# Speech Recognition AudioCapture

## Part 1 of the Speech Recognition System Series

Real-time 3-microphone beamforming and frequency analysis system for Raspberry Pi Pico (RP2040).

## Overview

This project implements a dual-core audio processing pipeline that:

- Captures audio from 3 microphones at 16 kHz with 12-bit precision
- Uses 31 mm microphone spacing (center-to-center, linear 3-mic array)
- Performs delay-and-sum beamforming for 5 directional beams (-60°, -30°, 0°, +30°, +60°)
- Computes 256-point FFT for frequency analysis (500-5500 Hz)
- Transmits 40 frequency bins per beam via I2C to downstream processors (8-bit)

## Architecture Highlights

### Dual-Core Parallelized Processing

**Core 0** (ADC + Beamforming + I2C)

- DMA-driven 3-channel ADC capture
- Delay-and-sum beamforming (5 beams)
- I2C transmission during idle time
- **Never blocks** Core 1's FFT computation

**Core 1** (FFT Only)

- 256-point Q15 fixed-point FFT
- Processes all 5 beams continuously
- Queues results for Core 0 transmission
- **No I2C delays** - maximum FFT throughput

### Key Benefits

- **Parallelized**: I2C transmission overlaps with next FFT cycle
- **No overruns**: ~5ms I2C time doesn't block FFT processing
- **Buffered**: Bidirectional 5-deep queues handle timing variations
- **High precision**: Full 12-bit ADC resolution preserved

## Hardware Requirements

- Raspberry Pi Pico (RP2040)
- 3× analog microphones (ADC-compatible)
- I2C slave devices at addresses 0x60-0x64 (optional, for data export)
- GPIO LEDs on pins 0-4 for error indication (optional)

## Pin Configuration

 | Pin       | Function    | Description                   |
 |-----------|-------------|-------------------------------|
 | 26-28     | ADC 0-2     | 3 microphone inputs           |
 | 8-9       | I2C SDA/SCL | FFT data export (@400kHz)     |
 | 0-4       | GPIO OUT    | I2C error LEDs (one per beam) |
 | 5, 10, 11 | GPIO OUT    | Debug timing signals          |
 |-----------|-------------|-------------------------------|

## Building

```bash
# Using VS Code task
Ctrl+Shift+B

# Or command line
ninja -C build
```

## Flashing

```bash
# Via picotool
picotool load build/Speech_Recognition_AudioCapture.uf2 -fx

# Or drag-and-drop .uf2 to BOOTSEL drive
```

## I2C Protocol

Each beam transmits a 41-byte packet:

- Byte 0: `0xAA` (header)
- Bytes 1-40: 40 frequency bins (8-bit)

**Slave addresses:**

- 0x60 = Beam -60°
- 0x61 = Beam -30°
- 0x62 = Beam 0°
- 0x63 = Beam +30°
- 0x64 = Beam +60°

## Performance Metrics

- **Sample rate**: 16 kHz per channel
- **Mic spacing**: 31 mm (adjacent microphones)
- **ADC precision**: 12-bit (0-4095)
- **FFT size**: 256 points
- **Frequency range**: 500-5500 Hz (40 bins)
- **Processing**: ~16ms per 5-beam cycle
- **I2C throughput**: ~5ms for all beams @ 400 kHz

## Beamforming Delay Model (Integer Math)

The beamformer uses integer sample shifts, with two effects combined:

- **Geometric steering delay** from microphone spacing and beam angle
- **Fixed ADC round-robin skew** compensation for sequential channel sampling

Round-robin ADC timing offset at 16 kHz/channel is:

- Mic 0: 0 samples
- Mic 1: +1/3 sample
- Mic 2: +2/3 sample

The implemented integer shift is computed as:

$$
	ext{delay}[\text{beam},\text{mic}] = -\mathrm{round}\left(\left(\tau_{geom}-\tau_{skew}\right)f_s\right)
$$

With 31 mm spacing, this yields the following delay table (samples):

| Beam angle | Mic 0 | Mic 1 | Mic 2 |
|------------|-------|-------|-------|
| -60°       | 0     | +2    | +3    |
| -30°       | 0     | +1    | +2    |
| 0°         | 0     | 0     | +1    |
| +30°       | 0     | 0     | -1    |
| +60°       | 0     | -1    | -2    |

### Geometric-Only vs Skew-Compensated Delays

If you ignore ADC round-robin skew, a beginner intuition is often:

| Beam angle | Mic 0 | Mic 1 | Mic 2 |
|------------|-------|-------|-------|
| -60°       | 0     | +2    | +3    |
| -30°       | 0     | +1    | +2    |
| 0°         | 0     | 0     | 0     |
| +30°       | 0     | -1    | -2    |
| +60°       | 0     | -2    | -3    |

The firmware table differs because it compensates the deterministic ADC channel skew (CH1 sampled +1/3 sample later than CH0, CH2 sampled +2/3 sample later than CH0).

### Equivalent Angle Shift from ADC Skew

At this geometry ($d=31\text{ mm}$, $f_s=16\text{ kHz}$), a $1/3$ sample timing offset is roughly equivalent to a steering offset of about $13^\circ$:

$$
\sin(\theta_{eq}) \approx \frac{(1/3)}{(d\,f_s/c)} = \frac{1/3}{1.446} \Rightarrow \theta_{eq}\approx 13.3^\circ
$$

So the skew term effectively shifts the integer-delay solution by about $\pm13^\circ$ unless compensated (which this firmware does).

## Debug & Monitoring

**GPIO timing signals:**

- GPIO 10: Pulses on each FFT completion (5× per cycle)
- GPIO 11: Pulses at start/end of 5-beam cycle
- GPIO 5: Core 0 activity heartbeat

**USB serial monitor** (115200 baud):

- Core 0/1 status messages
- Queue full warnings
- Cycle counters

### Runtime Serial Diagnostic Commands (Beginner)

You can enable/disable debug streams at runtime without reflashing:

- `help` → show command list
- `status` → show current mode states
- `raw on|off` → show/hide RAW ADC table
- `beam on|off` → show/hide BEAMFORMED table
- `tone on|off` → show/hide tone monitor line
- `i2c on|off` → show/hide I2C failure log prints
- `bin N` or just `N` → set tone-monitor FFT bin (`0..39`)

Useful tone bin examples:

- `bin 4`  → ~1000 Hz
- `bin 12` → ~2000 Hz
- `bin 28` → ~4000 Hz

Approximate mapping:

$$f_{bin}(n)\approx 500+125n\text{ Hz}$$

### Expected RAW ADC Values (What "Good" Looks Like)

For correctly biased single-supply microphone signals into RP2040 ADC:

- `AVG(mV)` for CH0/CH1/CH2 should be near mid-rail (typically ~1600–1700 mV)
- `MIN(mV)` should stay above 0 mV
- `MAX(mV)` should stay below 3300 mV
- `P2P(mV)` should increase clearly when tone/speech is present

Why these limits matter:

- RP2040 ADC input range is 0–3.3 V (no negative input range)
- Signals outside this range clip and distort beamforming/FFT
- See RP2040 datasheet (ADC/electrical limits) and Raspberry Pi Pico datasheet (board electrical characteristics)

If `AVG(mV)` is far from ~1.65 V, adjust analog bias network before tuning beamforming.

## How it works (summary)

- Core 0 continuously captures 3-channel ADC data via DMA at 16 kHz.
- Core 0 performs delay-and-sum beamforming for 5 angles (-60°, -30°, 0°, +30°, +60°).
- Core 1 runs a 256-point fixed-point FFT (Q15), computes magnitudes, and extracts 40 frequency bins.
- Core 0 transmits the 40-bin packet over I2C to slaves at addresses 0x60–0x64.

## Frequency bin centers (Hz)

Derived from $f_s=16000$, $N=256$, start bin 8 (500 Hz), with 40 output bins formed by summing pairs of FFT bins across the 500–5500 Hz range. Approximate bin centers are:

$$f_{bin}(n) = 500 + 125n\;\text{Hz},\quad n=0\ldots39$$

### 40 Bin Center Frequencies (Hz)

 | Bin | Center (Hz) |
 |-----|-------------|
 | 0   | 500         |
 | 1   | 625         |
 | 2   | 750         |
 | 3   | 875         |
 | 4   | 1000        |
 | 5   | 1125        |
 | 6   | 1250        |
 | 7   | 1375        |
 | 8   | 1500        |
 | 9   | 1625        |
 | 10  | 1750        |
 | 11  | 1875        |
 | 12  | 2000        |
 | 13  | 2125        |
 | 14  | 2250        |
 | 15  | 2375        |
 | 16  | 2500        |
 | 17  | 2625        |
 | 18  | 2750        |
 | 19  | 2875        |
 | 20  | 3000        |
 | 21  | 3125        |
 | 22  | 3250        |
 | 23  | 3375        |
 | 24  | 3500        |
 | 25  | 3625        |
 | 26  | 3750        |
 | 27  | 3875        |
 | 28  | 4000        |
 | 29  | 4125        |
 | 30  | 4250        |
 | 31  | 4375        |
 | 32  | 4500        |
 | 33  | 4625        |
 | 34  | 4750        |
 | 35  | 4875        |
 | 36  | 5000        |
 | 37  | 5125        |
 | 38  | 5250        |
 | 39  | 5375        |

## Documentation

See [.github/copilot-instructions.md](.github/copilot-instructions.md) for detailed architecture, algorithms, and beginner-friendly explanations.

## Part of Speech Recognition System

This is the first module in a series:

1. **AudioCapture** (this project) - Beamforming and frequency analysis
2. *Feature extraction* - TBD
3. *Recognition engine* - TBD
4. *Command interface* - TBD

## License

MIT License - See LICENSE file

## Authors

Ray Edgley & Copilot.

Developed as part of a modular speech recognition system for embedded platforms.
