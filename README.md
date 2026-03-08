# Speech Recognition AudioCapture

## Part 1 of the Speech Recognition System Series

Real-time 3-microphone beamforming and frequency analysis system for Raspberry Pi Pico (RP2040).

## Overview

This project implements a dual-core audio processing pipeline that:

- Captures audio from 3 microphones at 16 kHz with 12-bit precision
- Uses 31 mm microphone spacing (center-to-center, linear 3-mic array)
- Performs delay-and-sum beamforming for 5 directional beams (-60°, -30°, 0°, +30°, +60°)
- Computes 256-point FFT for frequency analysis (0-5500 Hz)
- Transmits 45 frequency bands per beam via I2C to downstream processors (8-bit)

Terminology convention used in this project:

- ADC front-end values: `mic` / `channel`.
- Post-beamforming values: `beam` / `beamformed channel`.
- Post-FFT values: `bin`.
- I2C payload values: `band`.
- 2D indexing convention: FFT arrays are `[beam][bin]`; I2C-output arrays are `[beam][band]`.

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

Each beam transmits a 46-byte packet:

- Byte 0: `0xAA` (header)
- Bytes 1-45: 45 frequency bands (8-bit)

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
- **Frequency range**: 0-5500 Hz (45 bins)
- **Processing**: ~16ms per 5-beam cycle
- **I2C throughput**: ~5ms for all beams @ 400 kHz

## Beamforming Delay Model (Integer Math)

The beamformer uses integer sample shifts, with two effects combined:

- **Geometric steering delay** from microphone spacing and beam angle
- **Fixed ADC round-robin skew** compensation for sequential channel sampling

### Why these delays work (beginner intuition)

Sound is not instantaneous: it propagates through air at about $c=343\text{ m/s}$ (around room temperature).

At this project sample rate, $f_s=16\text{ kHz}$, one sample period is:

$$
T_s=\frac{1}{f_s}=62.5\,\mu s
$$

In one sample period, sound travels:

$$
d_{sample}=c\,T_s=\frac{343}{16000}\text{ m}\approx 21.4\text{ mm}
$$

Because adjacent microphones are spaced by $d=31\text{ mm}$, a wave from the side reaches one mic first, then the next after a small delay.
The adjacent-mic geometric delay is:

$$
\Delta t(\theta)=\frac{d\,\sin\theta}{c}
$$

Equivalent delay in samples:

$$
\Delta n(\theta)=\Delta t\,f_s=\frac{d\,\sin\theta}{c}f_s
$$

Useful numbers for this geometry:

- $\theta=0^\circ$: $\Delta n\approx 0$ (arrives nearly together)
- $\theta=30^\circ$: $\Delta n\approx 0.72$ samples between adjacent mics
- $\theta=60^\circ$: $\Delta n\approx 1.25$ samples between adjacent mics
- End-to-end (mic0 to mic2) delay is about $2\times$ adjacent delay

Beamforming applies compensating delays so the chosen-direction waveforms line up in time before summing.
When aligned, they add constructively (larger output). For other directions, timing mismatch creates phase mismatch, so summation is partially destructive (reduced output). That is the directional selectivity.

Simple arrival-order sketch (3 mics in a line):

```text
Mic positions:  Mic0 -------- 31 mm -------- Mic1 -------- 31 mm -------- Mic2

Source at -60° (from left side):
Wave travel --->  hits Mic0 first, then Mic1, then Mic2
				 t0            t0+Δt         t0+2Δt

Source at +60° (from right side):
Wave travel <---  hits Mic2 first, then Mic1, then Mic0
				 t0            t0+Δt         t0+2Δt

Beam steering idea:
- To listen LEFT (-60°): delay Mic0 least and Mic2 most, so all three align before sum.
- To listen RIGHT (+60°): delay Mic2 least and Mic0 most, so all three align before sum.
- For other angles, signals are not perfectly aligned, so summed amplitude drops.

Phase intuition: when peaks line up with peaks, amplitudes add; when peaks line up with troughs, they partially cancel.
```

Round-robin ADC timing offset at 16 kHz/channel is:

- Mic 0: 0 samples
- Mic 1: +1/3 sample
- Mic 2: +2/3 sample

The implemented integer shift is computed as:

$$
\\text{delay}[\\text{beam},\\text{mic}] = -\\mathrm{round}\\left(\\left(\\tau_{geom}-\\tau_{skew}\\right)f_s\\right)
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

- **Mic / ADC input stage**
- `micgain M P` → set microphone gain for mic `M` (`0..2`) in percent `P` (`25..800`, starts at `150%`)
- `rawclip L H` → set RAW ADC clip window in mV (`L=low`, `H=high`, default `150 3150`)
- `micstatus` → show compact per-mic gain + clipping status
- `raw on|off` → show/hide RAW ADC table

- **Beamforming stage**
- `skew on|off` → enable/disable ADC round-robin skew correction in beamforming
- `beam on|off` → show/hide BEAMFORMED table
- `bgain B N` → set gain for one beam `B` (`0..4`) to `N` (`1..64`)
- `benable B on|off` → enable/disable processing for one beam `B` (`0..4`)

- **FFT stage**
- `fft on|off` → show/hide FFT section in periodic diagnostics
- `fftbeam B on|off` → enable/disable RAW FFT table for beam `B` (`0..4`, default only `B2` enabled)
- `fftraw on|off` → show/hide RAW FFT bin tables
- `fftrawfmt hex|pct` → RAW FFT display as 16-bit hex or `%` of full-scale (`65535`)
- `nfloor N` → set FFT post-bin noise floor for all beams (`0..255`, lower = more speech detail)
- `bnfloor B N` → set FFT post-bin noise floor for one beam `B` (`0..4`)
- `bfilter on|off` → enable/disable long-term per-bin background suppression (auto-tunes from 5s average)
- `bsec N` → set background suppression time constant in seconds (`1..8`)
- `bstrength N` → set background suppression subtraction strength for all beams (`0..100`)
- `bbstrength B N` → set subtraction strength for one beam `B` (`0..4`)
- `bcap N` → set max per-bin subtraction cap (`0..95`, lower preserves more speech)

- **Output stage**
- `i2clog on|off` → select I2C 16→8 mapping mode: logarithmic companding (ON) or linear mapping (OFF), default ON
- `i2coutput [B] on|off` → show/hide post-conversion I2C output table for beam `B` (`0..4`, if `B` omitted defaults to `2`, all beams start OFF)
- `i2coutputfmt hex|pct` → I2C output display as 8-bit hex or `%` of full-scale (`255`), with `%` shown in `3.1` format
- `i2c on|off` → show/hide I2C failure log prints
- `tone on|off` → show/hide tone monitor line
- `band N` or just `N` → set tone-monitor output band index (`0..44`) (not raw FFT bin `0..127`)
- `bin N` → alias for `band N` (legacy command)

- **General / diagnostics**
- `help` → show command list
- `status` → show current mode states
- `diagrate N` → set diagnostic print period in seconds (`1..60`)
- `diagcompact on|off` → switch periodic RAW/BEAM tables between compact and verbose formats
- `agc on|off` → currently disabled (gain control is manual)
- `agcclip N` → set AGC backoff threshold in clipped samples/frame (`1..64`)
- `savecfg` → force-save current mic/beam settings to flash
- `loadcfg` → reload settings from flash

Gain and band-filter behavior:

- Pre-FFT gain is manual and starts at `20x`.
- Each beam has independent gain, noise floor, and subtraction strength controls.
- Band-filter subtraction auto-adjusts from a ~`5s` average of each beam’s own raw FFT bins.
- Low 5s average increases suppression on that beam; high 5s average reduces suppression to preserve speech visibility.
- Mic gains are applied pre-beamforming with integer Q8 math and are persisted in flash.
- I2C packing performs the 16-bit-to-8-bit stage in one selectable mode: linear full-scale mapping (`i2clog off`) or logarithmic companding (`i2clog on`) to give low-amplitude bins more visual impact.
- Post-FFT bin filtering is currently bypassed while FFT tuning is in progress (for cleaner raw FFT diagnostics).

Per-beam field semantics (shown in diagnostics):

- `en`: enables/disables spectral processing for that beam. When OFF, that beam still participates in beamforming timing and packet flow, but FFT output bins are forced to zero.
- `nFloor`: per-beam FFT post-bin magnitude floor (`0..255` internal magnitude units). Bins below this threshold are zeroed.
- `bStr`: manual/base per-beam subtraction strength (% of estimated baseline).
- `bAuto`: runtime auto-adjusted subtraction strength (%), derived from `bStr` using each beam's ~5s average level.
- `clip`: clipped input sample count seen in the pre-FFT scaling stage for that beam.

Periodic diagnostic report order (single print burst per `diagrate`):

1. `STATUS` section
	- Shows ON/OFF state for diagnostic sections (`status`, `watchdog`, `raw`, `beam`, `fft`) and stream toggles.
2. `WATCHDOG` section
	- Shows ADC/DMA health snapshot (`dmaBusy`, block diff, DMA remaining count, DMA rearm count, queue depths).
3. `RAW ADC` section
	- Shows AGC state and clip threshold, then per-mic values:
	- manual gain, auto gain (currently fixed at 100%), average voltage, RAW/POST P2P min/avg/max, clip count.
4. `BEAMFORMED` section
	- Global line: AGC, skew compensation state, beam averaging time constant.
	- Per-beam rows: beam index, angle, mic delays (D0/D1/D2), gain, RAW/POST P2P min/max/avg, clip, `bStr`, `bAuto`.
5. `FFT` section
	- Shows RAW FFT diagnostics for enabled beams.
	- RAW table prints all 128 FFT bins in 8 rows × 16 columns.
	- Output-band bins are highlighted; lower single bins and upper paired bins are bracketed.
6. `I2C_OUTPUT` section
	- Shows post-conversion (16-bit FFT to 8-bit I2C payload) values for enabled beams.
	- Per-beam table header is `I2C_Output_Beam X`.
	- Displays 45 output bands using a 5×8 grid (`0..39`) plus a tail row (`40..44`).

Combined BEAMFORMED table:

- Periodic output now combines configuration and measured amplitude stats into one `BEAMFORMED (combined)` table when `beam on` is enabled.
- If `beam on` is OFF, the status stream still shows the compact `Per-beam` table.

RAW ADC diagnostics now include per-microphone:

- current mic gain (%),
- average input voltage (for bias verification),
- average voltage error from target bias (`1650 mV`),
- raw min/max absolute voltage (for clip-window checks),
- raw P2P level,
- post-gain P2P level,
- post-gain clipping count.

Useful tone band examples:

- `band 8`  → ~980 Hz
- `band 16` → ~1960 Hz
- `band 33` → ~4030 Hz

Approximate mapping:

$$f_{band}(n)\approx \frac{5500}{45}n\text{ Hz}\approx 122.2n\text{ Hz},\quad n=0\ldots44$$

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
- Core 1 runs a 256-point fixed-point FFT (Q15), computes magnitudes, and extracts 45 FFT bins.
- Core 0 transmits the 45-band packet over I2C to slaves at addresses 0x60–0x64.

## Frequency bin centers (Hz)

To avoid ambiguity:

- **FFT bins** = raw FFT indices ($k=0..127$) with spacing $\Delta f = f_s/N = 16000/256 = 62.5$ Hz.
- **Band 0..7** use FFT bins **1..8** directly (single-bin bands).
- **Band 8..42** use paired bins: (9,10), (11,12), ... (77,78).
- **Band 43..44** use single bins 79 and 80.
- FFT bin 0 is DC and excluded from output bands.
- **Output bands** = downstream values transmitted over I2C.

Representative band centers used by this mapping:

- Band 0 = 62.5 Hz (FFT bin 1)
- Band 7 = 500.0 Hz (FFT bin 8)
- Band 8 = 593.75 Hz (avg of FFT bins 9 and 10)
- Band 42 = 4843.75 Hz (avg of FFT bins 77 and 78)
- Band 43 = 4937.5 Hz (FFT bin 79)
- Band 44 = 5000.0 Hz (FFT bin 80)

### 45 Band Center Frequencies (Hz)

 | Band | Center (Hz) |
 |-----|-------------|
 | 0   | 62.5        |
 | 1   | 125.0       |
 | 2   | 187.5       |
 | 3   | 250.0       |
 | 4   | 312.5       |
 | 5   | 375.0       |
 | 6   | 437.5       |
 | 7   | 500.0       |
 | 8   | 593.75      |
 | 9   | 718.75      |
 | 10  | 843.75      |
 | 11  | 968.75      |
 | 12  | 1093.75     |
 | 13  | 1218.75     |
 | 14  | 1343.75     |
 | 15  | 1468.75     |
 | 16  | 1593.75     |
 | 17  | 1718.75     |
 | 18  | 1843.75     |
 | 19  | 1968.75     |
 | 20  | 2093.75     |
 | 21  | 2218.75     |
 | 22  | 2343.75     |
 | 23  | 2468.75     |
 | 24  | 2593.75     |
 | 25  | 2718.75     |
 | 26  | 2843.75     |
 | 27  | 2968.75     |
 | 28  | 3093.75     |
 | 29  | 3218.75     |
 | 30  | 3343.75     |
 | 31  | 3468.75     |
 | 32  | 3593.75     |
 | 33  | 3718.75     |
 | 34  | 3843.75     |
 | 35  | 3968.75     |
 | 36  | 4093.75     |
 | 37  | 4218.75     |
 | 38  | 4343.75     |
 | 39  | 4468.75     |
 | 40  | 4593.75     |
 | 41  | 4718.75     |
 | 42  | 4843.75     |
 | 43  | 4937.5      |
 | 44  | 5000.0      |

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
