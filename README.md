# Speech Recognition AudioCapture

**Part 1 of the Speech Recognition System Series**

Real-time 3-microphone beamforming and frequency analysis system for Raspberry Pi Pico (RP2040).

## Overview

This project implements a dual-core audio processing pipeline that:
- Captures audio from 3 microphones at 16 kHz with 12-bit precision
- Performs delay-and-sum beamforming for 5 directional beams (-60°, -30°, 0°, +30°, +60°)
- Computes 256-point FFT for frequency analysis (500-5000 Hz)
- Transmits 20 frequency bins per beam via I2C to downstream processors

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

| Pin | Function | Description |
|-----|----------|-------------|
| 26-28 | ADC 0-2 | 3 microphone inputs |
| 8-9 | I2C SDA/SCL | FFT data export (@400kHz) |
| 0-4 | GPIO OUT | I2C error LEDs (one per beam) |
| 5, 10, 11 | GPIO OUT | Debug timing signals |

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
- Bytes 1-40: 20 frequency bins (16-bit big-endian)

**Slave addresses:**
- 0x60 = Beam -60°
- 0x61 = Beam -30°
- 0x62 = Beam 0°
- 0x63 = Beam +30°
- 0x64 = Beam +60°

## Performance Metrics

- **Sample rate**: 16 kHz per channel
- **ADC precision**: 12-bit (0-4095)
- **FFT size**: 256 points
- **Frequency range**: 500-5000 Hz (20 bins)
- **Processing**: ~16ms per 5-beam cycle
- **I2C throughput**: ~5ms for all beams @ 400 kHz

## Debug & Monitoring

**GPIO timing signals:**
- GPIO 10: Pulses on each FFT completion (5× per cycle)
- GPIO 11: Pulses at start/end of 5-beam cycle
- GPIO 5: Core 0 activity heartbeat

**USB serial monitor** (115200 baud):
- Core 0/1 status messages
- Queue full warnings
- Cycle counters

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

Ray Edgley and Copilot.
Developed as part of a modular speech recognition system for embedded platforms.

