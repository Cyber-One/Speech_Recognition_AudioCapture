# Copilot Instructions for Speech Recognition AudioCapture

## What Is This Project? 🎤

This is a **real-time audio beamforming and frequency analysis system** built on a Raspberry Pi Pico microcontroller. It listens to sounds through 3 microphones, determines where the sound is coming from (direction), and analyzes its frequency content.

**Think of it like**: A smart microphone that not only hears sound, but also figures out which direction it's coming from and what frequencies are present (like knowing if you're hearing bass, mid-range, or treble).

### Real-World Applications
- **Voice-activated devices** that can identify which person in a group is speaking
- **Audio surveillance** systems that can locate where sounds originate
- **Sound source tracking** for robotics and autonomous systems
- **Speech recognition preprocessing** to enhance audio from a specific direction

## How It Works (Beginner Overview)

### The Hardware Setup
```
Microphone 1 -----> ADC Channel 0
Microphone 2 -----> ADC Channel 1  (spaced 50mm apart)
Microphone 3 -----> ADC Channel 2
```
Three microphones are placed in a line, 50mm (about 2 inches) apart.

### The Processing Pipeline (Simplified)
1. **Capture**: Read microphone signals at 16 kHz (16,000 times per second) with 12-bit precision
2. **Beamform**: Apply time delays to create 5 virtual "directional listeners" at angles: -60°, -30°, 0°, +30°, +60°
3. **Analyze**: Run FFT (Fast Fourier Transform) on each beam to extract frequency information
4. **Send Out**: Transmit frequency data via I2C to other processors

### Processing Flow Diagram
```
┌─────────────────────────────────────────────────────────────────┐
│                           CORE 0                                 │
│  ┌──────────┐    ┌────────────┐    ┌──────────────┐           │
│  │   DMA    │───>│ Beamforming│───>│ Queue to C1  │           │
│  │ ADC FIFO │    │  (5 beams) │    │ (beamformed) │           │
│  └──────────┘    └────────────┘    └──────────────┘           │
│       ↑                                      ↓                   │
│   3 mics                          ┌─────────────────┐           │
│  @ 16kHz                          │  Idle Time?     │           │
│  12-bit                           │  Check FFT      │           │
│                                   │  Result Queue   │           │
│                                   └────────┬────────┘           │
│                                            ↓                     │
│                                   ┌─────────────────┐           │
│                                   │  I2C Transmit   │──> 5 Slaves
│                                   │   (when ready)  │   0x60-0x64
│                                   └─────────────────┘           │
└─────────────────────────────────────────────────────────────────┘
                                    ↕ Queues ↕
┌─────────────────────────────────────────────────────────────────┐
│                           CORE 1                                 │
│  ┌──────────────┐    ┌────────────┐    ┌──────────────┐        │
│  │ Queue from C0│───>│ 256-pt FFT │───>│ Queue to C0  │        │
│  │ (beamformed) │    │  (5 beams) │    │ (FFT results)│        │
│  └──────────────┘    └────────────┘    └──────────────┘        │
│                      Q15 Fixed-Point                             │
│                      40 freq bins                                │
│                      500-5500 Hz                                 │
└─────────────────────────────────────────────────────────────────┘

Key Benefits:
• FFT never blocks on I2C (Core 1 runs at max speed)
• I2C uses Core 0's idle time (parallelized processing)
• Bidirectional queues buffer bursts and timing variations
```

### Why Beamforming Works
When sound comes from a specific direction, it reaches the microphones at **slightly different times**. By delaying each microphone's signal appropriately and adding them together, we can "point" the system in any direction. This is called **delay-and-sum beamforming**:
- Sound from the **right** arrives at mic 3 first, then mic 2, then mic 1 → delay the right mics
- Sound from the **left** arrives at mic 1 first → delay the left mics
- Sound from **straight ahead** arrives at all 3 simultaneously → no delay

### The Frequency Analysis (FFT)
FFT (Fast Fourier Transform) converts time-domain audio into **frequency-domain** representation:
- Input: 256 audio samples over time
- Output: 40 frequency bins representing power in each frequency range (500-5500 Hz)
- This range covers human speech well (formants, consonants, vowels)

---

## System Architecture

### Multi-Core Design (Dual-Core RP2040)
The system splits work between **two CPU cores** for maximum performance with **parallelized processing**:

**Core 0: Audio Capture + Beamforming + I2C Transmission**
- Reads ADC samples from 3 microphones continuously via DMA (time-critical)
- Performs delay-and-sum beamforming to create 5 directional "beams"
- Queues beamformed data for Core 1 to process
- **NEW**: Handles I2C transmission of FFT results from Core 1
- Uses idle time (after beamforming) to transmit data without blocking Core 1

**Core 1: FFT Processing Only**
- Waits for beamformed data from Core 0
- Runs 256-point FFT analysis on each of 5 beams (computationally intensive)
- Queues FFT results for Core 0 to transmit via I2C
- **Never blocks on I2C** - can continuously process FFT at maximum speed

**Why This Architecture?**
- **Core 0**: Time-critical ADC capture never interrupted + idle time used for I2C
- **Core 1**: Heavy FFT computation runs at maximum speed without I2C delays
- **Parallelized**: While Core 1 computes next FFT, Core 0 transmits previous results
- **Prevents overruns**: I2C transmission time (205 bytes @ 400 kHz) no longer blocks FFT
- **Bidirectional queues**: Beamformed data (Core 0→1) and FFT results (Core 1→0)

### Components

**Analog Input Capture (ADC)**
- 3 parallel ADC channels sample microphone signals
- Sample rate: 16 kHz (16,000 samples per second per channel)
- **12-bit resolution** (0-4095 range) for optimal SNR and dynamic range
- No byte-shift: preserves full ADC precision for FFT processing
- Uses DMA (Direct Memory Access) for automatic background sampling without CPU overhead

**Data Buffering (DMA + Circular Buffer)**
- DMA continuously streams ADC samples to a circular buffer in RAM
- 4096-sample buffer (4 ms of audio at 16 kHz × 3 channels)
- No CPU involvement; hardware handles the copying

**Beamforming (Delay-and-Sum)**
- Pre-computed delays for 5 angles: -60°, -30°, 0°, +30°, +60°
- For each beam: shift each microphone signal by calculated delay, then average
- Produces 5 "virtual directional microphones"

**FFT Processing (Fixed-Point)**
- 256-point Cooley-Tukey radix-2 FFT
- Uses Q15 fixed-point math (no floating-point, faster on Cortex-M0+)
- Pre-computed sine lookup table (128 values) with symmetry optimization
- Outputs magnitude spectrum (power at each frequency)

**I2C Export (Inter-Processor Communication)**
- Master I2C on Pico sends FFT results to 5 remote RP2040 slave processors
- **Handled by Core 0** during idle time (after beamforming completes)
- Each beam sends to a different slave: 0x60, 0x61, 0x62, 0x63, 0x64
- Packet: header byte (0xAA) + 20 frequency bins (2 bytes each) = 41 bytes total
- Transmission time: ~1 ms per beam @ 400 kHz (5 ms total if all slaves responding)
- Graceful error handling: if slave doesn't respond, light error LED and continue
- Non-blocking for Core 1: FFT computation never waits for I2C

**UART Monitor (Debugging Output)**
- Real-time stream of FFT data at 115200 baud
- Useful for viewing live frequency analysis on a serial monitor

**Error Indication (GPIO LEDs)**
- GPIO pins 0-4 indicate I2C communication failures for each beam
- LED turns ON if corresponding slave device doesn't acknowledge
- 5 LEDs total: one per beam angle

---

## Developer Workflows

### Building the Project
```bash
# Option 1: VS Code task (easiest)
- Press Ctrl+Shift+B or click "Compile Project" in Tasks menu

# Option 2: Command line
ninja -C build
```

### Flashing to Pico
```bash
# Via VS Code "Flash" task (requires CMSIS-DAP debugger)
# Or via picotool
picotool load build/Speech_Recognition_AudioCapture.uf2 -fx
```

### Debugging
- Use OpenOCD with CMSIS-DAP debugger for GDB debugging
- Use "Rescue Reset" task if Pico becomes unresponsive

### Serial Monitoring (Viewing Live Data via USB)
```bash
# USB serial output via CDC (no additional hardware needed)
# Connect USB cable from Pico to computer
# The device will appear as a virtual COM port
# Open with any serial monitor at 115200 baud (or just read the USB CDC output)
# Tools: PuTTY, minicom, VS Code Serial Monitor extension, or any USB serial terminal
# Output: 205 bytes per FFT cycle (5 beams × 41-byte packets)
```

### I2C Testing
- Monitor GPIO 0-4 for error LEDs (light = slave not responding for that beam)
- Use logic analyzer on GPIO 8/9 (I2C bus) to verify packet transmission

### Performance & Timing Measurement
**Debug GPIO pins for oscilloscope/logic analyzer:**
- **GPIO 10 (FFT Complete)**: Toggles every time a single FFT completes (5 toggles per full processing cycle)
- **GPIO 11 (At start and All Beams Complete)**: Pulses every time all 5 FFT processes complete (2 pulses per full processing cycle)

**Expected timing relationship:**
- GPIO 10 should toggle **2.5 times faster** than GPIO 11 if processing is working correctly
- Use an oscilloscope or logic analyzer to measure:
  - Frequency ratio between the two signals (should be ~2.5:1)
  - Total cycle time: Time between GPIO 11 toggles = 1 full processing cycle
  - Individual FFT time: Time between GPIO 10 toggles = 1/5 of full cycle
  - This helps diagnose performance issues or queue bottlenecks

---

## Key Implementation Details

### ADC Setup
- 3-channel round-robin sampling at 16 kHz per channel
- **12-bit precision** retained (byte_shift disabled for full resolution)
- ADC clock divider calculated to achieve target rate: 48 MHz / (3 × 16 kHz)
- Free-running mode with DMA triggering
- DMA configured for 16-bit transfers to match ADC FIFO width

### Beamforming Algorithm
1. Calculate time delay (in samples) for each microphone pair and angle
2. For each output sample: delay each channel's signal and sum/average
3. Clamp output to valid ADC range

### Fixed-Point FFT
- Q15 format: 16-bit signed values representing -1.0 to +0.9999
- Twiddle factors pre-computed and stored in lookup table
- Bit-reversal permutation for input ordering
- 8 stages of butterfly operations (log₂ 256)

### I2C Communication
- Standard I2C protocol at 400 kHz
- Packet: 41 bytes (0xAA header + 40 bins × 1 byte)
- **Handled by Core 0**: Transmission parallelized with Core 1's FFT computation
- Core 1 queues FFT results; Core 0 transmits during its idle time
- Blocking write (per packet); on NACK the corresponding error LED stays ON
- Total transmission time: ~5 ms for all 5 beams if all slaves respond
- Prevents FFT timing overruns by moving I2C off the critical FFT path

### Inter-Core Queues (Bidirectional)

**Beamformed Data Queue (Core 0 → Core 1)**
- 5-buffer circular queue for beamformed audio samples
- Core 0 produces after beamforming; Core 1 consumes for FFT
- If queue fills, oldest data is dropped (indicates Core 1 can't keep up)

**FFT Result Queue (Core 1 → Core 0)**
- 5-buffer circular queue for FFT frequency bins
- Core 1 produces after FFT computation; Core 0 consumes for I2C transmission
- If queue fills, indicates Core 0's I2C transmission is falling behind
- Allows burst absorption during temporary I2C delays

### Debug Timing Signals
- **GPIO 10**: Pulses at the start/end of each beam FFT (5 pulses per cycle)
- **GPIO 11**: Pulses at start and completion of the 5-beam batch (cycle marker)
- **GPIO 5**: Core 0 heartbeat/per-beam activity indicator
- Useful for real-time performance measurement with oscilloscope/logic analyzer

---

## Pin Assignments

| Pin | Function | Purpose |
|-----|----------|---------|
| 26 | ADC CH0 | Microphone 1 input |
| 27 | ADC CH1 | Microphone 2 input |
| 28 | ADC CH2 | Microphone 3 input |
| 8  | I2C SDA | I2C serial data (Core 0 transmits to slaves) |
| 9  | I2C SCL | I2C serial clock (Core 0 controls) |
| 0  | GPIO OUT | Error LED for beam -60° (Core 0 updates) |
| 1  | GPIO OUT | Error LED for beam -30° (Core 0 updates) |
| 2  | GPIO OUT | Error LED for beam 0° (Core 0 updates) |
| 3  | GPIO OUT | Error LED for beam +30° (Core 0 updates) |
| 4  | GPIO OUT | Error LED for beam +60° (Core 0 updates) |
| 5  | GPIO OUT | Debug: Core 0 heartbeat / per-beam activity |
| 10 | GPIO OUT | Debug: Core 1 FFT complete (each beam) |
| 11 | GPIO OUT | Debug: Core 1 All beams complete (start/end of cycle) |

---

## I2C Packet Structure

**Message Format to Slaves:**
```
Byte 0:     0xAA (packet header for sync)
Bytes 1-40: 20 frequency bins (2 bytes each, big-endian)
```

**Frequency Mapping:**
- Bin 0: ~500 Hz
- Bin 1: ~725 Hz
- ...
- Bin 19: ~5000 Hz
- Resolution: ~11.7 Hz per bin ((5000-500) / 20)

**Slave Addresses:**
- 0x60 = Beam -60°
- 0x61 = Beam -30°
- 0x62 = Beam 0° (center)
- 0x63 = Beam +30°
- 0x64 = Beam +60°

---

## Conventions & Hardware Constants

```c
// Sampling
TARGET_SAMPLE_RATE      = 16000   // Hz
CAPTURE_DEPTH           = 4096    // samples per buffer

// Beamforming
MIC_SPACING             = 0.05    // meters (50 mm)
SPEED_OF_SOUND          = 343.0   // m/s at 20°C
BEAM_ANGLES             = -60°, -30°, 0°, 30°, 60°

// FFT
FFT_SIZE                = 256     // samples
NUM_FREQ_BINS           = 20      // output bins
FREQ_MIN                = 500     // Hz
FREQ_MAX                = 5000    // Hz

// I2C
I2C_BAUDRATE            = 400000  // 400 kHz
I2C_PACKET_SIZE         = 41      // bytes
I2C_TIMEOUT             = 10000   // microseconds

// UART
BAUD_RATE               = 115200  // baud
```

---

## Debugging Tips

1. **No FFT output via USB serial?**
   - Ensure USB cable is connected from Pico to computer
   - Verify the device appears as a virtual COM port
   - Open serial monitor at 115200 baud
   - Check if Core 1 started (look for startup message in serial output)

2. **I2C error LEDs lighting up?**
   - Verify slave devices are powered and connected
   - Check I2C pull-ups on SDA/SCL
   - Look for bus collisions with logic analyzer
   - Note: GPIO 0 corresponds to beam -60°, GPIO 1 to -30°, GPIO 2 to 0°, GPIO 3 to +30°, GPIO 4 to +60°

3. **Dropped beamformed blocks?**
   - Core 1 (FFT) is too slow, beamform queue filling up
   - Try increasing QUEUE_SIZE in code
   - Check for FFT optimization opportunities

4. **FFT queue full errors?**
   - Core 0 can't transmit I2C fast enough, FFT result queue filling
   - Check if I2C slaves are responding (error LEDs)
   - Verify I2C bus speed (should be 400 kHz)
   - Increase FFT_QUEUE_SIZE if burst absorption needed

4. **Noisy/garbled audio analysis?**
   - Check microphone connections
   - Verify ADC is sampling at correct rate
   - Ensure no electromagnetic interference near microphones

---

## Critical Files

- **`Speech_Recognition_AudioCapture.c`**: Main firmware with all processing logic
- **`CMakeLists.txt`**: Build configuration (links Pico SDK libraries)
- **`pico_sdk_import.cmake`**: Pico SDK integration script

## Build Dependencies

Linked Pico SDK libraries:
- `hardware_adc`: ADC peripheral driver
- `hardware_dma`: DMA controller driver
- `hardware_i2c`: I2C peripheral driver
- `hardware_uart`: UART peripheral driver
- `pico_multicore`: Multi-core synchronization
- `pico_stdlib`: Standard library and utilities
- `m` (libm): Math library (sin, cos, sqrt, etc.)

---

## How it works (summary)

- Core 0 continuously captures 3-channel ADC data via DMA at 16 kHz.
- Core 0 performs delay-and-sum beamforming for 5 angles (-60°, -30°, 0°, +30°, +60°).
- Core 1 runs a 256-point fixed-point FFT (Q15), computes magnitudes, and extracts 40 frequency bins.
- Core 0 transmits the 40-bin packet over I2C to slaves at addresses 0x60–0x64.

## Frequency bin centers (Hz)

Derived from $f_s=16000$, $N=256$, start bin 8 (500 Hz), with 40 output bins formed by summing pairs of FFT bins across the 500–5500 Hz range. Approximate bin centers are:

$$f_{bin}(n) = 500 + 125n\;\text{Hz},\quad n=0\ldots39$$
