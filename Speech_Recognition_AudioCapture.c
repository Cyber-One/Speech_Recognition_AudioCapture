#include <stdio.h>
#include <math.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/stdio_usb.h"
#include "hardware/i2c.h"
#include "hardware/dma.h"
#include "hardware/uart.h"
#include "hardware/adc.h"
#include "hardware/timer.h"
#include "hardware/sync.h"

/*
 * DUAL-CORE PROCESSING ARCHITECTURE:
 * 
 * Core 0 (ADC + Beamforming + I2C Transmission):
 *   1. DMA continuously captures 3-channel ADC data
 *   2. Extracts channels and performs beamforming (5 beams)
 *   3. Queues beamformed data for Core 1
 *   4. Handles I2C transmission of FFT results from Core 1
 *   -> Parallelizes I2C transmission with Core 1's FFT computation
 * 
 * Core 1 (FFT Processing):
 *   1. Waits for beamformed data from Core 0
 *   2. Computes 256-point FFT for each of 5 beams
 *   3. Queues FFT results for Core 0 to transmit
 *   -> No I2C blocking, can continuously process FFT
 * 
 * Benefits:
 *   - Prevents Core 1 FFT timing overruns during I2C transmission
 *   - Core 0's idle time (after beamforming) now used for I2C
 *   - Parallelized processing maximizes throughput
 */

// ADC Configuration (3 microphone channels in round-robin)
#define ADC_PIN_0 26  // Channel 0 (Microphone 1)
#define ADC_PIN_1 27  // Channel 1 (Microphone 2)
#define ADC_PIN_2 28  // Channel 2 (Microphone 3)
#define ADC_NUM_CHANNELS 3
#define ADC_CHANNEL_0 0
#define ADC_CHANNEL_1 1
#define ADC_CHANNEL_2 2

// Raw ADC debug monitor (prints what Pico actually sees at ADC inputs)
#define RAW_ADC_DEBUG 1
#define RAW_ADC_DEBUG_INTERVAL_CYCLES 62  // ~1 second at 16ms processing cycles

// Narrowband tone monitor (uses FFT bin magnitudes for beam direction testing)
// Set frequency to test sweep behavior (e.g. 1000, 2000, 4000 Hz)
#define TONE_MONITOR_ENABLE 1
#define TONE_MONITOR_FREQ_HZ 1000
#define TONE_MONITOR_INTERVAL_SETS 62      // print roughly once per second

// Sample rate configuration
#define TARGET_SAMPLE_RATE 16000  // 16 kHz for speech recognition
#define CAPTURE_DEPTH 256         // Samples per channel per DMA transfer (per mic)
// DMA ring must be power-of-two sized; choose 32KB ring (16K uint16_t samples)
#define DMA_RING_BITS 15
#define BUFFER_SIZE ((1 << DMA_RING_BITS) / sizeof(uint16_t))  // 16,384 halfwords
#define BLOCK_SAMPLES (CAPTURE_DEPTH * ADC_NUM_CHANNELS)       // 12,288 halfwords needed per processing block

// Beamforming configuration
#define SPEED_OF_SOUND 343.0f     // m/s at 20°C
#define MIC_SPACING 0.031f        // 31mm spacing between microphones
#define NUM_BEAMS 5               // 5 steered beams
#define MAX_DELAY_SAMPLES ((int)(MIC_SPACING / SPEED_OF_SOUND * TARGET_SAMPLE_RATE) + 1)

// FFT configuration for frequency analysis (Fixed-point Q15 implementation)
#define FFT_SIZE 256              // 256-point FFT for frequency resolution
#define NUM_FREQ_BINS 40          // Extract 40 frequency bins
#define FREQ_MIN 500              // 500 Hz lower bound
#define FREQ_MAX 5500             // 5500 Hz upper bound
#define BIN_WIDTH ((float)(FREQ_MAX - FREQ_MIN) / NUM_FREQ_BINS)  // ~125 Hz per bin

// Fixed-point Q15 format (16-bit signed: -1.0 to 0.9999)
#define Q15_SCALE 32767           // 2^15 - 1

// Twiddle factor lookup table (sine values for 256-point FFT)
// Pre-computed to avoid runtime sin() calls
static const int16_t twiddle_sin_lut[128] = {
    0, 402, 804, 1206, 1608, 2009, 2409, 2808, 3207, 3605, 4001, 4395, 
    4787, 5177, 5565, 5950, 6332, 6710, 7085, 7456, 7822, 8184, 8541, 8894, 
    9242, 9585, 9922, 10254, 10580, 10900, 11214, 11521, 11822, 12116, 12402, 
    12681, 12952, 13216, 13471, 13718, 13957, 14188, 14410, 14623, 14827, 15022, 
    15207, 15383, 15548, 15704, 15850, 15985, 16110, 16224, 16327, 16419, 16500, 
    16569, 16627, 16673, 16707, 16729, 16739, 16738, 16724, 16698, 16660, 16610, 
    16548, 16472, 16384, 16283, 16170, 16044, 15905, 15754, 15590, 15414, 15225, 
    15024, 14810, 14583, 14345, 14094, 13831, 13557, 13271, 12974, 12665, 12345, 
    12014, 11671, 11318, 10954, 10580, 10195, 9801, 9397, 8983, 8559, 8126, 
    7685, 7235, 6777, 6311, 5837, 5357, 4870, 4377, 3876, 3371, 2860, 
    2344, 1825, 1302, 776, 248
};

// Output buffers for each beamformed channel
uint16_t beam_output[NUM_BEAMS][CAPTURE_DEPTH];

// FFT output: 40 frequency bins for each of 5 beams
uint16_t fft_output[NUM_BEAMS][NUM_FREQ_BINS];

// FFT working buffers (fixed-point Q15)
int16_t fft_real[FFT_SIZE];
int16_t fft_imag[FFT_SIZE];
uint16_t fft_magnitude[FFT_SIZE];

// Beamforming angles: -60°, -30°, 0°, 30°, 60°
static const float beam_angles[NUM_BEAMS] = {-60.0f, -30.0f, 0.0f, 30.0f, 60.0f};

// Calculated delays in samples for each beam and microphone pair
// Positive delay means shift the signal backward in time
static int beam_delays[NUM_BEAMS][ADC_NUM_CHANNELS];
// Runtime debug switch: enable/disable ADC round-robin skew compensation in beam delay math
static bool beam_skew_comp_enabled = true;

// DMA circular buffer (power-of-two sized to match ring, aligned for DMA)
// Ring size = 2^15 = 32768 bytes, so buffer MUST be aligned to 32KB boundary
__attribute__((aligned(32768))) uint16_t adc_buffer[BUFFER_SIZE];
volatile uint32_t buffer_pos = 0;
int g_dma_chan = -1;  // Global DMA channel for access in main loop

// Core 0 processing buffers - moved to static storage to avoid stack overflow
static uint16_t ch0_data[CAPTURE_DEPTH];
static uint16_t ch1_data[CAPTURE_DEPTH];
static uint16_t ch2_data[CAPTURE_DEPTH];
static uint16_t local_beam_output[NUM_BEAMS][CAPTURE_DEPTH];

// I2C Configuration (for exporting beamformed FFT data)
#define I2C_PORT i2c0
#define I2C_SDA 8
#define I2C_SCL 9
#define I2C_BAUDRATE 400000       // 400 kHz for timely per-beam transfers
#define I2C_BEAM_BASE_ADDR 0x60   // Base I2C slave address (0x60, 0x61, 0x62, 0x63, 0x64)
#define I2C_TIMEOUT_US 10000      // 10ms timeout per transaction

// Error indicator GPIO pins (unused outputs for I2C failure indication)
#define ERROR_LED_PIN_0 0         // Beam 0 error
#define ERROR_LED_PIN_1 1         // Beam 1 error
#define ERROR_LED_PIN_2 2         // Beam 2 error
#define ERROR_LED_PIN_3 3         // Beam 3 error
#define ERROR_LED_PIN_4 4         // Beam 4 error

// Debug timing GPIO pins for oscilloscope/logic analyzer measurements
#define DEBUG_FFT_COMPLETE_PIN 10  // Toggles on each FFT completion (5x per cycle)
#define DEBUG_ALL_BEAMS_COMPLETE_PIN 11  // Toggles when all 5 beams complete (1x per cycle)

// I2C packet format: 1 byte header + 40 bins * 1 byte (40 bytes) = 41 bytes total
#define I2C_PACKET_HEADER 0xAA    // Packet header for identification
#define I2C_PACKET_SIZE 41        // Header (1) + FFT bins (40)
#define I2C_ERROR_SERIAL_LOG 0    // 0 = suppress per-failure serial spam, 1 = print failures

// USB Serial output (via stdio_init_all in main)

// I2C packet buffer
uint8_t i2c_packet[I2C_PACKET_SIZE];

// Inter-core communication: beamformed data queue
// Core 0 produces beamformed samples, Core 1 consumes for FFT
#define QUEUE_SIZE 5
typedef struct {
    uint16_t beam_data[NUM_BEAMS][CAPTURE_DEPTH];
    bool valid;
} BeamformedBlock;

BeamformedBlock beamform_queue[QUEUE_SIZE];
volatile int queue_write_idx = 0;
volatile int queue_read_idx = 0;
volatile int queue_count = 0;

static spin_lock_t *beam_queue_lock;

// Reverse queue: FFT results from Core 1 to Core 0 for I2C transmission
// Core 1 produces FFT results, Core 0 consumes and transmits via I2C
#define FFT_QUEUE_SIZE 5
typedef struct {
    uint16_t freq_bins[NUM_FREQ_BINS];  // FFT output for one beam
    int beam_idx;                        // Which beam (0-4)
    bool valid;
} FFTResult;

FFTResult fft_result_queue[FFT_QUEUE_SIZE];
volatile int fft_queue_write_idx = 0;
volatile int fft_queue_read_idx = 0;
volatile int fft_queue_count = 0;

static spin_lock_t *fft_queue_lock;

static bool queue_add_beamformed(uint16_t beam_data[NUM_BEAMS][CAPTURE_DEPTH]) {
    uint32_t irq_state = spin_lock_blocking(beam_queue_lock);
    if (queue_count >= QUEUE_SIZE) {
        spin_unlock(beam_queue_lock, irq_state);
        return false;
    }

    int write_idx = queue_write_idx;

    // Copy data while locked to avoid consumer seeing partial writes
    for (int beam = 0; beam < NUM_BEAMS; beam++) {
        for (int i = 0; i < CAPTURE_DEPTH; i++) {
            beamform_queue[write_idx].beam_data[beam][i] = beam_data[beam][i];
        }
    }
    beamform_queue[write_idx].valid = true;

    queue_write_idx = (queue_write_idx + 1) % QUEUE_SIZE;
    queue_count++;

    spin_unlock(beam_queue_lock, irq_state);
    return true;
}

static bool queue_get_beamformed(uint16_t beam_data[NUM_BEAMS][CAPTURE_DEPTH]) {
    uint32_t irq_state = spin_lock_blocking(beam_queue_lock);
    if (queue_count == 0) {
        spin_unlock(beam_queue_lock, irq_state);
        return false;
    }

    int read_idx = queue_read_idx;

    for (int beam = 0; beam < NUM_BEAMS; beam++) {
        for (int i = 0; i < CAPTURE_DEPTH; i++) {
            beam_data[beam][i] = beamform_queue[read_idx].beam_data[beam][i];
        }
    }
    beamform_queue[read_idx].valid = false;

    queue_read_idx = (queue_read_idx + 1) % QUEUE_SIZE;
    queue_count--;

    spin_unlock(beam_queue_lock, irq_state);
    return true;
}

// FFT result queue functions (Core 1 produces, Core 0 consumes)
static bool fft_queue_add(int beam_idx, uint16_t *freq_bins) {
    uint32_t irq_state = spin_lock_blocking(fft_queue_lock);
    if (fft_queue_count >= FFT_QUEUE_SIZE) {
        spin_unlock(fft_queue_lock, irq_state);
        return false;
    }

    int write_idx = fft_queue_write_idx;
    fft_result_queue[write_idx].beam_idx = beam_idx;
    for (int i = 0; i < NUM_FREQ_BINS; i++) {
        fft_result_queue[write_idx].freq_bins[i] = freq_bins[i];
    }
    fft_result_queue[write_idx].valid = true;

    fft_queue_write_idx = (fft_queue_write_idx + 1) % FFT_QUEUE_SIZE;
    fft_queue_count++;

    spin_unlock(fft_queue_lock, irq_state);
    return true;
}

static bool fft_queue_get(int *beam_idx, uint16_t *freq_bins) {
    uint32_t irq_state = spin_lock_blocking(fft_queue_lock);
    if (fft_queue_count == 0) {
        spin_unlock(fft_queue_lock, irq_state);
        return false;
    }

    int read_idx = fft_queue_read_idx;
    *beam_idx = fft_result_queue[read_idx].beam_idx;
    for (int i = 0; i < NUM_FREQ_BINS; i++) {
        freq_bins[i] = fft_result_queue[read_idx].freq_bins[i];
    }
    fft_result_queue[read_idx].valid = false;

    fft_queue_read_idx = (fft_queue_read_idx + 1) % FFT_QUEUE_SIZE;
    fft_queue_count--;

    spin_unlock(fft_queue_lock, irq_state);
    return true;
}

// Calculate time delay in seconds for a given angle and microphone pair
// angle: steering angle in degrees (-90 to +90)
// mic_idx: microphone index (0, 1, or 2)
// returns: delay in seconds
static float calculate_delay_seconds(float angle, int mic_idx) {
    // Convert angle to radians
    float angle_rad = angle * 3.14159265f / 180.0f;
    
    // Distance from reference (mic 0) to mic at mic_idx
    float distance = mic_idx * MIC_SPACING;
    
    // Time delay = distance * sin(angle) / speed_of_sound
    // Positive delay for positive angles (sound from right)
    float delay_sec = distance * sinf(angle_rad) / SPEED_OF_SOUND;
    
    return delay_sec;
}

// ADC runs round-robin CH0->CH1->CH2 at 48k conversions/s.
// At 16k samples/s/channel this creates fixed sampling skew:
// mic0 = 0 samples, mic1 = +1/3 sample, mic2 = +2/3 sample (later in time).
// Convert this deterministic skew into an equivalent per-channel time offset.
static float calculate_adc_skew_seconds(int mic_idx) {
    float skew_samples = (float)mic_idx / (float)ADC_NUM_CHANNELS;
    return skew_samples / (float)TARGET_SAMPLE_RATE;
}

// Symmetric round-to-nearest for signed values (avoids truncation bias around 0)
static int round_to_int(float x) {
    return (x >= 0.0f) ? (int)(x + 0.5f) : (int)(x - 0.5f);
}

// Initialize beamforming delay indices for all angles
static void init_beam_delays(void) {
    for (int beam = 0; beam < NUM_BEAMS; beam++) {
        float angle = beam_angles[beam];
        
        for (int mic = 0; mic < ADC_NUM_CHANNELS; mic++) {
            float steer_delay_sec = calculate_delay_seconds(angle, mic);
            float adc_skew_sec = beam_skew_comp_enabled ? calculate_adc_skew_seconds(mic) : 0.0f;

            // Effective delay = steering geometry delay - deterministic ADC sampling skew
            // Convert to integer sample shift for delay-and-sum indexing.
            float effective_delay_samples = (steer_delay_sec - adc_skew_sec) * TARGET_SAMPLE_RATE;
            int delay_samples = -round_to_int(effective_delay_samples);
            beam_delays[beam][mic] = delay_samples;
        }
    }
}

// Extract single-channel samples from interleaved ADC buffer
// mic: which microphone (0, 1, or 2)
// adc_data: pointer to start of ADC data
// num_samples: number of samples to extract
// output: output array for single channel
static void extract_channel(int mic, uint16_t *adc_data, int num_samples, uint16_t *output) {
    for (int i = 0; i < num_samples; i++) {
        output[i] = adc_data[i * ADC_NUM_CHANNELS + mic];
    }
}

// Extract the latest BLOCK_SAMPLES from the DMA ring and de-interleave into CH0/CH1/CH2.
// Uses absolute sample phase so channel ordering remains correct even when ring wraps.
static void extract_channels_from_ring(uint32_t end_pos_samples,
                                       uint16_t *ch0_out,
                                       uint16_t *ch1_out,
                                       uint16_t *ch2_out) {
    const uint32_t ring_mask = BUFFER_SIZE - 1;
    uint32_t start_sample = end_pos_samples - BLOCK_SAMPLES;
    uint32_t start_idx = start_sample & ring_mask;
    uint32_t phase = start_sample % ADC_NUM_CHANNELS;  // 0=CH0,1=CH1,2=CH2

    int idx0 = 0;
    int idx1 = 0;
    int idx2 = 0;

    for (uint32_t i = 0; i < BLOCK_SAMPLES; i++) {
        uint16_t v = adc_buffer[(start_idx + i) & ring_mask] & 0x0FFF;
        uint32_t ch = (phase + i) % ADC_NUM_CHANNELS;

        if (ch == 0) {
            if (idx0 < CAPTURE_DEPTH) ch0_out[idx0++] = v;
        } else if (ch == 1) {
            if (idx1 < CAPTURE_DEPTH) ch1_out[idx1++] = v;
        } else {
            if (idx2 < CAPTURE_DEPTH) ch2_out[idx2++] = v;
        }
    }
}

static inline float adc_counts_to_millivolts(uint16_t counts) {
    return ((float)counts * 3300.0f) / 4095.0f;
}

static void compute_adc_stats(uint16_t *samples, int num_samples, uint16_t *min_val, uint16_t *max_val, uint16_t *avg_val) {
    uint16_t local_min = 0x0FFF;
    uint16_t local_max = 0;
    uint32_t sum = 0;
    for (int i = 0; i < num_samples; i++) {
        uint16_t v = samples[i] & 0x0FFF;
        if (v < local_min) local_min = v;
        if (v > local_max) local_max = v;
        sum += v;
    }
    *min_val = local_min;
    *max_val = local_max;
    *avg_val = (uint16_t)(sum / (uint32_t)num_samples);
}

static int tone_monitor_bin_from_freq(int freq_hz) {
    float relative = ((float)freq_hz - (float)FREQ_MIN) / BIN_WIDTH;
    int idx = (int)(relative + 0.5f);
    if (idx < 0) idx = 0;
    if (idx >= NUM_FREQ_BINS) idx = NUM_FREQ_BINS - 1;
    return idx;
}

static float tone_monitor_hz_from_bin(int bin_idx) {
    if (bin_idx < 0) bin_idx = 0;
    if (bin_idx >= NUM_FREQ_BINS) bin_idx = NUM_FREQ_BINS - 1;
    return (float)FREQ_MIN + (BIN_WIDTH * (float)bin_idx);
}

static bool diag_raw_enabled = (RAW_ADC_DEBUG != 0);
static bool diag_beam_enabled = (RAW_ADC_DEBUG != 0);
static bool diag_tone_enabled = (TONE_MONITOR_ENABLE != 0);
static bool diag_i2c_error_log_enabled = (I2C_ERROR_SERIAL_LOG != 0);
static volatile int tone_monitor_bin_idx_runtime = 0;
// Runtime digital gain applied before FFT (Q15 conversion stage)
static volatile int fft_input_gain = 6;
static bool agc_enabled = true;
// Runtime shift used when packing 16-bit FFT bins to 8-bit I2C payload
// Smaller shift = more sensitive LCD response (speech visibility)
static volatile int i2c_output_shift = 6;

#define FFT_GAIN_MIN 1
#define FFT_GAIN_MAX 64
#define AGC_HEADROOM_Q15 ((int32_t)(32767 * 8 / 10))  // 80% of full-scale
#define AGC_RAMP_UP_HOLD_MS 2500

static volatile uint16_t agc_last_peak_q15 = 0;
static volatile uint32_t agc_last_clip_ms = 0;
static volatile uint32_t agc_stable_start_ms = 0;
static volatile uint32_t agc_clip_events = 0;

static bool parse_onoff(const char *s, bool *value_out) {
    if (!s || !value_out) return false;
    if ((s[0] == '1' && s[1] == '\0') ||
        ((s[0] == 'o' || s[0] == 'O') && (s[1] == 'n' || s[1] == 'N') && s[2] == '\0')) {
        *value_out = true;
        return true;
    }
    if ((s[0] == '0' && s[1] == '\0') ||
        ((s[0] == 'o' || s[0] == 'O') && (s[1] == 'f' || s[1] == 'F') && (s[2] == 'f' || s[2] == 'F') && s[3] == '\0')) {
        *value_out = false;
        return true;
    }
    return false;
}

static bool str_ieq(const char *a, const char *b) {
    while (*a && *b) {
        char ca = *a;
        char cb = *b;
        if (ca >= 'A' && ca <= 'Z') ca = (char)(ca - 'A' + 'a');
        if (cb >= 'A' && cb <= 'Z') cb = (char)(cb - 'A' + 'a');
        if (ca != cb) return false;
        a++;
        b++;
    }
    return (*a == '\0' && *b == '\0');
}

static void print_diag_status(void) {
    printf("DIAG status: raw=%s beam=%s tone=%s i2c=%s skew=%s agc=%s gain=%dx i2cShift=%d peak=%u bin=%d (~%.0f Hz)\n",
           diag_raw_enabled ? "ON" : "OFF",
           diag_beam_enabled ? "ON" : "OFF",
           diag_tone_enabled ? "ON" : "OFF",
           diag_i2c_error_log_enabled ? "ON" : "OFF",
           beam_skew_comp_enabled ? "ON" : "OFF",
           agc_enabled ? "ON" : "OFF",
           fft_input_gain,
           i2c_output_shift,
           agc_last_peak_q15,
           tone_monitor_bin_idx_runtime,
           tone_monitor_hz_from_bin(tone_monitor_bin_idx_runtime));
}

static void print_diag_help(void) {
    printf("DIAG commands:\n");
    printf("  help                 -> show this help\n");
    printf("  status               -> show current diagnostic mode status\n");
    printf("  raw on|off           -> enable/disable RAW ADC table\n");
    printf("  beam on|off          -> enable/disable BEAMFORMED table\n");
    printf("  tone on|off          -> enable/disable tone monitor line\n");
    printf("  i2c on|off           -> enable/disable I2C failure serial logs\n");
    printf("  skew on|off          -> enable/disable ADC skew correction in beamforming\n");
    printf("  agc on|off           -> enable/disable auto gain control\n");
    printf("  gain N               -> set pre-FFT digital gain multiplier (1-64)\n");
    printf("  i2cshift N           -> set 16->8 bit I2C pack shift (4-12, lower=more sensitive)\n");
    printf("  bin N                -> set tone monitor bin (0-%d)\n", NUM_FREQ_BINS - 1);
    printf("  N                    -> shorthand for bin N\n");
}

static void process_diag_command(const char *cmd_buf) {
    char arg[16] = {0};
    int new_bin = -1;

    if (sscanf(cmd_buf, "%d", &new_bin) == 1 || sscanf(cmd_buf, "bin %d", &new_bin) == 1 || sscanf(cmd_buf, "BIN %d", &new_bin) == 1) {
        if (new_bin >= 0 && new_bin < NUM_FREQ_BINS) {
            tone_monitor_bin_idx_runtime = new_bin;
            printf("Tone monitor bin set: %d (~%.0f Hz)\n", new_bin, tone_monitor_hz_from_bin(new_bin));
        } else {
            printf("Invalid bin: %d (valid range 0-%d)\n", new_bin, NUM_FREQ_BINS - 1);
        }
    } else if (sscanf(cmd_buf, "raw %15s", arg) == 1 || sscanf(cmd_buf, "RAW %15s", arg) == 1) {
        bool v;
        if (parse_onoff(arg, &v)) {
            diag_raw_enabled = v;
            printf("RAW ADC diagnostics: %s\n", diag_raw_enabled ? "ON" : "OFF");
        } else {
            printf("Usage: raw on|off\n");
        }
    } else if (sscanf(cmd_buf, "beam %15s", arg) == 1 || sscanf(cmd_buf, "BEAM %15s", arg) == 1) {
        bool v;
        if (parse_onoff(arg, &v)) {
            diag_beam_enabled = v;
            printf("BEAMFORMED diagnostics: %s\n", diag_beam_enabled ? "ON" : "OFF");
        } else {
            printf("Usage: beam on|off\n");
        }
    } else if (sscanf(cmd_buf, "tone %15s", arg) == 1 || sscanf(cmd_buf, "TONE %15s", arg) == 1) {
        bool v;
        if (parse_onoff(arg, &v)) {
            diag_tone_enabled = v;
            printf("Tone monitor: %s\n", diag_tone_enabled ? "ON" : "OFF");
        } else {
            printf("Usage: tone on|off\n");
        }
    } else if (sscanf(cmd_buf, "i2c %15s", arg) == 1 || sscanf(cmd_buf, "I2C %15s", arg) == 1) {
        bool v;
        if (parse_onoff(arg, &v)) {
            diag_i2c_error_log_enabled = v;
            printf("I2C failure logs: %s\n", diag_i2c_error_log_enabled ? "ON" : "OFF");
        } else {
            printf("Usage: i2c on|off\n");
        }
    } else if (sscanf(cmd_buf, "skew %15s", arg) == 1 || sscanf(cmd_buf, "SKEW %15s", arg) == 1) {
        bool v;
        if (parse_onoff(arg, &v)) {
            beam_skew_comp_enabled = v;
            init_beam_delays();
            printf("Beamforming skew correction: %s\n", beam_skew_comp_enabled ? "ON" : "OFF");
        } else {
            printf("Usage: skew on|off\n");
        }
    } else if (sscanf(cmd_buf, "agc %15s", arg) == 1 || sscanf(cmd_buf, "AGC %15s", arg) == 1) {
        bool v;
        if (parse_onoff(arg, &v)) {
            agc_enabled = v;
            agc_stable_start_ms = to_ms_since_boot(get_absolute_time());
            printf("Auto gain control: %s\n", agc_enabled ? "ON" : "OFF");
        } else {
            printf("Usage: agc on|off\n");
        }
    } else if (sscanf(cmd_buf, "gain %d", &new_bin) == 1 || sscanf(cmd_buf, "GAIN %d", &new_bin) == 1) {
        if (new_bin >= FFT_GAIN_MIN && new_bin <= FFT_GAIN_MAX) {
            fft_input_gain = new_bin;
            agc_stable_start_ms = to_ms_since_boot(get_absolute_time());
            printf("Pre-FFT digital gain set: %dx\n", fft_input_gain);
        } else {
            printf("Invalid gain: %d (valid range %d-%d)\n", new_bin, FFT_GAIN_MIN, FFT_GAIN_MAX);
        }
    } else if (sscanf(cmd_buf, "i2cshift %d", &new_bin) == 1 || sscanf(cmd_buf, "I2CSHIFT %d", &new_bin) == 1) {
        if (new_bin >= 4 && new_bin <= 12) {
            i2c_output_shift = new_bin;
            printf("I2C output shift set: %d (lower = more sensitive)\n", i2c_output_shift);
        } else {
            printf("Invalid i2cshift: %d (valid range 4-12)\n", new_bin);
        }
    } else if (str_ieq(cmd_buf, "help") || str_ieq(cmd_buf, "h") || str_ieq(cmd_buf, "?")) {
        print_diag_help();
    } else if (str_ieq(cmd_buf, "status") || str_ieq(cmd_buf, "st")) {
        print_diag_status();
    } else {
        printf("Unknown command: %s\n", cmd_buf);
        print_diag_help();
    }
    fflush(stdout);
}

static void tone_monitor_poll_serial_commands(void) {
    static char cmd_buf[32];
    static int cmd_len = 0;
    static uint32_t last_char_ms = 0;
    const uint32_t CMD_IDLE_EXECUTE_MS = 120;

    int ch;
    while ((ch = getchar_timeout_us(0)) != PICO_ERROR_TIMEOUT) {
        if (ch == '\r' || ch == '\n') {
            if (cmd_len > 0) {
                cmd_buf[cmd_len] = '\0';
                process_diag_command(cmd_buf);
                cmd_len = 0;
            }
        } else if (ch >= 32 && ch <= 126) {
            if (cmd_len < (int)(sizeof(cmd_buf) - 1)) {
                cmd_buf[cmd_len++] = (char)ch;
            }
            last_char_ms = to_ms_since_boot(get_absolute_time());
        }
    }

    // Some serial terminals send text without CR/LF; execute after short idle gap.
    if (cmd_len > 0) {
        uint32_t now_ms = to_ms_since_boot(get_absolute_time());
        if ((now_ms - last_char_ms) > CMD_IDLE_EXECUTE_MS) {
            cmd_buf[cmd_len] = '\0';
            process_diag_command(cmd_buf);
            cmd_len = 0;
        }
    }
}

// Apply delay-and-sum beamforming
// beam_idx: which beam to process (0-4)
// channel_data: array of 3 pointers to individual channel samples
// num_samples: number of samples to process
// output: output beamformed samples
// Bit-reversal permutation for FFT
static void bit_reverse_permute(int16_t *real, int16_t *imag) {
    int n = FFT_SIZE;
    int j = 0;
    
    for (int i = 0; i < n - 1; i++) {
        if (i < j) {
            // Swap
            int16_t temp_r = real[i];
            int16_t temp_i = imag[i];
            real[i] = real[j];
            imag[i] = imag[j];
            real[j] = temp_r;
            imag[j] = temp_i;
        }
        
        int k = n >> 1;
        while (k <= j) {
            j -= k;
            k >>= 1;
        }
        j += k;
    }
}

// Get sine value from lookup table (uses symmetry)
// angle_idx: 0-256 represents 0 to 2*pi
static int16_t sin_lookup(int angle_idx) {
    angle_idx = angle_idx & 0xFF;  // Wrap to 0-255
    
    if (angle_idx < 64) {
        return twiddle_sin_lut[angle_idx];
    } else if (angle_idx < 128) {
        return twiddle_sin_lut[128 - angle_idx];
    } else if (angle_idx < 192) {
        return -twiddle_sin_lut[angle_idx - 128];
    } else {
        return -twiddle_sin_lut[256 - angle_idx];
    }
}

// Get cosine value from lookup table (cos = sin(x + pi/2))
static int16_t cos_lookup(int angle_idx) {
    return sin_lookup((angle_idx + 64) & 0xFF);
}

// Fixed-point multiplication with Q15 scaling
static int16_t fixed_mult(int16_t a, int16_t b) {
    int32_t result = ((int32_t)a * (int32_t)b) >> 15;
    if (result > 32767) result = 32767;
    if (result < -32768) result = -32768;
    return (int16_t)result;
}

// Cooley-Tukey FFT (radix-2, decimation-in-time)
static void fft_radix2(int16_t *real, int16_t *imag) {
    int n = FFT_SIZE;
    
    // Bit-reversal permutation
    bit_reverse_permute(real, imag);
    
    // Butterfly operations
    for (int stage = 1; stage <= 8; stage++) {  // log2(256) = 8 stages
        int m = 1 << stage;
        int half_m = m >> 1;
        
        for (int k = 0; k < n; k += m) {
            for (int j = 0; j < half_m; j++) {
                int w_idx = (256 * j) / m;  // Twiddle factor index
                int16_t w_sin = sin_lookup(w_idx);
                int16_t w_cos = cos_lookup(w_idx);
                
                int t_idx = k + j + half_m;
                int a_idx = k + j;
                
                // t = w * u
                int16_t t_real = fixed_mult(w_cos, real[t_idx]) - fixed_mult(w_sin, imag[t_idx]);
                int16_t t_imag = fixed_mult(w_sin, real[t_idx]) + fixed_mult(w_cos, imag[t_idx]);
                
                // Butterfly
                real[t_idx] = (real[a_idx] - t_real) >> 1;
                imag[t_idx] = (imag[a_idx] - t_imag) >> 1;
                real[a_idx] = (real[a_idx] + t_real) >> 1;
                imag[a_idx] = (imag[a_idx] + t_imag) >> 1;
            }
        }
    }
}

// Extract magnitude spectrum and map to 40 frequency bins
static void extract_freq_bins_fixed(uint16_t *magnitude, uint16_t *bins) {
    float bin_width_hz = (float)TARGET_SAMPLE_RATE / FFT_SIZE;
    int start_bin = (int)(FREQ_MIN / bin_width_hz);
    int end_bin = (int)(FREQ_MAX / bin_width_hz);
    int num_input_bins = end_bin - start_bin;
    
    // Noise floor threshold: suppress FFT magnitudes below this level
    // This removes 12-bit ADC quantization noise while preserving real signals
    const uint16_t NOISE_FLOOR = 25;  // Suppress quantization noise while preserving signals

    
    // Sum pairs of FFT bins to form 40 output bins (500 Hz to 5500 Hz)
    // FFT bin width: 16k / 256 = 62.5 Hz, so 2 bins ≈ 125 Hz per output bin
    for (int out_bin = 0; out_bin < NUM_FREQ_BINS; out_bin++) {
        int bin_base = start_bin + (out_bin * 2);
        if (bin_base < 0) bin_base = 0;
        if (bin_base >= (FFT_SIZE / 2 - 1)) bin_base = (FFT_SIZE / 2 - 2);

        uint32_t sum = (uint32_t)magnitude[bin_base] + (uint32_t)magnitude[bin_base + 1];
        uint16_t val = (sum > 65535) ? 65535 : (uint16_t)sum;

        // Apply noise floor: suppress weak signals that are likely noise
        if (val < NOISE_FLOOR) {
            val = 0;
        }

        bins[out_bin] = val;
    }
}

// Compute FFT for a single beamformed channel (fixed-point)
static void compute_fft_spectrum(uint16_t *input, uint16_t *output) {
    uint16_t local_peak_q15 = 0;
    bool clipped = false;

    // Prepare input: convert to fixed-point Q15
    for (int i = 0; i < FFT_SIZE; i++) {
        // Beamformed output is uint16_t (0-4095 range from 12-bit ADC)
        // Convert to signed Q15: center around 0 and scale up
        // Range: 0-4095 → -2048 to +2047 → -32768 to +32752 in Q15
        int32_t centered = (int32_t)input[i] - 2048;
        int32_t scaled = centered * 16 * fft_input_gain;
        if (scaled > 32767) {
            scaled = 32767;
            clipped = true;
        }
        if (scaled < -32768) {
            scaled = -32768;
            clipped = true;
        }

        uint16_t abs_q15 = (scaled < 0) ? (uint16_t)(-scaled) : (uint16_t)scaled;
        if (abs_q15 > local_peak_q15) local_peak_q15 = abs_q15;

        int16_t adc_q15 = (int16_t)scaled;
        fft_real[i] = adc_q15;
        fft_imag[i] = 0;
    }

    agc_last_peak_q15 = local_peak_q15;

    if (agc_enabled) {
        uint32_t now_ms = to_ms_since_boot(get_absolute_time());

        if (clipped) {
            agc_clip_events++;
            agc_last_clip_ms = now_ms;
            agc_stable_start_ms = now_ms;
            if (fft_input_gain > FFT_GAIN_MIN) {
                fft_input_gain -= 1;
            }
        } else {
            // Only consider ramp-up when we are below 80% full-scale.
            if (local_peak_q15 <= AGC_HEADROOM_Q15) {
                if (agc_stable_start_ms == 0) agc_stable_start_ms = now_ms;
                if ((now_ms - agc_stable_start_ms) >= AGC_RAMP_UP_HOLD_MS) {
                    if (fft_input_gain < FFT_GAIN_MAX) {
                        fft_input_gain += 1;
                    }
                    agc_stable_start_ms = now_ms;
                }
            } else {
                // We are already near full-scale; hold gain steady and restart stable timer.
                agc_stable_start_ms = now_ms;
            }
        }
    }
    
    // Compute FFT
    fft_radix2(fft_real, fft_imag);
    
    // Compute magnitude spectrum (skip DC component)
    fft_magnitude[0] = 0;
    for (int i = 1; i < FFT_SIZE / 2; i++) {
        int32_t real_sq = (int32_t)fft_real[i] * fft_real[i];
        int32_t imag_sq = (int32_t)fft_imag[i] * fft_imag[i];
        int32_t mag_sq = real_sq + imag_sq;

        // Protect against divide-by-zero when signal is silent
        if (mag_sq == 0) {
            fft_magnitude[i] = 0;
            continue;
        }
        
        // Integer square root approximation (Newton's method, fast)
        uint32_t mag = (uint32_t)mag_sq;
        uint32_t x = mag ? mag : 1;  // Ensure non-zero initial guess
        x = (x + mag / x) >> 1;
        x = (x + mag / x) >> 1;
        
        // Scale down: >> 8 keeps magnitude in reasonable range for 12-bit ADC input
        // Original >> 16 was too aggressive, losing signal information
        uint32_t scaled = x >> 8;
        fft_magnitude[i] = (uint16_t)(scaled > 65535 ? 65535 : scaled);  // Cap at 16-bit max
    }
    
    // Extract 40 frequency bins
    extract_freq_bins_fixed(fft_magnitude, output);
}

// Initialize error indicator GPIO pins
static void init_error_leds(void) {
    uint8_t error_pins[] = {ERROR_LED_PIN_0, ERROR_LED_PIN_1, ERROR_LED_PIN_2, ERROR_LED_PIN_3, ERROR_LED_PIN_4};
    printf("Initializing error LEDs (pins 0-4)...\n");
    fflush(stdout);
    for (int i = 0; i < 5; i++) {
        printf("  LED %d (GPIO %d)\n", i, error_pins[i]);
        fflush(stdout);
        gpio_init(error_pins[i]);
        gpio_set_dir(error_pins[i], GPIO_OUT);
        gpio_put(error_pins[i], 0);
    }
    printf("Error LEDs initialized\n");
    fflush(stdout);
}

// Initialize debug timing GPIO pins
static void init_debug_gpios(void) {
static bool beam_skew_comp_enabled = true;
    // GPIO 5 for Core 1 heartbeat
    printf("Initializing GPIO 5...\n");
    fflush(stdout);
            diag_raw_enabled ? "ON" : "OFF",
           beam_skew_comp_enabled ? "ON" : "OFF",
    printf("GPIO 5 initialized\n");
    fflush(stdout);
    gpio_set_dir(5, GPIO_OUT);
    printf("GPIO 5 direction set\n");
    fflush(stdout);
    gpio_put(5, 0);
    printf("GPIO 5 set to 0\n");
    fflush(stdout);
    
    printf("Initializing GPIO %d...\n", DEBUG_FFT_COMPLETE_PIN);
    fflush(stdout);
    gpio_init(DEBUG_FFT_COMPLETE_PIN);
    printf("GPIO %d initialized\n", DEBUG_FFT_COMPLETE_PIN);
    fflush(stdout);
    gpio_set_dir(DEBUG_FFT_COMPLETE_PIN, GPIO_OUT);
    printf("GPIO %d direction set\n", DEBUG_FFT_COMPLETE_PIN);
    fflush(stdout);
    gpio_put(DEBUG_FFT_COMPLETE_PIN, 0);
    printf("GPIO %d set to 0\n", DEBUG_FFT_COMPLETE_PIN);
    fflush(stdout);
    
    printf("Initializing GPIO %d...\n", DEBUG_ALL_BEAMS_COMPLETE_PIN);
    fflush(stdout);
    gpio_init(DEBUG_ALL_BEAMS_COMPLETE_PIN);
    printf("GPIO %d initialized\n", DEBUG_ALL_BEAMS_COMPLETE_PIN);
    fflush(stdout);
    gpio_set_dir(DEBUG_ALL_BEAMS_COMPLETE_PIN, GPIO_OUT);
    printf("GPIO %d direction set\n", DEBUG_ALL_BEAMS_COMPLETE_PIN);
    fflush(stdout);
    gpio_put(DEBUG_ALL_BEAMS_COMPLETE_PIN, 0);
    printf("GPIO %d set to 0\n", DEBUG_ALL_BEAMS_COMPLETE_PIN);
    fflush(stdout);
}

// Format I2C packet: header + 40 frequency bins (1 byte each)
static void format_i2c_packet(uint16_t *freq_bins) {
    i2c_packet[0] = I2C_PACKET_HEADER;
    for (int i = 0; i < NUM_FREQ_BINS; i++) {
        // Convert 16-bit magnitude to 8-bit by right shifting.
        // Runtime shift allows tuning speech visibility on downstream LCD.
        uint16_t val = freq_bins[i];
        uint16_t packed = val >> i2c_output_shift;
        if (packed > 255) packed = 255;
        uint8_t out = (uint8_t)packed;
        i2c_packet[1 + i] = out;
    }
    // Debug: log packet formatting (first time only)
    static bool logged = false;
    if (!logged) {
        fflush(stdout);
        logged = true;
    }
}

// Send FFT data via I2C to remote RP2040
// Returns true if successful, false if device not responding
static bool send_fft_via_i2c(int beam_idx, uint16_t *freq_bins) {
    uint8_t slave_addr = I2C_BEAM_BASE_ADDR + beam_idx;
    
    // Format the packet
    format_i2c_packet(freq_bins);
    
    // Attempt I2C write (blocking)
    // i2c_write_blocking returns number of bytes written, or PICO_ERROR_GENERIC if NACK
    int result = i2c_write_blocking(
        I2C_PORT,
        slave_addr,
        i2c_packet,
        I2C_PACKET_SIZE,
        false
    );
    
    // Check if write succeeded (all bytes written)
    if (result == I2C_PACKET_SIZE) {
        // Success - clear error LED if any
        uint8_t error_pins[] = {ERROR_LED_PIN_0, ERROR_LED_PIN_1, ERROR_LED_PIN_2, ERROR_LED_PIN_3, ERROR_LED_PIN_4};
        if (beam_idx < 5) {
            gpio_put(error_pins[beam_idx], 0);  // Turn OFF LED on successful recovery
        }
        return true;
    } else {
        // I2C device not responding - turn on error LED
        uint8_t error_pins[] = {ERROR_LED_PIN_0, ERROR_LED_PIN_1, ERROR_LED_PIN_2, ERROR_LED_PIN_3, ERROR_LED_PIN_4};
        if (beam_idx < 5) {
            gpio_put(error_pins[beam_idx], 1);  // Turn ON LED to indicate failure
        }
        if (diag_i2c_error_log_enabled) {
            printf("I2C Beam %d (addr 0x%02x) failed: result=%d (expected %d)\n", beam_idx, slave_addr, result, I2C_PACKET_SIZE);
            fflush(stdout);
        }
        return false;
    }
}

// Send all 5 beamformed FFT results via I2C to respective slave devices
// Core 1: FFT processing only (I2C now handled by Core 0)
static void core1_main(void) {
    // Core 1 waits for beamformed data, processes FFT, queues results for Core 0
    uint16_t local_beam_data[NUM_BEAMS][CAPTURE_DEPTH];
    uint32_t cycle_count = 0;
    uint32_t fft_queue_full_count = 0;
    
    // Immediate GPIO 5 pulse to confirm Core 1 is running
    gpio_put(5, 1);
    sleep_us(10);
    gpio_put(5, 0);
    sleep_us(10);
    
    while (true) {
        // Wait for beamformed data from Core 0 (non-blocking check)
        if (queue_get_beamformed(local_beam_data)) {
            cycle_count++;
            
            // All 5 beams FFT Start - toggle GPIO 11 as pulse start
            gpio_put(DEBUG_ALL_BEAMS_COMPLETE_PIN, 1);
            
            // Process FFT for each beamformed channel and queue for Core 0
            for (int beam = 0; beam < NUM_BEAMS; beam++) {
                gpio_put(DEBUG_FFT_COMPLETE_PIN, 1);
                compute_fft_spectrum(local_beam_data[beam], fft_output[beam]);
                gpio_put(DEBUG_FFT_COMPLETE_PIN, 0);

                // Queue FFT result for Core 0 to transmit via I2C
                if (!fft_queue_add(beam, fft_output[beam])) {
                    // Queue full - Core 0 can't keep up with I2C transmission
                    fft_queue_full_count++;
                    // Continue anyway - drop oldest result
                }
            }
            
            // All 5 beams FFT complete
            gpio_put(DEBUG_ALL_BEAMS_COMPLETE_PIN, 0);
        } else {
            // No data yet, keep spinning (Core 1 runs at 100% waiting for Core 0)
        }
    }
}

static void broadcast_fft_results(void) {
    // Deprecated: FFT broadcast now handled by Core 1
    // This function kept for compatibility
}

static void beamform_delay_sum(int beam_idx, uint16_t *ch0, uint16_t *ch1, uint16_t *ch2, int num_samples, uint16_t *output) {
    uint16_t *channels[3] = {ch0, ch1, ch2};
    
    for (int i = 0; i < num_samples; i++) {
        int32_t sum = 0;
        
        for (int mic = 0; mic < ADC_NUM_CHANNELS; mic++) {
            int delay = beam_delays[beam_idx][mic];
            int idx = i + delay;

            // Edge-safe delay: clamp index instead of dropping samples to zero.
            // Dropping out-of-range samples causes artificial low values at block edges,
            // which inflates p2p and distorts beam statistics.
            if (idx < 0) {
                idx = 0;
            } else if (idx >= num_samples) {
                idx = num_samples - 1;
            }

            sum += (int32_t)channels[mic][idx];
        }

        // Average across 3 channels and clamp to 12-bit ADC range.
        int32_t avg = sum / ADC_NUM_CHANNELS;
        if (avg < 0) avg = 0;
        if (avg > 4095) avg = 4095;
        output[i] = (uint16_t)avg;
    }
}

// Core 0: ADC capture and beamforming
int main()
{
    stdio_init_all();
    
    // Small initial delay to allow USB enumeration
    sleep_ms(5000);
    
    // Wait for USB CDC to be ready (timeout 5 seconds with status updates)
    uint32_t start_time = time_us_32();
    int connection_attempts = 0;
    while (!stdio_usb_connected() && (time_us_32() - start_time) < 5000000) {
        connection_attempts++;
        if ((connection_attempts % 100000) == 0) {
            // Try to force USB reconnection by accessing stdout
            putchar('.');
            fflush(stdout);
        }
        tight_loop_contents();
    }
    
    // Give USB a moment to fully stabilize
    sleep_ms(100);
    
    printf("\n=== Speech Recognition AudioCapture Starting ===\n");
    printf("USB CDC Connected: %s\n", stdio_usb_connected() ? "Yes" : "No");
    printf("USB enumeration took %u ms\n", (time_us_32() - start_time) / 1000);
    fflush(stdout);

    // Initialize ADC
    adc_init();
    adc_gpio_init(ADC_PIN_0);
    adc_gpio_init(ADC_PIN_1);
    adc_gpio_init(ADC_PIN_2);

    // Configure ADC for free-running round-robin mode
    adc_select_input(ADC_CHANNEL_0);
    adc_set_round_robin(0x07);  // Enable round-robin on channels 0,1,2 (bits 0-2 set)
    
    // Set ADC clock divider to achieve ~16 kHz sample rate per channel
    // ADC clock is typically 48 MHz; divider = 48MHz / (3 channels * 16kHz)
    float adc_clk_div = 48000000.0f / (TARGET_SAMPLE_RATE * ADC_NUM_CHANNELS);
    printf("Setting ADC clock divider to %.1f\n", adc_clk_div);
    fflush(stdout);
    adc_set_clkdiv(adc_clk_div);
    
    // Enable FIFO and set threshold to trigger DMA
    adc_fifo_setup(
        true,   // Enable FIFO
        true,   // Enable DMA
        1,      // DMA threshold (transfer on each sample)
        false,  // Don't error on overflow
        false   // NO byte shift - keep full 12-bit precision for FFT
    );
    
    printf("ADC FIFO configured\n");
    fflush(stdout);
    
    // Drain FIFO and disable IRQ to ensure clean state
    adc_fifo_drain();
    adc_irq_set_enabled(false);
    printf("ADC FIFO drained and IRQ disabled\n");
    fflush(stdout);
    
    // Setup DMA for continuous ADC FIFO to memory transfer
    g_dma_chan = dma_claim_unused_channel(true);
    printf("DMA channel %d claimed\n", g_dma_chan);
    fflush(stdout);
    dma_channel_config dma_cfg = dma_channel_get_default_config(g_dma_chan);
    
    // Configure DMA: read from ADC FIFO, write to circular buffer
    channel_config_set_transfer_data_size(&dma_cfg, DMA_SIZE_16);
    channel_config_set_read_increment(&dma_cfg, false);  // ADC FIFO doesn't increment
    channel_config_set_write_increment(&dma_cfg, true);  // Buffer increments
    channel_config_set_dreq(&dma_cfg, DREQ_ADC);         // Use ADC as trigger
    channel_config_set_ring(&dma_cfg, true, DMA_RING_BITS); // Circular buffer sized to BUFFER_SIZE
    
    dma_channel_configure(
        g_dma_chan,
        &dma_cfg,
        adc_buffer,                 // Write address (circular)
        &adc_hw->fifo,              // Read address (ADC FIFO)
        0xFFFFFFFFu,                // Effectively continuous transfers
        true                        // Start immediately
    );
    
    printf("DMA configured and started\n");
    fflush(stdout);

    // Initialize I2C (Core 1 will use this for FFT export)
    printf("Initializing I2C SDA %d SCL %d...\n", I2C_SDA, I2C_SCL);
    fflush(stdout);
    
    i2c_init(I2C_PORT, I2C_BAUDRATE);
    printf("I2C bus initialized\n");
    fflush(stdout);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    printf("I2C SDA configured\n");
    fflush(stdout);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    printf("I2C SCL configured using SDA %d and SCL %d\n", I2C_SDA, I2C_SCL);
    fflush(stdout);
    gpio_pull_up(I2C_SDA);
    printf("I2C SDA pull-up enabled\n");
    fflush(stdout);
    gpio_pull_up(I2C_SCL);
    printf("I2C SCL pull-up enabled\n");
    fflush(stdout);
    printf("I2C Initialized for FFT Export\n");
    fflush(stdout);

    // Initialize debug timing GPIO pins
    init_debug_gpios();
    printf("Debug GPIO Initialized (pins %d and %d)\n", DEBUG_FFT_COMPLETE_PIN, DEBUG_ALL_BEAMS_COMPLETE_PIN);
    
    // Test GPIO 5 and 6 with a few pulses to verify they work
    printf("Testing GPIO %d...\n", DEBUG_FFT_COMPLETE_PIN);
    fflush(stdout);
    for (int i = 0; i < 3; i++) {
        printf("  GPIO %d pulse %d\n", DEBUG_FFT_COMPLETE_PIN, i+1);
        fflush(stdout);
        gpio_put(DEBUG_FFT_COMPLETE_PIN, 1);
        sleep_us(10);
        gpio_put(DEBUG_FFT_COMPLETE_PIN, 0);
        sleep_us(10);
    }
    
    printf("GPIO %d test complete\n", DEBUG_FFT_COMPLETE_PIN);
    fflush(stdout);
    
    printf("Testing GPIO %d...\n", DEBUG_ALL_BEAMS_COMPLETE_PIN);
    fflush(stdout);
    for (int i = 0; i < 3; i++) {
        printf("  GPIO %d pulse %d\n", DEBUG_ALL_BEAMS_COMPLETE_PIN, i+1);
        fflush(stdout);
        gpio_put(DEBUG_ALL_BEAMS_COMPLETE_PIN, 1);
        sleep_us(10);
        gpio_put(DEBUG_ALL_BEAMS_COMPLETE_PIN, 0);
        sleep_us(10);
    }
    
    printf("GPIO %d test complete\n", DEBUG_ALL_BEAMS_COMPLETE_PIN);
    fflush(stdout);

    // USB serial output is enabled via stdio_init_all() above
    printf("Core 0: ADC Capture & Beamforming Started\n");
    fflush(stdout);

    // Initialize inter-core queue locks
    beam_queue_lock = spin_lock_instance(0);
    fft_queue_lock = spin_lock_instance(1);

    // Initialize beamforming delays for all 5 angles
    init_beam_delays();
    printf("Beamforming Initialized\n");
    fflush(stdout);
    
    printf("About to launch Core 1...\n");
    fflush(stdout);
    
    
    // Launch Core 1 for FFT processing (I2C now handled by Core 0)
    multicore_launch_core1(core1_main);
    printf("Core 1: FFT Processor Started (I2C handled by Core 0)\n");
    fflush(stdout);
    
    // Start free-running mode
    printf("About to call adc_run(true)...\n");
    fflush(stdout);
    adc_run(true);
    printf("adc_run(true) completed successfully\n");
    fflush(stdout);
    
    // Check ADC and DMA status before entering main loop
    printf("Pre-loop diagnostics:\n");
    fflush(stdout);
    sleep_ms(5);
    
    // Quick diagnostic check - no blocking sleeps!
    printf("Pre-loop: DMA busy=%d, FIFO=%d\n", dma_channel_is_busy(g_dma_chan), adc_fifo_get_level());
    fflush(stdout);

        // Short delay then re-check DMA/FIFO to see if samples are flowing
        sleep_ms(10);
        printf("After 10ms: xfer=%u, FIFO=%d, DMA busy=%d\n",
            dma_hw->ch[g_dma_chan].transfer_count,
            adc_fifo_get_level(),
            dma_channel_is_busy(g_dma_chan));
        fflush(stdout);

    // Core 0 Main Loop: capture ADC data and beamforming
    // Beamformed data is queued for Core 1 to process FFT
    uint32_t last_pos = 0;
    uint32_t core0_cycle_count = 0;
    uint32_t heartbeat_counter = 0;
    uint32_t initial_transfer_count = 0xFFFFFFFFu;  // Track how much DMA has done

#if TONE_MONITOR_ENABLE
    uint16_t tone_mag[NUM_BEAMS] = {0};
    uint8_t tone_seen_mask = 0;
    uint32_t tone_set_count = 0;
    tone_monitor_bin_idx_runtime = tone_monitor_bin_from_freq(TONE_MONITOR_FREQ_HZ);
#endif
    
    // Simple busy-wait loop - check DMA position continuously
    uint32_t log_counter = 0;

#if TONE_MONITOR_ENABLE
    printf("Tone monitor enabled: target=%d Hz, bin=%d (~%.0f Hz), interval=%d sets\n",
           TONE_MONITOR_FREQ_HZ,
           tone_monitor_bin_idx_runtime,
           tone_monitor_hz_from_bin(tone_monitor_bin_idx_runtime),
           TONE_MONITOR_INTERVAL_SETS);
    print_diag_help();
    print_diag_status();
    fflush(stdout);
#endif

    while (true) {
#if TONE_MONITOR_ENABLE
        tone_monitor_poll_serial_commands();
#endif

        // Check if new data is available (every CAPTURE_DEPTH samples)
        uint32_t remaining = dma_hw->ch[g_dma_chan].transfer_count;
        uint32_t current_pos = initial_transfer_count - remaining;
        
        // Handle circular buffer wraparound - check if we have enough new interleaved samples (all 3 channels)
        uint32_t diff = (current_pos >= last_pos) ? (current_pos - last_pos) : (BUFFER_SIZE - last_pos + current_pos);
        
        
        log_counter++;
        
        // Need BLOCK_SAMPLES (3*CAPTURE_DEPTH) samples before processing
        if (diff >= BLOCK_SAMPLES) {
            core0_cycle_count++;
            
            // Blink GPIO 11 when we process data
            //gpio_put(DEBUG_ALL_BEAMS_COMPLETE_PIN, 1);
            
            // Extract latest channels from DMA ring with correct phase handling.
            extract_channels_from_ring(current_pos, ch0_data, ch1_data, ch2_data);

#if RAW_ADC_DEBUG
            if ((core0_cycle_count % RAW_ADC_DEBUG_INTERVAL_CYCLES) == 0 && (diag_raw_enabled || diag_beam_enabled)) {
                uint16_t ch_min[ADC_NUM_CHANNELS];
                uint16_t ch_max[ADC_NUM_CHANNELS];
                uint16_t ch_avg[ADC_NUM_CHANNELS];

                if (diag_raw_enabled) {
                    compute_adc_stats(ch0_data, CAPTURE_DEPTH, &ch_min[0], &ch_max[0], &ch_avg[0]);
                    compute_adc_stats(ch1_data, CAPTURE_DEPTH, &ch_min[1], &ch_max[1], &ch_avg[1]);
                    compute_adc_stats(ch2_data, CAPTURE_DEPTH, &ch_min[2], &ch_max[2], &ch_avg[2]);

                    printf("RAW ADC:\n");
                    printf("  AGC:%s  GAIN:%2dx  PEAK:%5u  CLIPS:%lu\n",
                           agc_enabled ? "ON " : "OFF",
                           fft_input_gain,
                           agc_last_peak_q15,
                           (unsigned long)agc_clip_events);
                    printf("  CH   MIN(mV)   MAX(mV)   AVG(mV)   P2P(mV)\n");
                    for (int ch = 0; ch < ADC_NUM_CHANNELS; ch++) {
                        uint16_t p2p = ch_max[ch] - ch_min[ch];
                        float min_mv = adc_counts_to_millivolts(ch_min[ch]);
                        float max_mv = adc_counts_to_millivolts(ch_max[ch]);
                        float avg_mv = adc_counts_to_millivolts(ch_avg[ch]);
                        float p2p_mv = adc_counts_to_millivolts(p2p);
                        printf("  CH%-1d %9.1f %9.1f %9.1f %9.1f\n",
                               ch,
                               min_mv,
                               max_mv,
                               avg_mv,
                               p2p_mv);
                    }
                }
                fflush(stdout);
            }
#endif
            
            // Beamform all 5 beams
            for (int beam = 0; beam < NUM_BEAMS; beam++) {
                beamform_delay_sum(beam, ch0_data, ch1_data, ch2_data, CAPTURE_DEPTH, local_beam_output[beam]);
                // Toggle GPIO 5 every iteration (no delay - instant pulse)
                gpio_put(5, 1);
                sleep_us(5);
                gpio_put(5, 0);
            }

#if RAW_ADC_DEBUG
            if ((core0_cycle_count % RAW_ADC_DEBUG_INTERVAL_CYCLES) == 0 && diag_beam_enabled) {
                uint16_t beam_min[NUM_BEAMS];
                uint16_t beam_max[NUM_BEAMS];
                uint16_t beam_avg[NUM_BEAMS];

                for (int beam = 0; beam < NUM_BEAMS; beam++) {
                    compute_adc_stats(local_beam_output[beam], CAPTURE_DEPTH, &beam_min[beam], &beam_max[beam], &beam_avg[beam]);
                }

                printf("BEAMFORMED:\n");
                printf("  B   ANGLE    MIN(mV)   MAX(mV)   AVG(mV)   P2P(mV)\n");
                for (int beam = 0; beam < NUM_BEAMS; beam++) {
                    uint16_t p2p = beam_max[beam] - beam_min[beam];
                    float min_mv = adc_counts_to_millivolts(beam_min[beam]);
                    float max_mv = adc_counts_to_millivolts(beam_max[beam]);
                    float avg_mv = adc_counts_to_millivolts(beam_avg[beam]);
                    float p2p_mv = adc_counts_to_millivolts(p2p);
                    printf("  B%-1d %7.0f %9.1f %9.1f %9.1f %9.1f\n",
                           beam, beam_angles[beam],
                           min_mv,
                           max_mv,
                           avg_mv,
                           p2p_mv);
                }
                fflush(stdout);
            }
#endif
            
            // Queue beamformed data for Core 1 to process
            queue_add_beamformed(local_beam_output);
            
            last_pos = current_pos;
        }
        
        // Core 0 now handles I2C transmission (parallelized with Core 1 FFT)
        // Check if Core 1 has any FFT results ready to transmit
        int beam_idx;
        uint16_t freq_bins[NUM_FREQ_BINS];
        if (fft_queue_get(&beam_idx, freq_bins)) {
            // Transmit this beam's FFT result via I2C
            send_fft_via_i2c(beam_idx, freq_bins);

#if TONE_MONITOR_ENABLE
            if (diag_tone_enabled && beam_idx >= 0 && beam_idx < NUM_BEAMS) {
                int tone_bin_idx = tone_monitor_bin_idx_runtime;
                tone_mag[beam_idx] = freq_bins[tone_bin_idx];
                tone_seen_mask |= (uint8_t)(1u << beam_idx);

                if (tone_seen_mask == ((1u << NUM_BEAMS) - 1u)) {
                    tone_seen_mask = 0;
                    tone_set_count++;

                    if ((tone_set_count % TONE_MONITOR_INTERVAL_SETS) == 0) {
                        uint16_t max_mag = tone_mag[0];
                        for (int b = 1; b < NUM_BEAMS; b++) {
                            if (tone_mag[b] > max_mag) max_mag = tone_mag[b];
                        }
                        if (max_mag == 0) max_mag = 1;

                           printf("TONE %4dHz BIN%02d(~%.0fHz) | B0:%5u %3u%% B1:%5u %3u%% B2:%5u %3u%% B3:%5u %3u%% B4:%5u %3u%% | AGC:%s G:%dx P:%u\n",
                               (int)tone_monitor_hz_from_bin(tone_bin_idx), tone_bin_idx, tone_monitor_hz_from_bin(tone_bin_idx),
                               tone_mag[0], (uint16_t)((tone_mag[0] * 100u) / max_mag),
                               tone_mag[1], (uint16_t)((tone_mag[1] * 100u) / max_mag),
                               tone_mag[2], (uint16_t)((tone_mag[2] * 100u) / max_mag),
                               tone_mag[3], (uint16_t)((tone_mag[3] * 100u) / max_mag),
                               tone_mag[4], (uint16_t)((tone_mag[4] * 100u) / max_mag),
                               agc_enabled ? "ON" : "OFF",
                               fft_input_gain,
                               agc_last_peak_q15);
                        fflush(stdout);
                    }
                }
            }
#endif
        }
    }
}
