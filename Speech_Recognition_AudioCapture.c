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
#include "hardware/flash.h"
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

// Narrowband tone monitor (uses output-band magnitudes for beam direction testing)
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
#define NUM_FREQ_BINS 45          // 45 output bands
#define FFT_BIN_HZ ((float)TARGET_SAMPLE_RATE / FFT_SIZE)  // 62.5 Hz/bin

// Band mapping (male-speech emphasis at low frequencies):
// band 0..7   -> fft bins 1..8 (single-bin bands)
// band 8..42  -> pairs (9,10), (11,12), ... (77,78)
// band 43..44 -> fft bins 79, 80 (single-bin bands; keeps <= 5 kHz and 45 total bands)
#define LOW_SINGLE_BANDS 8
#define LOW_SINGLE_START_BIN 1
#define PAIR_START_BIN 9
#define FINAL_SINGLE0_BIN 79
#define FINAL_SINGLE1_BIN 80

static void band_to_fft_range(int band_idx, int *fft_bin_start, int *fft_bin_end) {
    if (!fft_bin_start || !fft_bin_end) return;

    if (band_idx < LOW_SINGLE_BANDS) {
        int b = LOW_SINGLE_START_BIN + band_idx;
        *fft_bin_start = b;
        *fft_bin_end = b;
        return;
    }

    if (band_idx <= 42) {
        int p0 = PAIR_START_BIN + ((band_idx - LOW_SINGLE_BANDS) * 2);
        *fft_bin_start = p0;
        *fft_bin_end = p0 + 1;
        return;
    }

    if (band_idx == 43) {
        *fft_bin_start = FINAL_SINGLE0_BIN;
        *fft_bin_end = FINAL_SINGLE0_BIN;
        return;
    }

    *fft_bin_start = FINAL_SINGLE1_BIN;
    *fft_bin_end = FINAL_SINGLE1_BIN;
}

static float band_center_hz_from_index(int band_idx) {
    int bin_start = 1;
    int bin_end = 1;
    band_to_fft_range(band_idx, &bin_start, &bin_end);
    return ((float)(bin_start + bin_end) * 0.5f) * FFT_BIN_HZ;
}

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

// Post-FFT output bins for each beam: [beam][bin]
uint16_t fft_beam_bins[NUM_BEAMS][NUM_FREQ_BINS];

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
static uint16_t ch0_gain_data[CAPTURE_DEPTH];
static uint16_t ch1_gain_data[CAPTURE_DEPTH];
static uint16_t ch2_gain_data[CAPTURE_DEPTH];
static uint16_t local_beam_output[NUM_BEAMS][CAPTURE_DEPTH];

#define DMA_CONTINUOUS_TRANSFERS 0xFFFFFFFFu

#define MIC_GAIN_Q8_UNITY 256
#define MIC_GAIN_Q8_DEFAULT 384
#define MIC_GAIN_Q8_MIN 64
#define MIC_GAIN_Q8_MAX 1024

#define SETTINGS_MAGIC 0x41554346u  // 'AUCF'
#define SETTINGS_VERSION 2u
#define SETTINGS_FLASH_OFFSET (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE)

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    uint16_t mic_gain_q8[ADC_NUM_CHANNELS];
    uint8_t beam_fft_gain[NUM_BEAMS];
    uint16_t beam_noise_floor[NUM_BEAMS];
    uint8_t beam_filter_strength_pct[NUM_BEAMS];
    uint8_t beam_processing_enabled[NUM_BEAMS];
    uint8_t bin_noise_filter_enabled;
    uint8_t bin_noise_filter_shift;
    uint8_t bin_filter_max_sub_pct;
    uint8_t reserved[5];
    uint32_t crc32;
} PersistedSettings;

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

// I2C packet format: 1 byte header + NUM_FREQ_BINS * 1 byte
#define I2C_PACKET_HEADER 0xAA    // Packet header for identification
#define I2C_PACKET_SIZE (1 + NUM_FREQ_BINS)
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
    uint16_t fft_bins[NUM_FREQ_BINS];  // Post-FFT bins for one beam
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
static bool fft_queue_add(int beam_idx, uint16_t *fft_bins) {
    uint32_t irq_state = spin_lock_blocking(fft_queue_lock);
    if (fft_queue_count >= FFT_QUEUE_SIZE) {
        spin_unlock(fft_queue_lock, irq_state);
        return false;
    }

    int write_idx = fft_queue_write_idx;
    fft_result_queue[write_idx].beam_idx = beam_idx;
    for (int i = 0; i < NUM_FREQ_BINS; i++) {
        fft_result_queue[write_idx].fft_bins[i] = fft_bins[i];
    }
    fft_result_queue[write_idx].valid = true;

    fft_queue_write_idx = (fft_queue_write_idx + 1) % FFT_QUEUE_SIZE;
    fft_queue_count++;

    spin_unlock(fft_queue_lock, irq_state);
    return true;
}

static bool fft_queue_get(int *beam_idx, uint16_t *fft_bins) {
    uint32_t irq_state = spin_lock_blocking(fft_queue_lock);
    if (fft_queue_count == 0) {
        spin_unlock(fft_queue_lock, irq_state);
        return false;
    }

    int read_idx = fft_queue_read_idx;
    *beam_idx = fft_result_queue[read_idx].beam_idx;
    for (int i = 0; i < NUM_FREQ_BINS; i++) {
        fft_bins[i] = fft_result_queue[read_idx].fft_bins[i];
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
    int best_idx = 0;
    float target_hz = (float)freq_hz;
    float best_err = 1e9f;

    for (int i = 0; i < NUM_FREQ_BINS; i++) {
        float center_hz = band_center_hz_from_index(i);
        float err = center_hz - target_hz;
        if (err < 0.0f) err = -err;
        if (err < best_err) {
            best_err = err;
            best_idx = i;
        }
    }

    return best_idx;
}

static float tone_monitor_hz_from_bin(int bin_idx) {
    if (bin_idx < 0) bin_idx = 0;
    if (bin_idx >= NUM_FREQ_BINS) bin_idx = NUM_FREQ_BINS - 1;
    return band_center_hz_from_index(bin_idx);
}

static bool diag_raw_enabled = (RAW_ADC_DEBUG != 0);
static bool diag_beam_enabled = (RAW_ADC_DEBUG != 0);
static bool diag_tone_enabled = (TONE_MONITOR_ENABLE != 0);
static bool diag_i2c_error_log_enabled = (I2C_ERROR_SERIAL_LOG != 0);
static bool diag_status_enabled = true;
static bool diag_compact_enabled = true;
static bool diag_fft_enabled = false;
static bool diag_fft_raw_enabled = true;
static bool diag_fft_beam_enabled[NUM_BEAMS] = {false, false, true, false, false};
static uint8_t diag_fft_raw_format = 0; // 0=hex, 1=pct
static bool diag_i2c_output_beam_enabled[NUM_BEAMS] = {false, false, false, false, false};
static uint8_t diag_i2c_output_format = 0; // 0=hex, 1=pct
static volatile uint16_t diag_interval_cycles = RAW_ADC_DEBUG_INTERVAL_CYCLES;
static volatile int tone_monitor_bin_idx_runtime = 0;
static uint16_t raw_p2p_min_period[ADC_NUM_CHANNELS] = {0xFFFFu, 0xFFFFu, 0xFFFFu};
static uint16_t raw_p2p_max_period[ADC_NUM_CHANNELS] = {0, 0, 0};
static uint32_t raw_p2p_sum_period[ADC_NUM_CHANNELS] = {0, 0, 0};
static uint16_t post_p2p_min_period[ADC_NUM_CHANNELS] = {0xFFFFu, 0xFFFFu, 0xFFFFu};
static uint16_t post_p2p_max_period[ADC_NUM_CHANNELS] = {0, 0, 0};
static uint32_t post_p2p_sum_period[ADC_NUM_CHANNELS] = {0, 0, 0};
static uint16_t beam_raw_p2p_min_period[NUM_BEAMS] = {0xFFFFu, 0xFFFFu, 0xFFFFu, 0xFFFFu, 0xFFFFu};
static uint16_t beam_raw_p2p_max_period[NUM_BEAMS] = {0, 0, 0, 0, 0};
static uint32_t beam_raw_p2p_sum_period[NUM_BEAMS] = {0, 0, 0, 0, 0};
static uint16_t beam_post_p2p_min_period[NUM_BEAMS] = {0xFFFFu, 0xFFFFu, 0xFFFFu, 0xFFFFu, 0xFFFFu};
static uint16_t beam_post_p2p_max_period[NUM_BEAMS] = {0, 0, 0, 0, 0};
static uint32_t beam_post_p2p_sum_period[NUM_BEAMS] = {0, 0, 0, 0, 0};
static uint32_t p2p_period_samples = 0;
static volatile bool settings_save_pending = false;
static volatile uint32_t settings_last_change_ms = 0;
static volatile uint32_t settings_save_ok_count = 0;
static volatile uint32_t settings_save_fail_count = 0;
static volatile uint32_t dma_rearm_count = 0;
static volatile uint32_t dma_remaining_snapshot = DMA_CONTINUOUS_TRANSFERS;
static volatile uint16_t mic_gain_q8[ADC_NUM_CHANNELS] = {MIC_GAIN_Q8_DEFAULT, MIC_GAIN_Q8_DEFAULT, MIC_GAIN_Q8_DEFAULT};
static volatile uint16_t mic_gain_last_clip_count[ADC_NUM_CHANNELS] = {0, 0, 0};
static volatile uint16_t fft_last_beam_bins[NUM_BEAMS][NUM_FREQ_BINS] = {0};
static volatile uint8_t fft_last_valid[NUM_BEAMS] = {0, 0, 0, 0, 0};
static volatile uint16_t fft_last_mag[NUM_BEAMS][FFT_SIZE / 2] = {0};
static volatile uint8_t i2c_last_beam_bands[NUM_BEAMS][NUM_FREQ_BINS] = {{0}};
static volatile uint8_t i2c_last_valid[NUM_BEAMS] = {0, 0, 0, 0, 0};
static uint8_t fft_bin_marker[FFT_SIZE / 2] = {0}; // 0=none,1=single,2=pair-open,3=pair-close
// Runtime digital gain applied before FFT (Q15 conversion stage)
static volatile int fft_input_gain = 20;
static volatile uint8_t beam_fft_gain[NUM_BEAMS] = {20, 20, 20, 20, 20};
static bool agc_enabled = false;
// Runtime FFT magnitude floor after bin-summing (lower helps speech harmonics show)
static volatile uint16_t fft_noise_floor = 2;
static volatile uint16_t beam_noise_floor[NUM_BEAMS] = {2, 2, 2, 2, 2};
// Optional log companding for I2C export so low-level signals are emphasized.
// Default ON.
static bool i2c_log_compand_enabled = true;
static uint8_t i2c_log_lut[256] = {0};
// RAW ADC hardware-bias clip window in mV (for diagnostics only).
static volatile uint16_t raw_clip_low_mv = 150;
static volatile uint16_t raw_clip_high_mv = 3150;
static const uint16_t raw_avg_target_mv = 1650;
// Long-term per-bin noise suppression (subtract slowly-tracked bin baseline)
static bool bin_noise_filter_enabled = true;
static volatile uint8_t bin_noise_filter_shift = 8;  // ~4.1s at 62.5 updates/sec
// Percentage of estimated baseline to subtract (0..100)
static volatile uint8_t bin_filter_strength_pct = 65;
static volatile uint8_t beam_filter_strength_pct[NUM_BEAMS] = {65, 65, 65, 65, 65};
// Max percentage of current bin magnitude that filter is allowed to subtract (0..95)
static volatile uint8_t bin_filter_max_sub_pct = 90;
// AGC backoff trigger sensitivity (minimum clipped samples per FFT frame)
static volatile uint8_t agc_clip_min_samples = 12;
static bool beam_processing_enabled[NUM_BEAMS] = {true, true, true, true, true};

// Q8 running baseline per beam/bin for long-term noise estimation
static uint32_t bin_noise_est_q8[NUM_BEAMS][NUM_FREQ_BINS] = {0};

#define FFT_GAIN_MIN 1
#define FFT_GAIN_MAX 64
#define AGC_HEADROOM_Q15 ((int32_t)(32767 * 8 / 10))  // 80% of full-scale
#define AGC_LOW_LEVEL_Q15 ((int32_t)(32767 * 35 / 100))  // 35% of full-scale
#define AGC_FAST_RAMP_UP_HOLD_MS 300
#define AGC_NORMAL_RAMP_UP_HOLD_MS 1200

static volatile uint16_t agc_last_peak_q15 = 0;
static volatile uint16_t agc_last_raw_bin_avg = 0;
static volatile uint32_t agc_last_clip_ms = 0;
static volatile uint32_t agc_stable_start_ms = 0;
static volatile uint32_t agc_clip_events = 0;
static volatile uint16_t beam_last_clipped_samples[NUM_BEAMS] = {0, 0, 0, 0, 0};
static uint32_t beam_raw_bin_avg_ema_q8[NUM_BEAMS] = {0};
static volatile uint16_t beam_filter_avg5s[NUM_BEAMS] = {0, 0, 0, 0, 0};
static volatile uint8_t beam_filter_auto_strength_pct[NUM_BEAMS] = {55, 55, 55, 55, 55};

#define AGC_RAW_AVG_EMA_SHIFT 3
#define AGC_RAW_AVG_LOW_OFFSET 24
#define AGC_RAW_AVG_HIGH_OFFSET 72
#define FILTER_BACKOFF_REF_GAIN 12
#define BAND_FILTER_AUTO_AVG_DIV 312
#define BAND_FILTER_AUTO_LOW_OFFSET 14
#define BAND_FILTER_AUTO_HIGH_OFFSET 84
#define BAND_FILTER_AUTO_MIN_STRENGTH 25
#define BAND_FILTER_AUTO_MAX_STRENGTH 92

static uint8_t compute_auto_filter_strength(uint16_t avg5s, uint16_t noise_floor, uint8_t base_strength) {
    uint16_t low = (uint16_t)(noise_floor + BAND_FILTER_AUTO_LOW_OFFSET);
    uint16_t high = (uint16_t)(noise_floor + BAND_FILTER_AUTO_HIGH_OFFSET);
    if (high <= low) high = (uint16_t)(low + 1u);

    uint32_t base = base_strength;
    if (avg5s <= low) {
        uint32_t s = base + 25u;
        if (s > BAND_FILTER_AUTO_MAX_STRENGTH) s = BAND_FILTER_AUTO_MAX_STRENGTH;
        return (uint8_t)s;
    }
    if (avg5s >= high) {
        int32_t s = (int32_t)base - 25;
        if (s < BAND_FILTER_AUTO_MIN_STRENGTH) s = BAND_FILTER_AUTO_MIN_STRENGTH;
        return (uint8_t)s;
    }

    uint32_t span = (uint32_t)(high - low);
    uint32_t pos = (uint32_t)(avg5s - low);
    int32_t hi_s = (int32_t)base + 25;
    int32_t lo_s = (int32_t)base - 25;
    if (hi_s > BAND_FILTER_AUTO_MAX_STRENGTH) hi_s = BAND_FILTER_AUTO_MAX_STRENGTH;
    if (lo_s < BAND_FILTER_AUTO_MIN_STRENGTH) lo_s = BAND_FILTER_AUTO_MIN_STRENGTH;
    int32_t delta = hi_s - lo_s;
    int32_t interp = hi_s - (int32_t)((delta * (int32_t)pos) / (int32_t)span);
    return (uint8_t)interp;
}

static uint32_t bin_filter_time_ms_from_shift(uint8_t shift) {
    // Beam FFT update rate is ~TARGET_SAMPLE_RATE / FFT_SIZE = 62.5 Hz
    const uint32_t updates_per_sec = TARGET_SAMPLE_RATE / FFT_SIZE;  // 62
    return (1000u * (1u << shift) + (updates_per_sec / 2u)) / updates_per_sec;
}

static uint8_t bin_filter_shift_from_seconds(uint32_t seconds) {
    uint32_t target_ms = seconds * 1000u;
    uint8_t best_shift = 8;
    uint32_t best_err = 0xFFFFFFFFu;

    for (uint8_t s = 4; s <= 12; s++) {
        uint32_t ms = bin_filter_time_ms_from_shift(s);
        uint32_t err = (ms > target_ms) ? (ms - target_ms) : (target_ms - ms);
        if (err < best_err) {
            best_err = err;
            best_shift = s;
        }
    }
    return best_shift;
}

static bool is_valid_beam_index(int beam_idx) {
    return (beam_idx >= 0 && beam_idx < NUM_BEAMS);
}

static uint32_t settings_crc32(const uint8_t *data, size_t length) {
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            uint32_t mask = (uint32_t)-(int32_t)(crc & 1u);
            crc = (crc >> 1) ^ (0xEDB88320u & mask);
        }
    }
    return ~crc;
}

static void apply_mic_gain_q8(const uint16_t *input, uint16_t *output, uint16_t gain_q8, uint16_t *clip_count_out) {
    uint16_t clip_count = 0;
    if (gain_q8 < MIC_GAIN_Q8_MIN) gain_q8 = MIC_GAIN_Q8_MIN;
    if (gain_q8 > MIC_GAIN_Q8_MAX) gain_q8 = MIC_GAIN_Q8_MAX;

    for (int i = 0; i < CAPTURE_DEPTH; i++) {
        int32_t centered = (int32_t)input[i] - 2048;
        int32_t scaled_q8 = centered * (int32_t)gain_q8;
        int32_t gained = (scaled_q8 >= 0) ? ((scaled_q8 + 128) >> 8) : ((scaled_q8 - 128) >> 8);
        int32_t shifted = gained + 2048;

        if (shifted < 0) {
            shifted = 0;
            clip_count++;
        } else if (shifted > 4095) {
            shifted = 4095;
            clip_count++;
        }
        output[i] = (uint16_t)shifted;
    }

    if (clip_count_out) *clip_count_out = clip_count;
}

static void refresh_global_from_per_beam_defaults(void) {
    fft_input_gain = beam_fft_gain[NUM_BEAMS / 2];
    fft_noise_floor = beam_noise_floor[NUM_BEAMS / 2];
    bin_filter_strength_pct = beam_filter_strength_pct[NUM_BEAMS / 2];
}

static bool load_settings_from_flash(void) {
    const PersistedSettings *stored = (const PersistedSettings *)(XIP_BASE + SETTINGS_FLASH_OFFSET);
    if (stored->magic != SETTINGS_MAGIC) return false;
    if (stored->version != SETTINGS_VERSION) return false;
    if (stored->size != sizeof(PersistedSettings)) return false;

    uint32_t crc_calc = settings_crc32((const uint8_t *)stored, sizeof(PersistedSettings) - sizeof(uint32_t));
    if (crc_calc != stored->crc32) return false;

    for (int i = 0; i < ADC_NUM_CHANNELS; i++) {
        uint16_t g = stored->mic_gain_q8[i];
        if (g < MIC_GAIN_Q8_MIN || g > MIC_GAIN_Q8_MAX) g = MIC_GAIN_Q8_DEFAULT;
        mic_gain_q8[i] = g;
    }

    for (int b = 0; b < NUM_BEAMS; b++) {
        uint8_t g = stored->beam_fft_gain[b];
        if (g < FFT_GAIN_MIN || g > FFT_GAIN_MAX) g = (uint8_t)FFT_GAIN_MIN;
        beam_fft_gain[b] = g;

        uint16_t floor = stored->beam_noise_floor[b];
        if (floor > 255) floor = 255;
        beam_noise_floor[b] = floor;

        uint8_t strength = stored->beam_filter_strength_pct[b];
        if (strength > 100) strength = 100;
        beam_filter_strength_pct[b] = strength;

        beam_processing_enabled[b] = (stored->beam_processing_enabled[b] != 0);
    }

    bin_noise_filter_enabled = (stored->bin_noise_filter_enabled != 0);
    if (stored->bin_noise_filter_shift >= 4 && stored->bin_noise_filter_shift <= 12) {
        bin_noise_filter_shift = stored->bin_noise_filter_shift;
    }

    if (stored->bin_filter_max_sub_pct <= 95) {
        bin_filter_max_sub_pct = stored->bin_filter_max_sub_pct;
    }

    refresh_global_from_per_beam_defaults();
    return true;
}

static bool save_settings_to_flash(void) {
    PersistedSettings settings;
    memset(&settings, 0, sizeof(settings));

    settings.magic = SETTINGS_MAGIC;
    settings.version = SETTINGS_VERSION;
    settings.size = sizeof(PersistedSettings);

    for (int i = 0; i < ADC_NUM_CHANNELS; i++) {
        settings.mic_gain_q8[i] = mic_gain_q8[i];
    }
    for (int b = 0; b < NUM_BEAMS; b++) {
        settings.beam_fft_gain[b] = beam_fft_gain[b];
        settings.beam_noise_floor[b] = beam_noise_floor[b];
        settings.beam_filter_strength_pct[b] = beam_filter_strength_pct[b];
        settings.beam_processing_enabled[b] = beam_processing_enabled[b] ? 1u : 0u;
    }
    settings.bin_noise_filter_enabled = bin_noise_filter_enabled ? 1u : 0u;
    settings.bin_noise_filter_shift = bin_noise_filter_shift;
    settings.bin_filter_max_sub_pct = bin_filter_max_sub_pct;
    settings.crc32 = settings_crc32((const uint8_t *)&settings, sizeof(PersistedSettings) - sizeof(uint32_t));

    multicore_lockout_start_blocking();
    uint32_t irq_state = save_and_disable_interrupts();
    flash_range_erase(SETTINGS_FLASH_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(SETTINGS_FLASH_OFFSET, (const uint8_t *)&settings, sizeof(PersistedSettings));
    restore_interrupts(irq_state);
    multicore_lockout_end_blocking();

    const PersistedSettings *verify = (const PersistedSettings *)(XIP_BASE + SETTINGS_FLASH_OFFSET);
    uint32_t verify_crc = settings_crc32((const uint8_t *)verify, sizeof(PersistedSettings) - sizeof(uint32_t));
    return (verify->magic == SETTINGS_MAGIC && verify->version == SETTINGS_VERSION && verify->size == sizeof(PersistedSettings) && verify->crc32 == verify_crc);
}

static void persist_settings_quietly(void) {
    settings_last_change_ms = to_ms_since_boot(get_absolute_time());
    settings_save_pending = true;
}

static void scale_bin_noise_estimates(uint16_t num, uint16_t den) {
    if (den == 0) return;

    for (int beam = 0; beam < NUM_BEAMS; beam++) {
        for (int i = 0; i < NUM_FREQ_BINS; i++) {
            uint32_t est = bin_noise_est_q8[beam][i];
            bin_noise_est_q8[beam][i] = (uint32_t)(((uint64_t)est * num) / den);
        }
    }
}

static void apply_bin_noise_filter(int beam_idx, uint16_t *bins) {
    if (!bin_noise_filter_enabled || !is_valid_beam_index(beam_idx)) return;

    uint8_t shift = bin_noise_filter_shift;
    if (shift < 1) shift = 1;

    // Auto strength uses a ~5s average of each beam's raw bins.
    uint32_t strength_pct = beam_filter_auto_strength_pct[beam_idx];

    // If gain is reduced below reference, further soften subtraction.
    int gain_now = beam_fft_gain[beam_idx];
    if (gain_now < FFT_GAIN_MIN) gain_now = FFT_GAIN_MIN;
    if (gain_now < FILTER_BACKOFF_REF_GAIN) {
        uint32_t gain_sq = (uint32_t)gain_now * (uint32_t)gain_now;
        uint32_t ref_sq = (uint32_t)FILTER_BACKOFF_REF_GAIN * (uint32_t)FILTER_BACKOFF_REF_GAIN;
        strength_pct = (strength_pct * gain_sq + (ref_sq / 2u)) / ref_sq;
    }

    for (int i = 0; i < NUM_FREQ_BINS; i++) {
        uint32_t est = bin_noise_est_q8[beam_idx][i];
        uint32_t sample_q8 = ((uint32_t)bins[i]) << 8;

        if (sample_q8 >= est) {
            est += (sample_q8 - est) >> shift;
        } else {
            est -= (est - sample_q8) >> shift;
        }

        bin_noise_est_q8[beam_idx][i] = est;

        uint16_t baseline = (uint16_t)(est >> 8);
        uint16_t sub = (uint16_t)(((uint32_t)baseline * strength_pct) / 100u);
        uint16_t max_sub = (uint16_t)(((uint32_t)bins[i] * bin_filter_max_sub_pct) / 100u);
        if (sub > max_sub) sub = max_sub;
        bins[i] = (bins[i] > sub) ? (bins[i] - sub) : 0;
    }
}

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

static uint16_t diag_cycles_from_seconds(uint16_t seconds) {
    if (seconds < 1) seconds = 1;
    if (seconds > 60) seconds = 60;
    uint32_t updates_per_sec = TARGET_SAMPLE_RATE / CAPTURE_DEPTH;  // ~62
    uint32_t cycles = (uint32_t)seconds * updates_per_sec;
    if (cycles < 1u) cycles = 1u;
    if (cycles > 65535u) cycles = 65535u;
    return (uint16_t)cycles;
}

static uint16_t diag_seconds_from_cycles(uint16_t cycles) {
    if (cycles < 1) cycles = 1;
    uint32_t updates_per_sec = TARGET_SAMPLE_RATE / CAPTURE_DEPTH;  // ~62
    uint32_t seconds = ((uint32_t)cycles + (updates_per_sec / 2u)) / updates_per_sec;
    if (seconds < 1u) seconds = 1u;
    if (seconds > 65535u) seconds = 65535u;
    return (uint16_t)seconds;
}

static void reset_period_p2p_stats(void) {
    for (int ch = 0; ch < ADC_NUM_CHANNELS; ch++) {
        raw_p2p_min_period[ch] = 0xFFFFu;
        raw_p2p_max_period[ch] = 0u;
        raw_p2p_sum_period[ch] = 0u;
        post_p2p_min_period[ch] = 0xFFFFu;
        post_p2p_max_period[ch] = 0u;
        post_p2p_sum_period[ch] = 0u;
    }
    for (int b = 0; b < NUM_BEAMS; b++) {
        beam_raw_p2p_min_period[b] = 0xFFFFu;
        beam_raw_p2p_max_period[b] = 0u;
        beam_raw_p2p_sum_period[b] = 0u;
        beam_post_p2p_min_period[b] = 0xFFFFu;
        beam_post_p2p_max_period[b] = 0u;
        beam_post_p2p_sum_period[b] = 0u;
    }
    p2p_period_samples = 0;
}

static void accumulate_period_p2p_stats(const uint16_t *raw_p2p, const uint16_t *post_p2p) {
    for (int ch = 0; ch < ADC_NUM_CHANNELS; ch++) {
        uint16_t r = raw_p2p[ch];
        uint16_t p = post_p2p[ch];

        if (r < raw_p2p_min_period[ch]) raw_p2p_min_period[ch] = r;
        if (r > raw_p2p_max_period[ch]) raw_p2p_max_period[ch] = r;
        raw_p2p_sum_period[ch] += r;

        if (p < post_p2p_min_period[ch]) post_p2p_min_period[ch] = p;
        if (p > post_p2p_max_period[ch]) post_p2p_max_period[ch] = p;
        post_p2p_sum_period[ch] += p;
    }
    p2p_period_samples++;
}

static void accumulate_beam_period_p2p_stats(const uint16_t *raw_p2p, const uint16_t *post_p2p) {
    for (int b = 0; b < NUM_BEAMS; b++) {
        uint16_t r = raw_p2p[b];
        uint16_t p = post_p2p[b];

        if (r < beam_raw_p2p_min_period[b]) beam_raw_p2p_min_period[b] = r;
        if (r > beam_raw_p2p_max_period[b]) beam_raw_p2p_max_period[b] = r;
        beam_raw_p2p_sum_period[b] += r;

        if (p < beam_post_p2p_min_period[b]) beam_post_p2p_min_period[b] = p;
        if (p > beam_post_p2p_max_period[b]) beam_post_p2p_max_period[b] = p;
        beam_post_p2p_sum_period[b] += p;
    }
}

static uint16_t compute_post_fft_gain_adc_p2p(uint16_t *samples, int gain) {
    int32_t min_v = 4095;
    int32_t max_v = 0;

    if (gain < FFT_GAIN_MIN) gain = FFT_GAIN_MIN;
    if (gain > FFT_GAIN_MAX) gain = FFT_GAIN_MAX;

    for (int i = 0; i < CAPTURE_DEPTH; i++) {
        int32_t centered = (int32_t)samples[i] - 2048;
        int32_t scaled = centered * gain;
        if (scaled > 2047) scaled = 2047;
        if (scaled < -2048) scaled = -2048;
        int32_t out = scaled + 2048;
        if (out < min_v) min_v = out;
        if (out > max_v) max_v = out;
    }

    return (uint16_t)(max_v - min_v);
}

static void print_diag_status(void) {
    printf("STATUS:\n");
    printf("  Sections: status=%-3s watchdog=%-3s raw=%-3s beam=%-3s fft=%-3s\n",
        diag_status_enabled ? "ON" : "OFF",
        diag_status_enabled ? "ON" : "OFF",
        diag_raw_enabled ? "ON" : "OFF",
        diag_beam_enabled ? "ON" : "OFF",
        diag_fft_enabled ? "ON" : "OFF");
    printf("  Streams : tone=%-3s i2c=%-3s i2cMapMode=%-6s diagrate=%2us\n",
        diag_tone_enabled ? "ON" : "OFF",
        diag_i2c_error_log_enabled ? "ON" : "OFF",
        i2c_log_compand_enabled ? "LOG" : "LINEAR",
        diag_seconds_from_cycles(diag_interval_cycles));
    printf("  Format  : diagcompact=%s\n", diag_compact_enabled ? "ON" : "OFF");
    printf("  SaveCfg: pending=%s ok=%lu fail=%lu\n",
        settings_save_pending ? "YES" : "NO",
        (unsigned long)settings_save_ok_count,
        (unsigned long)settings_save_fail_count);
    printf("  FFTDbg : raw=%s fmt=%s beams=[%s %s %s %s %s]\n",
        diag_fft_raw_enabled ? "ON" : "OFF",
        (diag_fft_raw_format == 0) ? "HEX" : "PCT",
        diag_fft_beam_enabled[0] ? "B0" : "--",
        diag_fft_beam_enabled[1] ? "B1" : "--",
        diag_fft_beam_enabled[2] ? "B2" : "--",
        diag_fft_beam_enabled[3] ? "B3" : "--",
        diag_fft_beam_enabled[4] ? "B4" : "--");
    printf("  I2COut : fmt=%s beams=[%s %s %s %s %s]\n",
        (diag_i2c_output_format == 0) ? "HEX" : "PCT",
        diag_i2c_output_beam_enabled[0] ? "B0" : "--",
        diag_i2c_output_beam_enabled[1] ? "B1" : "--",
        diag_i2c_output_beam_enabled[2] ? "B2" : "--",
        diag_i2c_output_beam_enabled[3] ? "B3" : "--",
        diag_i2c_output_beam_enabled[4] ? "B4" : "--");
}

static void init_fft_debug_bin_markers(void) {
    memset(fft_bin_marker, 0, sizeof(fft_bin_marker));

    for (int out_bin = 0; out_bin < NUM_FREQ_BINS; out_bin++) {
        int fft_bin0 = 1;
        int fft_bin1 = 1;
        band_to_fft_range(out_bin, &fft_bin0, &fft_bin1);

        if (fft_bin0 < 0 || fft_bin0 >= (FFT_SIZE / 2) || fft_bin1 < 0 || fft_bin1 >= (FFT_SIZE / 2)) {
            continue;
        }

        if (fft_bin0 == fft_bin1) {
            fft_bin_marker[fft_bin0] = 1;
        } else {
            fft_bin_marker[fft_bin0] = 2;
            fft_bin_marker[fft_bin1] = 3;
        }
    }
}

static void print_fft_raw_row_cell(uint16_t val, uint8_t marker) {
    const char *inv_on = "\x1b[7m";
    const char *inv_off = "\x1b[0m";

    if (diag_fft_raw_format == 0) {
        if (marker == 1) {
            printf("[%s%04X%s] ", inv_on, val, inv_off);
        } else if (marker == 2) {
            printf("[%s%04X%s  ", inv_on, val, inv_off);
        } else if (marker == 3) {
            printf(" %s%04X%s] ", inv_on, val, inv_off);
        } else {
            printf(" %04X  ", val);
        }
    } else {
        uint32_t pct100 = ((uint32_t)val * 10000u + 32767u) / 65535u;
        uint32_t whole = pct100 / 100u;
        uint32_t frac = pct100 % 100u;
        if (marker == 1) {
            printf("[%s%3u.%02u%s] ", inv_on, (unsigned)whole, (unsigned)frac, inv_off);
        } else if (marker == 2) {
            printf("[%s%3u.%02u%s  ", inv_on, (unsigned)whole, (unsigned)frac, inv_off);
        } else if (marker == 3) {
            printf(" %s%3u.%02u%s] ", inv_on, (unsigned)whole, (unsigned)frac, inv_off);
        } else {
            printf(" %3u.%02u  ", (unsigned)whole, (unsigned)frac);
        }
    }
}

static void print_fft_raw_table_for_beam(int beam_idx) {
    if (!is_valid_beam_index(beam_idx)) return;

    printf("  Beam B%d RAW FFT bins 0..127 (%s):\n", beam_idx, (diag_fft_raw_format == 0) ? "HEX" : "PCT");
    printf("  (highlighted entries are output-band bins; [] marks single bins or bin pairs)\n");

    for (int row = 0; row < 8; row++) {
        int start = row * 16;
        int end = start + 15;
        printf("%03d-%03d: ", start, end);
        for (int i = start; i <= end; i++) {
            uint16_t v = fft_last_mag[beam_idx][i];
            uint8_t marker = fft_bin_marker[i];
            print_fft_raw_row_cell(v, marker);
        }
        printf("\n");
    }
}

static void print_i2c_output_row_cell(uint8_t val) {
    if (diag_i2c_output_format == 0) {
        printf(" %02X ", val);
    } else {
        uint32_t pct10 = ((uint32_t)val * 1000u + 127u) / 255u;
        uint32_t whole = pct10 / 10u;
        uint32_t frac = pct10 % 10u;
        printf("%3u.%1u", (unsigned)whole, (unsigned)frac);
    }
}

static void print_i2c_output_table_for_beam(int beam_idx) {
    if (!is_valid_beam_index(beam_idx)) return;

    printf("  I2C_Output_Beam %d (%s, 45 bands):\n", beam_idx, (diag_i2c_output_format == 0) ? "HEX" : "PCT");
    printf("  Grid: 5 bands x 8 rows (+ tail for bands 40..44)\n");

    for (int row = 0; row < 8; row++) {
        int start = row * 5;
        int end = start + 4;
        printf("  Band %02d-%02d: ", start, end);
        for (int i = start; i <= end; i++) {
            print_i2c_output_row_cell(i2c_last_beam_bands[beam_idx][i]);
            printf(" ");
        }
        printf("\n");
    }

    printf("  Band 40-44: ");
    for (int i = 40; i < NUM_FREQ_BINS; i++) {
        print_i2c_output_row_cell(i2c_last_beam_bands[beam_idx][i]);
        printf(" ");
    }
    printf("\n");
}

static void print_mic_status(void) {
    printf("Mic status:\n");
    printf("  M   Gain(%%)  Gain(Q8)  ClipCnt\n");
    for (int m = 0; m < ADC_NUM_CHANNELS; m++) {
        uint16_t gain_pct = (uint16_t)((mic_gain_q8[m] * 100u) / MIC_GAIN_Q8_UNITY);
        printf("  M%-1d %8u %9u %8u\n", m, gain_pct, mic_gain_q8[m], mic_gain_last_clip_count[m]);
    }
}

static void print_diag_help(void) {
    printf("DIAG commands:\n");
    printf("  help                 -> show this help\n");
    printf("\n");
    printf("[Mic / ADC input stage]\n");
    printf("  micgain M P          -> set mic M gain in percent P (M=0..2, P=25..800, starts at 150%%)\n");
    printf("  rawclip L H          -> set RAW clip window in mV (L=low, H=high; default 150 3150)\n");
    printf("  micstatus            -> show compact per-mic gain + clipping status\n");
    printf("  raw on|off           -> enable/disable RAW ADC table\n");
    printf("\n");
    printf("[Beamforming stage]\n");
    printf("  skew on|off          -> enable/disable ADC skew correction in beamforming\n");
    printf("  beam on|off          -> enable/disable BEAMFORMED table\n");
    printf("  bgain B N            -> set gain for one beam B (0-4), N in 1-64\n");
    printf("  benable B on|off     -> enable/disable processing for one beam B\n");
    printf("\n");
    printf("[FFT stage]\n");
    printf("  fft on|off           -> enable/disable FFT diagnostic section\n");
    printf("  fftbeam B on|off     -> enable/disable RAW FFT table for beam B (default: only B2 ON)\n");
    printf("  fftraw on|off        -> show/hide raw FFT bin values for selected beam\n");
    printf("  fftrawfmt hex|pct    -> RAW FFT display as hex or %% of full-scale\n");
    printf("  nfloor N             -> set FFT noise floor for all beams (0-255)\n");
    printf("  bnfloor B N          -> set FFT noise floor for one beam B (0-255)\n");
    printf("  bfilter on|off       -> enable/disable long-term per-bin noise suppression\n");
    printf("  bsec N               -> set bin-filter time constant in seconds (1-8)\n");
    printf("  bstrength N          -> set bin-filter subtraction strength for all beams (0-100)\n");
    printf("  bbstrength B N       -> set subtraction strength for one beam B (0-100)\n");
    printf("  bcap N               -> set max per-bin subtraction cap in %% (0-95)\n");
    printf("\n");
    printf("[Output stage]\n");
    printf("  i2clog on|off        -> select I2C 16->8 mapping mode: log (ON) or linear (OFF), default ON\n");
    printf("  i2coutput B on|off   -> enable/disable I2C output table for beam B (default beam=2 if omitted)\n");
    printf("  i2coutput on|off     -> shorthand for i2coutput 2 on|off\n");
    printf("  i2coutputfmt hex|pct -> I2C output display as 8-bit hex or %% of full-scale\n");
    printf("  i2c on|off           -> enable/disable I2C failure serial logs\n");
    printf("  tone on|off          -> enable/disable tone monitor line\n");
    printf("  band N               -> set tone monitor output band index (0-%d)\n", NUM_FREQ_BINS - 1);
    printf("  bin N                -> alias for band N (legacy command)\n");
    printf("  N                    -> shorthand for band N (output band index)\n");
    printf("\n");
    printf("[General / diagnostics]\n");
    printf("  status               -> show current diagnostic mode status\n");
    printf("  status on|off        -> enable/disable periodic status with RAW/BEAM blocks\n");
    printf("  diagcompact on|off   -> compact or verbose RAW/BEAM tables\n");
    printf("  diagrate N           -> set diagnostic print period in seconds (1-60)\n");
    printf("  agc on|off           -> disabled (gain control is manual)\n");
    printf("  agcclip N            -> AGC backoff threshold: clipped samples/frame (1-64)\n");
    printf("  savecfg              -> save mic/beam settings to flash\n");
    printf("  loadcfg              -> load settings from flash\n");
}

static void process_diag_command(const char *cmd_buf) {
    char arg[16] = {0};
    int new_bin = -1;
    int beam_idx = -1;
    int value = -1;

    if (sscanf(cmd_buf, "%d", &new_bin) == 1 ||
        sscanf(cmd_buf, "band %d", &new_bin) == 1 || sscanf(cmd_buf, "BAND %d", &new_bin) == 1 ||
        sscanf(cmd_buf, "bin %d", &new_bin) == 1 || sscanf(cmd_buf, "BIN %d", &new_bin) == 1) {
        if (new_bin >= 0 && new_bin < NUM_FREQ_BINS) {
            tone_monitor_bin_idx_runtime = new_bin;
            printf("Tone monitor band set: %d (~%.0f Hz)\n", new_bin, tone_monitor_hz_from_bin(new_bin));
        } else {
            printf("Invalid band: %d (valid range 0-%d)\n", new_bin, NUM_FREQ_BINS - 1);
        }
    } else if (sscanf(cmd_buf, "status %15s", arg) == 1 || sscanf(cmd_buf, "STATUS %15s", arg) == 1) {
        bool v;
        if (parse_onoff(arg, &v)) {
            diag_status_enabled = v;
            printf("Periodic status stream: %s\n", diag_status_enabled ? "ON" : "OFF");
        } else {
            printf("Usage: status on|off\n");
        }
    } else if (sscanf(cmd_buf, "diagcompact %15s", arg) == 1 || sscanf(cmd_buf, "DIAGCOMPACT %15s", arg) == 1) {
        bool v;
        if (parse_onoff(arg, &v)) {
            diag_compact_enabled = v;
            printf("Diagnostic table format: %s\n", diag_compact_enabled ? "COMPACT" : "VERBOSE");
        } else {
            printf("Usage: diagcompact on|off\n");
        }
    } else if (sscanf(cmd_buf, "diagrate %d", &new_bin) == 1 || sscanf(cmd_buf, "DIAGRATE %d", &new_bin) == 1) {
        if (new_bin >= 1 && new_bin <= 60) {
            diag_interval_cycles = diag_cycles_from_seconds((uint16_t)new_bin);
            printf("Diagnostic period set: %u s (~%u cycles)\n", diag_seconds_from_cycles(diag_interval_cycles), diag_interval_cycles);
        } else {
            printf("Invalid diagrate: %d (valid range 1-60 seconds)\n", new_bin);
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
    } else if (sscanf(cmd_buf, "fftbeam %d %15s", &new_bin, arg) == 2 || sscanf(cmd_buf, "FFTBEAM %d %15s", &new_bin, arg) == 2) {
        bool v;
        if (!is_valid_beam_index(new_bin)) {
            printf("Invalid fftbeam: %d (valid range 0-%d)\n", new_bin, NUM_BEAMS - 1);
        } else if (!parse_onoff(arg, &v)) {
            printf("Usage: fftbeam B on|off\n");
        } else {
            diag_fft_beam_enabled[new_bin] = v;
            printf("FFT RAW table for B%d: %s\n", new_bin, v ? "ON" : "OFF");
        }
    } else if (sscanf(cmd_buf, "fftrawfmt %15s", arg) == 1 || sscanf(cmd_buf, "FFTRAWFMT %15s", arg) == 1) {
        if (str_ieq(arg, "hex")) {
            diag_fft_raw_format = 0;
            printf("FFT RAW format: HEX\n");
        } else if (str_ieq(arg, "pct") || str_ieq(arg, "percent")) {
            diag_fft_raw_format = 1;
            printf("FFT RAW format: PCT\n");
        } else {
            printf("Usage: fftrawfmt hex|pct\n");
        }
    } else if (sscanf(cmd_buf, "fftraw %15s", arg) == 1 || sscanf(cmd_buf, "FFTRAW %15s", arg) == 1) {
        bool v;
        if (parse_onoff(arg, &v)) {
            diag_fft_raw_enabled = v;
            printf("FFT raw bins: %s\n", diag_fft_raw_enabled ? "ON" : "OFF");
        } else {
            printf("Usage: fftraw on|off\n");
        }
    } else if (sscanf(cmd_buf, "i2coutputfmt %15s", arg) == 1 || sscanf(cmd_buf, "I2COUTPUTFMT %15s", arg) == 1) {
        if (str_ieq(arg, "hex")) {
            diag_i2c_output_format = 0;
            printf("I2C output format: HEX\n");
        } else if (str_ieq(arg, "pct") || str_ieq(arg, "percent")) {
            diag_i2c_output_format = 1;
            printf("I2C output format: PCT\n");
        } else {
            printf("Usage: i2coutputfmt hex|pct\n");
        }
    } else if (sscanf(cmd_buf, "i2coutput %d %15s", &new_bin, arg) == 2 || sscanf(cmd_buf, "I2COUTPUT %d %15s", &new_bin, arg) == 2) {
        bool v;
        if (!is_valid_beam_index(new_bin)) {
            printf("Invalid i2coutput beam: %d (valid range 0-%d)\n", new_bin, NUM_BEAMS - 1);
        } else if (!parse_onoff(arg, &v)) {
            printf("Usage: i2coutput B on|off\n");
        } else {
            diag_i2c_output_beam_enabled[new_bin] = v;
            printf("I2C output table for B%d: %s\n", new_bin, v ? "ON" : "OFF");
        }
    } else if (sscanf(cmd_buf, "i2coutput %15s", arg) == 1 || sscanf(cmd_buf, "I2COUTPUT %15s", arg) == 1) {
        bool v;
        int default_beam = 2;
        if (!parse_onoff(arg, &v)) {
            printf("Usage: i2coutput [B] on|off\n");
        } else {
            diag_i2c_output_beam_enabled[default_beam] = v;
            printf("I2C output table for B%d: %s\n", default_beam, v ? "ON" : "OFF");
        }
    } else if (sscanf(cmd_buf, "fft %15s", arg) == 1 || sscanf(cmd_buf, "FFT %15s", arg) == 1) {
        bool v;
        if (parse_onoff(arg, &v)) {
            diag_fft_enabled = v;
            printf("FFT diagnostics: %s\n", diag_fft_enabled ? "ON" : "OFF");
        } else {
            printf("Usage: fft on|off\n");
        }
    } else if (sscanf(cmd_buf, "tone %15s", arg) == 1 || sscanf(cmd_buf, "TONE %15s", arg) == 1) {
        bool v;
        if (parse_onoff(arg, &v)) {
            diag_tone_enabled = v;
            printf("Tone monitor: %s\n", diag_tone_enabled ? "ON" : "OFF");
        } else {
            printf("Usage: tone on|off\n");
        }
    } else if (sscanf(cmd_buf, "i2clog %15s", arg) == 1 || sscanf(cmd_buf, "I2CLOG %15s", arg) == 1) {
        bool v;
        if (parse_onoff(arg, &v)) {
            i2c_log_compand_enabled = v;
            printf("I2C log companding: %s\n", i2c_log_compand_enabled ? "ON" : "OFF");
        } else {
            printf("Usage: i2clog on|off\n");
        }
    } else if (sscanf(cmd_buf, "rawclip %d %d", &beam_idx, &value) == 2 || sscanf(cmd_buf, "RAWCLIP %d %d", &beam_idx, &value) == 2) {
        int low_mv = beam_idx;
        int high_mv = value;
        if (low_mv < 0 || low_mv > 3299 || high_mv < 1 || high_mv > 3300 || low_mv >= high_mv) {
            printf("Invalid rawclip: %d %d (valid: 0<=L<H<=3300)\n", low_mv, high_mv);
            printf("Usage: rawclip L H\n");
        } else {
            raw_clip_low_mv = (uint16_t)low_mv;
            raw_clip_high_mv = (uint16_t)high_mv;
            printf("RAW clip window set: %umV..%umV (P2P=%umV)\n",
                   raw_clip_low_mv,
                   raw_clip_high_mv,
                   (uint16_t)(raw_clip_high_mv - raw_clip_low_mv));
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
        (void)arg;
        agc_enabled = false;
        printf("AGC disabled in this mode; using manual gain\n");
    } else if (sscanf(cmd_buf, "micgain %d %d", &beam_idx, &value) == 2 || sscanf(cmd_buf, "MICGAIN %d %d", &beam_idx, &value) == 2) {
        if (beam_idx < 0 || beam_idx >= ADC_NUM_CHANNELS) {
            printf("Invalid mic index: %d (valid range 0-%d)\n", beam_idx, ADC_NUM_CHANNELS - 1);
        } else if (value < 25 || value > 800) {
            printf("Invalid mic gain percent: %d (valid range 25-800)\n", value);
        } else {
            mic_gain_q8[beam_idx] = (uint16_t)((value * MIC_GAIN_Q8_UNITY + 50) / 100);
            printf("Mic %d gain set: %u%% (Q8=%u)\n", beam_idx, value, mic_gain_q8[beam_idx]);
            persist_settings_quietly();
        }
    } else if (sscanf(cmd_buf, "bgain %d %d", &beam_idx, &value) == 2 || sscanf(cmd_buf, "BGAIN %d %d", &beam_idx, &value) == 2) {
        if (!is_valid_beam_index(beam_idx)) {
            printf("Invalid beam index: %d (valid range 0-%d)\n", beam_idx, NUM_BEAMS - 1);
        } else if (value < FFT_GAIN_MIN || value > FFT_GAIN_MAX) {
            printf("Invalid beam gain: %d (valid range %d-%d)\n", value, FFT_GAIN_MIN, FFT_GAIN_MAX);
        } else {
            beam_fft_gain[beam_idx] = (uint8_t)value;
            printf("Beam %d pre-FFT digital gain set: %dx\n", beam_idx, beam_fft_gain[beam_idx]);
            persist_settings_quietly();
        }
    } else if (sscanf(cmd_buf, "bnfloor %d %d", &beam_idx, &value) == 2 || sscanf(cmd_buf, "BNFLOOR %d %d", &beam_idx, &value) == 2) {
        if (!is_valid_beam_index(beam_idx)) {
            printf("Invalid beam index: %d (valid range 0-%d)\n", beam_idx, NUM_BEAMS - 1);
        } else if (value < 0 || value > 255) {
            printf("Invalid beam nfloor: %d (valid range 0-255)\n", value);
        } else {
            beam_noise_floor[beam_idx] = (uint16_t)value;
            printf("Beam %d FFT noise floor set: %u\n", beam_idx, beam_noise_floor[beam_idx]);
            persist_settings_quietly();
        }
    } else if (sscanf(cmd_buf, "nfloor %d", &new_bin) == 1 || sscanf(cmd_buf, "NFLOOR %d", &new_bin) == 1) {
        if (new_bin >= 0 && new_bin <= 255) {
            fft_noise_floor = (uint16_t)new_bin;
            for (int b = 0; b < NUM_BEAMS; b++) {
                beam_noise_floor[b] = (uint16_t)new_bin;
            }
            printf("FFT noise floor set: %u\n", fft_noise_floor);
            persist_settings_quietly();
        } else {
            printf("Invalid nfloor: %d (valid range 0-255)\n", new_bin);
        }
    } else if (sscanf(cmd_buf, "bfilter %15s", arg) == 1 || sscanf(cmd_buf, "BFILTER %15s", arg) == 1) {
        bool v;
        if (parse_onoff(arg, &v)) {
            bin_noise_filter_enabled = v;
            printf("Bin noise filter: %s\n", bin_noise_filter_enabled ? "ON" : "OFF");
            persist_settings_quietly();
        } else {
            printf("Usage: bfilter on|off\n");
        }
    } else if (sscanf(cmd_buf, "bsec %d", &new_bin) == 1 || sscanf(cmd_buf, "BSEC %d", &new_bin) == 1) {
        if (new_bin >= 1 && new_bin <= 8) {
            bin_noise_filter_shift = bin_filter_shift_from_seconds((uint32_t)new_bin);
            printf("Bin filter time set: ~%u ms\n", bin_filter_time_ms_from_shift(bin_noise_filter_shift));
            persist_settings_quietly();
        } else {
            printf("Invalid bsec: %d (valid range 1-8)\n", new_bin);
        }
    } else if (sscanf(cmd_buf, "bstrength %d", &new_bin) == 1 || sscanf(cmd_buf, "BSTRENGTH %d", &new_bin) == 1) {
        if (new_bin >= 0 && new_bin <= 100) {
            bin_filter_strength_pct = (uint8_t)new_bin;
            for (int b = 0; b < NUM_BEAMS; b++) {
                beam_filter_strength_pct[b] = (uint8_t)new_bin;
            }
            printf("Bin filter strength set: %u%%\n", bin_filter_strength_pct);
            persist_settings_quietly();
        } else {
            printf("Invalid bstrength: %d (valid range 0-100)\n", new_bin);
        }
    } else if (sscanf(cmd_buf, "bbstrength %d %d", &beam_idx, &value) == 2 || sscanf(cmd_buf, "BBSTRENGTH %d %d", &beam_idx, &value) == 2) {
        if (!is_valid_beam_index(beam_idx)) {
            printf("Invalid beam index: %d (valid range 0-%d)\n", beam_idx, NUM_BEAMS - 1);
        } else if (value < 0 || value > 100) {
            printf("Invalid beam strength: %d (valid range 0-100)\n", value);
        } else {
            beam_filter_strength_pct[beam_idx] = (uint8_t)value;
            printf("Beam %d filter strength set: %u%%\n", beam_idx, beam_filter_strength_pct[beam_idx]);
            persist_settings_quietly();
        }
    } else if (sscanf(cmd_buf, "benable %d %15s", &beam_idx, arg) == 2 || sscanf(cmd_buf, "BENABLE %d %15s", &beam_idx, arg) == 2) {
        bool v;
        if (!is_valid_beam_index(beam_idx)) {
            printf("Invalid beam index: %d (valid range 0-%d)\n", beam_idx, NUM_BEAMS - 1);
        } else if (!parse_onoff(arg, &v)) {
            printf("Usage: benable B on|off\n");
        } else {
            beam_processing_enabled[beam_idx] = v;
            printf("Beam %d processing: %s\n", beam_idx, v ? "ON" : "OFF");
            persist_settings_quietly();
        }
    } else if (sscanf(cmd_buf, "bcap %d", &new_bin) == 1 || sscanf(cmd_buf, "BCAP %d", &new_bin) == 1) {
        if (new_bin >= 0 && new_bin <= 95) {
            bin_filter_max_sub_pct = (uint8_t)new_bin;
            printf("Bin filter subtraction cap set: %u%%\n", bin_filter_max_sub_pct);
            persist_settings_quietly();
        } else {
            printf("Invalid bcap: %d (valid range 0-95)\n", new_bin);
        }
    } else if (str_ieq(cmd_buf, "savecfg") || str_ieq(cmd_buf, "save")) {
        if (save_settings_to_flash()) {
            printf("Settings saved to flash\n");
        } else {
            printf("Settings save FAILED\n");
        }
    } else if (str_ieq(cmd_buf, "loadcfg") || str_ieq(cmd_buf, "load")) {
        if (load_settings_from_flash()) {
            printf("Settings loaded from flash\n");
            print_diag_status();
        } else {
            printf("No valid settings found in flash\n");
        }
    } else if (sscanf(cmd_buf, "agcclip %d", &new_bin) == 1 || sscanf(cmd_buf, "AGCCLIP %d", &new_bin) == 1) {
        if (new_bin >= 1 && new_bin <= 64) {
            agc_clip_min_samples = (uint8_t)new_bin;
            printf("AGC clip threshold set: %u clipped samples/frame\n", agc_clip_min_samples);
        } else {
            printf("Invalid agcclip: %d (valid range 1-64)\n", new_bin);
        }
    } else if (str_ieq(cmd_buf, "micstatus") || str_ieq(cmd_buf, "ms")) {
        print_mic_status();
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

// Extract magnitude spectrum and map to NUM_FREQ_BINS output bands.
// Mapping emphasizes low-frequency resolution for speech:
//  - bands 0..7: single bins 1..8
//  - bands 8..42: pairs from (9,10) to (77,78)
//  - bands 43..44: single bins 79 and 80
static void extract_freq_bins_fixed(uint16_t *magnitude, uint16_t *bins, int beam_idx) {
    int max_fft_bin = (FFT_SIZE / 2) - 1;
    uint16_t noise_floor = fft_noise_floor;
    if (is_valid_beam_index(beam_idx)) {
        noise_floor = beam_noise_floor[beam_idx];
    }

    for (int out_bin = 0; out_bin < NUM_FREQ_BINS; out_bin++) {
        int fft_bin0 = 1;
        int fft_bin1 = 1;
        band_to_fft_range(out_bin, &fft_bin0, &fft_bin1);

        uint32_t sum = 0;
        if (fft_bin0 >= 1 && fft_bin0 <= max_fft_bin) sum += magnitude[fft_bin0];
        if (fft_bin1 != fft_bin0 && fft_bin1 >= 1 && fft_bin1 <= max_fft_bin) sum += magnitude[fft_bin1];

        uint16_t val = (sum > 65535u) ? 65535u : (uint16_t)sum;

        // Apply noise floor: suppress weak signals that are likely noise
        if (val < noise_floor) {
            val = 0;
        }

        bins[out_bin] = val;
    }
}

// Compute FFT for a single beamformed channel (fixed-point)
static void compute_fft_spectrum(uint16_t *input, uint16_t *output, int beam_idx) {
    if (is_valid_beam_index(beam_idx) && !beam_processing_enabled[beam_idx]) {
        memset(output, 0, sizeof(uint16_t) * NUM_FREQ_BINS);
        beam_last_clipped_samples[beam_idx] = 0;
        memset((void *)fft_last_mag[beam_idx], 0, sizeof(uint16_t) * (FFT_SIZE / 2));
        fft_last_valid[beam_idx] = 1u;
        return;
    }

    uint16_t local_peak_q15 = 0;
    uint16_t clipped_samples = 0;
    int gain_now = fft_input_gain;
    if (is_valid_beam_index(beam_idx)) {
        gain_now = beam_fft_gain[beam_idx];
    }
    if (gain_now < FFT_GAIN_MIN) gain_now = FFT_GAIN_MIN;
    if (gain_now > FFT_GAIN_MAX) gain_now = FFT_GAIN_MAX;

    // Prepare input: convert to fixed-point Q15
    for (int i = 0; i < FFT_SIZE; i++) {
        // Beamformed output is uint16_t (0-4095 range from 12-bit ADC)
        // Convert to signed Q15: center around 0 and scale up
        // Range: 0-4095 → -2048 to +2047 → -32768 to +32752 in Q15
        int32_t centered = (int32_t)input[i] - 2048;
        int32_t scaled = centered * 16 * gain_now;
        if (scaled > 32767) {
            scaled = 32767;
            clipped_samples++;
        }
        if (scaled < -32768) {
            scaled = -32768;
            clipped_samples++;
        }

        uint16_t abs_q15 = (scaled < 0) ? (uint16_t)(-scaled) : (uint16_t)scaled;
        if (abs_q15 > local_peak_q15) local_peak_q15 = abs_q15;

        int16_t adc_q15 = (int16_t)scaled;
        fft_real[i] = adc_q15;
        fft_imag[i] = 0;
    }

    agc_last_peak_q15 = local_peak_q15;
    if (is_valid_beam_index(beam_idx)) {
        beam_last_clipped_samples[beam_idx] = clipped_samples;
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
    
    // Extract NUM_FREQ_BINS frequency bins
    if (is_valid_beam_index(beam_idx)) {
        for (int i = 0; i < (FFT_SIZE / 2); i++) {
            fft_last_mag[beam_idx][i] = fft_magnitude[i];
        }
        fft_last_valid[beam_idx] = 1u;
    }

    extract_freq_bins_fixed(fft_magnitude, output, beam_idx);

    // AGC control metric from raw, unfiltered frequency bins.
    uint32_t raw_sum = 0;
    for (int i = 0; i < NUM_FREQ_BINS; i++) {
        raw_sum += output[i];
    }
    uint16_t raw_bin_avg = (uint16_t)(raw_sum / NUM_FREQ_BINS);
    agc_last_raw_bin_avg = raw_bin_avg;

    // Use each beam's own ~5 second average for auto filter adjustment.
    if (is_valid_beam_index(beam_idx)) {
        uint32_t ema_q8 = beam_raw_bin_avg_ema_q8[beam_idx];
        uint32_t sample_q8 = ((uint32_t)raw_bin_avg) << 8;
        if (ema_q8 == 0) {
            ema_q8 = sample_q8;
        } else if (sample_q8 >= ema_q8) {
            ema_q8 += (sample_q8 - ema_q8) / BAND_FILTER_AUTO_AVG_DIV;
        } else {
            ema_q8 -= (ema_q8 - sample_q8) / BAND_FILTER_AUTO_AVG_DIV;
        }
        beam_raw_bin_avg_ema_q8[beam_idx] = ema_q8;

        beam_filter_avg5s[beam_idx] = (uint16_t)(ema_q8 >> 8);
        beam_filter_auto_strength_pct[beam_idx] = compute_auto_filter_strength(
            beam_filter_avg5s[beam_idx],
            beam_noise_floor[beam_idx],
            beam_filter_strength_pct[beam_idx]);

        if (clipped_samples >= agc_clip_min_samples) {
            agc_clip_events++;
            agc_last_clip_ms = to_ms_since_boot(get_absolute_time());
        }
    }

    // Post-FFT bin filter intentionally bypassed during FFT tuning stage.
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
    // GPIO 5 for Core 1 heartbeat
    printf("Initializing GPIO 5...\n");
    fflush(stdout);
    gpio_init(5);
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

static void init_i2c_log_lut(void) {
    // y = log(1 + a*x) / log(1 + a), x in [0,1], y in [0,1]
    // a controls how much low-level detail is emphasized.
    const float alpha = 24.0f;
    const float denom = log1pf(alpha);

    for (int i = 0; i < 256; i++) {
        float x = (float)i / 255.0f;
        float y = log1pf(alpha * x) / denom;
        int out = (int)(y * 255.0f + 0.5f);
        if (out < 0) out = 0;
        if (out > 255) out = 255;
        i2c_log_lut[i] = (uint8_t)out;
    }
    i2c_log_lut[0] = 0;
    i2c_log_lut[255] = 255;
}

// Format I2C packet: header + NUM_FREQ_BINS frequency bins (1 byte each)
static void format_i2c_packet(int beam_idx, uint16_t *fft_bins) {
    i2c_packet[0] = I2C_PACKET_HEADER;

    for (int i = 0; i < NUM_FREQ_BINS; i++) {
        // Convert 16-bit magnitude to 8-bit for I2C transport.
        uint16_t val = fft_bins[i];

        // Base 16->8 full-scale mapping (linear mode).
        uint8_t in8 = (uint8_t)((((uint32_t)val * 255u) + 32767u) / 65535u);
        uint8_t out = in8;
        // If i2clog is enabled, use logarithmic 16->8 companding behavior.
        if (i2c_log_compand_enabled) {
            out = i2c_log_lut[in8];
        }
        i2c_packet[1 + i] = out;
        if (is_valid_beam_index(beam_idx)) {
            i2c_last_beam_bands[beam_idx][i] = out;
        }
    }
    if (is_valid_beam_index(beam_idx)) {
        i2c_last_valid[beam_idx] = 1u;
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
static bool send_fft_via_i2c(int beam_idx, uint16_t *fft_bins) {
    uint8_t slave_addr = I2C_BEAM_BASE_ADDR + beam_idx;
    
    // Format the packet
    format_i2c_packet(beam_idx, fft_bins);
    
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
    multicore_lockout_victim_init();

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
                compute_fft_spectrum(local_beam_data[beam], fft_beam_bins[beam], beam);
                gpio_put(DEBUG_FFT_COMPLETE_PIN, 0);

                // Queue FFT result for Core 0 to transmit via I2C
                if (!fft_queue_add(beam, fft_beam_bins[beam])) {
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

    init_i2c_log_lut();
    init_fft_debug_bin_markers();
    
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
        DMA_CONTINUOUS_TRANSFERS,   // Effectively continuous transfers
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

    if (load_settings_from_flash()) {
        printf("Loaded persisted settings from flash\n");
    } else {
        printf("No valid persisted settings found; using defaults\n");
    }
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
    uint32_t initial_transfer_count = DMA_CONTINUOUS_TRANSFERS;  // Track how much DMA has done

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

        // RP2040 DMA transfer_count is finite; with 48 kS/s aggregate ADC this reaches zero in ~24.8h.
        // If the channel stops, immediately re-arm to preserve long-run operation.
        if (!dma_channel_is_busy(g_dma_chan)) {
            uintptr_t write_addr = (uintptr_t)dma_hw->ch[g_dma_chan].write_addr;
            uintptr_t base_addr = (uintptr_t)adc_buffer;
            uint32_t ring_pos = 0;
            if (write_addr >= base_addr) {
                ring_pos = (uint32_t)((write_addr - base_addr) / sizeof(uint16_t));
            }
            ring_pos &= (BUFFER_SIZE - 1u);

            dma_channel_set_trans_count(g_dma_chan, DMA_CONTINUOUS_TRANSFERS, true);
            initial_transfer_count = DMA_CONTINUOUS_TRANSFERS;
            last_pos = ring_pos;
            dma_rearm_count++;

            if (diag_status_enabled) {
                printf("Core0: DMA auto-rearmed (ring_pos=%lu)\n", (unsigned long)ring_pos);
                fflush(stdout);
            }
        }

        // Check if new data is available (every CAPTURE_DEPTH samples)
        uint32_t remaining = dma_hw->ch[g_dma_chan].transfer_count;
        dma_remaining_snapshot = remaining;
        uint32_t current_pos = initial_transfer_count - remaining;
        
        // Handle circular buffer wraparound - check if we have enough new interleaved samples (all 3 channels)
        uint32_t diff = (current_pos >= last_pos) ? (current_pos - last_pos) : (BUFFER_SIZE - last_pos + current_pos);
        
        
        log_counter++;
        
        // Need BLOCK_SAMPLES (3*CAPTURE_DEPTH) samples before processing
        if (diff >= BLOCK_SAMPLES) {
            core0_cycle_count++;
            bool period_boundary = ((core0_cycle_count % diag_interval_cycles) == 0);
            
            // Blink GPIO 11 when we process data
            //gpio_put(DEBUG_ALL_BEAMS_COMPLETE_PIN, 1);
            
            // Extract latest channels from DMA ring with correct phase handling.
            extract_channels_from_ring(current_pos, ch0_data, ch1_data, ch2_data);

            // Apply per-microphone digital gain in integer Q8 before beamforming.
            apply_mic_gain_q8(ch0_data, ch0_gain_data, mic_gain_q8[0], (uint16_t *)&mic_gain_last_clip_count[0]);
            apply_mic_gain_q8(ch1_data, ch1_gain_data, mic_gain_q8[1], (uint16_t *)&mic_gain_last_clip_count[1]);
            apply_mic_gain_q8(ch2_data, ch2_gain_data, mic_gain_q8[2], (uint16_t *)&mic_gain_last_clip_count[2]);

            // Track RAW/POST P2P stats each processing cycle for period min/max/avg reporting.
            uint16_t p2p_raw_min[ADC_NUM_CHANNELS], p2p_raw_max[ADC_NUM_CHANNELS], p2p_raw_avg_dummy[ADC_NUM_CHANNELS];
            uint16_t p2p_post_min[ADC_NUM_CHANNELS], p2p_post_max[ADC_NUM_CHANNELS], p2p_post_avg_dummy[ADC_NUM_CHANNELS];
            uint16_t p2p_raw_now[ADC_NUM_CHANNELS];
            uint16_t p2p_post_now[ADC_NUM_CHANNELS];

            compute_adc_stats(ch0_data, CAPTURE_DEPTH, &p2p_raw_min[0], &p2p_raw_max[0], &p2p_raw_avg_dummy[0]);
            compute_adc_stats(ch1_data, CAPTURE_DEPTH, &p2p_raw_min[1], &p2p_raw_max[1], &p2p_raw_avg_dummy[1]);
            compute_adc_stats(ch2_data, CAPTURE_DEPTH, &p2p_raw_min[2], &p2p_raw_max[2], &p2p_raw_avg_dummy[2]);
            compute_adc_stats(ch0_gain_data, CAPTURE_DEPTH, &p2p_post_min[0], &p2p_post_max[0], &p2p_post_avg_dummy[0]);
            compute_adc_stats(ch1_gain_data, CAPTURE_DEPTH, &p2p_post_min[1], &p2p_post_max[1], &p2p_post_avg_dummy[1]);
            compute_adc_stats(ch2_gain_data, CAPTURE_DEPTH, &p2p_post_min[2], &p2p_post_max[2], &p2p_post_avg_dummy[2]);

            for (int ch = 0; ch < ADC_NUM_CHANNELS; ch++) {
                p2p_raw_now[ch] = (uint16_t)(p2p_raw_max[ch] - p2p_raw_min[ch]);
                p2p_post_now[ch] = (uint16_t)(p2p_post_max[ch] - p2p_post_min[ch]);
            }
            accumulate_period_p2p_stats(p2p_raw_now, p2p_post_now);

            // Beamform all 5 beams
            uint16_t beam_raw_p2p_now[NUM_BEAMS] = {0, 0, 0, 0, 0};
            uint16_t beam_post_p2p_now[NUM_BEAMS] = {0, 0, 0, 0, 0};
            for (int beam = 0; beam < NUM_BEAMS; beam++) {
                beamform_delay_sum(beam, ch0_gain_data, ch1_gain_data, ch2_gain_data, CAPTURE_DEPTH, local_beam_output[beam]);
                uint16_t bmin = 0, bmax = 0, bavg = 0;
                compute_adc_stats(local_beam_output[beam], CAPTURE_DEPTH, &bmin, &bmax, &bavg);
                beam_raw_p2p_now[beam] = (uint16_t)(bmax - bmin);
                beam_post_p2p_now[beam] = compute_post_fft_gain_adc_p2p(local_beam_output[beam], beam_fft_gain[beam]);
                // Toggle GPIO 5 every iteration (no delay - instant pulse)
                gpio_put(5, 1);
                sleep_us(5);
                gpio_put(5, 0);
            }
            accumulate_beam_period_p2p_stats(beam_raw_p2p_now, beam_post_p2p_now);

#if RAW_ADC_DEBUG
            if (period_boundary && diag_status_enabled) {
                uint16_t ch_min[ADC_NUM_CHANNELS];
                uint16_t ch_max[ADC_NUM_CHANNELS];
                uint16_t ch_avg[ADC_NUM_CHANNELS];
                uint16_t ch_gain_min[ADC_NUM_CHANNELS];
                uint16_t ch_gain_max[ADC_NUM_CHANNELS];
                uint16_t ch_gain_avg[ADC_NUM_CHANNELS];
                uint16_t beam_min[NUM_BEAMS];
                uint16_t beam_max[NUM_BEAMS];
                uint16_t beam_avg[NUM_BEAMS];

                compute_adc_stats(ch0_data, CAPTURE_DEPTH, &ch_min[0], &ch_max[0], &ch_avg[0]);
                compute_adc_stats(ch1_data, CAPTURE_DEPTH, &ch_min[1], &ch_max[1], &ch_avg[1]);
                compute_adc_stats(ch2_data, CAPTURE_DEPTH, &ch_min[2], &ch_max[2], &ch_avg[2]);
                compute_adc_stats(ch0_gain_data, CAPTURE_DEPTH, &ch_gain_min[0], &ch_gain_max[0], &ch_gain_avg[0]);
                compute_adc_stats(ch1_gain_data, CAPTURE_DEPTH, &ch_gain_min[1], &ch_gain_max[1], &ch_gain_avg[1]);
                compute_adc_stats(ch2_gain_data, CAPTURE_DEPTH, &ch_gain_min[2], &ch_gain_max[2], &ch_gain_avg[2]);

                for (int beam = 0; beam < NUM_BEAMS; beam++) {
                    compute_adc_stats(local_beam_output[beam], CAPTURE_DEPTH, &beam_min[beam], &beam_max[beam], &beam_avg[beam]);
                }

                printf("\n=== DIAGNOSTIC REPORT ===\n");

                // 1) Status Section
                print_diag_status();

                // 2) Watchdog Section
                printf("WATCHDOG:\n");
                printf("  ADC: dmaBusy=%d blockDiff=%lu needed=%u cycle=%lu\n",
                       dma_channel_is_busy(g_dma_chan),
                       (unsigned long)diff,
                       (unsigned)BLOCK_SAMPLES,
                       (unsigned long)core0_cycle_count);
                printf("  DMA: rem=%lu rearm=%lu qC0toC1=%d qC1toC0=%d\n",
                       (unsigned long)remaining,
                       (unsigned long)dma_rearm_count,
                       queue_count,
                       fft_queue_count);

                // 3) Raw ADC Section
                if (diag_raw_enabled) {
                    printf("RAW ADC:\n");
                    printf("  AGC=%s clipWin=%u..%umV p2pClip=%umV avgTarget=%umV clipCntThr=%u\n",
                           agc_enabled ? "ON" : "OFF",
                           raw_clip_low_mv,
                           raw_clip_high_mv,
                           (uint16_t)(raw_clip_high_mv - raw_clip_low_mv),
                           raw_avg_target_mv,
                           agc_clip_min_samples);
                    if (diag_compact_enabled) {
                        printf("  CH-  ManG  AutoG AvgmV ErrmV Min/Max | RAW m/a/x (mV) p2p   | POST m/a/x (mV) p2p  | Clip\n");
                    } else {
                        printf("  CH  ManG(%%) AutoG(%%) AVG(mV) Err(mV) Min/Max(mV) RAW_P2P(mV) min/avg/max POST_P2P(mV) min/avg/max Clips\n");
                    }
                    for (int ch = 0; ch < ADC_NUM_CHANNELS; ch++) {
                        uint16_t raw_min = (p2p_period_samples > 0u) ? raw_p2p_min_period[ch] : p2p_raw_now[ch];
                        uint16_t raw_max = (p2p_period_samples > 0u) ? raw_p2p_max_period[ch] : p2p_raw_now[ch];
                        uint16_t raw_avg = (p2p_period_samples > 0u) ? (uint16_t)(raw_p2p_sum_period[ch] / p2p_period_samples) : p2p_raw_now[ch];
                        uint16_t post_min = (p2p_period_samples > 0u) ? post_p2p_min_period[ch] : p2p_post_now[ch];
                        uint16_t post_max = (p2p_period_samples > 0u) ? post_p2p_max_period[ch] : p2p_post_now[ch];
                        uint16_t post_avg = (p2p_period_samples > 0u) ? (uint16_t)(post_p2p_sum_period[ch] / p2p_period_samples) : p2p_post_now[ch];
                        uint16_t man_gain_pct = (uint16_t)((mic_gain_q8[ch] * 100u) / MIC_GAIN_Q8_UNITY);
                        uint16_t auto_gain_pct = 100;
                        float avg_mv = adc_counts_to_millivolts(ch_avg[ch]);
                        float min_mv = adc_counts_to_millivolts(ch_min[ch]);
                        float max_mv = adc_counts_to_millivolts(ch_max[ch]);
                        float avg_err_mv = avg_mv - (float)raw_avg_target_mv;

                        if (diag_compact_enabled) {
                            printf("  CH%-1d %4u%% %5u%% %6.1f %+5.1f %6.1f/%6.1f | %6.1f/%6.1f/%6.1f | %6.1f/%6.1f/%6.1f | %4u\n",
                                   ch,
                                   man_gain_pct,
                                   auto_gain_pct,
                                   avg_mv,
                                   avg_err_mv,
                                   min_mv,
                                   max_mv,
                                   adc_counts_to_millivolts(raw_min),
                                   adc_counts_to_millivolts(raw_avg),
                                   adc_counts_to_millivolts(raw_max),
                                   adc_counts_to_millivolts(post_min),
                                   adc_counts_to_millivolts(post_avg),
                                   adc_counts_to_millivolts(post_max),
                                   mic_gain_last_clip_count[ch]);
                        } else {
                            printf("  CH%-1d %8u %8u %8.1f %+7.1f %7.1f/%7.1f %7.1f/%7.1f/%7.1f  %7.1f/%7.1f/%7.1f %6u\n",
                                   ch,
                                   man_gain_pct,
                                   auto_gain_pct,
                                avg_mv,
                                avg_err_mv,
                                min_mv,
                                max_mv,
                                   adc_counts_to_millivolts(raw_min),
                                   adc_counts_to_millivolts(raw_avg),
                                   adc_counts_to_millivolts(raw_max),
                                   adc_counts_to_millivolts(post_min),
                                   adc_counts_to_millivolts(post_avg),
                                   adc_counts_to_millivolts(post_max),
                                   mic_gain_last_clip_count[ch]);
                        }
                    }
                } else {
                    printf("RAW ADC: OFF\n");
                }

                // 4) BeamFormed Section
                if (diag_beam_enabled) {
                    printf("BEAMFORMED:\n");
                    printf("  Global: AGC=%s skew=%s beamAvg=%ums\n",
                           agc_enabled ? "ON" : "OFF",
                           beam_skew_comp_enabled ? "ON" : "OFF",
                           bin_filter_time_ms_from_shift(bin_noise_filter_shift));
                    if (diag_compact_enabled) {
                        printf("  B   Ang  D0  D1  D2 Gain | RAW m/x/a (mV)      | POST m/x/a (mV)     | Clip bStr bAuto\n");
                    } else {
                        printf("  B  Angle  D0 D1 D2 Gain RAW_P2P(mV) min/max/avg POST_P2P(mV) min/max/avg Clip bStr bAuto\n");
                    }
                    for (int beam = 0; beam < NUM_BEAMS; beam++) {
                        uint16_t raw_min = (p2p_period_samples > 0u) ? beam_raw_p2p_min_period[beam] : beam_raw_p2p_now[beam];
                        uint16_t raw_max = (p2p_period_samples > 0u) ? beam_raw_p2p_max_period[beam] : beam_raw_p2p_now[beam];
                        uint16_t raw_avg = (p2p_period_samples > 0u) ? (uint16_t)(beam_raw_p2p_sum_period[beam] / p2p_period_samples) : beam_raw_p2p_now[beam];
                        uint16_t post_min = (p2p_period_samples > 0u) ? beam_post_p2p_min_period[beam] : beam_post_p2p_now[beam];
                        uint16_t post_max = (p2p_period_samples > 0u) ? beam_post_p2p_max_period[beam] : beam_post_p2p_now[beam];
                        uint16_t post_avg = (p2p_period_samples > 0u) ? (uint16_t)(beam_post_p2p_sum_period[beam] / p2p_period_samples) : beam_post_p2p_now[beam];

                        if (diag_compact_enabled) {
                            printf("  B%-1d %5.0f %3d %3d %3d %3ux | %7.1f/%7.1f/%7.1f | %7.1f/%7.1f/%7.1f | %4u %4u%% %5u%%\n",
                                   beam,
                                   beam_angles[beam],
                                   beam_delays[beam][0],
                                   beam_delays[beam][1],
                                   beam_delays[beam][2],
                                   beam_fft_gain[beam],
                                   adc_counts_to_millivolts(raw_min),
                                   adc_counts_to_millivolts(raw_max),
                                   adc_counts_to_millivolts(raw_avg),
                                   adc_counts_to_millivolts(post_min),
                                   adc_counts_to_millivolts(post_max),
                                   adc_counts_to_millivolts(post_avg),
                                   beam_last_clipped_samples[beam],
                                   beam_filter_strength_pct[beam],
                                   beam_filter_auto_strength_pct[beam]);
                        } else {
                            printf("  B%-1d %6.0f %3d %3d %3d %4ux %7.1f/%7.1f/%7.1f   %7.1f/%7.1f/%7.1f %4u %4u%% %5u%%\n",
                                   beam,
                                   beam_angles[beam],
                                   beam_delays[beam][0],
                                   beam_delays[beam][1],
                                   beam_delays[beam][2],
                                   beam_fft_gain[beam],
                                   adc_counts_to_millivolts(raw_min),
                                   adc_counts_to_millivolts(raw_max),
                                   adc_counts_to_millivolts(raw_avg),
                                   adc_counts_to_millivolts(post_min),
                                   adc_counts_to_millivolts(post_max),
                                   adc_counts_to_millivolts(post_avg),
                                   beam_last_clipped_samples[beam],
                                   beam_filter_strength_pct[beam],
                                   beam_filter_auto_strength_pct[beam]);
                        }
                    }
                } else {
                    printf("BEAMFORMED: OFF\n");
                }

                // 5) FFT Section (placeholder)
                if (diag_fft_enabled) {
                    printf("FFT:\n");
                    printf("  Raw=%s Format=%s\n", diag_fft_raw_enabled ? "ON" : "OFF", (diag_fft_raw_format == 0) ? "HEX" : "PCT");
                    bool any_beam = false;
                    for (int b = 0; b < NUM_BEAMS; b++) {
                        if (!diag_fft_beam_enabled[b]) continue;
                        any_beam = true;
                        if (!fft_last_valid[b]) {
                            printf("  Beam B%d: no data yet\n", b);
                        } else if (diag_fft_raw_enabled) {
                            print_fft_raw_table_for_beam(b);
                        } else {
                            printf("  Beam B%d: RAW OFF\n", b);
                        }
                    }
                    if (!any_beam) {
                        printf("  No FFT beams enabled (use: fftbeam B on)\n");
                    }
                } else {
                    printf("FFT: OFF\n");
                }

                // 6) I2C Output Section (post 16->8 conversion)
                bool any_i2c_beam = false;
                for (int b = 0; b < NUM_BEAMS; b++) {
                    if (diag_i2c_output_beam_enabled[b]) {
                        any_i2c_beam = true;
                        break;
                    }
                }

                if (any_i2c_beam) {
                    printf("I2C_OUTPUT:\n");
                    printf("  Format=%s\n", (diag_i2c_output_format == 0) ? "HEX" : "PCT");
                    for (int b = 0; b < NUM_BEAMS; b++) {
                        if (!diag_i2c_output_beam_enabled[b]) continue;
                        if (!i2c_last_valid[b]) {
                            printf("  I2C_Output_Beam %d: no data yet\n", b);
                            continue;
                        }
                        print_i2c_output_table_for_beam(b);
                    }
                } else {
                    printf("I2C_OUTPUT: OFF (all beams disabled; use i2coutput B on)\n");
                }
                fflush(stdout);
            }
#endif

            if (period_boundary) {
                reset_period_p2p_stats();
            }
            
            // Queue beamformed data for Core 1 to process
            queue_add_beamformed(local_beam_output);
            
            last_pos = current_pos;
        }

        // Defer flash writes so command handling remains responsive and writes are batched.
        if (settings_save_pending) {
            uint32_t now_ms = to_ms_since_boot(get_absolute_time());
            if ((now_ms - settings_last_change_ms) >= 1200u) {
                if (save_settings_to_flash()) {
                    settings_save_ok_count++;
                } else {
                    settings_save_fail_count++;
                    printf("Warning: failed to persist settings\n");
                    fflush(stdout);
                }
                settings_save_pending = false;
            }
        }
        
        // Core 0 now handles I2C transmission (parallelized with Core 1 FFT)
        // Check if Core 1 has any FFT results ready to transmit
        int beam_idx;
        uint16_t fft_bins[NUM_FREQ_BINS];
        if (fft_queue_get(&beam_idx, fft_bins)) {
            if (is_valid_beam_index(beam_idx)) {
                for (int i = 0; i < NUM_FREQ_BINS; i++) {
                    fft_last_beam_bins[beam_idx][i] = fft_bins[i];
                }
                fft_last_valid[beam_idx] = 1u;
            }
            // Transmit this beam's FFT result via I2C
            send_fft_via_i2c(beam_idx, fft_bins);

#if TONE_MONITOR_ENABLE
            if (diag_tone_enabled && beam_idx >= 0 && beam_idx < NUM_BEAMS) {
                int tone_band_idx = tone_monitor_bin_idx_runtime;
                tone_mag[beam_idx] = fft_bins[tone_band_idx];
                tone_seen_mask |= (uint8_t)(1u << beam_idx);

                if (tone_seen_mask == ((1u << NUM_BEAMS) - 1u)) {
                    tone_seen_mask = 0;
                    tone_set_count++;

                    if ((tone_set_count % TONE_MONITOR_INTERVAL_SETS) == 0) {
                        uint32_t b0_pct100 = ((uint32_t)tone_mag[0] * 10000u + 32767u) / 65535u;
                        uint32_t b1_pct100 = ((uint32_t)tone_mag[1] * 10000u + 32767u) / 65535u;
                        uint32_t b2_pct100 = ((uint32_t)tone_mag[2] * 10000u + 32767u) / 65535u;
                        uint32_t b3_pct100 = ((uint32_t)tone_mag[3] * 10000u + 32767u) / 65535u;
                        uint32_t b4_pct100 = ((uint32_t)tone_mag[4] * 10000u + 32767u) / 65535u;

                           printf("TONE %4dHz \x1b[7mBAND%02d\x1b[0m(~%.0fHz) | \x1b[7mB0\x1b[0m:%5u %3u.%02u%% \x1b[7mB1\x1b[0m:%5u %3u.%02u%% \x1b[7mB2\x1b[0m:%5u %3u.%02u%% \x1b[7mB3\x1b[0m:%5u %3u.%02u%% \x1b[7mB4\x1b[0m:%5u %3u.%02u%%\n",
                               (int)tone_monitor_hz_from_bin(tone_band_idx), tone_band_idx, tone_monitor_hz_from_bin(tone_band_idx),
                               tone_mag[0], (unsigned)(b0_pct100 / 100u), (unsigned)(b0_pct100 % 100u),
                               tone_mag[1], (unsigned)(b1_pct100 / 100u), (unsigned)(b1_pct100 % 100u),
                               tone_mag[2], (unsigned)(b2_pct100 / 100u), (unsigned)(b2_pct100 % 100u),
                               tone_mag[3], (unsigned)(b3_pct100 / 100u), (unsigned)(b3_pct100 % 100u),
                               tone_mag[4], (unsigned)(b4_pct100 / 100u), (unsigned)(b4_pct100 % 100u));
                        fflush(stdout);
                    }
                }
            }
#endif
        }
    }
}
