#include <stdio.h>
#include <math.h>
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

// Sample rate configuration
#define TARGET_SAMPLE_RATE 16000  // 16 kHz for speech recognition
#define CAPTURE_DEPTH 256         // Samples per channel per DMA transfer (per mic)
// DMA ring must be power-of-two sized; choose 32KB ring (16K uint16_t samples)
#define DMA_RING_BITS 15
#define BUFFER_SIZE ((1 << DMA_RING_BITS) / sizeof(uint16_t))  // 16,384 halfwords
#define BLOCK_SAMPLES (CAPTURE_DEPTH * ADC_NUM_CHANNELS)       // 12,288 halfwords needed per processing block

// Beamforming configuration
#define SPEED_OF_SOUND 343.0f     // m/s at 20°C
#define MIC_SPACING 0.05f         // 50mm spacing between microphones
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

// Initialize beamforming delay indices for all angles
static void init_beam_delays(void) {
    for (int beam = 0; beam < NUM_BEAMS; beam++) {
        float angle = beam_angles[beam];
        
        for (int mic = 0; mic < ADC_NUM_CHANNELS; mic++) {
            float delay_sec = calculate_delay_seconds(angle, mic);
            // Convert to sample delay (negative for delay)
            int delay_samples = -(int)(delay_sec * TARGET_SAMPLE_RATE + 0.5f);
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
    // Prepare input: convert to fixed-point Q15
    for (int i = 0; i < FFT_SIZE; i++) {
        // Beamformed output is uint16_t (0-4095 range from 12-bit ADC)
        // Convert to signed Q15: center around 0 and scale up
        // Range: 0-4095 → -2048 to +2047 → -32768 to +32752 in Q15
        int32_t centered = (int32_t)input[i] - 2048;
        int16_t adc_q15 = (int16_t)(centered * 16);  // Scale by ~16 to use full Q15 range
        fft_real[i] = adc_q15;
        fft_imag[i] = 0;
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

// Format I2C packet: header + 40 frequency bins (1 byte each)
static void format_i2c_packet(uint16_t *freq_bins) {
    i2c_packet[0] = I2C_PACKET_HEADER;
    for (int i = 0; i < NUM_FREQ_BINS; i++) {
        // Convert 16-bit magnitude to 8-bit by right shifting
        uint16_t val = freq_bins[i];
        uint8_t out = (uint8_t)(val >> 8);
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
        // Debug: print I2C error to serial
        printf("I2C Beam %d (addr 0x%02x) failed: result=%d (expected %d)\n", beam_idx, slave_addr, result, I2C_PACKET_SIZE);
        fflush(stdout);
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
            
            // Clamp to valid range
            if (idx >= 0 && idx < num_samples) {
                sum += (int32_t)channels[mic][idx];
            }
        }
        
        // Average across 3 channels and store
        output[i] = (uint16_t)((sum / 3) & 0xFFF);
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
    
    // Simple busy-wait loop - check DMA position continuously
    uint32_t log_counter = 0;
    while (true) {
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
            
            // Extract channels
            extract_channel(0, adc_buffer, CAPTURE_DEPTH, ch0_data);
            extract_channel(1, adc_buffer, CAPTURE_DEPTH, ch1_data);
            extract_channel(2, adc_buffer, CAPTURE_DEPTH, ch2_data);
            
            // Beamform all 5 beams
            for (int beam = 0; beam < NUM_BEAMS; beam++) {
                beamform_delay_sum(beam, ch0_data, ch1_data, ch2_data, CAPTURE_DEPTH, local_beam_output[beam]);
                // Toggle GPIO 5 every iteration (no delay - instant pulse)
                gpio_put(5, 1);
                sleep_us(5);
                gpio_put(5, 0);
            }
            
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
        }
    }
}
