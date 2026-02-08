#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "hardware/gpio.h"

// Debug GPIO pins
#define DEBUG_FFT_COMPLETE_PIN 10
#define DEBUG_ALL_BEAMS_COMPLETE_PIN 11

int main() {
    stdio_init_all();
    
    // Wait for USB to enumerate
    sleep_ms(500);
    
    printf("\n=== GPIO Test Started ===\n");
    fflush(stdout);
    
    // Initialize GPIO 5
    printf("Initializing GPIO 5...\n");
    fflush(stdout);
    gpio_init(DEBUG_FFT_COMPLETE_PIN);
    gpio_set_dir(DEBUG_FFT_COMPLETE_PIN, GPIO_OUT);
    gpio_put(DEBUG_FFT_COMPLETE_PIN, 0);
    printf("GPIO 5 initialized\n");
    fflush(stdout);
    
    // Initialize GPIO 6
    printf("Initializing GPIO 6...\n");
    fflush(stdout);
    gpio_init(DEBUG_ALL_BEAMS_COMPLETE_PIN);
    gpio_set_dir(DEBUG_ALL_BEAMS_COMPLETE_PIN, GPIO_OUT);
    gpio_put(DEBUG_ALL_BEAMS_COMPLETE_PIN, 0);
    printf("GPIO 6 initialized\n");
    fflush(stdout);
    
    // Test GPIO 5
    printf("Testing GPIO 5...\n");
    fflush(stdout);
    for (int i = 0; i < 3; i++) {
        printf("  GPIO 5 pulse %d\n", i+1);
        fflush(stdout);
        gpio_put(DEBUG_FFT_COMPLETE_PIN, 1);
        sleep_us(10);
        gpio_put(DEBUG_FFT_COMPLETE_PIN, 0);
        sleep_us(10);
    }
    printf("GPIO 5 test complete\n");
    fflush(stdout);
    
    // Test GPIO 6
    printf("Testing GPIO 6...\n");
    fflush(stdout);
    for (int i = 0; i < 3; i++) {
        printf("  GPIO 6 pulse %d\n", i+1);
        fflush(stdout);
        gpio_put(DEBUG_ALL_BEAMS_COMPLETE_PIN, 1);
        sleep_us(10);
        gpio_put(DEBUG_ALL_BEAMS_COMPLETE_PIN, 0);
        sleep_us(10);
    }
    printf("GPIO 6 test complete\n");
    fflush(stdout);
    
    // Loop printing counter
    uint32_t counter = 0;
    while (true) {
        counter++;
        printf("Counter: %u\n", counter);
        fflush(stdout);
        sleep_ms(1000);
    }
    
    return 0;
}
