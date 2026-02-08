#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"

// Minimal USB CDC test - no ADC, no FFT, no multicore
int main() {
    stdio_init_all();
    
    // Wait for USB to enumerate
    sleep_ms(500);
    
    printf("\n=== USB CDC Test Started ===\n");
    fflush(stdout);
    
    uint32_t counter = 0;
    while (true) {
        counter++;
        printf("Counter: %u\n", counter);
        fflush(stdout);
        sleep_ms(1000);  // Print every second
    }
    
    return 0;
}
