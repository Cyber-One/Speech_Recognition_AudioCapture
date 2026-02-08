#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "hardware/gpio.h"

int main() {
    stdio_init_all();
    
    sleep_ms(5000);
    
    printf("\n=== GPIO Scan Test (pins 5-15) ===\n");
    fflush(stdout);
    
    // Test GPIO 5-15 to find which ones work
    for (int pin = 5; pin <= 15; pin++) {
        printf("\nTesting GPIO %d:\n", pin);
        fflush(stdout);
        
        printf("  Initializing...\n");
        fflush(stdout);
        gpio_init(pin);
        
        printf("  Setting output...\n");
        fflush(stdout);
        gpio_set_dir(pin, GPIO_OUT);
        
        printf("  Setting LOW...\n");
        fflush(stdout);
        gpio_put(pin, 0);
        sleep_ms(50);
        
        printf("  Pulsing HIGH...\n");
        fflush(stdout);
        gpio_put(pin, 1);
        sleep_ms(100);
        
        printf("  Setting LOW...\n");
        fflush(stdout);
        gpio_put(pin, 0);
        sleep_ms(50);
        
        printf("  GPIO %d OK\n", pin);
        fflush(stdout);
    }
    
    printf("\n=== Scan complete ===\n");
    fflush(stdout);
    
    uint32_t counter = 0;
    while (true) {
        counter++;
        printf("Counter: %u\n", counter);
        fflush(stdout);
        sleep_ms(1000);
    }
    
    return 0;
}
