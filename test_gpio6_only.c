#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "hardware/gpio.h"

int main() {
    stdio_init_all();
    
    // Wait for USB to enumerate
    sleep_ms(5000);
    
    printf("\n=== GPIO 6 Only Test ===\n");
    fflush(stdout);
    
    // Test only GPIO 6
    printf("Initializing GPIO 6...\n");
    fflush(stdout);
    gpio_init(6);
    printf("GPIO 6 init done\n");
    fflush(stdout);
    
    gpio_set_dir(6, GPIO_OUT);
    printf("GPIO 6 set to output\n");
    fflush(stdout);
    
    gpio_put(6, 0);
    printf("GPIO 6 set to 0\n");
    fflush(stdout);
    
    printf("Testing GPIO 6 pulses...\n");
    fflush(stdout);
    
    for (int i = 0; i < 5; i++) {
        printf("  Pulse %d - setting HIGH\n", i+1);
        fflush(stdout);
        gpio_put(6, 1);
        sleep_ms(100);
        
        printf("  Pulse %d - setting LOW\n", i+1);
        fflush(stdout);
        gpio_put(6, 0);
        sleep_ms(100);
    }
    
    printf("GPIO 6 test complete\n");
    fflush(stdout);
    
    // Loop
    uint32_t counter = 0;
    while (true) {
        counter++;
        printf("Counter: %u\n", counter);
        fflush(stdout);
        sleep_ms(1000);
    }
    
    return 0;
}
