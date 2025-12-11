#include <stdio.h>
// #include "C:\Users\16073\OneDrive\Documents\Pico-v1.5.0\FatFs_SPI\include\f_util.h"
// #include "C:\Program Files\Raspberry Pi\Pico SDK v1.5.0\pico-sdk\lib\tinyusb\lib\fatfs\source\ff.h"
// #include "C:\Users\16073\OneDrive\Documents\Pico-v1.5.0\FatFs_SPI\include\rtc.h"
// #include "C:\Users\16073\OneDrive\Documents\Pico-v1.5.0\FatFs_SPI\sd_driver\hw_config.h"
#include "stdlib.h"
#include "f_util.h"
#include "ff.h" 
#include "my_rtc.h"
#include "hw_config.h"

// Pico W devices use a GPIO on the WIFI chip for the LED,
// so when building for Pico W, CYW43_WL_GPIO_LED_PIN will be defined
#ifdef CYW43_WL_GPIO_LED_PIN
#include "pico/cyw43_arch.h"
#endif

#ifndef LED_DELAY_MS
#define LED_DELAY_MS 250
#endif

// Perform initialisation
int pico_led_init(void) {
#if defined(PICO_DEFAULT_LED_PIN)
    // A device like Pico that uses a GPIO for the LED will define PICO_DEFAULT_LED_PIN
    // so we can use normal GPIO functionality to turn the led on and off
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    return PICO_OK;
#elif defined(CYW43_WL_GPIO_LED_PIN)
    // For Pico W devices we need to initialise the driver etc
    return cyw43_arch_init();
#endif
}

// Turn the led on or off
void pico_set_led(bool led_on) {
#if defined(PICO_DEFAULT_LED_PIN)
    // Just set the GPIO on or off
    gpio_put(PICO_DEFAULT_LED_PIN, led_on);
#elif defined(CYW43_WL_GPIO_LED_PIN)
    // Ask the wifi "driver" to set the GPIO on or off
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_on);
#endif
}

int main() {
    stdio_init_all();
    sleep_ms(100);
    // time_init();

    int rc = pico_led_init();
    // hard_assert(rc == PICO_OK);
    pico_set_led(true);
    puts("Hello, world!");

    if (!sd_init_driver()) {
        while (true){
            printf("SD init driver failed\n");
            sleep_ms(1000);
        }
    }

    // See FatFs - Generic FAT Filesystem Module, "Application Interface",
    // http://elm-chan.org/fsw/ff/00index_e.html
    // sd_card_t *pSD = sd_get_by_num(0);
    // FRESULT fr = f_mount(&pSD->fatfs, pSD->pcName, 1);
    // FRESULT fr = f_mount(&pSD->state.fatfs, pSD->state.drive_prefix, 1);

    FATFS fs;
    FRESULT fr = f_mount(&fs,"0:",1);

    // if (FR_OK != fr) panic("f_mount error: %s (%d)\n", FRESULT_str(fr), fr);
    if (FR_OK != fr){
        while(true){
            // pico_set_led(false);
            printf("f_mount error: %s (%d)\n", FRESULT_str(fr),fr);
            sleep_ms(2000);
        }
    }

    FIL fil;
    const char* const filename = "filename.txt";
    fr = f_open(&fil, filename, FA_OPEN_APPEND | FA_WRITE);
    if (FR_OK != fr && FR_EXIST != fr){
        // panic("f_open(%s) error: %s (%d)\n", filename, FRESULT_str(fr), fr);
        while(true){
            printf("f_open(%s) error: %s (%d)\n", filename, FRESULT_str(fr));
            sleep_ms(2000);
        }
    }
        
    if (f_printf(&fil, "Hello, world!\n") < 0) {
        printf("f_printf failed\n");
    }
    fr = f_close(&fil);
    if (FR_OK != fr) {
        printf("f_close error: %s (%d)\n", FRESULT_str(fr), fr);
    }
    // f_unmount(pSD->state.drive_prefix);
    f_unmount("0:");

    puts("Goodbye, world!");
    while(true){
        printf("done");
        sleep_ms(2000);
    }
}
