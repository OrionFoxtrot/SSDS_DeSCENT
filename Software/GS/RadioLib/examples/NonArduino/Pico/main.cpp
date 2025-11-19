/*
  RadioLib Non-Arduino Raspberry Pi Pico library example

  Licensed under the MIT License

  Copyright (c) 2024 Cameron Goddard

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/

// define pins to be used
#define SPI_PORT spi0
#define SPI_MISO 4
#define SPI_MOSI 3
#define SPI_SCK 2
// #define SPI_MISO 8
// #define SPI_MOSI 7
// #define SPI_SCK 6

// #define RFM_NSS 26
#define RFM_NSS 26
#define RFM_RST 22
#define RFM_DIO0 14
#define RFM_DIO1 15

#include <pico/stdlib.h>
#include <stdio.h>

// include the library
#include <RadioLib.h>

// include the hardware abstraction layer
#include "hal/RPiPico/PicoHal.h"

// create a new instance of the HAL class
PicoHal* hal = new PicoHal(SPI_PORT, SPI_MISO, SPI_MOSI, SPI_SCK);

// now we can create the radio module
RFM95 radio = new Module(hal, RFM_NSS, RFM_DIO0, RFM_RST, RFM_DIO1);

// ================================== blink led =================================
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
// ==============================================================================

int main() {
  stdio_init_all();
  sleep_ms(100);
  int rc = pico_led_init();
  hard_assert(rc == PICO_OK);
  pico_set_led(true);
  // initialize just like with Arduino
  printf("[SX1276] Initializing ... ");
  int state = radio.begin(900);

  if (state != RADIOLIB_ERR_NONE) {
    printf("initialization failed, code %d\n", state);
    // pico_set_led(false);
    while(1){
      printf("initialization failed, code %d\n", state);
      sleep_ms(2000);
    }

    // return(1);
  }

  // // retry until successful
  // while (state != RADIOLIB_ERR_NONE) {
  //   printf("initialization failed, code %d\n", state);
  //   pico_set_led(false);
  //   sleep_ms(2000);
  //   state = radio.begin();
  // }

  // pico_set_led(false);
  printf("success!\n");

  // loop forever
  for(;;) {
    // send a packet
    printf("[SX1276] Transmitting packet ... ");
    state = radio.transmit("Hello World!");
    if(state == RADIOLIB_ERR_NONE) {
      // the packet was successfully transmitted
      printf("success!\n");
      // pico_set_led(true);

      // wait for a second before transmitting again
      hal->delay(1000);
      // pico_set_led(false);

    } else {
      printf("transmit failed, code %d\n", state);

    }
    sleep_ms(1000);

  }

  // return(0);
}
