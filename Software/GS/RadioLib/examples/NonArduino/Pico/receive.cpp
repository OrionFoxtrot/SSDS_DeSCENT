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

// Signal Parameters
float freq = 915;
float bw = 125.0;
int sf = 9;
int cr = 7;
int sw = RADIOLIB_SX126X_SYNC_WORD_PRIVATE;
int pwr = 10; //20 for flight; shouldn't matter on receive end
int pl = 8;
int gn = 1;
volatile bool receivedFlag = false;

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

// called when received
void setFlag(void)
{
    receivedFlag = true;
}


int main() {
  stdio_init_all();
  sleep_ms(100);
  int rc = pico_led_init();
  hard_assert(rc == PICO_OK);
  pico_set_led(true);
  
  // Initialize radio with parameters
  printf("[SX1276] Initializing ... ");
  int state = radio.begin(freq,bw,sf,cr,sw,pwr,pl,gn);
  if (state != RADIOLIB_ERR_NONE) {
    printf("initialization failed, code %d\n", state);
    pico_set_led(false);
    while(1){
      printf("initialization failed, code %d\n", state);
      sleep_ms(2000);
    }
    // return(1);
  }
  radio.setPacketReceivedAction(setFlag);

  printf("success!\n");

  // loop forever
  for(;;) {
    receivedFlag = false;
    printf("[SX1276] Starting receive ... ");
    int state = radio.startReceive();
    if (state == RADIOLIB_ERR_NONE) {;
        printf("success!\n");
    } else {
        printf("failed, code %d\n", state);
    }

    printf("Waiting for packet\n");
    while(!receivedFlag){
    }
    printf("done\n");

    uint8_t str[100] = {0};
    int state1 = radio.readData(str,100);
     if (state1 == RADIOLIB_ERR_NONE) {;
        printf("Output: %x\n", str);
        printf("Output: %s\n", str);
    } else {
        printf("transmit failed, code %d\n", state1);
    }

    sleep_ms(1000);
  }

  // return(0);
}
