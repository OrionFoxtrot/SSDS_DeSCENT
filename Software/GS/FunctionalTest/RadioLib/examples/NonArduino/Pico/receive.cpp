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

#include <pico/stdlib.h>
#include <stdio.h>
#include <format>
#include <vector>
#include <string>

#include "f_util.h"
#include "ff.h" 
#include "my_rtc.h"
#include "hw_config.h"

#include <RadioLib.h>
#include "hal/RPiPico/PicoHal.h"

// define pins to be used
#define SPI_PORT spi0
#define SPI_MISO 4
#define SPI_MOSI 3
#define SPI_SCK 2
// #define SPI_MISO 8
// #define SPI_MOSI 7
// #define SPI_SCK 6
// SD card pins in hw_config.c (uses spi1)

#define RFM_NSS 26
#define RFM_RST 22
#define RFM_DIO0 14
#define RFM_DIO1 15

// create a new instance of the HAL class
PicoHal* hal = new PicoHal(SPI_PORT, SPI_MISO, SPI_MOSI, SPI_SCK);
// now we can create the radio module
RFM95 radio = new Module(hal, RFM_NSS, RFM_DIO0, RFM_RST, RFM_DIO1);
// create new SD card object
FATFS fs;

// Signal Parameters
float freq = 915;
float bw = 125.0;
int sf = 9;
int cr = 7;
int sw = RADIOLIB_SX126X_SYNC_WORD_PRIVATE;
int pwr = 10; //20 for flight; shouldn't matter on receive end
int pl = 8;
int gn = 1;
int bufferlen = 100;
volatile bool receivedFlag = false;

// LED initialisation
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

// Write string to file in SD card
FRESULT write_data(char* str){
  // Mount SD Card
  printf("Mount SD Card\n");
  FRESULT fr = f_mount(&fs,"0:",1);
  if (FR_OK != fr){
    pico_set_led(false);
    printf("f_mount error: %s (%d)\n", FRESULT_str(fr),fr);
    return fr;
  }
  
  // Open file
  printf("Open File\n");
  FIL fil;
  const char* const filename = "filename.txt";
  fr = f_open(&fil, filename, FA_CREATE_ALWAYS | FA_WRITE);
  if (FR_OK != fr && FR_EXIST != fr){
    printf("f_open(%s) error: %s (%d)\n", filename, FRESULT_str(fr));
    return fr;
  }

  // Write String
  printf("Writing string: %s\n",str);
  if (f_printf(&fil, str) < 0) {
        printf("f_printf failed\n");
  }

  // Close file and unmount
  printf("Close file and unmount\n");
  fr = f_close(&fil);
  if (FR_OK != fr) {
    printf("f_close error: %s (%d)\n", FRESULT_str(fr), fr);
    return fr;
  }
  fr = f_unmount("0:");

  return fr;
}

// called when packet is received
void setFlag(void){
  receivedFlag = true;
}

int main() {
  // Initialize pico
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
  }
  radio.setPacketReceivedAction(setFlag);
  printf("[SX1276] init success!\n");

  // Initialize SD Card Writer
  if (!sd_init_driver()) {
    while (true){
      printf("SD init driver failed\n");
      sleep_ms(1000);
    }
  }

  // Track received packets
  int packetnum = 0;

  // loop forever
  for(;;) {
    // Start listening
    receivedFlag = false;
    printf("[SX1276] Starting receive ... ");
    int state = radio.startReceive();
    if (state == RADIOLIB_ERR_NONE) {;
        printf("success!\n");
    } else {
        printf("failed, code %d\n", state);
    }

    // Wait for packet
    printf("Waiting for packet ... ");
    while(!receivedFlag){
    }
    printf("done\n");
    packetnum = packetnum + 1;

    // Read packet data and write to SD card
    uint8_t str[bufferlen] = {0};
    int state1 = radio.readData(str,bufferlen);
     if (state1 == RADIOLIB_ERR_NONE) {;
        printf("Output: %x\n", str);
        printf("Output: %s\n", str);
    } else {
        printf("transmit failed, code %d\n", state1);
    }

    // Format packet as char array
    char buf[bufferlen];
    std::snprintf(buf, sizeof(buf), "Packet Num: %d, Contents: %s\n", packetnum, str);
    std::string str_formatted(buf);
    // Write to SD card
    FRESULT sd_status = write_data(buf);
    printf("SD card status: %s (%d)\n", FRESULT_str(sd_status), sd_status);

    sleep_ms(1000);
  }

  // return(0);
}
