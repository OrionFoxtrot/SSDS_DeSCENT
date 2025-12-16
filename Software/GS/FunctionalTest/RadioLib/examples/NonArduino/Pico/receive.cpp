// Default Pico + C libraries
#include <pico/stdlib.h>
#include <stdio.h>
#include <format>
#include <vector>
#include <string>
#include "hardware/clocks.h"

// SD card libraries
#include "f_util.h"
#include "ff.h" 
#include "my_rtc.h"
#include "hw_config.h"

// LoRa libraries
#include <RadioLib.h>
#include "hal/RPiPico/PicoHal.h"

// Define LoRa Module Pins (see hw_config.c for SD card pins)
#define SPI_PORT spi0
#define SPI_MISO 4
#define SPI_MOSI 3
#define SPI_SCK 2
#define RFM_NSS 26 //CS

#define RFM_RST 22
#define RFM_DIO0 14 //G0
#define RFM_DIO1 15 //G1

// Create a new instance of the HAL class
PicoHal* hal = new PicoHal(SPI_PORT, SPI_MISO, SPI_MOSI, SPI_SCK);
// Create radio module using hal
RFM95 radio = new Module(hal, RFM_NSS, RFM_DIO0, RFM_RST, RFM_DIO1);
// Create new SD card object
FATFS fs;

// Signal Parameters
float freq = 915; 
float bw = 125; // Bandwidth
int sf = 9; // Spreading factor
int cr = 7; // Coding rate
int sw = RADIOLIB_SX126X_SYNC_WORD_PRIVATE;
int pwr = 10; // Doesn't matter on receive end
int pl = 8; // Preamble length
int gn = 0; // Gain, doesn't matter in receive end
int bufferlen = 100; // Buffer size needs to be greater than packet size
int packetlen;
volatile bool receivedFlag = false;

// LED initialisation, for built in Pico LED
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
  // Open file
  printf("Open File\n");
  FIL fil;
  const char* const filename = "logdata.txt";
  FRESULT fr = f_open(&fil, filename, FA_OPEN_ALWAYS | FA_OPEN_APPEND | FA_WRITE);
  // FA_OPEN_ALWAYS - creates file if it doesn't exist
  // FA_OPEN_APPEND - read/write set to end of file, prevents overwriting existing contents
  if (FR_OK != fr && FR_EXIST != fr){
    printf("f_open(%s) error: %s (%d)\n", filename, FRESULT_str(fr));
    return fr;
  }

  // Write string
  printf("Writing string: %s\n",str);
  if (f_printf(&fil, str) < 0) {
        printf("f_printf failed\n");
  }

  // Close file
  printf("Close file\n");
  fr = f_close(&fil);
  if (FR_OK != fr) {
    printf("f_close error: %s (%d)\n", FRESULT_str(fr), fr);
    return fr;
  }

  return fr;
}

// LoRa packet received callback function
void setFlag(void){
  receivedFlag = true;
}

int main() {
  // Initialize pico
  stdio_init_all();
  sleep_ms(100);

  // Initialize LED
  int rc = pico_led_init();
  hard_assert(rc == PICO_OK);
  pico_set_led(true);
  
  // Initialize radio with parameters set above
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
  // Set LoRa receive callback function
  radio.setPacketReceivedAction(setFlag);
  printf("[SX1276] init success!\n");

  // Initialize SD Card Writer
  if (!sd_init_driver()) {
    while (true){
      printf("SD init driver failed\n");
      sleep_ms(1000);
    }
  }

  // Mount SD Card
  printf("Mount SD Card\n");
  FRESULT fr = f_mount(&fs,"0:",1);
  if (FR_OK != fr){
    pico_set_led(false);
    printf("f_mount error: %s (%d)\n", FRESULT_str(fr),fr);
    return fr;
  }

  // Write header to SD card
  printf("Writing header to SD card\n");
  char header[] = "PLACEHOLDER HEADER\n";
  // char header[] = "PacketNum,ChipID,GPSlat,GPSlong,GPSalt,IMUgyroX,IMUgyroY,IMUgyroZ,IMUaccelX,IMUaccelY,IMUaccelZ,IMUmagX,IMUmagY,IMUmagZ,Temp,Humidity,Pressure\n";
  FRESULT sd_status = write_data(header);
  printf("SD card status: %s (%d)\n", FRESULT_str(sd_status), sd_status);

  // Count received packets
  int packetnum = 0;

  // Loop forever
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

    // Wait for packet to arrive
    printf("Waiting for packet ... ");
    while(!receivedFlag){
      sleep_ms(1);
    }
    printf("done\n");
    // Count received packets
    packetnum = packetnum + 1;

    // Print packet stats
    printf("Packet Len: %d\n", radio.getPacketLength());
    printf("SNR: %d\n", radio.getSNR());
    printf("RSSI: %d\n", radio.getRSSI());

    // Read packet data into buffer
    uint8_t str[bufferlen] = {0};
    int state1 = radio.readData(str,bufferlen);
    str[radio.getPacketLength()] = 0;
    if (state1 == RADIOLIB_ERR_NONE) {
      printf("Output: %x\n", str);
      printf("Output: %s\n", str);
    } else {
      printf("received failed, code %d\n", state1);
    }

    // Parse packet data (as CSV), format as char array
    char buf[bufferlen];
    std::snprintf(buf, sizeof(buf), "%d,%s\n", packetnum, str);
    std::string str_formatted(buf);
    // TODO: implement bit parsing, concatenate as comma separated values

    // Write to SD card
    sd_status = write_data(buf);
    printf("SD card status: %s (%d)\n", FRESULT_str(sd_status), sd_status);

    sleep_ms(1000);
  }
}
