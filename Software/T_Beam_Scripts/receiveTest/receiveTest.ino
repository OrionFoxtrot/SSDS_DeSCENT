
//Board names: 
//T1 V1: TTGO LoRa32 OLED
//

#include <RadioLib.h>
#include "boards.h"

 // conservative option

// Standard Options:
// float freq = 915;
// float bw = 125.0;
// int sf = 9;
// int cr = 7;
// int sw = RADIOLIB_SX126X_SYNC_WORD_PRIVATE;
// int pwr = 20;
// int pl = 8;
// int gn = 1;

// Josh Recomended:
float freq = 915;
float bw = 125;
int sf = 9;
int cr = 7;
int sw = RADIOLIB_SX126X_SYNC_WORD_PRIVATE;
int pwr = 20;
int pl = 8;
int gn = 1;

#define PACKET_LEN 10
#define SYNC_LEN 1
#define SYNC_INFO_LEN 5
#define NUM_SATS 3

const float br = sf*((bw*1000)/(1 << sf)); 
const float userBR = br*(4.0/cr);
const float estTime = (PACKET_LEN*8)/userBR;

uint8_t sync_packet[SYNC_INFO_LEN];
uint8_t ack_packet[SYNC_LEN];
int active[NUM_SATS] = { NULL };

SX1276 radio = new Module(RADIO_CS_PIN, RADIO_DI0_PIN, RADIO_RST_PIN, RADIO_BUSY_PIN);

// flag to indicate that a packet was received
volatile bool receivedFlag = false;

// disable interrupt when it's not needed
volatile bool enableInterrupt2 = true;

//volatile unsigned long uStartReceive;
volatile unsigned long lastPacketT = 0;

// called when received
void setFlag(void)
{
    // check if the interrupt is enabled
    if (!enableInterrupt2) {
        return;
    }
    //uStartReceive = micros();
    // we got a packet, set the flag
    receivedFlag = true;
}

inline void alertFailure(int state, String fail, const char* displayMsg) {
  if(state == RADIOLIB_ERR_NONE){
    Serial.println(F("success!"));
  } else {
    Serial.print(fail); Serial.print(" "); Serial.println(state);
    #ifdef HAS_DISPLAY
    if (u8g2) {
        if (state != RADIOLIB_ERR_NONE) {
            u8g2->clearBuffer();
            u8g2->drawStr(0, 12, displayMsg);
            u8g2->sendBuffer();
        }
    }
    #endif
    while(true);
  }
}

void initRadio(){
  // initialize SX1278 with default settings

    int state = radio.begin(freq,bw,sf,cr,sw,pwr,pl,gn);
    
    alertFailure(state, "failed, code ", "Initializing: FAIL");

    Serial.print(F("[RF96] Setting frequency ... "));
    state = radio.setFrequency(freq);
    alertFailure(state, "failed, code ", "Initializing: FAIL");

    Serial.print(F("[RF96] Setting SF ... "));
    state = radio.setSpreadingFactor(sf);
    alertFailure(state, "failed, code ", "Initializing: FAIL");

    // set CRC parameter to true so it matches the CRC parameter on the TinyGS side
    Serial.print(F("[RF96] Setting CRC parameter ... "));
    state = radio.setCRC(true); 
    alertFailure(state, "CRC initialization error", "Initializing: FAIL");
  
    /*
    // set forceLDRO parameter to true so it matches the forceLDRO parameter on the TinyGS side
    Serial.print(F("[RF96] Setting forceLDRO parameter ... "));
    state = radio.forceLDRO(true); // FLDRO = 1 on TinyGS side
    alertFailure(state, "forceLDRO initialization error", "Initializing: FAIL");
    */
}

void listenBegin() {
    int state = radio.startReceive();
    if (state == RADIOLIB_ERR_NONE) {
        //Serial.println(F("start receive success!"));
    } else {
        Serial.print(F("failed receive start, code "));
        Serial.println(state);
    }
}

void setup()
{
    initBoard();
    // When the power is turned on, a delay is required.
    delay(1500);
    initRadio();
    
    //radio.setDio0Action(setFlag, 1);
    radio.setPacketReceivedAction(setFlag);
    Serial.print("Est time: "); Serial.println(estTime);
    Serial.print("Time on air: "); Serial.println(radio.getTimeOnAir(PACKET_LEN)); 
    listenBegin();
     #ifdef HAS_DISPLAY
    if (u8g2) {
        u8g2->clearBuffer();
        u8g2->drawStr(0, 12, "Waiting to receive");
        u8g2->sendBuffer();
    }
    #endif
}

float fromBuffer(uint8_t *buffer, int start, int end){
  int tmp = 0;
  int i;
  //Serial.print("Section:");
  for(i = start; i <= end; i++){
    tmp |= buffer[i] << (8*(i-start));
    
    //Serial.print(buffer[i]); Serial.print(" ");
  }
  //Serial.print("\n");
  return *((float*)&tmp);
}

String gPSData = "No GPS";
String iMUData = "No IMU";
String bMEData = "No BME";
void loop()
{
     if (receivedFlag) {
        //Serial.println("Received data --------------------------------------------->");
        //Serial.print("Time since last: ");
        //Serial.println(millis() - lastPacketT);
        //lastPacketT = millis();

        enableInterrupt2 = false;
        receivedFlag = false;

        String str;
        int state = radio.readData(str);

        //uint8_t packet[PACKET_LEN];
        //int state = radio.readData(packet, PACKET_LEN);

        if (state == RADIOLIB_ERR_NONE) {
            Serial.println("----------Data Recieved----------");
            //Serial.println("Received: " + str); 

            gPSData = getValue(str,';',0);
            iMUData = getValue(str,';',1);
            bMEData = getValue(str,';',2);
            Serial.println("GPS Data: "+gPSData);
            Serial.println("IMU Data: "+iMUData);
            Serial.println("BME Data: "+bMEData);

            Serial.print("RSSI: "); Serial.print(radio.getRSSI()); Serial.println(F(" dBm")); 
            Serial.print("SNR: "); Serial.print(radio.getSNR()); Serial.println(F(" dB"));


    
        
        } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
            // packet was received, but is malformed
            Serial.println(F("[SX1278] CRC error!"));

        } else {
            // some other error occurred
            Serial.print(F("[SX1278] Failed, code "));
            Serial.println(state);
        }

        enableInterrupt2 = true;
        // put module back to listen mode
        listenBegin();

      }
}

String getValue(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length()-1;

  for(int i=0; i<=maxIndex && found<=index; i++){
    if(data.charAt(i)==separator || i==maxIndex){
        found++;
        strIndex[0] = strIndex[1]+1;
        strIndex[1] = (i == maxIndex) ? i+1 : i;
    }
  }

  return found>index ? data.substring(strIndex[0], strIndex[1]) : "";
}
