#include <Adafruit_INA260.h>

Adafruit_INA260 ina260 = Adafruit_INA260();

void setup() {
  Serial.begin(115200);
  // Wait until serial port is opened
  while (!Serial) { delay(10); }

  Serial.println("Adafruit INA260 Test");

  if (!ina260.begin()) {
    Serial.println("Couldn't find INA260 chip");
    while (1);
  }
  Serial.println("Found INA260 chip");
}
float peak = 0;
float busV = 0;
float busI = 0;
void loop() {
  busV = ina260.readBusVoltage();
  if(busI>peak){
    peak = busI;
  }
  busI = ina260.readCurrent();
  Serial.println(String(busV)+','+String(busI)+","+String(peak));
  

  //Serial.println();
  delay(100);
}
