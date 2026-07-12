#include <Adafruit_INA260.h>

#define DEBUG false

#define relay1Switch 2
#define relay2Switch 3
#define relay3Switch 4

#define relay1Addr 0x40
#define relay2Addr 0x41
#define relay3Addr 0x45

Adafruit_INA260 relay1INA = Adafruit_INA260();
Adafruit_INA260 relay2INA = Adafruit_INA260();
Adafruit_INA260 relay3INA = Adafruit_INA260();

float ina1BusV = 0;
float ina2BusV = 0;
float ina3BusV = 0;
float ina1Curr = 0;
float ina2Curr = 0;
float ina3Curr = 0;

unsigned long lastINARead = 0;

const int RELAY_ON = HIGH;
const int RELAY_OFF = LOW;

bool relay1State = false;
bool relay2State = false;
bool relay3State = false;

char serialBuffer[32];
byte serialIndex = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  pinMode(relay1Switch, OUTPUT);
  pinMode(relay2Switch, OUTPUT);
  pinMode(relay3Switch, OUTPUT);

  applyRelayStates();

  if (!relay1INA.begin(relay1Addr)) {
    Serial.println("Couldn't find INA260 chip 1");
    while (1);
  }

  if (!relay2INA.begin(relay2Addr)) {
    Serial.println("Couldn't find INA260 chip 2");
    while (1);
  }

  if (!relay3INA.begin(relay3Addr)) {
    Serial.println("Couldn't find INA260 chip 3");
    while (1);
  }

  Serial.println("Enter 1, 2, 3, or combinations like 1,2 to toggle relays.");
  Serial.println("Enter 0 to turn all relays off.");
}

void loop() {
  handleSerialInput();

  if (millis() - lastINARead >= 250) {
    lastINARead = millis();

    readINAData(relay1INA, ina1BusV, ina1Curr);
    readINAData(relay2INA, ina2BusV, ina2Curr);
    readINAData(relay3INA, ina3BusV, ina3Curr);

    printINAData();
  }
}

void readINAData(Adafruit_INA260 &ina, float &busV, float &current) {
  busV = ina.readBusVoltage();
  current = ina.readCurrent();
}

void printINAData() {
  Serial.print(String(ina1BusV) + " , " + String(ina1Curr) + " , ");
  Serial.print(String(ina2BusV) + " , " + String(ina2Curr) + " , ");
  Serial.println(String(ina3BusV) + " , " + String(ina3Curr));
}

void applyRelayStates() {
  digitalWrite(relay1Switch, relay1State ? RELAY_ON : RELAY_OFF);
  digitalWrite(relay2Switch, relay2State ? RELAY_ON : RELAY_OFF);
  digitalWrite(relay3Switch, relay3State ? RELAY_ON : RELAY_OFF);
}

void handleSerialInput() {
  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      if (serialIndex > 0) {
        serialBuffer[serialIndex] = '\0';
        processRelayCommand(serialBuffer);
        serialIndex = 0;
      }
    } else {
      if (serialIndex < sizeof(serialBuffer) - 1) {
        serialBuffer[serialIndex++] = c;
      }
    }
  }
}

void processRelayCommand(char *cmd) {
  bool toggleRelay1 = false;
  bool toggleRelay2 = false;
  bool toggleRelay3 = false;
  bool turnAllOff = false;

  for (int i = 0; cmd[i] != '\0'; i++) {
    if (cmd[i] == '1') toggleRelay1 = true;
    if (cmd[i] == '2') toggleRelay2 = true;
    if (cmd[i] == '3') toggleRelay3 = true;
    if (cmd[i] == '0') turnAllOff = true;
  }

  if (turnAllOff) {
    relay1State = false;
    relay2State = false;
    relay3State = false;
  } else {
    if (toggleRelay1) relay1State = !relay1State;
    if (toggleRelay2) relay2State = !relay2State;
    if (toggleRelay3) relay3State = !relay3State;
  }

  applyRelayStates();

  if (DEBUG){
    Serial.print("Relays enabled: ");
    if (relay1State) Serial.print("1 ");
    if (relay2State) Serial.print("2 ");
    if (relay3State) Serial.print("3 ");
    if (!relay1State && !relay2State && !relay3State) Serial.print("none");
    Serial.println();
  }
}