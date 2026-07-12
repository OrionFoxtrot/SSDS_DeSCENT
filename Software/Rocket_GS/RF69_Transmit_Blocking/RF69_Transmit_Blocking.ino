 /*
  RadioLib RF69 Blocking Transmit Example

  This example transmits packets using RF69 FSK radio module.
  Each packet contains up to 64 bytes of data, in the form of:
  - Arduino String
  - null-terminated char array (C-string)
  - arbitrary binary data (byte array)

  Using blocking transmit is not recommended, as it will lead
  to inefficient use of processor time!
  Instead, interrupt transmit is recommended.

  For default module settings, see the wiki page
  https://github.com/jgromes/RadioLib/wiki/Default-configuration#rf69sx1231

  For full API reference, see the GitHub Pages
  https://jgromes.github.io/RadioLib/
*/

// include the library
#include <RadioLib.h>

// RF69 has the following connections:
// CS pin:    7 => 10 (PCB)
// DIO0 pin:  2
// RESET pin: 3 => 4 (PCB)
#define RFM_CS 10
#define RFM_DIO0 2
#define RFM_RST 4
RF69 radio = new Module(RFM_CS, RFM_DIO0, RFM_RST);

#define blinkypin 9
void manualRFM69Reset() {
  pinMode(RFM_RST, OUTPUT);

  digitalWrite(RFM_RST, LOW);
  delay(10);

  digitalWrite(RFM_RST, HIGH);   // active high reset
  delay(10);

  digitalWrite(RFM_RST, LOW);    // release reset
  delay(100);
}
void setup() {
  Serial.begin(115200);

  pinMode(blinkypin, OUTPUT);
  digitalWrite(blinkypin, LOW);    // keep translator disabled first

  pinMode(RFM_CS, OUTPUT);
  digitalWrite(RFM_CS, HIGH);   // deselect radio

  pinMode(RFM_RST, OUTPUT);
  digitalWrite(RFM_RST, LOW);   // do not hold radio in reset

  delay(500);                   // let rails settle

  digitalWrite(blinkypin, HIGH);   // enable translator only after pins are sane
  delay(500);

  manualRFM69Reset();
  // initialize RF69 with default settings
  Serial.println(F("[RF69] Initializing ... "));

  
  int state = radio.begin(915.0);
  while(state != RADIOLIB_ERR_NONE){
    Serial.print(F("failed, code "));
    Serial.println(state);
    Serial.println("Trying to initialize again");
    Serial.println("Hint, if you're connected to the CityLabs Programmer, press the power rest button");

    manualRFM69Reset();
    digitalWrite(blinkypin, LOW);
    delay(100);
    digitalWrite(blinkypin, HIGH);

    state = radio.begin(915.0);
  }


  Serial.print(F("[RF69] Setting high power module ... "));
  state = radio.setOutputPower(20, true);
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true) { delay(10); }
  }

  // delay(9999999999999999999);
}

// counter to keep track of transmitted packets
int count = 0;

void loop() {
  Serial.print(F("[RF69] Transmitting packet ... "));

  // you can transmit C-string or Arduino string up to 64 characters long
  String str = "Hello World! #" + String(count++);
  int state = radio.transmit(str);


  if (state == RADIOLIB_ERR_NONE) {
    // the packet was successfully transmitted
    Serial.println(F("success!"));

  } else if (state == RADIOLIB_ERR_PACKET_TOO_LONG) {
    // the supplied packet was longer than 64 bytes
    Serial.println(F("too long!"));

  } else {
    // some other error occurred
    Serial.print(F("failed, code "));
    Serial.println(state);
  }
  delay(1000);
}
