/*
  Blink

  Turns an LED on for one second, then off for one second, repeatedly.

  Most Arduinos have an on-board LED you can control. On the UNO, MEGA and ZERO
  it is attached to digital pin 13, on MKR1000 on pin 6. LED_BUILTIN is set to
  the correct LED pin independent of which board is used.
  If you want to know what pin the on-board LED is connected to on your Arduino
  model, check the Technical Specs of your board at:
  https://www.arduino.cc/en/Main/Products

  modified 8 May 2014
  by Scott Fitzgerald
  modified 2 Sep 2016
  by Arturo Guadalupi
  modified 8 Sep 2016
  by Colby Newman

  This example code is in the public domain.

  https://www.arduino.cc/en/Tutorial/BuiltInExamples/Blink
*/


//#define rxPin PC1
//#define txPin PC0

#define Print_rxPin PB7
#define Print_txPin PB6
#define blinkypin PA9
#define APinInQuestion PB3
HardwareSerial soft_tx_rx = HardwareSerial(Print_rxPin, Print_txPin);


// the setup function runs once when you press reset or power the board
void setup() {



  soft_tx_rx.begin(115200);
  soft_tx_rx.print("Blinky Pin #: ");
  soft_tx_rx.println(blinkypin);

  soft_tx_rx.print("APinInQuestion #:");
  soft_tx_rx.println(APinInQuestion);
  
  
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(blinkypin, OUTPUT);
  
}
int counter = 0;
// the loop function runs over and over again forever
void loop() {
  counter ++;
  soft_tx_rx.print("hello world counter k:");
  soft_tx_rx.println(counter);

  digitalWrite(blinkypin, HIGH);  // turn the LED on (HIGH is the voltage level)
  delay(2000);                      // wait for a second
  digitalWrite(blinkypin, LOW);   // turn the LED off by making the voltage LOW
  delay(2000);                      // wait for a second
}
