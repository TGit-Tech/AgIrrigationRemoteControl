/************************************************************************//**
 *  @brief  Arduino Sketch to be loaded onto the Irrigation Pump Remote Hand-Held unit.
 *    see:
  note: 2x16 characters on display
  Arduino/Genuino Uno is a microcontroller board based on the ATmega328P
  https://www.arduino.cc/en/Main/ArduinoBoardUno
  1KB of EEPROM memory
 *  @code
 *    exmaple code
 *  @endcode
 *  @authors 
 *    tgit23        12/2016       Original
******************************************************************************/
#define DEBUG 1                 // 1 for DEBUG
#include <PeerIOSerialControl.h>

#define SS_TX_PIN 8             // XBee DIN 
#define SS_RX_PIN 9             // XBee DOUT

// Setup a Software Serial for XBEE (Allows Debug)
#include <SoftwareSerial.h>
SoftwareSerial IOSerial(SS_RX_PIN,SS_TX_PIN);
PeerIOSerialControl XBee(10,IOSerial,Serial);    // ArduinoID, IOSerial, DebugSerial

//vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
// Setup()
// See:     Q1-Backlit shorting issue; http://forum.arduino.cc/index.php?topic=96747.0
//-----------------------------------------------------------------------------------------
void setup(){
    pinMode(SS_RX_PIN, INPUT);
    pinMode(SS_TX_PIN, OUTPUT);
    IOSerial.begin(9600);
    Serial.begin(9600);         // Start UART Communications with the XBee->Module
}

//vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
// Loop()
//-----------------------------------------------------------------------------------------
void loop(){
  XBee.Available();
}
