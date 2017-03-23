/************************************************************************//**
 *  @brief  Arduino Sketch to be loaded onto the Irrigation Pump Controller.
 *    see:
 *  @code
 *    exmaple code
 *  @endcode
 *  @authors 
 *    tgit23        12/2016       Original
******************************************************************************/
#define DEBUG 1                 // 1 for DEBUG
#define SS_TX_PIN 2             // XBee DIN
#define SS_RX_PIN 3             // XBee DOUT
#define US_PRESENT 1            // 1=UltraSonic Level Monitor Attached, 0=No UltraSonic
#define US_TRIG_PIN 6			      // UltraSonic Trigger Pin
#define US_ECHO_PIN	5			      // UltraSonic Echo Pin
#define US_MAX_DIST 400         // Longest Distance to Measure

// Setup a Software Serial for XBEE (Allows Debug)
#include <PeerIOSerialControl.h>
#include <SoftwareSerial.h>
SoftwareSerial IOSerial(SS_RX_PIN,SS_TX_PIN);
PeerIOSerialControl XBee(10,IOSerial,Serial);    // ArduinoID, IOSerial, DebugSerial

#if US_PRESENT>0
#include <NewPing.h>
NewPing sonar(US_TRIG_PIN, US_ECHO_PIN, US_MAX_DIST);
unsigned long ulLastPing = 0;
#endif

//vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
// Setup()
// See:     Q1-Backlit shorting issue; http://forum.arduino.cc/index.php?topic=96747.0
//-----------------------------------------------------------------------------------------
void setup(){
    pinMode(SS_RX_PIN, INPUT);
    pinMode(SS_TX_PIN, OUTPUT);
    pinMode(4, OUTPUT);
    pinMode(5, INPUT);
    pinMode(6, OUTPUT);
    pinMode(7, OUTPUT);
	  pinMode(A0, INPUT);
    pinMode(A1, INPUT);
    pinMode(A2, INPUT);
    pinMode(A3, INPUT);
    pinMode(A4, INPUT);
    pinMode(A5, INPUT);
    IOSerial.begin(9600);
    Serial.begin(9600);
}

//vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
// Loop()
//-----------------------------------------------------------------------------------------
void loop(){
  XBee.Available();

#if US_PRESENT>0
  // Read UltraSonic water level
  int ulCurrentTime = millis();
  if ( ulCurrentTime > ulLastPing + 1000 ) {
    XBee.VirtualPin(64, sonar.ping_in() );
    ulLastPing = ulCurrentTime;
  }
#endif
  
}
