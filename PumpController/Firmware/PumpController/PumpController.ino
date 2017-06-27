/******************************************************************************************************************//**
 *  @brief  Arduino Sketch to be loaded onto the AgIrrigationRemoteControl Pump Controller device.
 *    see:  https://github.com/tgit23/AgIrrigationRemoteControl
 *  @code
 *    exmaple code
 *  @endcode
 *  @authors 
 *    tgit23        12/2016       Original
**********************************************************************************************************************/
#include <PeerIOSerialControl.h>        //See https://github.com/tgit23/PeerIOSerialControl
#include <SoftwareSerial.h>

#define TRANSCEIVER_ID 10               // Unique Transceiver ID for this Device (1-15)
#define XBEECONFIG 0                    // 1 to enter XBEE Configuration Mode, 0 Normal Operation
#define DEBUG 0                         // 1 for DEBUG

//---[ PIN SETTINGS ]--------------------------------------------------------------------------------------------------
#define SS_TX_PIN 2                     // XBee DIN
#define SS_RX_PIN 3                     // XBee DOUT
#define PUMP_POWER_PIN 7                // Pump Power Pin ( Blue twisted pair )
#define PUMP_AUX_CONTACT 0              // Pump Power Aux Contact ( Green twisted pair ) 0-to deactivate
#define ULTRASONIC_PRESENT 0            // 1=UltraSonic Level Monitor Attached, 0=No UltraSonic
#define ULTRASONIC_TRIG_PIN 4           // UltraSonic Trigger Pin
#define ULTRASONIC_ECHO_PIN 5           // UltraSonic Echo Pin
#define ULTRASONIC_MAX_DIST 400         // Longest Distance to Measure

// Setup a Software Serial for XBEE (Allows Debug)
SoftwareSerial IOSerial(SS_RX_PIN,SS_TX_PIN);
PeerIOSerialControl XBee(TRANSCEIVER_ID,IOSerial,Serial);    // ArduinoID, IOSerial, DebugSerial

#if ULTRASONIC_PRESENT>0
#include <NewPing.h>
NewPing sonar(US_TRIG_PIN, ULTRASONIC_ECHO_PIN, ULTRASONIC_MAX_DIST);
unsigned long ulLastPing = 0;
#endif

//vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
// Setup()
// See:     Q1-Backlit shorting issue; http://forum.arduino.cc/index.php?topic=96747.0
//-----------------------------------------------------------------------------------------
void setup(){
    pinMode(SS_RX_PIN, INPUT);
    pinMode(SS_TX_PIN, OUTPUT);
    pinMode(4,OUTPUT);
    pinMode(5,OUTPUT);
    pinMode(6,OUTPUT);
    pinMode(7,OUTPUT);
#if ULTRASONIC_PRESENT>0
    pinMode(US_TRIG_PIN, OUTPUT);
    pinMode(US_ECHO_PIN, INPUT);
#endif
#if PUMP_AUX_CONTACT>0
    pinMode(PUMP_AUX_CONTACT, INPUT_PULLUP);
#endif
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
#if XBEECONFIG>0
  if ( IOSerial.available()>0 ) Serial.write(IOSerial.read());
  if ( Serial.available()>0 ) IOSerial.write(Serial.read());
#else
  XBee.Available();
#endif

#if ULTRASONIC_PRESENT>0
  // Read UltraSonic water level
  int ulCurrentTime = millis();
  if ( ulCurrentTime > ulLastPing + 1000 ) {
    XBee.VirtualPin(64, sonar.ping_in() );
    ulLastPing = ulCurrentTime;
  }
#endif
  
}
