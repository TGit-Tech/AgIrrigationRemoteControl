/******************************************************************************************************************//**
 * @file AgFirmware.ino
 * @brief Arduino Sketch Firmware for the Devices of the AgIrrigationRemoteControl project
 * @see https://github.com/tgit23/AgIrrigationRemoteControl
 * @authors 
 *    tgit23        8/2017       Original
 **********************************************************************************************************************/
#include "Device.h"

#define RELEASE                   20170706        // Release Version used to Build the Units
#define XBEECONFIG                0               // Set to 'true' to configure Xbee with XCTU
//#define DUMB_CONTROLLER                         // Dumb-Controllers only Receive and Send raw pin data
//#define DUMB_DEVICEID             6


/******************************************************************************************************************//**
 * [[[ DEVICES ]]] - Each defined by:   Device <IdName>(<DeviceName>, <DeviceID#(1-15)> );
**********************************************************************************************************************/
Device HandRemote(        "Remote",   1     );
Device Pump(              "Pump",     6     );
//Device HeadGate(          "HeadGate", 7     );
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
#ifndef DUMB_CONTROLLER 
void setup() {


//---------------------------------------------------------------------------------------------------------------------
ThisDevice::DeviceID = HandRemote.DeviceID;             // Which device is the Firmware FOR?
//---------------------------------------------------------------------------------------------------------------------
HandRemote.LCD(8,9,4,5,6,7,XBEECONFIG);                               // HandRemote uses pins (4-9) for LCD
Pump.LCD(12,13,8,9,10,11,XBEECONFIG);                                 // Pump-Controller uses pins (8-13) for LCD
Pump.Communications(3,2,3000,XBEECONFIG);                             // Pump-Controller Rx = 3, Tx = 2, UpdateInterval
#if RELEASE<20170525                                                  // OLD RELEASE HandRemote ---------------
HandRemote.Communications(12,11,3000,XBEECONFIG);                     //  Rx = 12, Tx = 11, UpdateInterval
HandRemote.Pin(2)->Mode(  OUTPUT_BUZZER,    "Buzz"          );        //  Buzzer Signal on Pin 2
#define BUZZ 2                                  
#else                                                                 // ELSE Newer RELEASES HandRemote -------
HandRemote.Communications(3,2,3000,XBEECONFIG);                       //  Rx = 3, Tx = 2, UpdateInterval
HandRemote.Pin(11)->Mode( OUTPUT_BUZZER,    "Buzz"          );        //  Buzzer Signal on Pin 11
#define BUZZ 11
#endif


/******************************************************************************************************************//**
 * [[[ DEVICE.PIN() DEFINITIONS ]]]
 *      Pin MODES -> INPUT, OUTPUT, INPUT_PULLUP, INPUT_SONIC, OUTPUT_BUZZER, OUTPUT_PWM, CONTROL_PIN
 *********************************************************************************************************************/
HandRemote.Pin(A1)->Mode(   INPUT,            "Battery(B)"      );
      Pump.Pin(7 )->Mode(   OUTPUT,           "Power(P)"        );
      Pump.Pin(64)->Mode(   INPUT_SONIC,      "Water(W)", 4, 5  );   // Trigger Pin = 4, Echo Pin = 5
      Pump.Pin(65)->Mode(   CONTROLLER,       "[WG]Pid"         );
//HeadGate.Pin(6)->Mode(  OUTPUT_PWM,       "HeadGate"        );
//Pivot.Pin(POWER)->Mode(           INPUT_PULLUP,     "OnOff"         );
//ChemicalPump.Pin(POWER)->Mode(    OUTPUT,           "Power"         );


/******************************************************************************************************************//**
 * [[[ CONTROLLERS ]]]
 *  - Settable()
 *  - TieToPin(PinPoint *_OutPin)
 *  - ControlSet(PinPoint *_OutPin)
 *  - PIDSetpoint(PinPoint *_OutPin, char *_ID, double Kp, double Ki, double Kd, int POn, int PDir, PinPoint *_ControlPin = NULL )
 *  - LessThanSetpoint(PinPoint *_OutPin, char *_ID, PinPoint *_ControlPin = NULL )
 *  - GreaterThanSetpoint(PinPoint *_OutPin, char *_ID, PinPoint *_ControlPin = NULL )
 *  - EqualToSetpoint(PinPoint *_OutPin, char *_ID, PinPoint *_ControlPin = NULL )
 *  - NotEqualToSetpoint(PinPoint *_OutPin, char *_ID, PinPoint *_ControlPin = NULL )
**********************************************************************************************************************/
//--DEVICE------------|--INPUT -----------,[ID]-)->CONTROLLER---------|----OUTPUT ---------(etc...)--[ControlPin] --
HandRemote.Control(   HandRemote.Pin(A1)  ,'b'  )->LessThanSetpoint(  HandRemote.Pin(BUZZ)                          );
HandRemote.Control(   Pump.Pin(7)               )->Settable();

HandRemote.Control(   Pump.Pin(65)              )->SetController();

Pump.Control(         Pump.Pin(7)             )->Settable();
//Pump.Control(         Pump.Pin(64)        )->PIDSetpoint(       HeadGate.Pin(6), 'g', 1, 1, 1, P_ON_M, REVERSE );
//Pivot.Control(        Pivot.Pin(7)        )->TieToPin(          Pump.Pin(7) );  
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


}
#endif
#ifndef DUMB_CONTROLLER
void loop() { ThisDevice::Update();  }
#endif
/*********************************************************************************************************************/
#ifdef DUMB_CONTROLLER
/*********************************************************************************************************************/
SSoftwareSerial IOSerial(3,2);
PeerIOSerialControl XBee(DUMB_DEVICEID,IOSerial,Serial);
void setup() {
  pinMode(3, INPUT);    // XBee DOUT Pin
  pinMode(2, OUTPUT);   // XBee DIN Pin
  Serial.begin(9600);   // Start Serial for Debug
  IOSerial.begin(9600); // Start UART Communications with the XBee->Module
}
void loop() {  
  if (XBEECONFIG) {
    if ( IOSerial.available()>0 ) Serial.write(IOSerial.read());    // Forward Serial to XBEE for XBEE Config
    if ( Serial.available()>0 ) IOSerial.write(Serial.read());
  } else {
    XBee.Available();                                             // Check Communications
  }
}
#endif

