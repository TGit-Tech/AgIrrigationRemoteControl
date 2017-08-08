/******************************************************************************************************************//**
 * @file AgFirmware.ino
 * @brief Arduino Sketch Firmware for the Devices of the AgIrrigationRemoteControl project
 * @see https://github.com/tgit23/AgIrrigationRemoteControl
 * @authors 
 *    tgit23        8/2017       Original
 **********************************************************************************************************************/
#include <LiquidCrystal.h>
#include "RemoteMenu.h"

//--------------------------------------------------------------------------------------------------
//vvvvvvvvvvvvvvvvvv[[[ DEVICES ]]]vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
//--------------------------------------------------------------------------------------------------
#define HAND_REMOTE               1     // Hand-Held Remote Unit              ( 1-5 )
//#define HAND_REMOTE               2    // Un-comment for the 2nd Hand-Remote
#define PUMP_CONTROLLER           6     // Controller Units                   ( 6-15 )
#define PIVOT_CONTROLLER          7     // Pivot On/Off Signal receiver to CHEM_CONTROLLER
#define DITCH_CONTROLLER          10    // 1st Pump; a 'Ditch' Pump-Controller
#define CANAL_CONTROLLER          11    // 2nd Pump; a 'Canal' Pump-Controller
#define GATE_CONTROLLER           13    // Head-gate Controller for PID-Set water level
#define CHEM_CONTROLLER           14    // Chemical pump controller driven by Pivot On/Off signal
//--------------------------------------------------------------------------------------------------
#define THISDEVICE            HAND_REMOTE     // The device this Firmware is for
#define RELEASE               20170706        // Release Version used to Build the Units
const bool XBEECONFIG =       false;          // Set to 'true' to configure Xbee with XCTU
//#define DUMB_CONTROLLER                     // Dumb-Controllers only Receive and Send raw pin data
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

#ifndef DUMB_CONTROLLER
//--------------------------------------------------------------------------------------------------
//vvvvvvvvvvvvvvvvvv[[[ USER-PIN DEFINITIONS ]]]vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
//--------------------------------------------------------------------------------------------------
//      PinTypes { INPIN, INPINHIGH, OUTPIN, BUZZPIN, SONICPIN, SETTABLE };
//                                                                              (for Ultrasonic)
//                  |---- DEVICE-----|-PIN-||-PinType---|--|--NAME--------|ID|--|Trig|Echo|-------
//--------------------------------------------------------------------------------------------------
PinPoint PumpPower  ( PUMP_CONTROLLER,    7,  SETTABLE,     "Power",      'P' );
PinPoint WaterLevel ( PUMP_CONTROLLER,   64,  SONICPIN,     "Water",      'L',    4,    5 );
PinPoint GateLevel  ( GATE_CONTROLLER,    6,  SETTABLE,     "Gate",       'G' );
PinPoint ChemPump   ( CHEM_CONTROLLER,    7,  OUTPIN,       "Chem",       'C' );
PinPoint PivotOnOff ( PIVOT_CONTROLLER,   7,  INPINHIGH,    "Pivot",      'P' );
PinPoint PidControl ( PUMP_CONTROLLER    65,  USERCONTROL,  "W-G Pid",    'g' );
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

  // Hardware Assembly Pin Definitions
  #if THISDEVICE<6
    LiquidCrystal LCD(8, 9, 4, 5, 6, 7);
    PinPoint Battery  ( HAND_REMOTE,       A1,  INPIN,     "Battery",   'B' );
    #if RELEASE<20170525    //------- Release 2017.05.24
      PinPoint Buzzer   ( HAND_REMOTE,        2,  BUZZPIN                     );
    #else                   //------- Release 2017.06.30
      PinPoint Buzzer   ( HAND_REMOTE,       11,  BUZZPIN                     );
    #endif
  #else
    LiquidCrystal LCD(12, 13, 8, 9, 10, 11);
  #endif
  
RemoteMenu Menu(&LCD);
void setup() {
  
  #if RELEASE<20170525 && THISDEVICE<6
    Menu.Setup(THISDEVICE, XBEECONFIG, 12, 11 ); // Force Rx,Tx to 12, 11
  #else
    Menu.Setup(THISDEVICE, XBEECONFIG);
  #endif

  //----------------------------------------------------------------------------------------
  //vvvvvvvvvvvvvvvvvv[[[  DEVICE NAMES  ]]]vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
  //----------------------------------------------------------------------------------------
  // Assign Display Names
  Menu.DeviceName(  HAND_REMOTE,        "Remote");
  Menu.DeviceName(  PUMP_CONTROLLER,    "Pump");
  Menu.DeviceName(  GATE_CONTROLLER,    "Gate");
  Menu.DeviceName(  CHEM_CONTROLLER,    "Chem");
  Menu.DeviceName(  PIVOT_CONTROLLER,   "Pivot");
  //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^



  //----------------------------------------------------------------------------------------
  //vvvvvvvvvvvvvvvvvv[[[  PIN CONTROLS  ]]]vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
  //----------------------------------------------------------------------------------------
  // ControlTypes { SET_PIN, PID_SET, DIRECTLY, LESS_THAN, GREATER_THAN, EQUAL_TO, NOT_EQUAL_TO };
  //----------------------- PIN --------- ControlType-------ID----|StorePin|-

  WaterLevel.Controls  (  &GateLevel,     PID_SET,          'g'           );
  PivotOnOff.Controls  (  &ChemPump,      DIRECTLY,         'e'           );
  //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


  // Hardware Assembly default Controls for Hand-Held Remote Units ( 1-5 )
  #if THISDEVICE<6
    Battery.Controls     (  &Buzzer,        LESS_THAN,        'b'           );
    Menu.AddPin(&Battery);
  #endif

  //----------------------------------------------------------------------------------------
  //vvvvvvvvvvvvvvvvvv[[[  MENU PINS  ]]]vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
  //----------------------------------------------------------------------------------------
  Menu.AddPin(&PumpPower);
  Menu.AddPin(&WaterLevel);
  Menu.AddPin(&GateLevel);
  Menu.AddPin(&ChemPump);
  Menu.Begin(&PumpPower);
  //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
}
#endif
#ifdef DUMB_CONTROLLER
SSoftwareSerial IOSerial(3,2);
PeerIOSerialControl XBee(THISDEVICE,IOSerial,Serial);
void setup() {
  pinMode(3, INPUT);    // XBee DOUT Pin
  pinMode(2, OUTPUT);   // XBee DIN Pin
  Serial.begin(9600);   // Start Serial for Debug
  IOSerial.begin(9600); // Start UART Communications with the XBee->Module
}
#endif

#ifndef DUMB_CONTROLLER
void loop() { Menu.loop(); }
#endif

#ifdef DUMB_CONTROLLER
void loop() {  
  if (XBEECONFIG) {
    if ( IOSerial.available()>0 ) Serial.write(IOSerial.read());    // Forward Serial to XBEE for XBEE Config
    if ( Serial.available()>0 ) IOSerial.write(Serial.read());
  } else {
    XBee.Available();                                             // Check Communications
  }
}
#endif
