/******************************************************************************************************************//**
 * @file AgFirmware.ino
 * @brief Arduino Sketch Firmware for the Devices of the AgIrrigationRemoteControl project
 * @see https://github.com/tgit23/AgIrrigationRemoteControl
 * @authors 
 *    tgit23        8/2017       Original
 **********************************************************************************************************************/
#include <LiquidCrystal.h>
#include "RemoteMenu.h"

//vvvvvvvvvvvvvvvvvv[[[ DEVICES ]]]vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
// Hand-Held Remote Units             ( 1-5 )
// Smart-Controller Units with LCD    ( 6-10 )
// Dumb-Controller Units without LCD  ( 11-15 )
#define HAND_REMOTE           1
#define PUMP_CONTROLLER       6
#define GATE_CONTROLLER       11
#define CHEM_CONTROLLER       12
#define PIVOT_CONTROLLER      8
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//------------------------------------------------------------------------------------------
#define THISDEVICE            HAND_REMOTE   // The device this Firmware is for
#define XBEECONFIG            0             // Set to 1 to configure RF Radio through XCTU
#define RELEASE               20170706      // Release Version used to Build the Units
//------------------------------------------------------------------------------------------

#if THISDEVICE<11
//vvvvvvvvvvvvvvvvvv[[[ USER PIN DEFINITIONS ]]]vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
// PinTypes { INPIN, INPINHIGH, OUTPIN, BUZZPIN, SONICPIN, SETTABLE };
//                  |---- DEVICE-----|-PIN-||-PinType---|----NAME---|ID|-------------------
//------------------------------------------------------------------------------------------
PinPoint PumpPower  ( PUMP_CONTROLLER,    7,  SETTABLE,  "Power",     'P' );
PinPoint WaterLevel ( PUMP_CONTROLLER,   64,  SONICPIN,  "Water",     'L' );
PinPoint GateLevel  ( GATE_CONTROLLER,    6,  SETTABLE,   "Gate",     'G' );
PinPoint ChemPump   ( CHEM_CONTROLLER,    7,  OUTPIN,    "Chem",      'C' );
PinPoint PivotOnOff ( PIVOT_CONTROLLER,   7,  INPINHIGH, "Pivot",     'P' );

//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

// Hardware Pin Definitions
#if THISDEVICE<6          //----------------- Hand-Held Remote Units -------
  LiquidCrystal LCD(8, 9, 4, 5, 6, 7);
  PinPoint Battery  ( HAND_REMOTE,       A1,  INPIN,     "Battery",   'B' );
  #if RELEASE<20170525
  PinPoint Buzzer   ( HAND_REMOTE,        2,  BUZZPIN                     );
  RemoteMenu Menu   (12, 11, THISDEVICE, &LCD);
  #else
  PinPoint Buzzer   ( HAND_REMOTE,       11,  BUZZPIN                     );
  RemoteMenu Menu   (3, 2, THISDEVICE, &LCD);
  #endif
#else                    //----------------- Smart-Controller with LCD -----
  LiquidCrystal LCD(12, 13, 8, 9, 10, 11);
  RemoteMenu Menu(3, 2, THISDEVICE, &LCD);
#endif

void setup() {
  Serial.begin(9600);LCD.begin(16, 2);

  //vvvvvvvvvvvvvvvvvv[[[  DEVICE NAMES  ]]]vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
  // Assign Display Names to the Devices
  Menu.DeviceDisplayName(HAND_REMOTE, "Remote");
  Menu.DeviceDisplayName(PUMP_CONTROLLER, "Pump");
  Menu.DeviceDisplayName(GATE_CONTROLLER, "Gate");
  Menu.DeviceDisplayName(CHEM_CONTROLLER, "Chem");
  Menu.DeviceDisplayName(PIVOT_CONTROLLER, "Pivot");
  //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

  //vvvvvvvvvvvvvvvvvv[[[  PIN CONTROLS  ]]]vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
  //
  // ControlTypes { SET_PIN, PID_SET, DIRECTLY, LESS_THAN, GREATER_THAN, EQUAL_TO, NOT_EQUAL_TO };
  //----------------------- PIN --------- ControlType-------ID----|StorePin|-
#if THISDEVICE<6
  Battery.Controls     (  &Buzzer,        LESS_THAN,        'b'           );
  Menu.AddPin(&Battery);
#endif
  WaterLevel.Controls  (  &GateLevel,     PID_SET,          'g'           );
  PivotOnOff.Controls  (  &ChemPump,      DIRECTLY,         'e'           );
  
  //--------- Add Pins to the Display Menu --------------------
  Menu.AddPin(&PumpPower);
  Menu.AddPin(&WaterLevel);
  Menu.AddPin(&GateLevel);
  Menu.AddPin(&ChemPump);
  Menu.StartDisplay(&PumpPower);
}

void loop() {
  Menu.loop();
}


#else         // THISDEVIC is (11-15)
//---------------------------------------------------------------------------------------------------------------------
//                       DUMB-CONTROLLER without LCD support or any local controls
//---------------------------------------------------------------------------------------------------------------------
SoftwareSerial IOSerial(3,2);                                 // SoftSerial for XBEE ( rxPin, txPin )
PeerIOSerialControl XBee(THISDEVICEID,IOSerial,Serial);

void setup() {
  pinMode(3, INPUT);    // XBee DOUT Pin
  pinMode(2, OUTPUT);   // XBee DIN Pin
  Serial.begin(9600);
  IOSerial.begin(9600);         // Start UART Communications with the XBee->Module
}

void loop(){

#if XBEECONFIG!=0
  if ( IOSerial.available()>0 ) Serial.write(IOSerial.read());    // Forward Serial to XBEE for XBEE Config
  if ( Serial.available()>0 ) IOSerial.write(Serial.read());
#else     
  XBee.Available();                                             // Check Communications
  #if UTRASONIC_METER_INSTALLED==1                          // Read UltraSonic water level if installed
    int ulCurrentTime = millis();
    if ( ulCurrentTime > ulLastPing + 1000 ) {
      XBee.VirtualPin(64, sonar.ping_in() );                    // Assign UltraSonic reading to Virt.Pin(64)
      ulLastPing = ulCurrentTime;
    }
  #endif
    
#endif
#endif
