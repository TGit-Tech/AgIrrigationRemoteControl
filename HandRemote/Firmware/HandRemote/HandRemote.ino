/******************************************************************************************************************//**
 * @brief  Arduino Sketch firmware to be uploaded onto the Ag-Irrigation Hand-Remote 
 * @see https://github.com/tgit23/AgIrrigationRemoteControl
 * @remarks Version 2017.04.26
 * @todo
 *  - Implement Firmata for Base/Desktop operation
 *  - Make so default on Set is Opposite of last reading
 * @authors 
 *    tgit23        01/2017       Original
**********************************************************************************************************************/
#include <LiquidCrystal.h>
#include <EEPROM.h>
#include <PeerIOSerialControl.h>        //See https://github.com/tgit23/PeerIOSerialControl
#include <SoftwareSerial.h>

//---[ COMMONLY CHANGED SETTINGS ]-------------------------------------------------------------------------------------
#define TRANSCEIVER_ID 2                // Unique numeric (ID)entity for this Unit(1-15)
#define XBEECONFIG 0                    // Configure the XBEE using XCTU Digi Software by setting this to 1
#define DEBUG 0                         // Set this to 1 for Serial DEBUGGING messages ( Firmware development use only )

//---[ PROGRAM BEHAVIOR SETTINGS ]-------------------------------------------------------------------------------------
#define WAIT_REPLY_MS         2000      // How long to wait for a XBee reply
#define START_STATUS_ITERATE  15000     // Start iterating Menu-Items after idle for (ms)
#define ITERATE_EVERY         5000      // Iterate Menu-Items every (ms); when idle

//---[ LOCAL-HARDWARE PIN SETTINGS ]-----------------------------------------------------------------------------------
#define SBUZZ 2                         // Signal-Pin on Buzzer ( D2 )
#define SS_TX_PIN 11                    // TX -> XBEE-DIN ( Closest to UNO )
#define SS_RX_PIN 12                    // RX -> XBEE-DOUT ( Farthest from UNO )
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);    // select the pins used on the LCD panel

//vvv[ PROGRAM CONSTANTS (DO NOT CHANGE!)]vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
enum Button { UP, DOWN, RIGHT, LEFT, SELECT, NONE };
enum eLocation {
  // PIN NUMBER = 0x007F Bits(0->6)     // Allows up-to value 127   0000 0000 0111 1111
  A             = 0x0000,               // bit(7), Analog Pin (0)   0000 0000 0### ####
  D             = 0x0080,               // bit(7), Digital Pin (1)  0000 0000 1### ####
  PUMP0_PIN     = 0x0100,               // bit(8), Pump-Option[0]   0000 0001 0000 0000
  PUMP1_PIN     = 0x0200,               // bit(9), Pump-Option[1]   0000 0010 0000 0000
  PUMP2_PIN     = 0x0400,               // bit(10), Pump-Option[2]  0000 0100 0000 0000
  PUMP3_PIN     = 0x0800,               // bit(11), Pump-Option[3]  0000 1000 0000 0000
  ALL_PUMPS_PIN = 0x0F00,               // 0x3E00 All-Pump-Bits     0000 1111 0000 0000
  HAND_PIN      = 0x1000,               // bit(12), Hand-Pin        0001 0000 0000 0000
  HAND_EPROM    = 0x2000,               // bit(13), Hand-EEPROM     0010 0000 0000 0000
  HAND_PROG     = 0x4000,               // bit(14), Hand-PROG       0100 0000 0000 0000
};
#define PIN         0x007F              // ( Location & PIN ) = Extract Pin Number from Location
#define PUMP_PIN    0x0F00              // ( Location & PUMP_PIN ) = true if any Pump-Pin
#define HAND_PIN    0x1000              // ( Location & HAND_PIN ) = true if any Hand-Pin
#define HAND        0xF000              // ( Location & HAND ) = Any Hand-Remote Location
#define DIGITALPIN  0x0080              // ( Location & DIGITALPIN ) = true if digital pin
#define MAIN 0                          // Menu[#].Sub[0-MAIN] for currently-read value of the Menu Item
#define VALID true                      // Is Menu[#].Sub[MAIN].State a 'VALID' (read) value?
#define SET 1                           // Menu[#].Sub[1-SET] for SET values to await being applied to the MAIN-Value
#define SETTABLE true                   // Does Menu[#].Sub[SET].State allow 'SETTABLE' values by the user
#define LOALARM 2                       // Menu[#].Sub[2-LOALARM] for storing Low Alarm value to test the MAIN-Value
#define HIALARM 3                       // Menu[#].Sub[3-HIALARM] for storing High Alarm value to test the MAIN-Value
#define ON true                         // Is the "Menu[#].Sub[??ALARM].State" (ALARM) ON or NOT-ON?
#if DEBUG>0
  #define DBL(x) Serial.println x
  #define DB(x) Serial.print x
  #define DBC Serial.print(", ")
#else
  #define DBL(x)
  #define DB(x)
  #define DBC  
#endif
SoftwareSerial IOSerial(SS_RX_PIN,SS_TX_PIN);                 // SoftSerial for XBEE ( rxPin, txPin )
PeerIOSerialControl XBee(TRANSCEIVER_ID,IOSerial,Serial);     // XBee(ArduinoID, IOSerial, DebugSerial)
void GetItem(int i = -1);                                     // Predefine header so SetupMenu() can be at top
unsigned long last_bpress = 0;              // Track last button press time
unsigned long last_change = 0;              // Track last status iteration time
int idx = 0;                                // Track Menu index item
int SubIdx = 0;                             // Track current menu value item
bool AlarmActive = false;                   // Track if an Active Alarm is present
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

//=====================================================================================================================
//------------------------------ MENU STRUCTURE ( CONFIGURABLE ) ------------------------------------------------------
//=====================================================================================================================
//---[ MENU-ITEMS USED IN THE FIRMWARE ]--
#define MONITOR 0                         // Menu[MONITOR] tells the firmware to monitor only selected or all pumps
#define PUMPIDX 1                         // Menu[PUMPIDX] the Menu-index of Selected Pump-Controller

#define NUM_MENU_ITEMS 7                  //<<<<<<<<<<<<<<< MUST MATCH NUMBER OF MENU ITEMS DEFINED !!!!!!!!!!!!!!!!!!
#define MAXOPTIONS 2                      //<<<<<<<<<<<<<<< MUST ALLOW MAXIMUM NO of OPTIONS USED !!!!!!!!!!!!!!!!!!!!
#define STARTIDX 3                        // First Menu item to display when powering on the Hand-Remote

//---[ SETUP the MENU[#].Sub[?] Structure ]----------------------------------------------------------------------------
struct uMenuOption {
  char            *Text;                  // Text display for this-one Menu-Item OPTION
  int             Value = LOW;            // A value used for this OPTION ( HIGH-1/LOW-0 for ON/OFF / TransieverID)
};
struct uSubVal {
  int             Value = -1;             // Value location for Val[MAIN, SET, HIALARM, LOALARM]
  int             ToneHz = 1000;          // A tone frequency associated with this alarm when activated
  char            ID = NULL;              // Character to IDentify an alarm; setting an ID makes an alarm SETTABLE
  bool            State = false;          // [MAIN]State=VALID, [SET]State=SETTABLE, [xxALARM]=ON
};
struct MenuItems {
  char            *Text;                        // The text to display on the LCD for this Menu item
  eLocation       Location = HAND_PROG;         // Where to get/set the MAIN-Value; LOCAL, REMOTE, EPROM, etc...
  uSubVal         Sub[4];                       // MAIN, SET, LOALARM, HIALARM - Value storage per Menu Item
  uMenuOption     Option[MAXOPTIONS];           // Selectable options like ON/OFF, Pump1 or 2 etc..
  byte            LastOptionIdx = 0;            // The number of "Option"s for this Menu-Item; or 0 = Numeric value
} Menu[NUM_MENU_ITEMS];                   

/******************************************************************************************************************//**
 * @brief  Setup the LCD menu
 * @remarks
 * - Allows a single spot customization to users configuration
 * - Be sure to change 'NUM_MENU_ITEMS' when adding/deleting items
 * - Display will show the items in the same order as they are defined here
 * @code
 *   exmaple code
 * @endcode
**********************************************************************************************************************/
void SetupMenu() {

  //vvvvvv[ HAND-REMOTE MENU ITEMS ]vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
  Menu[MONITOR].Text = "Monitor";                       // Create a menu item for monitoring ALL Pump-Controllers
  Menu[MONITOR].Location = HAND_EPROM;                  // Value is stored in the Hand-Remotes EEPROM memory
  Menu[MONITOR].Sub[SET].State = SETTABLE;              // User can SET the value
  
  Menu[MONITOR].Option[0].Text = "Sel";                 // Monitor "Sel" Pump at a time   - Option #0 = Selected
  Menu[MONITOR].Option[0].Value = 0;                    // "Sel" will be the value 0      - One = 0
  Menu[MONITOR].Option[1].Text = "All";                 // Monitor "All" Pumps            - Option #1 = All
  Menu[MONITOR].Option[1].Value = 1;                    // "All" will be the value 1      - All = 1
  Menu[MONITOR].LastOptionIdx = 1;                      // Last Option Index defined      - Number of Options - 1
  
  //-------------------------------------------------------------------------------------------------------------------
  Menu[PUMPIDX].Text = "Pump";                          // Menu Item used to select the Pump-Controller
  Menu[PUMPIDX].Location = HAND_EPROM;                  // Selected pump is stored in EEPROM
  Menu[PUMPIDX].Sub[SET].State = SETTABLE;              // Allow this value to be 'SET' by the user
  
  // Define the Pump-Controllers ( A maximum of 4; index 0->3 )
  // Check that "#define MAXOPTIONS" allows the number of indexes used ( default; 2 )
  Menu[PUMPIDX].Option[0].Text = "Canal";               // Pump can be "Canal"            - Option #0 = Canal
  Menu[PUMPIDX].Option[0].Value = 10;                   // "Canal" Value=TRANSCEIVER_ID   - Canal = TransceiverID #10
  Menu[PUMPIDX].Option[1].Text = "Ditch";               // Pump can be "Ditch"            - Option #1 = Ditch
  Menu[PUMPIDX].Option[1].Value = 11;                   // "Ditch" Value=TRANSCEIVER_ID   - Ditch = TrasceiverID #11
  Menu[PUMPIDX].LastOptionIdx = 1;                      // Last Pump Option Index         - # of Pump-Controllers - 1

  //-----------------------------------------
  Menu[2].Text = "Battery(B)";                    // Create a menu item for monitoring the Battery
  Menu[2].Location = HAND_PIN+ A+1;               // Battery level is gotten from the Hand-Remote pin A1
  Menu[2].Sub[LOALARM].ID = 'b';                  // A Low Alarm is identified by a lower-case 'b'
  
  //vvvvvv[ PUMP-SPECIFIC MENU ITEMS ]vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
  //-----------------------------------------
  Menu[3].Text = "Power(P)";                      // Create a menu item for Power Control
  Menu[3].Location = ALL_PUMPS_PIN+ D+7;          // Power is set/got on all Pump-Controller's on pin [D7]
    
  Menu[3].Sub[SET].State = SETTABLE;              // Allow this Value to be 'SET' by the user
  Menu[3].Sub[LOALARM].ID = 'p';                  // A Low Alarm is identified by a lower-case 'p'
  Menu[3].Sub[HIALARM].ID = 'P';                  // A High Alarm is identified by an upper-case 'P'
 
  Menu[3].Option[0].Text = "Off";                 // Power can be "Off"             - Option #0 = Off
  Menu[3].Option[0].Value = LOW;                  // "Off" will be the value 'LOW"  - Off = LOW
  Menu[3].Option[1].Text = "On";                  // Power can be "On"              - Option #1 = On
  Menu[3].Option[1].Value = HIGH;                 // "On" will be the value 'HIGH'  - On = HIGH
  Menu[3].LastOptionIdx = 1;                      // Last Option Index defined      - Number of Options - 1  
  
  //-----------------------------------------
  Menu[4].Text = "Water (L)";                     // Create a menu item for Water Level Transducer
  Menu[4].Location = PUMP1_PIN+ A+64;             // Water Level is read from VIRTUAL (Pump-Controllers firmware) pin 64           
  Menu[4].Sub[LOALARM].ID = 'l';                  // A Low Alarm is identified by a lower-case 'l'
  Menu[4].Sub[HIALARM].ID = 'L';                  // A High Alarm is identified by an upper-cse 'L'
  
  //-----------------------------------------
  Menu[5].Text = "Pressure(R)";                   // Create a menu item for the Primary Pressure Transducer
  Menu[5].Location = ALL_PUMPS_PIN+ A+3;          // The 'signal' is gotten on all Pump-Controllers on pin [A3]
  Menu[5].Sub[LOALARM].ID='r';                    // A Low Pressure alarm is identified by a lower-case 'r'
  Menu[5].Sub[HIALARM].ID='R';                    // A High Pressure alarm is identified by an upper-case 'R'
  
  //-----------------------------------------
  Menu[6].Text = "Pressure(S)";                   // Create a menu item for the Secondary Pressure Transducer
  Menu[6].Location = PUMP0_PIN+ A+4;              // Menu-item is for Pump-Option #0 (Canal) on Pin (A4)
  Menu[6].Sub[LOALARM].ID='s';                    // A Low Pressure alarm is identified by a lower-case 's'
  Menu[6].Sub[HIALARM].ID='S';                    // A High Pressure alarm is identified by an upper-case 'S'
  
  //------------[ Start-Up the Display ( DO NOT CHANGE! )]-------------
  GetItem(PUMPIDX);                               // Read the last selected Pump-Controller from EEPROM
  idx = STARTIDX;SubIdx=MAIN;                     // Set where the Menu will start
  GetItem();                                      // Get starting Menu item
  LCD_display();                                  // Update the display
}

/******************************************************************************************************************//**
 * @brief  Record Menu Item Values in EEPROM non-volitale memory.
 * @remarks
 * - Arduino UNO offers 1024-bytes or (146)7-Byte Menu Item Storage
 * - Pump-Controller Menu Items need 5-Alarm Bytes per Selectable Pump
 * - Hand_EPROM Menu Items need 7-Bytes ( 2-Value, 5-Alarms )
 * - Limits (6)Pumps with (30)5-Byte Values = (150)Bytes per Pump or (900)Bytes Total for Pump Alarms
 * - (1024 - 900)/7 = 0->17 Menu Item Idx can be used for HAND items.
 *  @code
 *    exmaple code
 *  @endcode
**********************************************************************************************************************/
void EEPROMSet(int i = -1) {
  if ( i == -1 ) i = idx;DB(("EEPROMSet("));DB((i));DBL((")"));
  if ( Menu[i].Location == HAND_PROG ) return;                  // PROG values stay in the volatile memory
  int iOffset = (i*5)+Menu[PUMPIDX].Sub[MAIN].Value * 150;      // Pumps @(150)Bytes/pump (5)Bytes/item
  
  if ( Menu[i].Location == HAND_EPROM ) {
    iOffset = i*7+900;                                          // Hand-Remote @900 (7)Bytes/item
    byte valLowByte = ((Menu[i].Sub[MAIN].Value >> 0) & 0xFF);
    byte valHighByte = ((Menu[i].Sub[MAIN].Value >> 8) & 0xFF);
      
    EEPROM.update( iOffset, valLowByte);iOffset++;
    DB(("EEPROM.update( "));DB((iOffset));DBC;DB((valLowByte, HEX));DBL((")"));
    EEPROM.update( iOffset, valHighByte);iOffset++;
    DB(("EEPROM.update( "));DB((iOffset));DBC;DB((valHighByte, HEX));DBL((")"));
    Menu[i].Sub[MAIN].State = VALID;                            // Validate the setting
  }
    
  // Store Alarm Values in EEPROM
  byte loAlarmLoByte = ((Menu[i].Sub[LOALARM].Value >> 0) & 0xFF);
  byte loAlarmHiByte = ((Menu[i].Sub[LOALARM].Value >> 8) & 0xFF);
  byte hiAlarmLoByte = ((Menu[i].Sub[HIALARM].Value >> 0) & 0xFF);
  byte hiAlarmHiByte = ((Menu[i].Sub[HIALARM].Value >> 8) & 0xFF);
  byte AlarmSet = 0x22;   // 2=OFF, A=ON
  if ( Menu[i].Sub[LOALARM].State == ON ) bitSet(AlarmSet,3);
  if ( Menu[i].Sub[HIALARM].State == ON ) bitSet(AlarmSet,7);
    
  EEPROM.update( iOffset, loAlarmLoByte);iOffset++;
  DB(("EEPROM.update( "));DB((iOffset));DBC;DB((loAlarmLoByte, HEX));DBL((")"));
  EEPROM.update( iOffset, loAlarmHiByte);iOffset++;
  DB(("EEPROM.update( "));DB((iOffset));DBC;DB((loAlarmHiByte, HEX));DBL((")"));
  EEPROM.update( iOffset, hiAlarmLoByte);iOffset++;
  DB(("EEPROM.update( "));DB((iOffset));DBC;DB((hiAlarmLoByte, HEX));DBL((")"));
  EEPROM.update( iOffset, hiAlarmHiByte);iOffset++;
  DB(("EEPROM.update( "));DB((iOffset));DBC;DB((hiAlarmHiByte, HEX));DBL((")"));
  EEPROM.update( iOffset, AlarmSet);iOffset++;
  DB(("EEPROM.update( "));DB((iOffset));DBC;DB((AlarmSet, HEX));DBL((")"));
}

/******************************************************************************************************************//**
 * @brief  Read Menu Item Values from Arduino EEPROM non-volitale memory.
 * @see    EEPROMSet for Addressing notation
 * @code
 *   exmaple code
 * @endcode
**********************************************************************************************************************/
void EEPROMGet(int i = -1) {
  byte StatusByte = 0;
  if ( i == -1 ) i = idx;DB(("EEPROMGet("));DB((i));DBL((")"));
  if ( Menu[i].Location == HAND_PROG ) return;                  // PROG values stay in the volatile memory
  int iOffset = (i*5)+Menu[PUMPIDX].Sub[MAIN].Value * 150;      // Pumps @(150)Bytes/pump (5)Bytes/item
  
  if ( Menu[i].Location == HAND_EPROM ) {                       // Get a "Unit Setting" value from EEPROM
    iOffset = i*7+900;                                          // Hand-Remote @900 (7)Bytes/item
    byte valLowByte = EEPROM.read( iOffset );iOffset++;
    byte valHighByte = EEPROM.read( iOffset );iOffset++;
    Menu[i].Sub[MAIN].Value = (int)((valLowByte << 0) & 0xFF) + ((valHighByte << 8) & 0xFF00);
  }

  // Get Alarm Values from EEPROM
  DB(("EEPROMGet() - LoAlarmByte Offset = "));DB((iOffset));DBL((")"));
  byte loAlarmLoByte = EEPROM.read( iOffset );iOffset++;
  byte loAlarmHiByte = EEPROM.read( iOffset );iOffset++;
  byte hiAlarmLoByte = EEPROM.read( iOffset );iOffset++;
  byte hiAlarmHiByte = EEPROM.read( iOffset );iOffset++;
  byte AlarmSet = EEPROM.read( iOffset );iOffset++;
  
  Menu[i].Sub[LOALARM].Value = (int)((loAlarmLoByte << 0) & 0xFF) + ((loAlarmHiByte << 8) & 0xFF00);
  Menu[i].Sub[HIALARM].Value = (int)((hiAlarmLoByte << 0) & 0xFF) + ((hiAlarmHiByte << 8) & 0xFF00);
  Menu[i].Sub[LOALARM].State = bitRead(AlarmSet,3);
  Menu[i].Sub[HIALARM].State = bitRead(AlarmSet,7);

  // EPROM Values are 'VALID' if the AlarmSet Bit Check passes ( Value was set/not just garbage )
  if ( Menu[i].Location == HAND_EPROM ) {
    if ( (AlarmSet & 0x77) == 0x22 ) Menu[i].Sub[MAIN].State = VALID;
  }
}

/******************************************************************************************************************//**
 * @brief  Preforms all needed changes to the system when changing a Selected Pump.  Loads alarm values
 * @code
 *   exmaple code
 * @endcode
**********************************************************************************************************************/
void SetPump(int TransceiverID) {
  
  if ( TransceiverID != XBee.TargetArduinoID() ) {
    for (int i=0;i<NUM_MENU_ITEMS;i++) {
      if ( i != PUMPIDX ) {                                         // Don't clear Selected Pump or PROG values
        Menu[i].Sub[MAIN].State = !VALID;                           // De-Activate previouse Read Value's
        EEPROMGet(i);                                               // Load Alarm Values
      }
    }
    XBee.TargetArduinoID(TransceiverID);
  }
}

/******************************************************************************************************************//**
 * @brief  Obtain menu values and compare them against their alarm values
 * @remarks
 *  - Battery read 458 when running off USB and 856 when running from 9VDC Battery.
 * @code
 *   exmaple code
 * @endcode
**********************************************************************************************************************/
void GetItem(int i = -1) {
  if ( i == -1 ) i = idx;DB(("GetItem("));DB((i));DBL((")"));                   // Default index = global 'idx'
  if ( i != PUMPIDX && Menu[PUMPIDX].Sub[MAIN].State != VALID ) return;         // Make sure a valid Pump is Selected

  //-------------------------------------------------------------------------------------------------------------------
  if ( Menu[i].Location == HAND_PROG ) {
    return;
  } else if ( Menu[i].Location == HAND_EPROM ) {                                // Read Value from EEPROM
      EEPROMGet(i);                                                             // EEPROMGet() does value validation
  } else {
    if ( Menu[i].Location & HAND_PIN ) {
      if ( Menu[i].Location & DIGITALPIN ) {                                    // Read value from Hand-Remote Pin
        Menu[i].Sub[MAIN].Value = digitalRead(Menu[i].Location & PIN);
      } else {
        Menu[i].Sub[MAIN].Value = analogRead(Menu[i].Location & PIN);
      }
    } else if ( Menu[i].Location & PUMP_PIN ) {
      if ( Menu[i].Location & DIGITALPIN ) {                                    // Read value from Pump Pin
        Menu[i].Sub[MAIN].Value = XBee.digitalReadB(Menu[i].Location & PIN);
      } else {
        Menu[i].Sub[MAIN].Value = XBee.analogReadB(Menu[i].Location & PIN);
      }
    }
    if ( Menu[i].Sub[MAIN].Value != -1 ) Menu[i].Sub[MAIN].State = VALID;       // Validate Pin Values
  }

  //-------------------------------------------------------------------------------------------------------------------
  if ( Menu[i].Sub[MAIN].State == VALID ) {
    if ( i == PUMPIDX && Menu[i].Sub[MAIN].State == VALID ) {                     // New Pump Selection
      SetPump( Menu[i].Option[ Menu[i].Sub[MAIN].Value ].Value );                 // SetPump(TRANSCEIER_ID)
    }            
        
    if ( Menu[i].Sub[LOALARM].State == ON && Menu[i].Sub[LOALARM].ID != NULL ) {  // Check the LOALARM
      if ( Menu[i].LastOptionIdx > 0 ) {                                             
        AlarmActive = ( Menu[i].Sub[MAIN].Value == Menu[i].Sub[LOALARM].Value );  // Option Compare EQUALS
      } else {                                                                    
        AlarmActive = ( Menu[i].Sub[MAIN].Value < Menu[i].Sub[LOALARM].Value );   // Value Compare LESS-THAN
      }
    }
    
    if ( Menu[i].Sub[HIALARM].State == ON && Menu[i].Sub[HIALARM].ID != NULL ) {  // Check the HIALARM
      if ( Menu[i].LastOptionIdx > 0 ) {
        AlarmActive = ( Menu[i].Sub[MAIN].Value != Menu[i].Sub[HIALARM].Value );  // Option Compare NOT-EQUAL
      } else {
        AlarmActive = ( Menu[i].Sub[MAIN].Value > Menu[i].Sub[HIALARM].Value );   // Value Compare GREATER-THAN
      }
    }
    if ( AlarmActive ) tone(SBUZZ,Menu[i].Sub[LOALARM].ToneHz);                   // Sound Buzzer
  } 
}

/******************************************************************************************************************//**
 * @brief  Preform value setting and recording when the 'Select' button is pressed.
 * @code
 *   exmaple code
 * @endcode
**********************************************************************************************************************/
void SetItem(int i = -1) {
  if ( i == -1 ) i = idx;int iSetValue = 0;DBL(("SetItem()"));
  if ( i != PUMPIDX && Menu[PUMPIDX].Sub[MAIN].State != VALID ) return;   // Disable the menu until a pump is selected

  if ( SubIdx == LOALARM || SubIdx == HIALARM ) {                         // ----------- ALARMS -----------------
    if (Menu[i].Sub[SubIdx].State == ON) {Menu[i].Sub[SubIdx].State = !ON;} else {Menu[i].Sub[SubIdx].State = ON;}
    EEPROMSet();                                                          // Save Alarm ON/OFF Status in EEPROM

  } else if ( SubIdx == MAIN ) {                                          // ----------- MAIN -------------------
    GetItem(i);                                                           // Select will Refresh item

  } else if ( SubIdx == SET ) {                                           // ----------- SET --------------------
    iSetValue = Menu[i].Sub[SET].Value;                                   // Record the current Set Value
    if ( Menu[i].LastOptionIdx > 0 ) {                                                // IF [SET] value is an OPTION
      if (Menu[i].Sub[SET].Value < 0 || Menu[i].Sub[SET].Value > MAXOPTIONS) return;  // Boundary Check the OPTION
      iSetValue = Menu[i].Option[Menu[i].Sub[SET].Value].Value;                       // Record the OPTIONS [SET] Value
    }
    Menu[i].Sub[MAIN].State = !VALID;                                     // UN-VALIDATE the [MAIN] Value; its changing
    
    if ( Menu[i].Location == HAND_PROG ) {
      Menu[i].Sub[MAIN].Value = Menu[i].Sub[SET].Value;                   // Set the new value in Menu Memory
      Menu[i].Sub[MAIN].State = VALID;                                    // Validate the new Value
      
    } else if ( Menu[i].Location == HAND_EPROM ) {
      Menu[i].Sub[MAIN].Value = Menu[i].Sub[SET].Value;                   //  [MAIN] Value becomes [SET] Value
      EEPROMSet(i);                                                       //  Write the Value to EEPROM
      Menu[i].Sub[MAIN].State = VALID;                                    //  Validate the new Value
      if ( i == PUMPIDX ) SetPump( Menu[i].Option[ Menu[i].Sub[MAIN].Value ].Value ); // SetPump(TRANSCEIER_ID)

    } else if ( Menu[i].Location & HAND_PIN ) {
      Menu[i].Sub[MAIN].State = VALID;                                    // Validate the new Value
      if ( Menu[i].Location & DIGITALPIN ) {
        digitalWrite(Menu[i].Location & PIN, iSetValue);                  // Set the Local Arduino Digital Pin
      } else {
        analogWrite(Menu[i].Location & PIN, iSetValue);                   // Set the Local Arduino Analog Pin  
      }
    } else if ( Menu[i].Location & PUMP_PIN ) {
      if ( Menu[i].Location & DIGITALPIN ) {
        XBee.digitalWriteNB(Menu[i].Location & PIN, iSetValue);           // Set the Remote Arduino Digital Pin
      } else {
        XBee.analogWriteNB(Menu[i].Location & PIN, iSetValue);            // Set the Remote Arduino Analog Pin
      }
    }
  }
}

/******************************************************************************************************************//**
 * @brief  Properly display a Menu Item on the LCD screen
 * @code
 *   exmaple code
 * @endcode
**********************************************************************************************************************/
void LCD_display() {
  
  // Top Row (Selected Pump)
  lcd.clear();lcd.setCursor(0,0);
  if( Menu[idx].Location & HAND ) {
    lcd.print("Hand-Remote");
  } else if(Menu[PUMPIDX].Sub[MAIN].State != VALID) {
    lcd.print("ERR Pump");
  } else {
    lcd.print(Menu[PUMPIDX].Option[Menu[PUMPIDX].Sub[MAIN].Value].Text);
    lcd.setCursor(strlen(Menu[PUMPIDX].Option[Menu[PUMPIDX].Sub[MAIN].Value].Text)+1,0);lcd.print("Pump");
  }
  
  // Right Side Alarm Identifiers
  int pos = 15;
  for (int i=0;i<NUM_MENU_ITEMS;i++) {
    if ( Menu[i].Location & HAND || bitRead(Menu[i].Location, Menu[PUMPIDX].Sub[MAIN].Value + 8) ) {
      if ( Menu[i].Sub[LOALARM].State == ON && Menu[i].Sub[LOALARM].ID != NULL ) {
        lcd.setCursor(pos,0);lcd.print(Menu[i].Sub[LOALARM].ID);pos--;}
      if ( Menu[i].Sub[HIALARM].State == ON && Menu[i].Sub[HIALARM].ID != NULL ) {
        lcd.setCursor(pos,0);lcd.print(Menu[i].Sub[HIALARM].ID);pos--;}
    }
  }
       
  // Bottom Row ( Menu Item Text )
  lcd.setCursor(0,1);lcd.print(Menu[idx].Text);
  lcd.setCursor(strlen(Menu[idx].Text), 1);
  
  if ( SubIdx == MAIN ) {
    lcd.print(" =");
    lcd.setCursor( strlen(Menu[idx].Text) + 2, 1);
  } else if ( SubIdx == SET ) {
    lcd.print(" SET");
    lcd.print((char)126); //Character '->'
    lcd.setCursor( strlen(Menu[idx].Text) + 5, 1);
  } else {
    if ( Menu[idx].LastOptionIdx > 0 ) {
      if ( SubIdx == LOALARM ) lcd.print(" !=");
      if ( SubIdx == HIALARM ) {lcd.print(" !");lcd.print((char)183);} // Slashed equal
    } else {
      if ( SubIdx == LOALARM) lcd.print(" !<");
      if ( SubIdx == HIALARM) lcd.print(" !>");
    }
    lcd.setCursor( strlen(Menu[idx].Text) + 3, 1);
  }
  
  // Value
  if ( SubIdx == MAIN && Menu[idx].Sub[MAIN].State != VALID ) {
    lcd.print("ERR");
  } else {
    if ( Menu[idx].LastOptionIdx > 0 ) {
      int OptionIdx = Menu[idx].Sub[SubIdx].Value;
      if( OptionIdx < 0 || OptionIdx > Menu[idx].LastOptionIdx ) {
        lcd.print("ERR");
      } else {
        lcd.print(Menu[idx].Option[OptionIdx].Text); 
      }
    } else {
        //---------------------- RAW VALUE DISPLAY MODIFICATIONS -----------------------------
        // Menu[4 and 5] are Pressure Transducers we change to PSI ( Pounds per square inch )
        if ( idx == 5 || idx == 6 ) {
          lcd.print( (int) ((Menu[idx].Sub[SubIdx].Value - 97) * 0.2137) );
        } else {
          lcd.print(Menu[idx].Sub[SubIdx].Value);
        }
    }
  }
  delay(250);
}

/******************************************************************************************************************//**
 * @brief  Arduino Sketch Setup routine - Initialize the environment.
 * @remarks
 * - pin#10 INPUT Backlit shorting see http://forum.arduino.cc/index.php?topic=96747.0
 * @code
 *   exmaple code
 * @endcode
**********************************************************************************************************************/
void setup(){
    pinMode(10,INPUT);            // Fix for Q1-LCD Backlit shorting issue
    pinMode(SBUZZ,OUTPUT);
    pinMode(SS_RX_PIN, INPUT);
    pinMode(SS_TX_PIN, OUTPUT);
    
    IOSerial.begin(9600);
    Serial.begin(9600);           // Start UART Communications with the XBee->Module
    lcd.begin(16, 2);             // Start the LCD library
    SetupMenu();                  // Setup the Menu Items
    XBee.Timeout(WAIT_REPLY_MS);  // Set the Timeout for XBEE communications
    LCD_display();                // Display the main menu
}

/******************************************************************************************************************//**
 * @brief  Arduino Sketch Loop() routine
 * @remarks
 * - This function is called automatically over-and-over by Arduino
 * - Handles incoming XBee communications
 * - Handles button presses and LCD response updates
 * - Handles Menu iteratation during idle.
**********************************************************************************************************************/
void loop(){

//--- FORWARD SERIAL TO XBEE for XBEECONFIG --------------------
#if XBEECONFIG>0
  if ( IOSerial.available()>0 ) Serial.write(IOSerial.read());
  if ( Serial.available()>0 ) IOSerial.write(Serial.read());
#else
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

  //--- Check for incoming communications
  XBee.Available();

  //--- Check for a button press
  Button bpress = NONE;
  int adc_key_in = analogRead(0);
  if (adc_key_in > 1000) { bpress = NONE; }
  else if (adc_key_in < 50) { bpress = RIGHT; }
  else if (adc_key_in < 195) { bpress = UP; }
  else if (adc_key_in < 380) { bpress = DOWN; }
  else if (adc_key_in < 555) { bpress = LEFT; }
  else if (adc_key_in < 790) { bpress = SELECT; }

  //--- Iterate Menu while Idle --------------------------------
  if(bpress == NONE) {
    if ( millis() - last_bpress > START_STATUS_ITERATE && 
         millis() - last_change > ITERATE_EVERY && !AlarmActive ) {
          
          SubIdx = MAIN;                                        // Switch to 'MAIN' menu items
          int i = idx;                                          // Mark current idx
          
          do {                                                  // Find the next menu item with an Alarm
            i++;                                                // Increment Menu Item
            if(i>NUM_MENU_ITEMS-1) {                            // When reaching the end of Menu Items
              i=0;                                              //    Reset to Menu-Item 0
              // If MONITOR = All; Change to next Pump-Controller
              if ( Menu[MONITOR].Option[Menu[MONITOR].Sub[MAIN].Value].Value == 1 ) {
                Menu[PUMPIDX].Sub[MAIN].Value++;
                if(Menu[PUMPIDX].Sub[MAIN].Value>Menu[PUMPIDX].LastOptionIdx) Menu[PUMPIDX].Sub[MAIN].Value = 0;    
                SetPump( Menu[PUMPIDX].Option[ Menu[PUMPIDX].Sub[MAIN].Value ].Value ); // SetPump(TRANSCEIER_ID)
              }
            }
            if(i == idx) break;                                 // break if we've made a full rotation
          } while (
            // Next item; IF not-a-unit-setting AND isn't selected OR doesn't have any Alarm ID's
            (Menu[i].Location & PUMP_PIN && !bitRead(Menu[i].Location, Menu[PUMPIDX].Sub[MAIN].Value + 8)) ||
            (Menu[i].Sub[LOALARM].ID == NULL && Menu[i].Sub[HIALARM].ID == NULL));
          
          idx=i;                        // --- Now; set new 'idx' to the one with an Alarm ID
          GetItem();                    // Get the value
          last_change=millis();         // Record time for next iteration
          LCD_display();                // Update the display
    }
  } else { last_bpress = millis(); }    // Record time for Start Iteration (Idle time)
  
  //--- Process Button Press -----------------------------------
  if(bpress !=NONE) {
    noTone(SBUZZ);AlarmActive = false;  // Turn off any alarms at button press
    
    //------- ( SELECT ) -------
    if (bpress == SELECT) {
      SetItem();
      SubIdx = MAIN;                    // Return to the MAIN items once a SET is done
    
    //------- (   UP   ) -------
    } else if (bpress == UP ) {
      if ( SubIdx == MAIN ) {                                                   // If viewing 'MAIN' Menu items
        idx--; if( idx<0 ) idx = NUM_MENU_ITEMS-1;                              // Change Menu Item to previous
        if ( Menu[idx].Location & PUMP_PIN && 
             !bitRead(Menu[idx].Location, Menu[PUMPIDX].Sub[MAIN].Value + 8)) { // If Pump but not this Pump
              idx--; }                                                          //    Skip this item
        if( idx<0 ) idx = NUM_MENU_ITEMS-1;                                     // Preform boundary check
        if ( Menu[idx].Sub[MAIN].State != VALID ) GetItem();                    // Retreive Value

      } else {                                                                  // ELSE
        Menu[idx].Sub[SubIdx].Value++;                                          // Change Value up
        if ( Menu[idx].LastOptionIdx>0 && 
             Menu[idx].Sub[SubIdx].Value > Menu[idx].LastOptionIdx) {
          Menu[idx].Sub[SubIdx].Value = 0; }                                    // Option - Boundary Check
      }

    //------- (  DOWN  ) -------
    } else if (bpress == DOWN) {
      if ( SubIdx == MAIN ) {                                                   // If viewing 'MAIN' menu items 
        idx++; if ( idx>NUM_MENU_ITEMS-1 ) idx=0;                               // Change Menu Item idx to NEXT item...
        if ( Menu[idx].Location & PUMP_PIN && 
             !bitRead(Menu[idx].Location, Menu[PUMPIDX].Sub[MAIN].Value + 8)) { // If Pump but not this Pump Skip
              idx++; }                                                          
        if ( idx>NUM_MENU_ITEMS-1 ) idx=0;                                      // Preform boundary check                                        
        if ( SubIdx == MAIN && Menu[idx].Sub[MAIN].State != VALID ) GetItem();
      } else {                                                                  // ELSE
        Menu[idx].Sub[SubIdx].Value--;                                          // Change Value down
        if ( Menu[idx].LastOptionIdx>0 && Menu[idx].Sub[SubIdx].Value < 0)         
          Menu[idx].Sub[SubIdx].Value = Menu[idx].LastOptionIdx;                   // Boundary Check
      }
    
    //------- (  RIGHT ) -------
    } else if (bpress == RIGHT) {
      SubIdx++;
      if ( SubIdx == SET && Menu[idx].Sub[SET].State != SETTABLE ) SubIdx++;    // Skip SET if not SETTABLE
      if ( SubIdx == LOALARM && Menu[idx].Sub[LOALARM].ID == NULL ) SubIdx++;   // Skip LOALARM if not Identified
      if ( SubIdx == HIALARM && Menu[idx].Sub[HIALARM].ID == NULL ) SubIdx++;   // Skip HIALARM if not Identified
      if ( SubIdx > HIALARM ) SubIdx = MAIN;
      if ( Menu[idx].Sub[MAIN].State == VALID ) {
        if ( SubIdx == SET && Menu[idx].Sub[SET].Value < 0 ) Menu[idx].Sub[SET].Value = Menu[idx].Sub[MAIN].Value; // Make SET value MAIN value instead of ERR
      } else {
        GetItem();
      }
    
    //------- (  LEFT  ) -------  
    } else if (bpress == LEFT) {
      SubIdx--;if (SubIdx<0) SubIdx=0;
      if ( SubIdx == SET && Menu[idx].Sub[SET].State != SETTABLE ) SubIdx--;    // Skip SET if not SETTABLE
      if ( SubIdx == LOALARM && Menu[idx].Sub[LOALARM].ID == NULL ) SubIdx--;   // Skip LOALARM if not identified
      if ( SubIdx == HIALARM && Menu[idx].Sub[HIALARM].ID == NULL ) SubIdx--;   // Skip HIALARM if not identified
      if ( SubIdx < MAIN ) SubIdx = MAIN;
      if ( SubIdx == MAIN && Menu[idx].Sub[MAIN].State != VALID ) GetItem();
    }

    // Update Display
    LCD_display();
  }
#endif
}
