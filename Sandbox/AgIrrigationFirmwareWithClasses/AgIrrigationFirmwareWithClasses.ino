/******************************************************************************************************************//**
 * @brief  Arduino Sketch firmware to be uploaded onto the AgIrrigationRemoteControl Hand-Remote Device.
 * @see https://github.com/tgit23/AgIrrigationRemoteControl
 * @remarks Version 2017.06.17
 * @todo
 *  - Implement Firmata for Base/Desktop operation
 * @authors 
 *    tgit23        01/2017       Original
 *    tgit23        07/2017       Implemented keypad button interrupts and Non-Blocking functionality
**********************************************************************************************************************/
#include "PeerRemoteMenu.h"

#define REMOTE 0
#define CONTROLLER 1
#define CONTROLLER_W_LCD 2
//=====================================================================================================================
//------------------------------ SIMPLE USER CONFIGURATION SETTINGS ---------------------------------------------------
//=====================================================================================================================
#define XBEECONFIG                            0   // Configure the XBEE using XCTU Digi Software by setting this to 1
//#define FIRMWARE_FOR           CONTROLLER_W_LCD   // What type of device this firmware is for.
#define FIRMWARE_FOR                      REMOTE

#if FIRMWARE_FOR==REMOTE
  #define THISDEVICEID                          1   // Unique numeric (ID)entity for this Unit(1-15)
#else
  #define THISDEVICEID                         10   // Unique numeric (ID)entity for this Unit(1-15)
#endif
#define BUILD_VERSION                  20170706   // Release Version used to Build the Unit ( without the dots )


#define UTRASONIC_METER_INSTALLED             0   // 0=NO, 1=YES; Is an Ultrasonic meter installed on THIS device?

//=====================================================================================================================
//------------------------------ PIN SETTINGS -------------------------------------------------------------------------
//=====================================================================================================================
#if BUILD_VERSION>20170524 || FIRMWARE_FOR>=CONTROLLER   // Controllers always used 2,3 for Rx,Tx

                                            //---------[ Newest UnReleased Build Pins ]-----------------
  #define SS_TX_PIN 2                       // TX -> XBEE-DIN ( Closest to UNO )
  #define SS_RX_PIN 3                       // RX -> XBEE-DOUT ( Farthest from UNO )

  #if FIRMWARE_FOR==REMOTE
    #define SBUZZ 12                          // Buzzer Signal Pin (S)
    #define PBUZZ 13                          // Buzzer Power Pin (+)
  #else
    #define SBUZZ NOPIN
  #endif
  
#else                                       //---------[ REMOTE Build Release 2017.05.24 Pins ]---------
  #define SS_TX_PIN 11                      // TX -> XBEE-DIN ( Closest to UNO )
  #define SS_RX_PIN 12                      // RX -> XBEE-DOUT ( Farthest from UNO )
  #define SBUZZ 2                           // Buzzer Signal Pin (S)
#endif                                      
                                            //---------[ REMOTE BATTERY RESISTORS ]----------------------
#define BATT_R1_VINTOA1   1500              // Integer Value of BATTVOLT Resistor VIN -> A1 in KOhms
#define BATT_R2_A1TOGND   510               // Integer Value of BATTVOLT Resistor A1 -> GND in KOhms

#if FIRMWARE_FOR==REMOTE                    //---------[ LCD Pins ]---------------------------------------
  LiquidCrystal LCD(8, 9, 4, 5, 6, 7);      // Pins used by the LCD Keypad Shield on the Hand-Remote
#elif FIRMWARE_FOR==CONTROLLER_W_LCD
  LiquidCrystal LCD(12, 13, 8, 9, 10, 11);  // Pins used by the LCD Keypad Shield on the Pump-Controller
#endif

//============================== INITIALIZE ===========================================================================
SSoftwareSerial IOSerial(SS_RX_PIN,SS_TX_PIN);              // SSoftSerial for XBEE ( rxPin, txPin ) - allows interrupts
PeerIOSerialControl XBee(THISDEVICEID,IOSerial,Serial);   // XBee(ArduinoID, IOSerial, DebugSerial)
#if XBEECONFIG==0 && (FIRMWARE_FOR==REMOTE || FIRMWARE_FOR==CONTROLLER_W_LCD)
PeerRemoteMenu Menu(&XBee, &LCD, THISDEVICEID, SBUZZ);    // Interrupts in RemoteMenu disturbs XBee Config.
#endif
#if UTRASONIC_METER_INSTALLED==1
#include <NewPing.h>
NewPing sonar(ULTRASONIC_TRIG_PIN, ULTRASONIC_ECHO_PIN, ULTRASONIC_MAX_DIST);
unsigned long ulLastPing = 0;
#endif
/******************************************************************************************************************//**
 * @brief  Arduino Sketch Setup routine - Initialize the environment.
 * @remarks
 * - Setup() is called once; automatically when Arduino UNO is first powered on or reset.
 * - pin#10 INPUT Backlit shorting see http://forum.arduino.cc/index.php?topic=96747.0
**********************************************************************************************************************/
void setup(){

#if XBEECONFIG!=0
  LCD.clear();LCD.setCursor(0,0);
  LCD.print( "XBEE Config Mode" );  // Display XBEE Config Mode when Mode is active
#else

  //=====================================================================================================================
  //------------------------------ I/O PIN-MODE SETTINGS ----------------------------------------------------------------
  //=====================================================================================================================
  pinMode(10, INPUT);           // Fix for Q1-LCD Backlit shorting issue
  pinMode(A1, INPUT);           // A0 Controlled by LCD-Display library.
  pinMode(A2, INPUT_PULLUP);    // A1 is used by the Battery Level Indicator.
  pinMode(A3, INPUT_PULLUP);    // Keep all other Analog pins from floating
  pinMode(A4, INPUT_PULLUP);    // so 'PCINT1_vect' interrupt only triggers
  pinMode(A5, INPUT_PULLUP);    // when analog pin A0 changes

#if FIRMWARE_FOR==REMOTE        // Determin if the Buzzer needs I/O pins to drive (+) or (-) supplies
  #ifdef PBUZZ
    pinMode(PBUZZ,OUTPUT);digitalWrite(PBUZZ, HIGH);  // Supply Power to Buzzer (+) if needed
  #endif
  #ifdef GBUZZ
    pinMode(GBUZZ,OUTPUT);digitalWrite(GBUZZ, LOW);   // Supply Ground to Buzzer (-) if needed
  #endif
  pinMode(SBUZZ,OUTPUT);        // Buzzer Signal Pin (S)
#endif
  pinMode(SS_RX_PIN, INPUT);    // XBee DOUT Pin
  pinMode(SS_TX_PIN, OUTPUT);   // XBee DIN Pin

  //=====================================================================================================================
  //------------------------------ SYSTEM / MENU CONFIGURATION SETTINGS -------------------------------------------------
  //=====================================================================================================================
  /**************************************************************************
   * DEFINE DEVICES
   *  Menu.AddDevice ( uint8_t _Device, char *_Name );
   *  - Device: the devices THISDEVICEID
   *  - Name: the device name to be displayed
   * **'Menu.ThisDevicesID ( uint8_t _DeviceID );' must be called to know what 'ThisDevice' is.
  ***************************************************************************/
  Menu.AddDevice( 1, "Remote");
  Menu.AddDevice( 10,"Pump" );
  Menu.AddDevice( 11,"Gate" );

  /**************************************************************************
   * MENU ITEMS
   *  Add the Device and Pin of the Items to be monitored (i.e. Read Items)
  ***************************************************************************/
  MenuItem *battItem, *powerItem, *pressItem, *waterItem, *gateItem, *pidItem, pOnItem;
  //               AddMenuItem(         Name, ID, Device, Pin, [IsOnOff], [IsVirtual] )
#if FIRMWARE_FOR==REMOTE
  battItem  = Menu.AddMenuItem( "Battery",   'B',      1,  A1);
  pidItem = Menu.AddMenuItem(   "WG-Pid",    'I',     10,  70);
#endif
  powerItem = Menu.AddMenuItem( "Power",     'P',     10,   7,    true );
  waterItem = Menu.AddMenuItem( "Water",     'W',     10,  64);
  pressItem = Menu.AddMenuItem( "Pressure",  'R',     10,  A3 );
  gateItem =  Menu.AddMenuItem( "Gate",      'G',     11,  A4);

  /**************************************************************************
   * ATTACH SET
   * ** If no arguments are passed; The Read Device and Pin are used to SET.
   * - [DriveDevice]  : Which device the SET will control
   * - [DrivePin]     : The Pin on the Device the SET will control
  ***************************************************************************/  
  //         AttachSet( [DriveDevice], [DrivePin] );
  powerItem->AttachSet( );
  gateItem->AttachSet( );
#if FIRMWARE_FOR==REMOTE
  pidItem->AttachSet(            10,        70  );    // Control PID through virtual control
#endif
  /**************************************************************************
   * ATTACH PID ( Output )
   * - NOTE: AttachSet() if needed; MUST BE CALLED BEFORE AttachPID()
   * -       this order allows the SET to shut-down the PID when set manually.
   * - Kp: Determines how aggressively the PID reacts to the current amount of error (Proportional) (double >=0)
   * - Ki: Determines how aggressively the PID reacts to error over time (Integral) (double>=0)
   * - Kd: Determines how aggressively the PID reacts to the change in error (Derivative) (double>=0)
   * - POn: Either P_ON_E (Default) or P_ON_M. Allows Proportional on Measurement to be specified. 
   * - Direction: Either DIRECT or REVERSE
  ***************************************************************************/
  //         AttachPID( OutputItem,  Kp,  Ki, Kd,     POn,  Direction,  [SetVPin] )
#if FIRMWARE_FOR==CONTROLLER_W_LCD
  waterItem->AttachPID(   gateItem,   1,   2,  3,  P_ON_M,     DIRECT,        70 );
#endif




  /**************************************************************************
   * ATTACH ALARMS
   *  Create Alarms for every Menu-Item that should monitor boundaries   
   *  - [DriveDevice]   : Which 'device' to active when an Alarm boundary is crossed
   *    -- Keyword 'BUZZER' can be used to activate a local BUZZER
   *    ---- The BUZZER Pin is set on 'PeerRemoteMenu' Initialization
   *    ---- Any 'DrivePin' Assignment with BUZZER is IGNORED
   *    -- The 'BUZZER' and 'ULTRASONIC_DISTANCE_METER' cannot be used together
   * - [DrivePin]       : The Pin on the Device the Alarm will activate
   * - [DriveValue]     : The Value the Alarm will activate
   * - [HaltOnAlarm]    : Determines is all monitoring should stop when a boundary is crossed
   * - [ViolationCount] : Alarm will not trigger until the boundary is crossed this many times consecutevely
   * - [SetValueVPin]      : A Virtual Pin the Boundary Value will be stored on
   * - [ID]             : A single character to identify the Alarm boundary
   *                      - If no 'ID' ( NULL ) is assigned; then the Items-ID 
   *                      -- lowercase is assigned for LESS - EQUAL compares
   *                      -- uppercase is assigned for GREATER - NOTEQUAL compares
  ***************************************************************************/  
  //        AttachAlarm(               = NODEVICE,    = NOPIN,          = 0,       = false,              = 1, = NULL,  = NOPIN )
  //        AttachAlarm(   Compare, [DriveDevice], [DrivePin], [DriveValue], [HaltOnAlarm], [ViolationCount],   [ID], [SetVPin] )
#if FIRMWARE_FOR==REMOTE
  battItem->AttachAlarm(      LESS,        BUZZER,      NOPIN,         1000 );
#endif
  powerItem->AttachAlarm(    EQUAL,        BUZZER,      NOPIN,         1000 );
  powerItem->AttachAlarm( NOTEQUAL,        BUZZER,      NOPIN,         1000 );
  waterItem->AttachAlarm(     LESS,        BUZZER,      NOPIN,         1000 );
  waterItem->AttachAlarm(  GREATER,        BUZZER,      NOPIN,         1000 );
  pressItem->AttachAlarm(     LESS,        BUZZER,      NOPIN,         1000 );
  pressItem->AttachAlarm(  GREATER,        BUZZER,      NOPIN,         1000 );



  /**************************************************************************
   * ATTACH VALUE-MODIFIER
   *  Used to change a RAW value into a meaningful value for the display
   *  All callback functions must be in the form 'int FunctionName(int raw)'
   *    - Where 'raw' is the value read from the device pin.
   *    - return is the value to be displayed
   *  AttachValueModifier(FunctionName)
  ***************************************************************************/ 
  //AttachValueModifier(int (*_ValueModifierCallback)(int))
  battItem->AttachValueModifier(ModifyBatteryValue);
  pressItem->AttachValueModifier(ModifyPressureValue);

  //=============== Start Communication and Display =====================================================================
  XBee.Timeout(3000);           // Set the Timeout for XBEE communications
  IOSerial.begin(9600);         // Start UART Communications with the XBee->Module
  Serial.begin(9600);           // Start Serial Monitor for debug
#if FIRMWARE_FOR==REMOTE || FIRMWARE_FOR==CONTROLLER_W_LCD
  LCD.begin(16, 2);             // Start the LCD library
#endif

  //=============== Start Menu with Starting Item =======================================================================
  // Start Menu(Starting Item) - This Function must be called to start the Menu
  Menu.Start(powerItem);
#endif
}

/******************************************************************************************************************//**
 * @brief  Arduino Sketch Loop() routine
 * @remarks
 * - This function is called automatically over-and-over again by the Arduino
 * - Handles incoming XBee communications
 * - Handles button presses and LCD response updates
 * - Handles Menu iteratation during idle.
**********************************************************************************************************************/
void loop(){

#if XBEECONFIG!=0
  if ( IOSerial.available()>0 ) Serial.write(IOSerial.read());    // Forward Serial to XBEE for XBEE Config
  if ( Serial.available()>0 ) IOSerial.write(Serial.read());
#else
  #if FIRMWARE_FOR==REMOTE || FIRMWARE_FOR==CONTROLLER_W_LCD
    Menu.loop();                                                  // RemoteMenu takes care of Communications
  #else       
                                                      
  //---- Loop() Calls when NO_LCD is Installed -------------------------------------------------------------
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
}

//=====================================================================================================================
//------------------------------ VALUE MODIFIER FUNCTIONS -------------------------------------------------------------
//=====================================================================================================================
// Modify the raw analog input value into voltage represented by the voltage divider
int ModifyBatteryValue(int raw) {
  //return raw;
  return (long(raw)*long(1.75*BATT_R2_A1TOGND)/BATT_R1_VINTOA1);
}

// Show pressure as PSI ( pounds per square inch )
int ModifyPressureValue(int raw) {
  return raw;
  //return (int) ((raw - 97) * 0.2137);
}

