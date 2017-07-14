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
#ifndef _PEERREMOTEMENU_h
#define _PEERREMOTEMENU_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif


#include <LiquidCrystal.h>
#include <EEPROM.h>
#include <PeerIOSerialControl.h>        //See https://github.com/tgit23/PeerIOSerialControl
#include <SSoftwareSerial.h>            //See https://github.com/tgit23/SSoftwareSerial
#include <PID_v1.h>                     //See https://github.com/br3ttb/Arduino-PID-Library/

enum Button       { UP, DOWN, RIGHT, LEFT, SELECT, NONE };
enum eCompare     { LESS, GREATER, EQUAL, NOTEQUAL, EMPTY };
enum eFunc        { MAIN, SET, ALARM };
enum eSetType     { SET_MENU_ITEM_DEVICE_AND_PIN, SET_DIRECTLY, SET_WITH_PID };

#define NOPIN                 0xFF      // A non-usable value to signify 'NOPIN' has been assigned
#define BUZZER                0xFF      // A non-usable value to signify the ALARM drive device is a local BUZZER
#define VALUE_ERR             -1        // A Value that marks an 'error'
#define DEBUG                 1         // Set this to 1 for Serial DEBUGGING messages ( Firmware development use only )
#define BLOCKING              0         // Blocking mode (0) stalls screen till item is gotten, (1) releases screen
#define START_STATUS_ITERATE  30000     // Start iterating Menu-Items after idle for (ms)
#define ITERATE_EVERY         5000      // Iterate Menu-Items every (ms); when idle
#define BUTTON_DEBOUNCE_MS    300       // How close in milliseconds two button presses will register

volatile static unsigned long last_bpress_millis = 0;   // Track last button press time
static Button last_bpress = NONE;                       // Store last button press for processing by the loop()
static Button prev_bpress = NONE;                       // Used with 'ButtonHeld' to see if same button is registered twice
static int ButtonHeld = 0;                              // Used to count how long a Button is held by counting


typedef struct uAlarm {
  char            ID = NULL;
  eCompare        Compare = LESS;
  int             Value = 0;
  uint8_t         StorePin = NOPIN;
  uint8_t         DriveDevice = 0;
  uint8_t         DrivePin = NOPIN;
  unsigned int    DriveValue = LOW;
  bool            IsOn = false;
  bool            HaltOnAlarm = false;
  uint8_t         ViolationCount = 1;
  uint8_t         Violations = 0;
  unsigned int    EpromOffset = 0;
  uAlarm          *Next = NULL;
  uAlarm          *Prev = NULL;
};

typedef struct uSet {
  eSetType        SetType= 0;
  int             Value = 0;
  uint8_t         DriveDevice = 0;
  uint8_t         DrivePin = NOPIN;
  uint8_t         ValueStorePin = NOPIN;
  unsigned int    EpromOffset = 0;
  PID             *SetPID = NULL;
  double          *PIDInput = 0;
  double          *PIDOutput = 0;
  double          *PIDSetpoint = 0;
};

class MenuItem {
  public:
    MenuItem();
    void AttachAlarm(char _ID, eCompare _Compare, uint8_t _DriveDevice = 0, uint8_t _DrivePin = 0, int _DriveValue = 0, bool _HaltOnAlarm = false, uint8_t _ViolationCount = 1, uint8_t _StorePin = NOPIN );
    void AttachSet(eSetType _SetType, uint8_t _DriveDevice = 0, uint8_t _DrivePin = NOPIN, uint8_t _ValueStorePin = NOPIN, double _PIDKp = 0, double _PIDKi = 0, double _PIDKd = 0, int _PIDPOn = 0, int _PIDDirection = 0 );
  
    char            *Text = NULL;                       // The text to display on the LCD for this Menu item
    uint8_t         Device = 0;                         // The device-ID the Menu Item reads
    uint8_t         Pin = NOPIN;                        // The Pin of the 'device' the Menu Item reads from
    int             Value = VALUE_ERR;                  // The last READ value of the 'Device' 'Pin' 0xFFFF is ERR
    bool            IsOnOff = false;                    // Determines if the Value is just an ON or OFF value
    int             (*ValueModifierCallback)(int) = 0;  // Allows sketch functions to modify the RAW value read into something meaningful
    unsigned int    EpromOffset = 0;                    // Location in EEPROM the Set and Alarm values are stored
    uSet            *Set = NULL;                        // Gives the Menu Item an option to SET the target value
    uAlarm          *FirstAlarm = NULL;                 // Tracks the first attached alarm
    uAlarm          *CurrAlarm = NULL;                  // Tracks the Current alarm being displayed on the LCD
    MenuItem        *Next = NULL;                       // Link-List Next Menu Item
    MenuItem        *Prev = NULL;                       // Link-List Previous Menu Item
};


class PeerRemoteMenu {
  public:
    PeerRemoteMenu(PeerIOSerialControl *_XBee, LiquidCrystal *_LCD, uint8_t _BuzzerPin = 0 );
    MenuItem* AddMenuItem( char *_Text, uint8_t _Device, uint8_t _Pin, bool _IsOnOff );
    void AddDevice ( uint8_t _Device, char *_Name );
    void SetStartingItem ( MenuItem *Item );
    void ThisDevicesID ( uint8_t _DeviceID );
    void ThisDevicesID ();
    int ValueModify(int Index, int SubIndex, int AddBy = 0);
    void loop();
    static void ButtonCheck(int adc_value);

  private:
    eFunc Func = MAIN;
    char *Devices[16];
    uint8_t ThisDeviceID = 0;
    char ActiveAlarm = 0;                                       // Track if an Active Alarm is present
    bool AlarmHalt = false;                                     // Track when an Alarm should Halt the iteration
    bool bIterating = false;                                    // Alarm only active while iterating the menu
    int PacketID = -1;                                          // For non-blocking communications
    unsigned long wait_reply = 0;                               // Track non-blocking reply time
    unsigned long last_iter_millis = 0;                         // Track last status iteration time
    uint8_t BuzzerPin = 0;
    
    void EEPROMSet(int i = -1);
    void EEPROMGet(int i = -1);
    void GetItem(MenuItem *Item);
    void SetPin(uint8_t _DriveDevice, uint8_t _DrivePin, int _Value, uint8_t _ValueStorePin = NOPIN );
    void CheckAlarmsUpdatePID(MenuItem *Item);
    LiquidCrystal *LCD;
    PeerIOSerialControl *XBee;
    void LCD_display();
    MenuItem *FirstItem = NULL;
    MenuItem *CurrItem = NULL;

};


#endif
