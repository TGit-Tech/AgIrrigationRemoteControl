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
enum eFunc        { MAIN, SETPID, SET, ALARM };

#define NODEVICE              0
#define NOPIN                 0xFF      // A non-usable value to signify 'NOPIN' has been assigned
#define BUZZER                0xFF      // A non-usable value to signify the ALARM drive device is a local BUZZER
#define VALUE_ERR             -1        // A Value that marks an 'error'
#define VALUE_WAIT            -2        // A Value waiting for a Reply Packet
#define DEBUG                 1         // Set this to 1 for Serial DEBUGGING messages ( Firmware development use only )
#define BLOCKING              0         // Blocking mode (1) stalls screen till item is gotten, (0) releases screen
#define START_STATUS_ITERATE  30000     // Start iterating Menu-Items after idle for (ms)
#define ITERATE_EVERY         5000      // Iterate Menu-Items every (ms); when idle
#define BUTTON_DEBOUNCE_MS    300       // How close in milliseconds two button presses will register
#define ISONBIT               13

volatile static unsigned long last_bpress_millis = 0;   // Track last button press time
static Button last_bpress = NONE;                       // Store last button press for processing by the loop()
static Button prev_bpress = NONE;                       // Used with 'ButtonHeld' to see if same button is registered twice
static int ButtonHeld = 0;                              // Used to count how long a Button is held by counting
static uint8_t ThisDeviceID = 0;
    
class MenuItem;
class uSetPID {
  public:
    MenuItem        *OutputItem = NULL;
    PID             *OPID = NULL;
    double          Input = 0;
    double          Output = 0;
    double          Setpoint = 0;
    int             SetVPin = NOPIN;
    bool            IsOn = false;
    unsigned int    EpromOffset = 0;
  private:
  
};

class uSet {
  public:
    int             Value = 0;
    uint8_t         DriveDevice = 0;
    uint8_t         DrivePin = NOPIN;
    unsigned int    EpromOffset = 0;
    uint8_t         SetVPin = NOPIN;
    bool            IsOn = false;
    uSetPID         *AttachedPID = NULL;
  private:
};

class uAlarm {
  public:
    char            ID = NULL;
    eCompare        Compare = LESS;
    int             Value = 0;
    uint8_t         SetVPin = NOPIN;
    uint8_t         DriveDevice = 0;
    uint8_t         DrivePin = NOPIN;
    unsigned int    DriveValue = LOW;
    bool            IsOn = false;
    bool            IsActive = false;
    bool            HaltOnAlarm = false;
    uint8_t         ViolationCount = 1;
    uint8_t         Violations = 0;
    unsigned int    EpromOffset = 0;
    uAlarm          *Next = NULL;
    uAlarm          *Prev = NULL;
  private:
  
};

class MenuItem {
  public:
    MenuItem();
    void AttachAlarm(eCompare _Compare, uint8_t _DriveDevice = NODEVICE, uint8_t _DrivePin = NOPIN, int _DriveValue = 0, bool _HaltOnAlarm = false, uint8_t _ViolationCount = 1, char _ID = NULL, uint8_t _SetVPin = NOPIN );
    void AttachSet(uint8_t _DriveDevice = NODEVICE, uint8_t _DrivePin = NOPIN );
    void AttachPID(MenuItem *_OutputItem, double _Kp, double _Ki, double _Kd, int _POn, int _Direction, uint8_t _SetVPin = NOPIN );
    void AttachValueModifier(int (*_ValueModifierCallback)(int));
    
    char            *Name = NULL;                           // The text to display on the LCD for this Menu item
    char            ID = NULL;
    uint8_t         Device = 0;                             // The device-ID the Menu Item reads
    uint8_t         Pin = NOPIN;                            // The Pin of the 'device' the Menu Item reads from
    int             Value = VALUE_ERR;                      // The last READ value of the 'Device' 'Pin' 0xFFFF is ERR
    bool            IsOnOff = false;                        // Determines if the Value is just an ON or OFF value
    int             (*ValueModifierCallback)(int) = NULL;   // Allows sketch functions to modify the RAW value read into something meaningful
    int             PacketID = -1;                          // For non-blocking communications
    uSet            *Set = NULL;                            // Gives the Menu Item an option to SET the target value
    uSetPID         *SetPID = NULL;                         // Menu Item read -> PID -> Menu Item Write
    uAlarm          *FirstAlarm = NULL;                     // Tracks the first attached alarm
    uAlarm          *CurrAlarm = NULL;                      // Tracks the Current alarm being displayed on the LCD
    MenuItem        *Next = NULL;                           // Link-List Next Menu Item
    MenuItem        *Prev = NULL;                           // Link-List Previous Menu Item
};

class PeerRemoteMenu {
  public:
    PeerRemoteMenu(PeerIOSerialControl *_XBee, LiquidCrystal *_LCD, uint8_t _ThisDeviceID, uint8_t _BuzzerPin = NOPIN );
    MenuItem* AddMenuItem( char *_Name, char _ID, uint8_t _Device, uint8_t _Pin, bool _IsOnOff = false );
    void AddDevice ( uint8_t _Device, char *_Name );        // Add System Devices
    void Start( MenuItem *StartItem );                      // Initialize Menu Setup and Set Starting Item
    static void ButtonCheck(int adc_value);                 // Button check must be public for ISR()
    void loop();

  private:
    // Class Variables
    eFunc Func = MAIN;
    char *Devices[16];
    bool AlarmHalt = false;                                     // Track when an Alarm should Halt the iteration
    bool bIterating = false;                                    // Alarm only active while iterating the menu
    unsigned long wait_reply = 0;                               // Track non-blocking reply time
    unsigned long last_iter_millis = 0;                         // Track last status iteration time
    uint8_t BuzzerPin = NOPIN;                                  // Track which pin the BUZZER is signalled on

    // Object pointers
    LiquidCrystal *LCD;
    PeerIOSerialControl *XBee;
    MenuItem *FirstItem = NULL;
    MenuItem *CurrItem = NULL;
        
    // Function prototypes
    void EEPROMSet(unsigned int Offset, int Value, int OnOff = -1);
    int EEPROMGet(unsigned int Offset, bool *IsOn = NULL);
    void GetItem(MenuItem *Item);
    void SetPin(uint8_t _DriveDevice, uint8_t _DrivePin, int _Value );
    void CheckValue(MenuItem *Item);
    void LCD_display();
};

#endif
