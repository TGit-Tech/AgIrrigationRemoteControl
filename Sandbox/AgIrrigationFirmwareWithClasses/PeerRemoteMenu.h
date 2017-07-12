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
#include <SSoftwareSerial.h>            //See
#include <PID_v1.h>

enum Button       { UP, DOWN, RIGHT, LEFT, SELECT, NONE };
enum eValModifier { RAW, PRESSURE, BATTVOLTS, ONOFF };
enum eCompare     { LESS, GREATER, EQUAL, NOTEQUAL, EMPTY };
enum eFunc        { MAIN, SET, ALARM };
enum eSetType     { READONLY, SETTABLE, PIDSET };

#define NOPIN     0xFF                // 255 (unit8_t) = 'NOPIN'
#define BUZZER    0xFF
#define READ_DEVICE_AND_PIN 0xFF;
#define DEBUG     1         // Set this to 1 for Serial DEBUGGING messages ( Firmware development use only )
#define BLOCKING  0         // Blocking mode (0) stalls screen till item is gotten, (1) releases screen
#define START_STATUS_ITERATE  30000     // Start iterating Menu-Items after idle for (ms)
#define ITERATE_EVERY         5000      // Iterate Menu-Items every (ms); when idle

volatile static unsigned long last_bpress_millis = 0;              // Track last button press time
static Button last_bpress = NONE;                                  // Store last button press for processing
static Button prev_bpress = NONE;                                  // Used to count ButtonHeld counter
static unsigned long Button_Debounce_ms = 500;
static int ButtonHeld = 0;                                         // Increment values by 10 when button is held


typedef struct uAlarm {
  char            ID = NULL;
  eCompare        Compare = LESS;
  int             Value = 0;
  uint8_t         StorePin = NOPIN;
  uint8_t         DriveDevice = 0;
  uint8_t         DrivePin = NOPIN;
  unsigned int    DriveValue = LOW;
  bool            IsOn = false;
  uAlarm          *Next = NULL;
  uAlarm          *Prev = NULL;
};

typedef struct uSet {
  int             Value = 0;
  uint8_t         DriveDevice = 0;
  uint8_t         DrivePin = NOPIN;
  uint8_t         ValueStorePin = NOPIN;
  PID             *SetPID;
};

class MenuItem {
  public:
    MenuItem();
    void AttachAlarm(char _ID, eCompare _Compare, uint8_t _StorePin = NOPIN, uint8_t _DriveDevice = 0, uint8_t _DrivePin = 0, int _DriveValue = 0);
    void AttachSet(uint8_t _DriveDevice = 0, uint8_t _DrivePin = NOPIN, uint8_t _ValueStorePin = NOPIN, PID *_SetPID = NULL );
  
    char            *Text;                  // The text to display on the LCD for this Menu item
    uint8_t         Device;               // The device the Menu Item is for
    uint8_t         Pin = NOPIN;            // Where to get/set the MAIN-Value; LOCAL, REMOTE, NOPIN, etc...
    unsigned int    Value = 0;
    bool            ValueValid = false;
    bool            IsOnOff = false;
    int             (*ValueModifierCallback)(int) = 0;
    unsigned int    EpromOffset = 0;
    uSet            *Set = NULL;
    uAlarm          *FirstAlarm = NULL;
    uAlarm          *CurrAlarm = NULL;
    MenuItem        *Next = NULL;
    MenuItem        *Prev = NULL;  
};


class PeerRemoteMenu {
  public:
    PeerRemoteMenu(PeerIOSerialControl *_XBee, LiquidCrystal *_LCD, uint8_t _BuzzerPin = 0 );
    MenuItem* AddMenuItem( char *_Text, uint8_t _Device, uint8_t _Pin, bool _IsOnOff );
    void AddDeviceName ( uint8_t _Device, char *_Name );
    void SetStartingItem ( MenuItem *Item );
    
    int ValueModify(int Index, int SubIndex, int AddBy = 0);
    void loop();
    static void ButtonCheck(int adc_value);

  private:
    eFunc Func = MAIN;
    char *Devices[16];
    bool AlarmActive = false;                                   // Track if an Active Alarm is present
    bool bIterating = false;                                    // Alarm only active while iterating the menu
    int PacketID = -1;                                          // For non-blocking communications
    unsigned long wait_reply = 0;                               // Track non-blocking reply time
    unsigned long last_iter_millis = 0;                         // Track last status iteration time
    uint8_t BuzzerPin = 0;
    void EEPROMSet(int i = -1);
    void EEPROMGet(int i = -1);
    void GetItem(int i = -1);
    void SetItem(int i = -1);
    void CheckAlarms(int i = -1);
    LiquidCrystal *LCD;
    PeerIOSerialControl *XBee;
    void LCD_display();
    MenuItem *FirstItem = NULL;
    MenuItem *CurrItem = NULL;

};


#endif
