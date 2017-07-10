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
enum eFunc        { MAIN, SET, LOALARM, HIALARM };
enum eSetType     { READONLY, SETTABLE, PIDSET };
#define NOPIN     0xFF                // 255 (unit8_t) = 'NOPIN'
#define BUZZER    0xFF

/*
 * Kp: Determines how aggressively the PID reacts to the current amount of error (Proportional) (double >=0)
Ki: Determines how aggressively the PID reacts to error over time (Integral) (double>=0)
Kd: Determines how aggressively the PID reacts to the change in error (Derivative) (double>=0)
POn: Either P_ON_E (Default) or P_ON_M. Allows Proportional on Measurement to be specified. 
#define P_ON_M 0
#define P_ON_E 1
 */
typedef struct uPIDController {
  uint8_t         DriveDevice = 0;
  uint8_t         DrivePin = NOPIN;
  double          Kp;
  double          Ki;
  double          Kd;
  int             POn;
};

typedef struct uAlarm {
  char            ID = NULL;
  eCompare        Compare = LESS;
  int             Value = 0;
  uint8_t         ValueStorePin = NOPIN;
  uint8_t         DriveDevice = 0;
  uint8_t         DrivePin = NOPIN;
  unsigned int    DriveValue = LOW;
  bool            Buzz = true;
  bool            IsOn = false;
};

typedef struct MenuItem {
  char            *Text;                  // The text to display on the LCD for this Menu item
  uint8_t         Device;               // The device the Menu Item is for
  uint8_t         Pin = NOPIN;            // Where to get/set the MAIN-Value; LOCAL, REMOTE, NOPIN, etc...
  unsigned int    Value = 0;
  bool            ValueValid = false;
  eValModifier    ValueModifier = RAW;    // Select value modifying equations in LCD_display()
  eSetType        SetType = READONLY;
  unsigned int    SetValue = 0;
  uint8_t         SetStorePin = NOPIN;
  unsigned int    EpromOffset = 0;
  uAlarm          *LoAlarm;
  uAlarm          *HiAlarm;
  uPIDController  *PIDController;
  MenuItem        *Next;
  MenuItem        *Prev;
};  

class PeerRemoteMenu {
  public:
    PeerRemoteMenu(PeerIOSerialControl *_XBee, LiquidCrystal *_LCD, uint8_t _BuzzerPin = 0 );
    MenuItem* AddMenuItem( char *_Text, uint8_t _Device, uint8_t _Pin, eValModifier _Modifier, eSetType _SetType, uint8_t SetStorePin = NOPIN );
    void AddLoAlarm( MenuItem *Item, char _ID, eCompare _Compare, uint8_t _DriveDevice, uint8_t _DrivePin, unsigned int _DriveValue, uint8_t ValueStorePin = NOPIN );
    void AddHiAlarm( MenuItem *Item, char _ID, eCompare _Compare, uint8_t _DriveDevice, uint8_t _DrivePin, unsigned int _DriveValue, uint8_t ValueStorePin = NOPIN );
    void AddPIDController( MenuItem *Item, uint8_t _DriveDevice, uint8_t _DrivePin, double _Kp, double _Ki, double _Kd, int _POn);
    void AddDeviceName ( uint8_t _Device, char *_Name );
    void SetStartingItem ( MenuItem *Item );
    
    int ValueModify(int Index, int SubIndex, int AddBy = 0);
    void loop();
    static void ButtonCheck(int adc_value);

  private:
    eFunc Func;
    uint8_t BuzzerPin = 0;
    void NextFunc();
    void PrevFunc();
    void NextItem();
    void PrevItem();
    void EEPROMSet(int i = -1);
    void EEPROMGet(int i = -1);
    void GetItem(int i = -1);
    void SetItem(int i = -1);
    void CheckAlarms(int i = -1);
    LiquidCrystal *LCD;
    PeerIOSerialControl *XBee;
    void LCD_display();
};


#endif
