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
#include <EEPROM.h>
#include <PID_v1.h>                     //See https://github.com/br3ttb/Arduino-PID-Library/
#include "PeerIOSerialControl.h"        //See https://github.com/tgit23/PeerIOSerialControl

#ifndef _MENUFUNCTION_h
#define _MENUFUNCTION_h

typedef enum eFunctionType { LESS_THAN_ALARM, GREATER_THAN_ALARM, EQUAL_TO_ALARM, NOT_EQUAL_TO_ALARM, SET_PIN, PID_SETPOINT };

// Options for DriveDevice & DrivePin ( uint8_t )
#define NODEVICE              0
#define NOPIN                 0xFF      // A non-usable value to signify 'NOPIN' has been assigned
#define BUZZER                0xFF      // A non-usable value to signify the ALARM drive device is a local BUZZER

// Options for DriveValue ( int )
#define USER_SELECT_NUMERIC     -1      // Signifies the User Selectable DriveValue is a number ( 0 - 1023 )
#define USER_SELECT_ONOROFF     -2      // Signifies the Seletable DriveValue is either On-or-Off
#define PID_OUTPUT              -3

class MenuFunction {
  public:
    MenuFunction(eFunctionType _FunctionType, uint8_t _DriveDevice = NODEVICE, uint8_t _DrivePin = NOPIN, int _DriveValue = 0, uint8_t _ValueOnPin = NOPIN, char _ID = NULL, bool _HaltOnAlarm = false, uint8_t _ViolationCount = 1 );
    int SetEpromOffset(unsigned int _EpromOffset);
    void Select();
    
    MenuFunction          *Next = NULL;
    MenuFunction          *Prev = NULL;
    PeerIOSerialControl   *XBee = NULL;
    PinStatus             Status = IsOff;             // A local non-pin mimic of PeerIOSerialControl's Value Status Protocol
    bool                  IsActive = false;
    
  private:
    eFunctionType   FunctionType = 0;
    unsigned int    EpromOffset = 0;
    uint8_t         DriveDevice = 0;
    uint8_t         DrivePin = NOPIN;
    uint8_t         ValueOnPin = NOPIN;
    
    // Alarm Specific Variables
    unsigned int    DriveValue = LOW;
    int             Value = 0;
    char            ID = NULL;
    bool            HaltOnAlarm = false;
    uint8_t         ViolationCount = 1;
    uint8_t         Violations = 0;
    
    // PID Special Variables
    PID             *OPID = NULL;
    double          Input = 0;
    double          Output = 0;
    double          Setpoint = 0;
};
#endif
