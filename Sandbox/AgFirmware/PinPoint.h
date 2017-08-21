/******************************************************************************************************************//**
 * @file PinPoint.h
 * @brief Class definition for a I/O PinPoint on a Device ( 37-bytes per pin )
 * @remarks
 * - DeviceID(0) is reserved for 'Local' Pins ( i.e. ThisDevice Pins )
 * @authors 
 *    tgit23        8/2017       Original
 **********************************************************************************************************************/
#ifndef _PINPOINT_H
#define _PINPOINT_H

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#define DEBUG 1
//----------------------------------------------------------------------------------------------------------------------
//From \Program Files (x86)\Arduino\hardware\arduino\avr\cores\arduino\Arduino.h
//Extend Arduinos pinMode(uint8_t, uint8_t) definitions
//#define INPUT 0x0
//#define OUTPUT 0x1
//#define INPUT_PULLUP 0x2
#define INPUT_SONIC 0x3
#define OUTPUT_BUZZER 0x4
#define OUTPUT_PWM 0x5
#define CONTROLLER 0x6
//----------------------------------------------------------------------------------------------------------------------
#include "PeerIOSerialControl.h"
#include "NewPing.h"                        // See https://bitbucket.org/teckel12/arduino-new-ping/wiki/Home
// Fix @ https://bitbucket.org/teckel12/arduino-new-ping/wiki/Multiple%20Definition%20of%20%22__vector_7%22%20Error 
//----------------------------------------------------------------------------------------------------------------------
//PeerIOSerialControl::enum PinStatus { ISOFF = 0, ISON = 1, ERR = 2, OKAY = 3 };
#ifndef _ECONTROL
typedef enum eState : byte { WAIT, READY, SETTING, PAUSE, SETPAUSE, COMPLETE };
#define _ECONTROL
#endif

//----------------------------------------------------------------------------------------------------------------------
class UserControl;      // Forward declaration
//----------------------------------------------------------------------------------------------------------------------
class PinPoint {
//----------------------------------------------------------------------------------------------------------------------
  public:
    PinPoint(byte *_Device, byte *_Pin, char *_DeviceName);                     // PinPoint Constructor

    void Mode(uint8_t _Mode);                                                   // No Name
    void Mode(uint8_t _Mode, char *_Name );                                     // Include Name
    void Mode(uint8_t _Mode, char *_Name, uint8_t _TrigPin, uint8_t _EchoPin);  // For INPUT_SONIC
    
    //------ Read Input Pin -------------
    void            ReadValue(bool _ForceBlocking = false);
    int             GetRawValue();
    int             GetModifiedValue();
    void            AttachValueModifier(int (*_ValueModifierCallback)(int));    // Value Modify Callback
    int             ModifyValue(int _Value);                                    // In-Place Value Modifier
    PinStatus       GetStatus();                                                // ISOFF, ISON, ERR, OKAY
    eState          GetState();                                                 // WAIT, READY, SETTING, COMPLETE
    void            SetState(eState _State);

    //------ Set Output Pin -------------
    void            SetTo(unsigned int _Value, PinStatus _Status = OKAY);
    
    //------ Public Pin Variables -------------
    char            *Name;                    // Name of the Pin
    byte            *Device;                  // DeviceID the Pin is On   ( 0 if Local Device )
    byte            *Pin;                     // Pin Number
    bool            IsOnOff = false;          // Numeric or On/Off
    char            *DeviceName = NULL;       // Text Name of the Device Pin is on
    
    //--------------------------------------------
    PinPoint        *Next = NULL;
    PinPoint        *Prev = NULL;

    //------- Input Pin Controllers --------------    
    UserControl     *FirstControl = NULL;
    UserControl     *CurrControl = NULL;

    static PeerIOSerialControl *XBee;
//----------------------------------------------------------------------------------------------------------------------        
  private:
    byte            PinMode = 0x0;
    eState          State = COMPLETE;
    int             (*ValueModifierCallback)(int) = NULL;
    int             mValue = -1;
    int             mPacketID = -1;
    PinStatus       mStatus = ERR;
    void            ApplyControls();

    // UltraSonic Reading
    NewPing         *Sonar = NULL;
    unsigned long   mWaitStart = 0;
    unsigned long   mLastPing = 0;
};
#endif
