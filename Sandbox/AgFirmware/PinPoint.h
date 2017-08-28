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

//-------------------------------------[ INCLUDES ]---------------------------------------------------------------------
#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif
#include "PeerIOSerialControl.h"
#include <LiquidCrystal.h>
#include "NewPing.h"                        // See https://bitbucket.org/teckel12/arduino-new-ping/wiki/Home
// Fix @ https://bitbucket.org/teckel12/arduino-new-ping/wiki/Multiple%20Definition%20of%20%22__vector_7%22%20Error 

#define DEBUG 1
//-------------------------------------[ PINMODES ]---------------------------------------------------------------------
//From \Program Files (x86)\Arduino\hardware\arduino\avr\cores\arduino\Arduino.h
//Extend Arduinos pinMode(uint8_t, uint8_t) definitions
//#define INPUT 0x0
//#define OUTPUT 0x1
//#define INPUT_PULLUP 0x2
#define INPUT_SONIC 0x3
#define OUTPUT_BUZZER 0x4
#define OUTPUT_PWM 0x5
#define CONTROLLER 0x6

//------------------------------------[ STATE ]-------------------------------------------------------------------------
//PeerIOSerialControl::enum PinStatus { ISOFF = 0, ISON = 1, ERR = 2, OKAY = 3 };
#ifndef _ESTATE
typedef enum eState : byte { WAIT, READY, SETTING, PAUSE, SETPAUSE, COMPLETE };
#define _ESTATE
#endif

//----------------------------------------------------------------------------------------------------------------------
class UserControl;      // Forward declaration
//----------------------------------------------------------------------------------------------------------------------
class PinPoint {
//----------------------------------------------------------------------------------------------------------------------
  public:
    PinPoint(uint8_t *_Device, uint8_t *_Pin, char *_DeviceName, LiquidCrystal *_LCD); // PinPoint Constructor

    void Mode(uint8_t _Mode);                                                   // No Name PinMode
    void Mode(uint8_t _Mode, char *_Name );                                     // Include Name PinMode
    void Mode(uint8_t _Mode, char *_Name, uint8_t _TrigPin, uint8_t _EchoPin);  // For INPUT_SONIC PinMode
    
/******************************************************************************************************************//**
 * @brief Request the Pin Input is Read.
 * @remarks Reading State is the State(); when State() is READY; Value can be gotten by GetRawValue() or GetModifiedValue()
 * @code
 *   exmaple code
 * @endcode
**********************************************************************************************************************/
    void            ReadValue(bool _ForceBlocking = false);
    
/******************************************************************************************************************//**
 * @brief Gets the Pin Input Read value un-modified
 * @remarks
 * @code
 *   exmaple code
 * @endcode
**********************************************************************************************************************/
    int             GetRawValue();

/******************************************************************************************************************//**
 * @brief Assigns a main-sketch function name to the Pin Value for displaying a modified user-desired value.
 * @param[in] ValueModifierCallback - A Main Sketch Function Name with an 'int FunctionName(int raw)' signature.
 * @return packetID
*  @remarks 
*  - Example uses would be changing raw sonic distance to inches or raw pressure to PSI.
*  - Raw values are still used behind the scene's only the displayed value is modified
 * <B>Example:</B>@code{.cpp}
 *    Pump.Pin(A3)->AttachValueModifier( PressureReadingToPSIFunction );
 * @endcode
**********************************************************************************************************************/
    void            AttachValueModifier(int (*_ValueModifierCallback)(int));

/******************************************************************************************************************//**
 * @brief Gets the Pin Input Read Value processed through any User-Assigned Value Modifier functions for Display
 * @remarks
 * @code
 *   exmaple code
 * @endcode
**********************************************************************************************************************/
    int             GetModifiedValue();
    
/******************************************************************************************************************//**
 * @brief Use a user-defined function in the main sketch to modify the raw value read from the arduino
 * @remarks
 * - Raw values are still used behind the scene's only the displayed value is modified
 * @code
 *   exmaple code
 * @endcode
**********************************************************************************************************************/
    int             ModifyValue(int _Value);                                    // In-Place Value Modifier

/******************************************************************************************************************//**
 * @brief Get the Value 'Status'; Signals the Pin Values status
 * @remarks An un-readable Pin will have an ERR status else it will be OKAY.  ControlPins use ISON and ISOFF.
 * @return ISOFF, ISON, ERR, OKAY
 * @code
 *   exmaple code
 * @endcode
**********************************************************************************************************************/
    PinStatus       Status();
    
/******************************************************************************************************************//**
 * @brief Gets the Pin Request 'State'; Signals the time-gap between a remote Pin requested and when its received.
 * @remarks Checks XBee communications and returns 'READY' if a Pin Value was received
 * @return WAIT, READY, SETTING, PAUSE, SETPAUSE, COMPLETE
 * @code
 *   Pin(1).ReadValue();                      // Launch a Pin Read Request
 *   while ( Pin(1).State() != READY ) {}  // Waits until Read Request has a reply
 *   Serial.print( Pin(1).GetRawValue() );
 * @endcode
**********************************************************************************************************************/
    eState          State();

/******************************************************************************************************************//**
 * @brief Sets the Pin 'State'; Signals the time-gap between a remote Pin requested and when its received.
 * @remarks Checks XBee communications and returns 'READY' if a Pin Value was received
 * @return WAIT, READY, SETTING, PAUSE, SETPAUSE, COMPLETE
 * @code
 *   Pin(1).ReadValue();                      // Launch a Pin Read Request
 *   while ( Pin(1).State() != READY ) {}  // Waits until Read Request has a reply
 *   Serial.print( Pin(1).GetRawValue() );
 * @endcode
**********************************************************************************************************************/
    void            State(eState _State);

/******************************************************************************************************************//**
 * @brief Drive the Pins output to a raw value
 * @remarks Optionally for 'ControlPins'; The Status can also be driven on a pin.
 * @code
 *   exmaple code
 * @endcode
**********************************************************************************************************************/
    void            SetTo(unsigned int _Value, PinStatus _Status = OKAY);
    
    //------ Public Pin Variables -------------
    char            *Name;                    // Name of the Pin
    byte            *Device;                  // DeviceID the Pin is On   ( 0 if Local Device )
    byte            *Pin;                     // Pin Number
    bool            IsOnOff = false;          // Numeric or On/Off
    char            *DeviceName = NULL;       // Text Name of the Device Pin is on
    
    //----- Link List Pointers -------------------
    PinPoint        *Next = NULL;
    PinPoint        *Prev = NULL;

    //----- Pin Controllers ----------------------
    UserControl     *FirstControl = NULL;
    UserControl     *CurrControl = NULL;

    static PeerIOSerialControl *XBee;
//----------------------------------------------------------------------------------------------------------------------        
  private:
  
    byte            PinMode = 0x0;
    eState          mState = COMPLETE;
    int             (*ValueModifierCallback)(int) = NULL;
    int             mValue = -1;
    int             mPacketID = -1;
    PinStatus       mStatus = ERR;
    LiquidCrystal   *LCD = NULL;
    void            Display();
    void            ApplyControls();
    

    // UltraSonic Reading
    NewPing         *Sonar = NULL;
    unsigned long   mWaitStart = 0;
    unsigned long   mLastPing = 0;
};
#endif
