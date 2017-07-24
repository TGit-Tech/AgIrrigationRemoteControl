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
#ifndef _MENUITEM_h
#define _MENUITEM_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include "PeerIOSerialControl.h"        //See https://github.com/tgit23/PeerIOSerialControl
#include "MenuFunction.h"

#define VALUE_ERR             -1        // A Value that marks an 'error'
#define VALUE_WAIT            -2        // A Value waiting for a Reply Packet
#define DEBUG                 1         // Set this to 1 for Serial DEBUGGING messages ( Firmware development use only )
#define BLOCKING              0         // Blocking mode (1) stalls screen till item is gotten, (0) releases screen
#define START_STATUS_ITERATE  30000     // Start iterating Menu-Items after idle for (ms)
#define ITERATE_EVERY         5000      // Iterate Menu-Items every (ms); when idle
#define BUTTON_DEBOUNCE_MS    400       // How close in milliseconds two button presses will register
#define ISON                  0x2000    // Bit-13; of Raw Value determines ON/OFF status
#define VALUE                 0x1FFF    // Clear Bit-13 for Value Only
//enum eCompare     { LESS, GREATER, EQUAL, NOTEQUAL, EMPTY };

class MenuItem {
  public:
    MenuItem(PeerIOSerialControl *_XBee, uint8_t _ThisDeviceID, char *_Name, char _ID, uint8_t _Device, uint8_t _Pin, bool _IsOnOff );
    MenuItem(); // For pointers
    
    void AttachFunction(eFunctionType _FunctionType, uint8_t _DriveDevice = NODEVICE, uint8_t _DrivePin = NOPIN, int _DriveValue = USER_SELECT_NUMERIC, uint8_t _ValueOnPin = NOPIN, char _ID = NULL, bool _HaltOnAlarm = false, uint8_t _ViolationCount = 1 );
    void AttachValueModifier(int (*_ValueModifierCallback)(int));
    void Get();
    void GetStatusLine(char *Status);
    void Check();

    void AttachSet(uint8_t _DriveDevice = NODEVICE, uint8_t _DrivePin = NOPIN, uint8_t _DriveOnOffVPin = NOPIN );
    void AttachPID(MenuItem *_OutputItem, double _Kp, double _Ki, double _Kd, int _POn, int _Direction, uint8_t _ValueOnPin = NOPIN );
    

    int                   (*ValueModifierCallback)(int) = NULL;   // Allows sketch functions to modify the RAW value read into something meaningful

    MenuFunction          *FirstFunction = NULL;                  // Tracks the first attached alarm
    MenuFunction          *CurrFunction = NULL;                   // Tracks the Current alarm being displayed on the LCD
    MenuItem              *Next = NULL;                           // Link-List Next Menu Item
    MenuItem              *Prev = NULL;                           // Link-List Previous Menu Item
  private:
    PeerIOSerialControl   *XBee;
    char                  *Name = NULL;                           // The text to display on the LCD for this Menu item
    char                  ID = NULL;
    uint8_t               Device = 0;                             // The device-ID the Menu Item reads
    uint8_t               Pin = NOPIN;                            // The Pin of the 'device' the Menu Item reads from
    bool                  IsOnOff = false;
    int                   Value = VALUE_ERR;                      // The last READ value of the 'Device' 'Pin' 0xFFFF is ERR
    unsigned long wait_reply = 0;
    int                   PacketID = -1;
    uint8_t               ThisDeviceID = 0;
};
#endif
