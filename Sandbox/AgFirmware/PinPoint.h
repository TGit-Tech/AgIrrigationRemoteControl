/******************************************************************************************************************//**
 * @file PinPoint.h
 * @brief Class definition for PinPoint used for I/O ControlType of a Pin on a Device
 * @authors 
 *    tgit23        8/2017       Original
 **********************************************************************************************************************/
#ifndef _PINPOINT_H
#define _PINPOINT_H

#include "PeerIOSerialControl.h"
#include "NewPing.h"                        // See https://bitbucket.org/teckel12/arduino-new-ping/wiki/Home
// Fix @ https://bitbucket.org/teckel12/arduino-new-ping/wiki/Multiple%20Definition%20of%20%22__vector_7%22%20Error 

#define DEBUG 1

//typedef enum PinStatus { ISOFF = 0, ISON = 1, WAIT = 2, ERR = 3, OKAY = 4 };
#ifndef _ECONTROL
typedef enum eControlType { SET_PIN, PID_SET, DIRECTLY, LESS_THAN, GREATER_THAN, EQUAL_TO, NOT_EQUAL_TO };
#define _ECONTROL
#endif
typedef enum ePinType { INPIN, INPINHIGH, OUTPIN, BUZZPIN, SONICPIN, SETTABLE, USERCONTROL };

class UserControl;      // Forward declaration
class PinPoint {
  public:
    PinPoint(uint8_t _Device, uint8_t _Pin, ePinType _PinType );
    PinPoint(uint8_t _Device, uint8_t _Pin, ePinType _PinType, char *_Name, char _ID );
    PinPoint(uint8_t _Device, uint8_t _Pin, ePinType _PinType, char *_Name, char _ID, uint8_t _TrigPin, uint8_t _EchoPin );
    void Controls(PinPoint* _OutPin, eControlType _ControlType, char _ID, PinPoint* _StorePin = NULL);
    
    static PeerIOSerialControl *XBee;
    static uint8_t ThisDeviceID;
      
    //------ Input ControlType -------------
    int GetRawValue();
    int GetModifiedValue();
    void SetTo(unsigned int _Value, PinStatus _Status = OKAY);
    
    PinStatus       GetStatus();                        // Protects Status for unintentional setting
    bool            UpdateAvailable();                  // Check for communication updates
    
    void            ReadValue(bool _ForceBlocking = false);                     // Returns PacketID or -1 for Received
    void            AttachValueModifier(int (*_ValueModifierCallback)(int));
    int             ModifyValue(int _Value);
    char            *Name;
    char            ID;
    uint8_t         Device;
    uint8_t         Pin;
    bool            IsOnOff = false;

    PinPoint        *Next = NULL;
    PinPoint        *Prev = NULL;
    UserControl     *FirstControl = NULL;
    UserControl     *CurrControl = NULL;
        
  private:
    int             (*ValueModifierCallback)(int) = NULL;
    int             mValue = -1;
    int             mPacketID = -1;
    unsigned long   mWaitStart = 0;
    unsigned long   mLastPing = 0;
    ePinType        PinType = INPIN;
    PinStatus       mStatus = ERR;

    NewPing         *Sonar = NULL;
    
};
#endif
