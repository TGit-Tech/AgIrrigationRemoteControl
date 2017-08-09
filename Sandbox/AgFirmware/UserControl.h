/******************************************************************************************************************//**
 * @file PinPoint.h
 * @brief Class definition for PinPoint used for I/O Control of a Pin on a Device
 * @authors 
 *    tgit23        8/2017       Original
 **********************************************************************************************************************/
#ifndef _USERCONTROL_H
#define _USERCONTROL_H

#include "PinPoint.h"
#include "PID_v1.h"         // see https://playground.arduino.cc/Code/PIDLibrary
#include <EEPROM.h>

//typedef enum PinStatus { ISOFF = 0, ISON = 1, WAIT = 2, ERR = 3, OKAY = 4 };
//typedef enum eControlType { SET_PIN, PID_SET, CONNECT_TO, LESS_THAN, GREATER_THAN, EQUAL_TO, NOT_EQUAL_TO };

#ifndef _ECONTROL
typedef enum eControlType { SET_PIN, PID_SET, DIRECTLY, LESS_THAN, GREATER_THAN, EQUAL_TO, NOT_EQUAL_TO };
#define _ECONTROL
#endif

#define DEBUG 1
class UserControl {
  public:
    UserControl(PinPoint * _InPin, eControlType _ControlType, PinPoint * _OutPin, char _ID, PinPoint * _StorePin = NULL);
    UserControl(PinPoint * _InPin, eControlType _ControlType, PinPoint * _OutPin, char _ID, double Kp, double Ki, double Kd, int POn, int PIDDirection, PinPoint * _StorePin = NULL);
    static char     OnControls[16];                   // Store 'ON' Controls for status display
    void            Save();
    void            SetPoint(int _Set);
    int             SetPoint();
    void            SetPointAdd(int AddValue);
    void            IsOn(bool _IsOn);
    bool            IsOn();
    void            Apply();
    
    PinStatus       Status = OKAY;
    UserControl     *Next = NULL;
    UserControl     *Prev = NULL;
    eControlType    ControlType;
    
  private:  
    static unsigned int NextEpromOffset;
    static int      ObjectCount;
    
    int             ObjectIndex = 0;
    char            ID = NULL;
    int             Setpoint = 0;
    unsigned int    EpromOffset = 0;
    
    
    PinPoint        *InPin = NULL;
    PinPoint        *OutPin = NULL;
    PinPoint        *StorePin = NULL;

    PID             *PIDControl = NULL;
    double          PIDInput = 0;
    double          PIDOutput = 0;
    double          PIDSetpoint = 0;
    double          Kp = 0;
    double          Ki = 0;
    double          Kd = 0;
    int             POn = 0;
    int             PIDDirection = 0;
    unsigned long   PIDLastCompute = 0;
};
#endif
