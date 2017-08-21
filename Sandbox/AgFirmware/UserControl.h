/******************************************************************************************************************//**
 * @file PinPoint.h
 * @brief UserControl ( Input->Controls(ThrewUserSetting)->Output )
 * @authors 
 *    tgit23        8/2017       Original
 **********************************************************************************************************************/
#ifndef _USERCONTROL_H
#define _USERCONTROL_H

#include "PinPoint.h"
#include "PID_v1.h"         // see https://playground.arduino.cc/Code/PIDLibrary
#include <EEPROM.h>
#include <LiquidCrystal.h>

//PeerIOSerialControl::enum PinStatus { ISOFF = 0, ISON = 1, ERR = 2, OKAY = 3 };
#ifndef _ECONTROL
typedef enum eState : byte { WAIT, READY, SETTING, PAUSE, SETPAUSE, COMPLETE };
#define _ECONTROL
#endif

typedef enum eControlType : byte { LCD_READONLY, SET_PIN, PID_SET, SET_CONTROLLER, TIE_PINS, LESS_THAN, GREATER_THAN, EQUAL_TO, NOT_EQUAL_TO };

#define DEBUG 1
class UserControl {
  public:
    UserControl(PinPoint *_InPin, LiquidCrystal *_LCD, char _ID );

    // Control Functions
    void                  Settable();
    void                  TieToPin(PinPoint *_OutPin);
    void                  PIDSetpoint(PinPoint *_OutPin, double Kp, double Ki, double Kd, int POn, int PDir, PinPoint *_ControlPin = NULL );
    void                  SetController();
    
    void                  LessThanSetpoint(PinPoint *_OutPin, PinPoint *_ControlPin = NULL );
    void                  GreaterThanSetpoint(PinPoint *_OutPin, PinPoint *_ControlPin = NULL );
    void                  EqualToSetpoint(PinPoint *_OutPin, PinPoint *_ControlPin = NULL );
    void                  NotEqualToSetpoint(PinPoint *_OutPin, PinPoint *_ControlPin = NULL );

    static char           OnControls[16];                   // Store 'ON' Controls for status display
    void                  Save();
    void                  SetPoint(int _Set);
    int                   SetPoint();
    void                  SetPointAdd(int AddValue);
    void                  IsOn(bool _IsOn);
    bool                  IsOn();
    void                  Start();
    void                  Apply(bool SetPin = false);
    
    eState                GetState();
    void                  SetState(eState _State);
    char                  ID = '?';
    
    eControlType          ControlType;
    PinStatus             Status = OKAY;
    
    UserControl           *Next = NULL;
    UserControl           *Prev = NULL;
    
    PinPoint              *InPin = NULL;
    PinPoint              *OutPin = NULL;
    PinPoint              *ControlPin = NULL;

  private:  
    eState                State = COMPLETE;
    int                   Setpoint = 0;             // Notice lower-case 'p'
    LiquidCrystal         *LCD = NULL;
    
    static unsigned int   NextEpromOffset;
    unsigned int          EpromOffset = 0;
    static byte           ObjectCount;
    byte                  ObjectIndex = 0;

    // PID-Control specific variables
    PID                   *PIDControl = NULL;
    double                PIDInput = 0;
    double                PIDOutput = 0;
    double                PIDSet = 0;
    double                Kp = 0;
    double                Ki = 0;
    double                Kd = 0;
    int                   POn = 0;
    int                   PDir = 0;
    unsigned long         PIDLastCompute = 0;

};
/*
class Test {
  public:
    Test(char _ID, PinPoint *_InPin) { Serial.print("Test Pin=");Serial.println(_InPin->Pin); ID = _ID; InPin = _InPin; }
    char ID;
    PinPoint *InPin = NULL;
    Test *Next = NULL;
    Test *Prev = NULL;
};
*/
#endif
