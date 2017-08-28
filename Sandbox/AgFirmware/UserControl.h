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
#define DEBUG 1
typedef enum eControlType : byte { LCD_READONLY, SET_PIN, PID_SET, SET_CONTROLLER, TIE_PINS, 
                                   LESS_THAN, GREATER_THAN, EQUAL_TO, NOT_EQUAL_TO };
class UserControl {
  public:
    UserControl(PinPoint *_InPin, LiquidCrystal *_LCD, char _ID );

    // Control Functions - Called via Device.Control(Device.Pin(InPin))->'ControlFunction'(Device.Pin(OutPin), etc...)
    void                  Settable();
    void                  TieToPin(PinPoint *_OutPin);
    void                  PIDSetpoint(PinPoint *_OutPin, double Kp, double Ki, double Kd, int POn, int PDir, PinPoint *_ControlPin = NULL );
    void                  SetController();
    
    void                  LessThanSetpoint(PinPoint *_OutPin, PinPoint *_ControlPin = NULL );
    void                  GreaterThanSetpoint(PinPoint *_OutPin, PinPoint *_ControlPin = NULL );
    void                  EqualToSetpoint(PinPoint *_OutPin, PinPoint *_ControlPin = NULL );
    void                  NotEqualToSetpoint(PinPoint *_OutPin, PinPoint *_ControlPin = NULL );
    ///////////////////////////
    
    void                  Save();                           // Save Setpoint and Status to EEPROM
    void                  SetPoint(int _Set);               // Set the SetPoint
    int                   SetPoint();                       // Get the SetPoint
    void                  SetPointAdd(int AddValue);        // Add or Subtract to the SetPoint
    void                  Status(PinStatus _Status);
    PinStatus             Status();
    void                  Apply(bool ManualApply = false);  // Apply the SetPoint Control SET_PIN requires ManualApply
    void                  Display();                        // Update the Display
    
    char                  ID = '?';                         // Single letter to identify this control
    eControlType          ControlType;                      // Type of Control
    
    static char           OnControls[16];                   // Store 'ON' Controls for status display
    
    UserControl           *Next = NULL;                     // Per-InPin Link List of Controls attached
    UserControl           *Prev = NULL;
    
    PinPoint              *InPin = NULL;                    // The Read Pin
    PinPoint              *OutPin = NULL;                   // The Drive Pin
    PinPoint              *ControlPin = NULL;               // The Controls SetPoint on a Store Pin for remote access to the Control

  private:  
    int                   Setpoint = 0;                     // Notice lower-case 'p'
    PinStatus             mStatus = OKAY;                   // PeerIOSerialControl::enum PinStatus { ISOFF = 0, ISON = 1, ERR = 2, OKAY = 3 };
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
#endif
