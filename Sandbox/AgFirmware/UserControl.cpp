/******************************************************************************************************************//**
 * @file UserControl.cpp
 * @brief Manages Input->Output Control Processes ( Read-InPin-Value > Control Process > Set-OutPin-Value )
 * @authors 
 *    tgit23        8/2017       Original
 **********************************************************************************************************************/
#include "UserControl.h"

// Initialize Static Variables
unsigned int  UserControl::NextEpromOffset = 0;
char          UserControl::OnControls[16] = {};
byte          UserControl::ObjectCount = 0;

#if DEBUG>0                             // Activate Debug Messages ( DEBUG defined in PeerRemoteMenu.h )
  #define DBL(x) Serial.println x
  #define DBFL(x) Serial.println F(x)
  #define DB(x) Serial.print x
  #define DBF(x) Serial.print F(x)
  #define DBC Serial.print(F(", "))
#else                                   // ELSE - Clear Debug Messages
  #define DBL(x)
  #define DBFL(x)
  #define DB(x)
  #define DBF(x)
  #define DBC  
#endif

/******************************************************************************************************************//**
 * @brief Create a new Control ( Constructor )
 * @remarks
 * @code
 *   exmaple code
 * @endcode
**********************************************************************************************************************/
UserControl::UserControl(PinPoint *_InPin, LiquidCrystal *_LCD, char _ID ) {
  if ( _InPin == NULL ) return;             // Requires InPin
  InPin = _InPin;  LCD = _LCD;  ID = _ID;   // Assign arguments
  
  DBF(("UserControl::UserControl("));DB((InPin->DeviceName));DBF((":"));DB((InPin->Name));DBF((":"));
  DB((ID));DBF((")"));
  
  EpromOffset = NextEpromOffset;                          // Assign EEPROM Offset to store User Setpoints
  DB((F(" @EpromOffset=")));DB((EpromOffset));
  byte LoByte = EEPROM.read( EpromOffset );               // Read Eprom when Offset is assigned
  byte HiByte = EEPROM.read( EpromOffset + 1 );
  if ( HiByte > 0x3F ) {                                  // Initialize EPROM to 0x00 if out-of-bounds
    EEPROM.update( EpromOffset, 0x00 );
    EEPROM.update( EpromOffset + 1, 0x00 );
    Setpoint = 0;
  } else {                                                // ELSE assign the Eprom Read values
    Status = ((HiByte & 0x30) >> 4);                      // Bit 12 & 13 make Status
    Setpoint = (int)(((HiByte & 0x0F) << 8) | LoByte);    // 0x0FFF 0->11-bits for value
  }
  NextEpromOffset = EpromOffset + 2;                      // Record the next available offset

  ObjectIndex = ObjectCount;                              // Assign an OnControls[ObjectIndex] & set to ID if ON
  if ( ObjectCount<15 ) ObjectCount++;                    // Stay within boundaries
  if ( Status == ISON ) OnControls[ObjectIndex] = ID;
  DB((F(" @ObjectIndex=")));DBL((ObjectIndex));
}

/******************************************************************************************************************//**
 * @brief  Save the Controls 'Setpoint' to Eprom and VirtualPins
 * @remarks
 *  @code
 *    exmaple code
 *  @endcode
**********************************************************************************************************************/
void UserControl::Save() {
  DBFL(("UserControl::Save()"));
    
  // EEPROM - Save; Store the Value and Status as two bytes in Eprom
  byte LoByte = ((Setpoint >> 0) & 0xFF);
  byte HiByte = ((Status << 4 & 0x30) | (Setpoint >> 8 & 0x0F));
  EEPROM.update( EpromOffset, LoByte );
  EEPROM.update( (EpromOffset + 1), HiByte );
  DB((F("EEPROM.update( ")));DB((EpromOffset));DBC;DB((LoByte, HEX));DBL((F(")")));
  DB((F("EEPROM.update( ")));DB((EpromOffset + 1));DBC;DB((HiByte, HEX));DBL((F(")")));

  // VirtualPin - Save; If 'ControlPin' Exists - Save to a Virtual Pin
  if ( ControlPin != NULL ) { ControlPin->SetTo(Setpoint, Status); }
}

//----------------------------------------------------------------------------------------------------------------------
//                      [[ CONTROLLERS ]]
//----------------------------------------------------------------------------------------------------------------------
/******************************************************************************************************************//**
 * @brief Control type 'Settable'
 * @remarks
 * @code
 *   exmaple code
 * @endcode
**********************************************************************************************************************/
void UserControl::Settable() {
  if ( InPin == NULL ) return;
  ControlType = SET_PIN; OutPin = InPin;
  
  // Debug
  DBF(("UserControl::Settable("));DB((InPin->DeviceName));DBF((":"));DB((InPin->Name));DBF((":"));
  DB((ID));DBFL((")"));
}

/******************************************************************************************************************//**
 * @brief Control type 'TieToPin'
 * @remarks
 * @code
 *   exmaple code
 * @endcode
**********************************************************************************************************************/
void UserControl::TieToPin(PinPoint *_OutPin) {
  if ( InPin == NULL ) return;
  ControlType = TIE_PINS; OutPin = _OutPin;
  
  // Debug
  DBF(("UserControl::TieToPin("));DB((InPin->DeviceName));DBF((":"));DB((InPin->Name));DBF((":"));
  DB((ID));DBFL((")"));
}

/******************************************************************************************************************//**
 * @brief Control type 'SetController'
 * @remarks
 * @code
 *   exmaple code
 * @endcode
**********************************************************************************************************************/
void UserControl::SetController() {
  if ( InPin == NULL ) return;
  ControlType = SET_CONTROLLER; OutPin = InPin;
  
  // Debug
  DBF(("UserControl::SetController("));
  DB((InPin->DeviceName));DBF((":"));DB((InPin->Name));DBF((":"));DB((ID));DBFL((")"));
}
/******************************************************************************************************************//**
 * @brief Control type 'PIDSetpoint'
 * @remarks
 * @code
 *   exmaple code
 * @endcode
**********************************************************************************************************************/
void UserControl::PIDSetpoint(PinPoint *_OutPin, double _Kp, double _Ki, double _Kd, int _POn, int _PDir, PinPoint *_ControlPin = NULL ) {
  if ( InPin == NULL ) return;
  ControlType = PID_SET; OutPin = _OutPin; Kp = _Kp; Ki = _Ki; Kd = _Kd; POn = _POn; PDir = _PDir; ControlPin = _ControlPin;
    
  // Debug
  DBF(("UserControl::PIDSetpoint("));
  DB((InPin->DeviceName));DBF((":"));DB((InPin->Name));DBF((":"));DB((ID));DBC;
  DB((OutPin->DeviceName));DBF((":"));DB((OutPin->Name));DBC;
  DB((Kp));DBC;DB((Ki));DBC;DB((Kd));DBC;DB((POn));DBC;DB((PDir));DBFL((")"));

  PIDControl = new PID(&PIDInput, &PIDOutput, &PIDSet, Kp, Ki, Kd, POn, PDir );
  PIDControl->SetOutputLimits(0,255);
  if ( Status == ISON ) { PIDControl->SetMode(AUTOMATIC); } else { PIDControl->SetMode(MANUAL); }
}

/******************************************************************************************************************//**
 * @brief Control type 'LessThanSetpoint'
 * @remarks
 * @code
 *   exmaple code
 * @endcode
**********************************************************************************************************************/
void UserControl::LessThanSetpoint(PinPoint *_OutPin, PinPoint *_ControlPin = NULL ) {
  if ( InPin == NULL ) return;
  ControlType = LESS_THAN;  OutPin = _OutPin;  ControlPin = _ControlPin;

  // Debug
  DBF(("UserControl::LessThanSetpoint("));DB((InPin->DeviceName));DBF((":"));DB((InPin->Name));DBF((":"));
  DB((ID));DBC;DB((OutPin->DeviceName));DBF((":"));DB((OutPin->Name));DBFL((")"));
}
/******************************************************************************************************************//**
 * @brief Control type 'GreaterThanSetpoint'
 * @remarks
 * @code
 *   exmaple code
 * @endcode
**********************************************************************************************************************/
void UserControl::GreaterThanSetpoint(PinPoint *_OutPin, PinPoint *_ControlPin = NULL ) {
  if ( InPin == NULL ) return;
  ControlType = GREATER_THAN;  OutPin = _OutPin;  ControlPin = _ControlPin;

  // Debug
  DBF(("UserControl::GreaterThanSetpoint("));DB((InPin->DeviceName));DBF((":"));DB((InPin->Name));DBF((":"));
  DB((ID));DBC;DB((OutPin->DeviceName));DBF((":"));DB((OutPin->Name));DBFL((")"));
}
/******************************************************************************************************************//**
 * @brief Control type 'EqualToSetpoint'
 * @remarks
 * @code
 *   exmaple code
 * @endcode
**********************************************************************************************************************/
void UserControl::EqualToSetpoint(PinPoint *_OutPin, PinPoint *_ControlPin = NULL ) {
  if ( InPin == NULL ) return;
  ControlType = EQUAL_TO;  OutPin = _OutPin;  ControlPin = _ControlPin;

  // Debug
  DBF(("UserControl::EqualToSetpoint("));DB((InPin->DeviceName));DBF((":"));DB((InPin->Name));
  DBC;DB((OutPin->DeviceName));DBF((":"));DB((OutPin->Name));
  DBC;DB((ID));DBFL((")"));
}
/******************************************************************************************************************//**
 * @brief Control type 'NotEqualToSetpoint'
 * @remarks
 * @code
 *   exmaple code
 * @endcode
**********************************************************************************************************************/
void UserControl::NotEqualToSetpoint(PinPoint *_OutPin, PinPoint *_ControlPin = NULL ) {
  if ( InPin == NULL ) return;
  ControlType = NOT_EQUAL_TO;  OutPin = _OutPin;  ControlPin = _ControlPin;

  // Debug
  DBF(("UserControl::NotEqualToSetpoint("));DB((InPin->DeviceName));DBF((":"));DB((InPin->Name));
  DBC;DB((OutPin->DeviceName));DBF((":"));DB((OutPin->Name));
  DBC;DB((ID));DBFL((")"));
}

//----------------------------------------------------------------------------------------------------------------------
//                      [[ PROCESS FUNCTIONS ]]
//----------------------------------------------------------------------------------------------------------------------
/******************************************************************************************************************//**
 * @brief Applies the selected Control - Drives the Output Pin
 * @remarks
 * @code
 *   exmaple code
 * @endcode
**********************************************************************************************************************/
void UserControl::Start() {
  DBF(("UserControl::Start("));;DB((InPin->DeviceName));DBF((":"));DB((InPin->Name));DBF((":"));DB((ID));DBFL((")"));

  InPin->ReadValue();
  State = WAIT;
}

/******************************************************************************************************************//**
 * @brief Sets/Gets a User Setpoint
 * @remarks returns WAIT, READY, SETTING, COMPLETE ( This is the State Machine of the Controller )
 * @code
 *   exmaple code
 * @endcode
**********************************************************************************************************************/
eState UserControl::GetState() {
  DBFL(("UserControl::GetState()"));
  
  if ( State == WAIT ) {                    // If Control is at WAIT
    if ( InPin->GetState() != WAIT ) {      // Check that InPin Reading isn't WAITing
      State = READY;                        // Move to Control State READY when InPin is no longer at WAIT
    }
  }
  // Calling Apply() moves Control to COMPLETE
  return State;
}

void UserControl::SetState(eState _State) {
  DBFL(("UserControl::SetState()"));

  State = _State;
}

/******************************************************************************************************************//**
 * @brief Applies the selected Control - Drives the Output Pin
 * @remarks
 * @code
 *   exmaple code
 * @endcode
**********************************************************************************************************************/
void UserControl::Apply(bool SetPin = false) {
  DBF(("UserControl::Apply("));;DB((OutPin->DeviceName));DBF((":"));DB((OutPin->Name));DBF((":"));DB((ID));DBFL((")"));

  int InputValue = InPin->GetRawValue();      // Moves PinPoint State to COMPLETE
  State = COMPLETE;
  if ( !SetPin && ControlType == SET_PIN ) return;
  if ( ControlType != ISON ) return;
  if ( InPin->GetState() == WAIT || InPin->GetStatus() == ERR ) return;    // If read value isn't available then return

  
  switch ( ControlType ) {
    
    case LESS_THAN:
      if ( InputValue < Setpoint ) { OutPin->SetTo(1000); }
      else { OutPin->SetTo(0); }
      break;
      
    case GREATER_THAN:
      if ( InputValue > Setpoint ) { OutPin->SetTo(1000); }
      else { OutPin->SetTo(0); }
      break;
      
    case EQUAL_TO:
      if ( InputValue == Setpoint ) { OutPin->SetTo(1000); }
      else { OutPin->SetTo(0); }
      break;
      
    case NOT_EQUAL_TO:
      if ( InputValue != Setpoint ) { OutPin->SetTo(1000); }
      else { OutPin->SetTo(0); }
      break;
      
    case SET_PIN:
      OutPin->SetTo(Setpoint, Status);
      break;

    case PID_SET:
      if ( PIDControl != NULL ) {
        PIDInput = double(InputValue);
        PIDSet = double(Setpoint);
        unsigned long now=millis();
        PIDControl->SetSampleTime( now - PIDLastCompute );
        PIDLastCompute = now;
        if ( PIDControl->Compute() ) { OutPin->SetTo(PIDOutput); }
        else { DBL((F("UserControl::Apply(PID_SET - No Compute)"))); }

        // Debug message
        DB((F("UserControl::Apply(PID_SET,")));DB((PIDInput));DBC;DB((Setpoint));DBC;DB((PIDOutput));DBC;
        if ( PIDControl->GetMode() == AUTOMATIC ) { DB((F("AUTOMATIC"))); } else { DB((F("MANUAL"))); }
        DBL((F(")")));
      }
      break;
  }
  if ( SetPin ) InPin->CurrControl = NULL;    // After Set return to Pin Read
}

/******************************************************************************************************************//**
 * @brief Sets/Gets a User Setpoint
 * @remarks
 * @code
 *   exmaple code
 * @endcode
**********************************************************************************************************************/
void UserControl::SetPoint(int _Set) {
  Setpoint = _Set;
  if ( ControlPin != NULL ) ControlPin->SetTo(Setpoint, Status);
}
int UserControl::SetPoint() {
  return Setpoint;
}

/******************************************************************************************************************//**
 * @brief Adds or Subtracts to the Setpoint using the Modified Display Value
 * @remarks
 * @code
 *   exmaple code
 * @endcode
**********************************************************************************************************************/
void UserControl::SetPointAdd(int AddValue) {
  if ( InPin->IsOnOff ) {
    if ( Setpoint != 0 ) { Setpoint = 0; }
    else { Setpoint = 1; }
  } else {
    // Adjust Setpoint according to Modified Value
    int ModStart = InPin->ModifyValue(Setpoint);
    if ( AddValue < 0 ) {
      while ( InPin->ModifyValue(Setpoint) > ModStart + AddValue ) { Setpoint--; }
    } else if ( AddValue > 0 ) {
      while ( InPin->ModifyValue(Setpoint) < ModStart + AddValue ) { Setpoint++; }
    }
    if ( Setpoint < 0 ) Setpoint = 0;
  }
  if ( ControlPin != NULL ) ControlPin->SetTo(Setpoint, Status);
}

/******************************************************************************************************************//**
 * @brief IsOn Get/Set wrappers
 * @remarks
 * - If 'ControlPin' is set the status is retreived from the Virtual Pin
 * - Else the Status is stored locally
 * @code
 *   exmaple code
 * @endcode
**********************************************************************************************************************/
void UserControl::IsOn(bool _IsOn) {
  DBF(("UserControl::IsOn("));DB((_IsOn));DBFL((")"));

  if ( _IsOn ) { 
    Status = ISON; 
    if ( ControlType == PID_SET ) PIDControl->SetMode(AUTOMATIC);
  } else { 
    Status = ISOFF;
    if ( ControlType == PID_SET ) PIDControl->SetMode(MANUAL); 
  }
  if ( ControlPin != NULL ) OutPin->SetTo(Setpoint, Status);
  if ( Status == ISON && ID != NULL ) { OnControls[ObjectIndex] = ID; } 
  else { OnControls[ObjectIndex] = ' '; }
  //DB((F("OnControls["));DB((ObjectIndex));DB((F("]="));DBL((OnControls[ObjectIndex]));
}
bool UserControl::IsOn() {
  if ( ControlPin != NULL ) return (OutPin->GetStatus() == ISON);
  return ( Status == ISON );
}
