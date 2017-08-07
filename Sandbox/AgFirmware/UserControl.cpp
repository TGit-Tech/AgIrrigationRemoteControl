/******************************************************************************************************************//**
 * @file UserControl.cpp
 * @brief Manages Input->Output Control Processes ( Read-InPin-Value > Control Process > Set-OutPin-Value )
 * @authors 
 *    tgit23        8/2017       Original
 **********************************************************************************************************************/
#include "UserControl.h"

/******************************************************************************************************************//**
 * @brief Creates a new Control ( Constructor )
 * @remarks
 * @code
 *   exmaple code
 * @endcode
**********************************************************************************************************************/
UserControl::UserControl(PinPoint * _InPin, eControlType _ControlType, PinPoint * _OutPin, char _ID, PinPoint * _StorePin = NULL) {
  InPin = _InPin;
  ControlType = _ControlType;
  OutPin = _OutPin;
  ID = _ID;
  StorePin = _StorePin;
  if ( ControlType == PID_SET ) {
    PIDControl = new PID(&PIDInput, &PIDOutput, &PIDSetpoint, Kp, Ki, Kd, POn, PIDDirection );
    PIDControl->SetOutputLimits(1,1023);
    PIDControl->SetMode(MANUAL);
  }
}

/******************************************************************************************************************//**
 * @brief Applies the Output ( Driven ) Pin
 * @remarks
 * @code
 *   exmaple code
 * @endcode
**********************************************************************************************************************/
void UserControl::Apply() {

  if ( ControlType != SET_PIN && Status != ISON ) return;                   // If control is not ON return
  if ( InPin->GetStatus() == WAIT || InPin->GetStatus() == ERR ) return;    // If read value isn't available then return

  int InputValue = InPin->GetRawValue();
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
        PIDSetpoint = double(Setpoint);
        PIDControl->Compute();
        OutPin->SetTo(PIDOutput);
      }
      break;
  }
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
  if ( StorePin != NULL ) StorePin->SetTo(Setpoint);
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
  }
}

/******************************************************************************************************************//**
 * @brief IsOn Get/Set wrappers
 * @remarks
 * - If 'StorePin' is set the status is retreived from the Virtual Pin
 * - Else the Status is stored locally
 * @code
 *   exmaple code
 * @endcode
**********************************************************************************************************************/
void UserControl::IsOn(bool _IsOn) {
  mIsOn = _IsOn;
  if ( StorePin != NULL ) OutPin->SetTo(Setpoint, Status); 
}
bool UserControl::IsOn() {
  if ( StorePin != NULL ) return OutPin->GetStatus();
  return mIsOn;
}

/******************************************************************************************************************//**
 * @brief Set an Arduino Eprom Offset Address where the Controls 'Setpoint' is stored
 * @remarks
 * @return  The next free Offset Address
 * @code
 *    // Loop through functions and return next available offset
 *    unsigned int Offset = StartingEpromOffset;
 *    MenuFunction *thisFunction = FirstFunction;
 *    while ( thisFunction != NULL ) { 
 *      Offset = thisFunction->SetEpromOffset(Offset); // Set Offset and return next available
 *      thisFunction = thisFunction->Next;
 *    }
 *    return Offset;
 * @endcode
**********************************************************************************************************************/
unsigned int UserControl::SetEpromOffset(unsigned int _EpromOffset) {
  DB(("UserControl::SetEpromOffset("));DB((_EpromOffset));DBL((")"));

  EpromOffset = _EpromOffset;
  byte LoByte = EEPROM.read( EpromOffset );                           // Read Eprom when Offset is assigned
  byte HiByte = EEPROM.read( EpromOffset + 1 );
  if ( HiByte > 0x3F ) {                                              // Initialize EPROM to 0x00 if out-of-bounds
    EEPROM.update( EpromOffset, 0x00 );
    EEPROM.update( EpromOffset + 1, 0x00 );
    Setpoint = 0;
  } else {                                                            // ELSE assign eprom values
    Status = ((HiByte & 0x30) >> 8);                                  // Bit 12 & 13 make Status
    Setpoint = (int)((LoByte << 0) & 0xFF) + ((HiByte << 8) & 0x0F00);   // 0x0FFF 0->11-bits for value
  }
  return EpromOffset + 2;                                             // Return next available offset
}

/******************************************************************************************************************//**
 * @brief  Save the Controls 'Setpoint' to Eprom and VirtualPins
 * @remarks
 *  @code
 *    exmaple code
 *  @endcode
**********************************************************************************************************************/
void UserControl::Save() {
  DBL(("UserControl::Save()"));
    
  // EEPROM - Save; Store the Value and Status as two bytes in Eprom
  byte LoByte = ((Setpoint >> 0) & 0xFF);
  byte HiByte = ((Status << 4 & 0x30) + (Setpoint >> 8) & 0x0F);
  EEPROM.update( EpromOffset, LoByte );
  EEPROM.update( (EpromOffset + 1), HiByte );
  DB(("EEPROM.update( "));DB((EpromOffset));DBC;DB((LoByte, HEX));DBL((")"));
  DB(("EEPROM.update( "));DB((EpromOffset + 1));DBC;DB((HiByte, HEX));DBL((")"));

  // VirtualPin - Save; If 'ValueOnPin' Exists - Save to Virtual Pin
  //if ( ValueOnPin != NOPIN ) { XBee->VirtualPin(ValueOnPin, Value, Status); }
}

