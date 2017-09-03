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

//----------------------------------------------------------------------------------------------------------------------
//                      [[ CREATE & SAVE ]]
//----------------------------------------------------------------------------------------------------------------------
/******************************************************************************************************************//**
 * @brief Create a new Control ( Constructor )
 * @remarks Sets the EEPROM storing location and retreives SetPoint values if they exist
 * @code
 *   exmaple code
 * @endcode
**********************************************************************************************************************/
UserControl::UserControl(PinPoint *_InPin, LiquidCrystal *_LCD, char _ID ) {
  if ( _InPin == NULL ) return;             // Requires InPin
  InPin = _InPin;  LCD = _LCD;  ID = _ID;   // Assign arguments
  
  DBF(("UserControl::UserControl("));DB((InPin->DeviceName));DBF((":"));DB((InPin->Name));DBF((":"));
  DB((ID));DBF((")"));
  
  EpromOffset = NextEpromOffset;                          // Assign EEPROM Offset to store User mSetpoints
  DB((F(" @EpromOffset=")));DB((EpromOffset));
  byte LoByte = EEPROM.read( EpromOffset );               // Read Eprom when Offset is assigned
  byte HiByte = EEPROM.read( EpromOffset + 1 );
  if ( HiByte > 0x3F ) {                                  // Initialize EPROM to 0x00 if out-of-bounds
    EEPROM.update( EpromOffset, 0x00 );
    EEPROM.update( EpromOffset + 1, 0x00 );
    mSetpoint = 0;
  } else {                                                // ELSE assign the Eprom Read values
    mStatus = ((HiByte & 0x30) >> 4);                     // Bit 12 & 13 make Status
    mSetpoint = (int)(((HiByte & 0x0F) << 8) | LoByte);    // 0x0FFF 0->11-bits for value
  }
  NextEpromOffset = EpromOffset + 2;                      // Record the next available offset

  ObjectIndex = ObjectCount;                              // Assign an OnControls[ObjectIndex] & set to ID if ON
  if ( ObjectCount<15 ) ObjectCount++;                    // Stay within boundaries
  if ( Status() == ISON ) OnControls[ObjectIndex] = ID;
  DB((F(" @ObjectIndex=")));DBL((ObjectIndex));
}

/******************************************************************************************************************//**
 * @brief Saves the Controls SetPoint and Status to EEPROM memory and Sets the ControlPin
 * @remarks 
 *  @code
 *    exmaple code
 *  @endcode
**********************************************************************************************************************/
void UserControl::Save() {
  DBFL(("UserControl::Save()"));
    
  // EEPROM - Save; Store the Value and Status as two bytes in Eprom
  byte LoByte = ((mSetpoint >> 0) & 0xFF);
  byte HiByte = ((mStatus << 4 & 0x30) | (mSetpoint >> 8 & 0x0F));
  EEPROM.update( EpromOffset, LoByte );
  EEPROM.update( (EpromOffset + 1), HiByte );
  DB((F("EEPROM.update( ")));DB((EpromOffset));DBC;DB((LoByte, HEX));DBL((F(")")));
  DB((F("EEPROM.update( ")));DB((EpromOffset + 1));DBC;DB((HiByte, HEX));DBL((F(")")));

  // VirtualPin - Save; If 'ControlPin' Exists - Save to a Virtual Pin
  if ( ControlPin != NULL ) { ControlPin->SetTo(mSetpoint, mStatus); }
  if ( ControlType == SET_CONTROLLER ) { Apply(true); } // Save to Virtual Pin
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
  ControlType = TIE_PINS;
  OutPin = _OutPin;
  
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
  ControlType = SET_CONTROLLER;
  OutPin = InPin;
  
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
  ControlType = PID_SET;
  OutPin = _OutPin;
  Kp = _Kp; Ki = _Ki; Kd = _Kd;
  POn = _POn; PDir = _PDir;
  ControlPin = _ControlPin;
    
  // Debug
  DBF(("UserControl::PIDSetpoint("));
  DB((InPin->DeviceName));DBF((":"));DB((InPin->Name));DBF((":"));DB((ID));DBC;
  DB((OutPin->DeviceName));DBF((":"));DB((OutPin->Name));DBC;
  DB((Kp));DBC;DB((Ki));DBC;DB((Kd));DBC;DB((POn));DBC;DB((PDir));DBFL((")"));

  PIDControl = new PID(&PIDInput, &PIDOutput, &PIDSet, Kp, Ki, Kd, POn, PDir );
  PIDControl->SetOutputLimits(0,255);
  if ( Status() == ISON ) { PIDControl->SetMode(AUTOMATIC); } else { PIDControl->SetMode(MANUAL); }
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
  ControlType = LESS_THAN;
  OutPin = _OutPin;
  ControlPin = _ControlPin;

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
  ControlType = GREATER_THAN;
  OutPin = _OutPin;
  ControlPin = _ControlPin;

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
  ControlType = EQUAL_TO;
  OutPin = _OutPin;
  ControlPin = _ControlPin;

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
  ControlType = NOT_EQUAL_TO;
  OutPin = _OutPin;
  ControlPin = _ControlPin;

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
 * @remarks Apply() takes for granite that the InPin Value was already retreived
 * @code
 *   exmaple code
 * @endcode
**********************************************************************************************************************/
void UserControl::Apply(bool ManualApply = false) {
  DBF(("UserControl::Apply("));;DB((OutPin->DeviceName));DBF((":"));DB((OutPin->Name));DBF((":"));DB((ID));
  DBC;DB((ManualApply));DBFL((")"));

//  State = COMPLETE;                                                       // Moves Control to COMPLETE
  int InputValue = InPin->GetRawValue();                                  // Read the InPin Value to Apply Auto-Control
  
  if ( !ManualApply ) {
    if ( Status() != ISON ) return;                                         // Make sure Auto-Applies ISON
    if ( InPin->State() == WAIT || InPin->Status() == ERR ) return; // If read value isn't available then return    
  }

  //-------------------------------------------------------------------------------------------------------------------
  switch ( ControlType ) {
    
    case LESS_THAN:
      if ( InputValue < mSetpoint ) { OutPin->SetTo(1000); }
      else { OutPin->SetTo(0); }
      break;
      
    case GREATER_THAN:
      if ( InputValue > mSetpoint ) { OutPin->SetTo(1000); }
      else { OutPin->SetTo(0); }
      break;
      
    case EQUAL_TO:
      if ( InputValue == mSetpoint ) { OutPin->SetTo(1000); }
      else { OutPin->SetTo(0); }
      break;
      
    case NOT_EQUAL_TO:
      if ( InputValue != mSetpoint ) { OutPin->SetTo(1000); }
      else { OutPin->SetTo(0); }
      break;
      
    case SET_PIN:
      if ( ManualApply ) {
        OutPin->SetTo(mSetpoint);                    // Only apply SET_PIN Manually
        InPin->CurrControl = NULL;                          // After Manual Set exit Control
      }
      break;

    case PID_SET:
      if ( PIDControl != NULL ) {
        PIDInput = double(InputValue);
        PIDSet = double(mSetpoint);
        unsigned long now=millis();
        PIDControl->SetSampleTime( now - PIDLastCompute );
        PIDLastCompute = now;
        if ( PIDControl->Compute() ) { OutPin->SetTo(PIDOutput); }
        else { DBL((F("UserControl::Apply(PID_SET - No Compute)"))); }

        // Debug message
        DB((F("UserControl::Apply(PID_SET,")));DB((PIDInput));DBC;DB((mSetpoint));DBC;DB((PIDOutput));DBC;
        if ( PIDControl->GetMode() == AUTOMATIC ) { DB((F("AUTOMATIC"))); } else { DB((F("MANUAL"))); }
        DBL((F(")")));
      }
      break;
      
    case SET_CONTROLLER:
      if ( ManualApply ) { OutPin->SetTo(mSetpoint, mStatus); }
      break;
  }
}
//----------------------------------------------------------------------------------------------------------------------
//                      [[ SetPoint Functions ]]
//----------------------------------------------------------------------------------------------------------------------
/******************************************************************************************************************//**
 * @brief Sets the Controls User-SetPoint
 * @remarks 
 * - Setting a new SetPoint will update the Display
 * - If a 'ControlPin' was supplied on the Controller the SetPoint is saved to the 'ControlPin'
 * @code
 *   exmaple code
 * @endcode
**********************************************************************************************************************/
void UserControl::SetPoint(int _Set) {
  mSetpoint = _Set;
  if ( ControlPin != NULL ) ControlPin->SetTo(mSetpoint, mStatus);    // Make Set on ControlPin
  Display();
}

/******************************************************************************************************************//**
 * @brief Gets the Controls User-SetPoint
 * @remarks
 * @code
 *   exmaple code
 * @endcode
**********************************************************************************************************************/
int UserControl::SetPoint() {
  if ( ControlPin != NULL ) { mSetpoint = ControlPin->GetRawValue(); } // Retreive SetPoint from ControlPin
  return mSetpoint;
}

/******************************************************************************************************************//**
 * @brief Adds or Subtracts to the mSetpoint using the Modified Display Value and updates the Display
 * @remarks
 * @code
 *   exmaple code
 * @endcode
**********************************************************************************************************************/
void UserControl::SetPointAdd(int AddValue) {
  if ( InPin->IsOnOff ) {
    if ( mSetpoint != 0 ) { mSetpoint = 0; }
    else { mSetpoint = 1; }
  } else {
    // Adjust mSetpoint according to Modified Value
    int ModStart = InPin->ModifyValue(mSetpoint);
    if ( AddValue < 0 ) {
      while ( InPin->ModifyValue(mSetpoint) > ModStart + AddValue ) { mSetpoint--; }
    } else if ( AddValue > 0 ) {
      while ( InPin->ModifyValue(mSetpoint) < ModStart + AddValue ) { mSetpoint++; }
    }
    if ( mSetpoint < 0 ) mSetpoint = 0;
  }
  if ( ControlPin != NULL ) ControlPin->SetTo(mSetpoint, mStatus);

  Display();
}

/******************************************************************************************************************//**
 * @brief Set the Control Status (
 * @remarks
 * - If 'ControlPin' is set the status is retreived from the Virtual Pin
 * - Else the Status is stored locally
 * @code
 *   exmaple code
 * @endcode
**********************************************************************************************************************/
void UserControl::Status(PinStatus _Status) {
  DBF(("UserControl::Status("));DB((_Status));DBFL((")"));

  if ( _Status == ISON ) { 
    mStatus = ISON;
    if ( ControlType == PID_SET ) PIDControl->SetMode(AUTOMATIC);
  } else { 
    mStatus = ISOFF;
    if ( ControlType == PID_SET ) PIDControl->SetMode(MANUAL); 
  }
  if ( ControlPin != NULL ) ControlPin->SetTo(mSetpoint, mStatus);
  if ( mStatus == ISON && ID != NULL ) { OnControls[ObjectIndex] = ID; } 
  else { OnControls[ObjectIndex] = ' '; }

  // Update the Display
  Display();
  //DB((F("OnControls["));DB((ObjectIndex));DB((F("]="));DBL((OnControls[ObjectIndex]));  
}

/******************************************************************************************************************//**
 * @brief Get the Control Status
 * @remarks
 * - If 'ControlPin' is set the status is retreived from the Virtual Pin
 * - Else the Status is stored locally
 * @code
 *   exmaple code
 * @endcode
**********************************************************************************************************************/
PinStatus UserControl::Status() {
  if ( ControlPin != NULL ) mStatus = ControlPin->Status();
  return mStatus;
}

/******************************************************************************************************************//**
 * @brief private. Displays the Control on an LCD
 * @remarks
 * @code
 *   exmaple code
 * @endcode
**********************************************************************************************************************/
void UserControl::Display() {
  if ( LCD == NULL ) return;
  DBFL(("UserControl::Display()"));
  
  // These controls don't need a Control display
  if ( ControlType == LCD_READONLY || ControlType == TIE_PINS ) return;
    
  // Display Device ( Top Row Left )
  LCD->clear();LCD->setCursor(0,0);
  LCD->print( InPin->DeviceName );

  // Display Control ON/OFF Status
  if ( ControlType != SET_PIN ) {
    if ( Status() == ISON ) { LCD->setCursor(14,0);LCD->print("On"); }
    else { LCD->setCursor(13,0);LCD->print("Off"); }
  }
  
  //Display the Pin and mSetpoint
  LCD->setCursor(0,1);LCD->print( InPin->Name );LCD->print(" ");
    
    switch ( ControlType ) {
      case SET_PIN:         LCD->print("SET");LCD->print(char(126));break; // 126 ->
      case PID_SET:         LCD->print("PID");LCD->print(char(126));break; // 126 ->
      case SET_CONTROLLER:  LCD->print("SET");LCD->print(char(126));break; // 126 ->
      case LESS_THAN:       LCD->print(char(225));LCD->print("<");break; // 225 = a-dots
      case GREATER_THAN:    LCD->print(char(225));LCD->print(">");break; // 225 = a-dots
      case EQUAL_TO:        LCD->print(char(225));LCD->print("=");break; // 225 = a-dots
      case NOT_EQUAL_TO:    LCD->print(char(225));LCD->print(char(183));break; // 225 = a-dots, 183 = slashed =
    }

    if ( Status() == ERR ) { LCD->print("ERR"); }
    else {
      if ( InPin->IsOnOff ) {
        if ( SetPoint() == 0 ) { LCD->print("Off"); }
        else { LCD->print("On"); }
      } else {
        LCD->print(SetPoint());
      }
    }
  
}

