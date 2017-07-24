#include "MenuFunction.h"


/******************************************************************************************************************//**
 * @brief New Function constructor
 * @remarks
 * - Raw values are still used behind the scene's only the displayed value is modified
 * @code
 *   exmaple code
 * @endcode
**********************************************************************************************************************/
MenuFunction::MenuFunction(eFunctionType _FunctionType, uint8_t _DriveDevice = NODEVICE, uint8_t _DrivePin = NOPIN, int _DriveValue = 0, uint8_t _ValueOnPin = NOPIN, char _ID = NULL, bool _HaltOnAlarm = false, uint8_t _ViolationCount = 1 ) {
  FunctionType = _FunctionType;
  DriveDevice = _DriveDevice;
  DrivePin = _DrivePin;
  DriveValue = _DriveValue;
  ValueOnPin = _ValueOnPin;
  ID = _ID;
  HaltOnAlarm = _HaltOnAlarm;
  ViolationCount = _ViolationCount;
}

/******************************************************************************************************************//**
 * @brief Use a user-defined function in the main sketch to modify the raw value read from the arduino
 * @remarks
 * - Raw values are still used behind the scene's only the displayed value is modified
 * - Returns the next available Eprom Offset
 * @code
 *   exmaple code
 * @endcode
**********************************************************************************************************************/
int MenuFunction::SetEpromOffset(unsigned int _EpromOffset) {
  DB(("MenuFunction::SetEpromOffset("));DB((_EpromOffset));DBL((")"));

  EpromOffset = _EpromOffset;
  byte LoByte = EEPROM.read( EpromOffset );                           // Read Eprom when Offset is assigned
  byte HiByte = EEPROM.read( EpromOffset + 1 );
  if ( HiByte > 0x3F ) {                                              // Initialize EPROM to 0x00 if out-of-bounds
    EEPROM.update( EpromOffset, 0x00 );
    EEPROM.update( EpromOffset + 1, 0x00 );
    Value = 0;
  } else {                                                            // ELSE assign eprom values
    Status = ((HiByte & 0x30) >> 8);                                  // Bit 12 & 13 make Status
    Value = (int)((LoByte << 0) & 0xFF) + ((HiByte << 8) & 0x0F00);   // 0x0FFF 0->11-bits for value
  }
  return _EpromOffset + 2;
}

/******************************************************************************************************************//**
 * @brief Preform duties for the 'Select' button press
 * @remarks
 * - Raw values are still used behind the scene's only the displayed value is modified
 * - Returns the next available Eprom Offset
 * @code
 *   exmaple code
 * @endcode
**********************************************************************************************************************/
void MenuFunction::Select() {
  if ( FunctionType == LESS_THAN_ALARM || FunctionType == GREATER_THAN_ALARM ||
       FunctionType == EQUAL_TO_ALARM || FunctionType == NOT_EQUAL_TO_ALARM ) {
    // Toggle On/Off
  } 
  else if ( FunctionType == SET_PIN ) {
    // Preform Pin set
  } 
  else if ( FunctionType == PID_SETPOINT ) {
    // Toggle On/Off
  }
}

/* bpress == SELECT
 *  if ( Func == SETPID ) {
      if ( CurrItem->SetPID->OPID->GetMode() == AUTOMATIC ) {
        CurrItem->SetPID->OPID->SetMode(MANUAL);IsOn = MANUAL;
      } else {
        CurrItem->SetPID->OPID->SetMode(AUTOMATIC);IsOn = AUTOMATIC;
      }
      // Set the VPins and Save changes to EEPROM
      if ( CurrItem->SetPID->ValueOnPin != NOPIN ) XBee->VirtualPin( CurrItem->SetPID->ValueOnPin, CurrItem->SetPID->Setpoint );
      if ( CurrItem->SetPID->SetOnOffVPin != NOPIN) XBee->VirtualPin( CurrItem->SetPID->SetOnOffVPin, CurrItem->SetPID->OPID->GetMode() );
      EEPROMSet(CurrItem->SetPID->EpromOffset, CurrItem->SetPID->Setpoint, IsOn);   // Save Value & IsOn in Eprom
    }
  
    // Drive the Device and Pin when Select is pressed during a SET
    else if ( Func == SET ) {
      
      // If the SET item is under PID control; turn off the PID
      if ( CurrItem->Set->AttachedPID != NULL ) {
        DBL(("SET shutdown PID"));
        CurrItem->Set->AttachedPID->OPID->SetMode(MANUAL);
        EEPROMSet(CurrItem->Set->AttachedPID->EpromOffset, CurrItem->Set->AttachedPID->Setpoint, MANUAL);
      }
      EEPROMSet(CurrItem->Set->EpromOffset, CurrItem->Set->Value);        // Just a convenience save
      SetPin( CurrItem->Set->DriveDevice, CurrItem->Set->DrivePin, CurrItem->Set->Value );
      
      // Check if SET item has DriveOnOffVPin
      if ( CurrItem->Set->DriveOnOffVPin ) {
        SetPin( CurrItem->Set->DriveDevice, CurrItem->Set->DriveOnOffVPin, CurrItem->Set->Value );
      } else {
        Func = MAIN;                                                        // Return to MAIN for convenience
        GetItem(CurrItem);                                                  // Get item again after SET
      }
    }    

    // Toggle Alarm (OFF<->ON) when Select is pressed
    else if ( Func == ALARM ) {
      if ( CurrItem->CurrAlarm->IsOn ) { CurrItem->CurrAlarm->IsOn = false; } else { CurrItem->CurrAlarm->IsOn = true; }
      if ( CurrItem->CurrAlarm->ValueOnPin != NOPIN ) { XBee->VirtualPin( CurrItem->CurrAlarm->ValueOnPin, CurrItem->CurrAlarm->GetValue() ); }
      if ( CurrItem->CurrAlarm->SetOnOffVPin != NOPIN ) { XBee->VirtualPin( CurrItem->CurrAlarm->SetOnOffVPin, CurrItem->CurrAlarm->IsOn ); }
      EEPROMSet(CurrItem->CurrAlarm->EpromOffset, CurrItem->CurrAlarm->GetValue(), (int)CurrItem->CurrAlarm->IsOn ); 
    }
 */


/******************************************************************************************************************//**
 * @brief  Record Menu Item Values in EEPROM non-volitale memory.
 * @remarks
 * - Arduino UNO offers 1024-bytes or (146)7-Byte Menu Item Storage
 * - Pump-Controller Menu Items need 5-Alarm Bytes per Selectable Pump
 * - Hand_NOPIN Menu Items need 7-Bytes ( 2-Value, 5-Alarms )
 *  @code
 *    exmaple code
 *  @endcode
**********************************************************************************************************************/
/*
void uAlarm::EEPROMSet(unsigned int Offset, int Value, int IsOn = -1) {
  DB(("EEPROMSet("));DB((Offset));DBC;DB((Value));DBC;DB((IsOn));DBL((")"));
    
  // Store the Value as two bytes
  byte LoByte = ((Value >> 0) & 0xFF);
  byte HiByte = ((Value >> 8) & 0xFF);
  EEPROM.update( Offset, LoByte );
  DB(("EEPROM.update( "));DB((Offset));DBC;DB((LoByte, HEX));DBL((")"));
  EEPROM.update( (Offset + 1), HiByte );
  DB(("EEPROM.update( "));DB((Offset + 1));DBC;DB((HiByte, HEX));DBL((")"));
  
  // If an 'IsOn' was given - store it in one byte
  if ( IsOn != -1 ) {
    byte IsOnByte = 0x22; // 2=OFF, A=ON
    if ( IsOn ) bitSet(IsOnByte,3);
    EEPROM.update ( (Offset + 2), IsOnByte ); 
    DB(("EEPROM.update( "));DB((Offset + 2));DBC;DB((IsOnByte, HEX));DBL((")"));
  }
}
*/
/******************************************************************************************************************//**
 * @brief  Read Menu Item Values from Arduino EEPROM non-volitale memory.
 * @see    EEPROMSet for Addressing notation
 * @code
 *   exmaple code
 * @endcode
**********************************************************************************************************************/
/*
int uAlarm::EEPROMGet(unsigned int Offset, bool *IsOn = NULL) {
  DB(("EEPROMGet("));DB((Offset));DBL((")"));

  // Get Value from the first 2-byets
  byte LoByte = EEPROM.read( Offset );
  byte HiByte = EEPROM.read( Offset + 1 );

  // If an 'IsOn' pointer was supplied - set it with 3rd byte
  if ( IsOn != NULL ) {
    if ( bitRead(EEPROM.read((Offset + 2)),3) ) *IsOn = true;
  }
  // Return the value read from Eprom
  return (int)((LoByte << 0) & 0xFF) + ((HiByte << 8) & 0xFF00);
}
*/
