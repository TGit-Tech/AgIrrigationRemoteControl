/******************************************************************************************************************//**
 * @file PinPoint.cpp
 * @brief Class definition for a PinPoint on the https://github.com/tgit23/AgIrrigationRemoteControl project.
 * @remarks 
 *  - PinPoints define any hardwired Accessory like relays, meters, buzzers, lights, ultrasonic, etc...
 *  - PinPoint defines Read / Write functions for the Pin
 *  - PinPoint acts as a 'UserControl' objects collection for this paticular Pins ( Read->Control ) process.
 * @authors 
 *    tgit23        8/2017       Original
 **********************************************************************************************************************/
#include "PinPoint.h"
#include "UserControl.h"

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

PeerIOSerialControl *PinPoint::XBee = NULL; // Static XBee for all PinPoints
//---------------------------------------------------------------------------------------------------------------------
// Constructor
//---------------------------------------------------------------------------------------------------------------------
PinPoint::PinPoint(uint8_t *_Device, uint8_t *_Pin, char *_DeviceName) {
  Device = _Device;
  Pin = _Pin;
  DeviceName = _DeviceName;
}

//---------------------------------------------------------------------------------------------------------------------
// Mode()
//---------------------------------------------------------------------------------------------------------------------
void PinPoint::Mode(uint8_t _Mode) {
  PinMode = _Mode;
  
  if ( !Device ) return; // Set pinMode() if Local device
  switch ( PinMode ) {
    case INPUT:           pinMode(Pin, INPUT);IsOnOff = ( Pin < A0 ); break;
    case INPUT_PULLUP:    pinMode(Pin, INPUT_PULLUP);IsOnOff = ( Pin < A0 ); break;
    case OUTPUT:          pinMode(Pin, OUTPUT);IsOnOff = ( Pin < A0 ); break;
    case INPUT_SONIC:     break;
    case OUTPUT_BUZZER:   pinMode(Pin, OUTPUT);break;
    case OUTPUT_PWM:      pinMode(Pin, OUTPUT);break;
    case CONTROLLER:      break;
  }
}

void PinPoint::Mode(uint8_t _Mode, char *_Name ) {
  Mode(_Mode);
  Name = _Name;
}

void PinPoint::Mode(uint8_t _Mode, char *_Name, uint8_t _TrigPin, uint8_t _EchoPin) {
  Mode(_Mode);
  Name = _Name;
  if ( !Device ) {                  // Create Sonar if on local device
    if ( _Mode == INPUT_SONIC ) {
      pinMode(_TrigPin, OUTPUT);
      pinMode(_EchoPin, INPUT);
      Sonar = new NewPing(_TrigPin, _EchoPin, 400);
    }
  }
}
   
/******************************************************************************************************************//**
 * @brief Reads the Value and stores it; Value is then gotten by calling GetRawValue() or GetModifiedValue()
 * @remarks
 * @code
 *   APin.ReadValue();
 *   while ( APin.Status() == WAIT ) { // Loop Till Value is Retreivable }
 *   GottenValue = APin.GetValue();
 * @endcode
**********************************************************************************************************************/
void PinPoint::ReadValue(bool _ForceBlocking = false) {
  DB((F("PinPoint::ReadValue("))); if ( _ForceBlocking ) { DB((F("Blocking")));DBC; }
  DB((F("Device=")));DB((int(Device)));DBC;DB((F("Pin=")));DB((int(Pin)));DBL((")"));

  mStatus = ERR; mPacketID = -1; 
  if ( Device > 16 || Pin > 127 ) return;    // Value Check
  State = WAIT;
  
  if ( !Device ) {                         //--- Local Pin Read ---
    if ( PinMode == INPUT_SONIC ) {                          // SonicPin Only works on local device
      if ( Sonar != NULL ) {
        mValue = Sonar->ping_in();                        // Measure the Distance
        if ( mValue != -1 ) mStatus = OKAY;               
        XBee->VirtualPin(Pin, mValue);                    // Record distance on the Virtual Pin
      }
    }
    else if ( PinMode == OUTPUT_PWM ) { mValue = XBee->analogReadOutput(Pin); }
    else if ( Pin >= A0 ) { mValue = analogRead(Pin); }
    else { mValue = digitalRead(Pin); }
    if ( mValue != -1 ) { mStatus = OKAY; }
    DB((F("PinPoint::ReadValue LOCAL=")));DBL((mValue));
    State = READY;
  }
  else if ( _ForceBlocking ) {
    if ( Device != XBee->TargetArduinoID() ) XBee->TargetArduinoID( Device ); //Set TransceiverID
    if ( Pin >= A0 || PinMode ==OUTPUT_PWM ) { mValue = XBee->analogReadB(Pin); }
    else { mValue = XBee->digitalReadB(Pin); }
    if ( mValue != -1 ) { mStatus = OKAY; }
    DB((F("PinPoint::ReadValue BLOCKED=")));DBL((mValue));
    State = READY;
  }
  else {                                                        //--- Remote Pin Read ---
    if ( Device != XBee->TargetArduinoID() ) XBee->TargetArduinoID( Device ); //Set TransceiverID
    mWaitStart = millis();
    if ( Pin >= A0 || PinMode ==OUTPUT_PWM || PinMode == CONTROLLER ) { mPacketID = XBee->analogReadNB(Pin); }
    else { mPacketID = XBee->digitalReadNB(Pin); }
    DBL((F("PinPoint::ReadValue REQUESTED")));
  }  
}

/******************************************************************************************************************//**
 * @brief Checks XBee communications and returns 'true' if the pin Status has changed
 * @remarks
 * @code
 *   exmaple code
 * @endcode
**********************************************************************************************************************/
eState PinPoint::GetState() {
  XBee->Available();
  if ( State == WAIT ) {
    // Check GetReply()
    if ( mPacketID != -1 ) {
      int Ret = XBee->GetReply(mPacketID);
      if ( Ret != -1 ) { mValue = Ret; mStatus = OKAY; State = READY; }
    }
    // Timeout
    if ( millis() - mWaitStart > XBee->Timeout() ) { mPacketID = -1; mStatus = ERR; State = READY; }
  }
  /* Debug
  switch ( State ) {
    case WAIT:      DBFL(("PinPoint::GetState(WAIT)"));break;
    case READY:     DBFL(("PinPoint::GetState(READY)"));break;
    case SETTING:   DBFL(("PinPoint::GetState(SETTING)"));break;
    case COMPLETE:  DBFL(("PinPoint::GetState(COMPLETE)"));break;
  }
  */
  return State;
}

void PinPoint::SetState(eState _State) {
  State = _State;
}

/******************************************************************************************************************//**
 * @brief Use a user-defined function in the main sketch to modify the raw value read from the arduino
 * @remarks
 * - Raw values are still used behind the scene's only the displayed value is modified
 * @code
 *   exmaple code
 * @endcode
**********************************************************************************************************************/
PinStatus PinPoint::GetStatus() {
  return mStatus;
}

/******************************************************************************************************************//**
 * @brief Use a user-defined function in the main sketch to modify the raw value read from the arduino
 * @remarks
 * - Raw values are still used behind the scene's only the displayed value is modified
 * @code
 *   exmaple code
 * @endcode
**********************************************************************************************************************/
int PinPoint::GetRawValue() {
  return mValue;
}

/******************************************************************************************************************//**
 * @brief Use a user-defined function in the main sketch to modify the raw value read from the arduino
 * @remarks
 * - Raw values are still used behind the scene's only the displayed value is modified
 * @code
 *   exmaple code
 * @endcode
**********************************************************************************************************************/
int PinPoint::GetModifiedValue() {
  if ( ValueModifierCallback == NULL ) return mValue;
  return (*ValueModifierCallback)(mValue);
}

int PinPoint::ModifyValue(int _Value) {
  if ( ValueModifierCallback == NULL ) return _Value;
  return (*ValueModifierCallback)(_Value);
}
/******************************************************************************************************************//**
 * @brief Use a user-defined function in the main sketch to modify the raw value read from the arduino
 * @remarks
 * - Raw values are still used behind the scene's only the displayed value is modified
 * @code
 *   exmaple code
 * @endcode
**********************************************************************************************************************/
void PinPoint::SetTo(unsigned int _Value, PinStatus _Status = OKAY) {
  DB((F("PinPoint::SetTo(")));DB((int(Device)));DBC;DB((int(Pin)));DBC;DB((_Value));DBL((")"));

  if ( Device > 16 || Pin > 127 ) return;    // Value Check

  // Drive Buzzer
  if ( PinMode == OUTPUT_BUZZER ) {
    if (_Value < 1) { noTone(Pin); }
    else { tone(Pin, _Value); }
  }

  // Drive Local Pin
  else if ( !Device ) {                                          // Drive a LOCAL Pin
    if ( Pin > 63 ) { XBee->VirtualPin(Pin, _Value, _Status); }                 // Drive a Virtual Pin
    else {
      if ( Pin >= A0 || PinMode == OUTPUT_PWM ) { 
        analogWrite(Pin, _Value); 
        DB((F("PinPoint::SetTo - analogWrite(")));DB((int(Pin)));DBC;DB((_Value));DBL((")"));
      } else { 
        digitalWrite(Pin, _Value);
        DB((F("PinPoint::SetTo - digitalWrite(")));DB((int(Pin)));DBC;DB((_Value));DBL((")"));
      }
    }
  }

  // Drive Remote Pin
  else {
    if ( XBee == NULL ) return;
    if ( Device != XBee->TargetArduinoID() ) XBee->TargetArduinoID( Device );
    if ( PinMode == OUTPUT_PWM ) { XBee->analogWriteB(Pin, _Value); }                  // Set the Remote ArduinoOUTPUT_PWM Pin
    else if ( PinMode == CONTROLLER ) { XBee->VirtualPin(Pin, _Value, _Status); }
    else { XBee->digitalWriteB(Pin, _Value); }                                  // Set the Remote Arduino Digital Pin
  }
}

/******************************************************************************************************************//**
 * @brief Use a user-defined function in the main sketch to modify the raw value read from the arduino
 * @remarks
 * - Raw values are still used behind the scene's only the displayed value is modified
 * @code
 *   exmaple code
 * @endcode
**********************************************************************************************************************/
void PinPoint::AttachValueModifier(int (*_ValueModifierCallback)(int)) {
  ValueModifierCallback = _ValueModifierCallback;
}

