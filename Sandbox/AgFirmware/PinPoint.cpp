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

#if DEBUG>0                                 // Activate Debug Messages ( DEBUG defined in PeerRemoteMenu.h )
  #define DBL(x) Serial.println x
  #define DB(x) Serial.print x
  #define DBC Serial.print(", ")
#else                                       // ELSE - Clear Debug Messages
  #define DBL(x)
  #define DB(x)
  #define DBC  
#endif

PeerIOSerialControl *PinPoint::XBee = NULL; // Static XBee for all PinPoints
uint8_t PinPoint::ThisDeviceID = 0;         // ThisDevice Identification Num for all PinPoints

/******************************************************************************************************************//**
 * @brief Default Constructor
 * @remarks
 * @code
 *   exmaple code
 * @endcode
**********************************************************************************************************************/
PinPoint::PinPoint(uint8_t _Device, uint8_t _Pin, ePinType _PinType, char *_Name, char _ID ) {
  
  // Assign Variables
  Device = _Device; Pin = _Pin; PinType = _PinType; Name = _Name; ID = _ID;  
  
  // Set whether pin is On/Off value and auto-create a SET_PIN control for pins that need set
  if ( Pin <= A0 && PinType != PWM ) IsOnOff = true;  
  if ( PinType == SETTABLE || PinType == USERCONTROL || PinType == PWM ) {
    FirstControl = new UserControl( this, SET_PIN, this, ID );
  }

  // Set the pinMode
  if ( Device == ThisDeviceID ) {
    switch ( PinType ) {
      case INPIN:       pinMode(Pin,INPUT);break;
      case INPINHIGH:   pinMode(Pin,INPUT_PULLUP);break;
      case OUTPIN:      pinMode(Pin,OUTPUT);break;
      case BUZZPIN:     pinMode(Pin,OUTPUT);break;
      case SETTABLE:    pinMode(Pin,OUTPUT);break;
      case PWM:         pinMode(Pin,OUTPUT);break;
    }
  }
}
//---------------------------------------------------------------------------------------------------------------------
// Constructor - No-Name and No-ID ( A pin NOT for Menu usage )
//---------------------------------------------------------------------------------------------------------------------
PinPoint::PinPoint(uint8_t _Device, uint8_t _Pin, ePinType _PinType )
:PinPoint(_Device, _Pin, _PinType, NULL, NULL ) {
}

//---------------------------------------------------------------------------------------------------------------------
// UltraSonic Special Constructor - uses two pins for one value
//---------------------------------------------------------------------------------------------------------------------
PinPoint::PinPoint(uint8_t _Device, uint8_t _Pin, ePinType _PinType, char *_Name, char _ID, uint8_t _TrigPin, uint8_t _EchoPin )
:PinPoint(_Device, _Pin, _PinType, _Name, _ID ) { 
  if ( Device == ThisDeviceID ) {
    pinMode(_TrigPin, OUTPUT);
    pinMode(_EchoPin, INPUT);
    Sonar = new NewPing(_TrigPin, _EchoPin, 400);
  }
}

/******************************************************************************************************************//**
 * @brief Reads the Value and stores it; Value is then gotten by calling GetRawValue() or GetModifiedValue()
 * @remarks
 * - Automatically calls ApplyControls() if a valid value is gotten
 * @code
 *   APin.ReadValue();
 *   while ( APin.Status() == WAIT ) { // Loop Till Value is Retreived }
 *   GottenValue = APin.GetValue();
 * @endcode
**********************************************************************************************************************/
void PinPoint::ReadValue(bool _ForceBlocking = false) {
  DB(("PinPoint::ReadValue(")); if ( _ForceBlocking ) { DB(("Blocking"));DBC; }
  DB(("Device="));DB((Device));DBC;DB(("Pin="));DB((Pin));DBL((")"));

  mStatus = ERR; mPacketID = -1;
  if ( Device > 16 || Pin > 127 ) return;    // Value Check
  
  if ( Device == ThisDeviceID ) {                         //--- Local Pin Read ---
    if ( PinType == SONICPIN ) {                          // SonicPin Only works on local device
      if ( Sonar != NULL ) {
        mValue = Sonar->ping_in();                        // Measure the Distance
        if ( mValue != -1 ) mStatus = OKAY;               
        XBee->VirtualPin(Pin, mValue);                    // Record distance on the Virtual Pin
      }
    }
    else if ( PinType == PWM ) { mValue = XBee->analogReadOutput(Pin); }
    else if ( Pin >= A0 ) { mValue = analogRead(Pin); }
    else { mValue = digitalRead(Pin); }
    if ( mValue != -1 ) { mStatus = OKAY; ApplyControls(); }
    DB(("PinPoint::ReadValue LOCAL="));DBL((mValue));
  }
  else if ( _ForceBlocking ) {
    if ( Device != XBee->TargetArduinoID() ) XBee->TargetArduinoID( Device ); //Set TransceiverID
    if ( Pin >= A0 || PinType == PWM ) { mValue = XBee->analogReadB(Pin); }
    else { mValue = XBee->digitalReadB(Pin); }
    if ( mValue != -1 ) { mStatus = OKAY; ApplyControls(); }
    DB(("PinPoint::ReadValue BLOCKED="));DBL((mValue));
  }
  else {                                                        //--- Remote Pin Read ---
    if ( Device != XBee->TargetArduinoID() ) XBee->TargetArduinoID( Device ); //Set TransceiverID
    mWaitStart = millis();
    if ( Pin >= A0 || PinType == PWM || PinType == USERCONTROL ) { mPacketID = XBee->analogReadNB(Pin); }
    else { mPacketID = XBee->digitalReadNB(Pin); }
    mStatus = WAIT;
    DBL(("PinPoint::ReadValue REQUESTED"));
  }  
}

/******************************************************************************************************************//**
 * @brief Checks XBee communications and returns 'true' if the pin Status has changed
 * @remarks
 * - Automatically calls ApplyControls() if value is available
 * @code
 *   exmaple code
 * @endcode
**********************************************************************************************************************/
bool PinPoint::UpdateAvailable() {
  XBee->Available();
  if ( mStatus == WAIT ) {
    if ( mPacketID != -1 ) {
      int Ret = XBee->GetReply(mPacketID);
      if ( Ret != -1 ) {
        mValue = Ret;
        mStatus = OKAY;
        ApplyControls();
        return true;
      }
    }
    if ( millis() - mWaitStart > XBee->Timeout() ) {
      mPacketID = -1;
      mStatus = ERR;
      return true;
    }
  }
  return false;  
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
  DB(("PinPoint::SetTo("));DB((Device));DBC;DB((Pin));DBC;DB((_Value));DBL((")"));

  if ( Device > 16 || Pin > 127 ) return;    // Value Check

  // Drive Buzzer
  if ( PinType == BUZZPIN ) {
    if (_Value < 1) { noTone(Pin); }
    else { tone(Pin, _Value); }
  }

  // Drive Local Pin
  else if ( Device == ThisDeviceID ) {                                          // Drive a LOCAL Pin
    if ( Pin > 63 ) { XBee->VirtualPin(Pin, _Value, _Status); }                 // Drive a Virtual Pin
    else {
      if ( Pin >= A0 || PinType == PWM ) { 
        analogWrite(Pin, _Value); 
        DB(("PinPoint::SetTo - analogWrite("));DB((Pin));DBC;DB((_Value));DBL((")"));
      } else { 
        digitalWrite(Pin, _Value);
        DB(("PinPoint::SetTo - digitalWrite("));DB((Pin));DBC;DB((_Value));DBL((")"));
      }
    }
  }

  // Drive Remote Pin
  else {
    if ( XBee == NULL ) return;
    if ( Device != XBee->TargetArduinoID() ) XBee->TargetArduinoID( Device );
    if ( PinType == PWM ) { XBee->analogWriteB(Pin, _Value); }                  // Set the Remote Arduino PWM Pin
    else if ( PinType == USERCONTROL ) { XBee->VirtualPin(Pin, _Value, _Status); }
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

/******************************************************************************************************************//**
 * @brief Adds a 'UserControl' to the Controls Link-List
 * @remarks
 * @code
 *   exmaple code
 * @endcode
**********************************************************************************************************************/
void PinPoint::Controls(PinPoint* _OutPin, eControlType _ControlType, char _ID, uint8_t OnDevice = 0, PinPoint* _StorePin = NULL) {
  if ( OnDevice == 0 || OnDevice == ThisDeviceID ) {
    if ( FirstControl == NULL ) {
      FirstControl = new UserControl( this, _ControlType, _OutPin, _ID, _StorePin );
    } else {
      UserControl *thisControl = FirstControl;
      while ( thisControl->Next != NULL ) { thisControl = thisControl->Next; }
      thisControl->Next = new UserControl( this, _ControlType, _OutPin, _ID, _StorePin );
      thisControl->Next->Prev = thisControl;
    }
  }
}

/******************************************************************************************************************//**
 * @brief Adds a 'UserControl' to the Controls Link-List
 * @remarks
 * @code
 *   exmaple code
 * @endcode
**********************************************************************************************************************/
void PinPoint::ApplyControls() {
  UserControl *thisControl = FirstControl;
  while ( thisControl != NULL ) {
    if ( thisControl->ControlType != SET_PIN ) thisControl->Apply();
    thisControl = thisControl->Next;
  }
}
/******************************************************************************************************************//**
 * @brief Adds a 'UserControl' to the Controls Link-List
 * @remarks
 * @code
 *   exmaple code
 * @endcode
**********************************************************************************************************************/
void PinPoint::Controls(PinPoint* _OutPin, eControlType _ControlType, char _ID, uint8_t OnDevice, double Kp, double Ki, double Kd, int POn, int PIDDirection, PinPoint * _StorePin = NULL) {
  if ( OnDevice == 0 || OnDevice == ThisDeviceID ) {
    if ( FirstControl == NULL ) {
      FirstControl = new UserControl( this, _ControlType, _OutPin, _ID, Kp, Ki, Kd, POn, PIDDirection, _StorePin );
    } else {
      UserControl *thisControl = FirstControl;
      while ( thisControl->Next != NULL ) { thisControl = thisControl->Next; }
      thisControl->Next = new UserControl( this, _ControlType, _OutPin, _ID, Kp, Ki, Kd, POn, PIDDirection, _StorePin );
      thisControl->Next->Prev = thisControl;
    }
  }
}
