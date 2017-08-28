/******************************************************************************************************************//**
 * @file PinPoint.cpp
 * @brief Class definition for a PinPoint on the https://github.com/tgit23/AgIrrigationRemoteControl project.
 * @remarks 
 *  - PinPoints define any hardwired Accessory like relays, meters, buzzers, lights, ultrasonic, etc...
 *  - PinPoint defines Read / Write functions for the Pin
 *  - PinPoint acts as a 'UserControl' objects collection for this paticular Pins ( Read->Control ) process.
 *  - CONTROLLER pin serves both local save of Controller SetPoint & Status
 *  - CONTROLLER pin also serves as remote get/set of Controller SetPoint & Status
 *  - Of Course the CONTROLLER 'ControlPin' is always on the local device!
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
PinPoint::PinPoint(uint8_t *_Device, uint8_t *_Pin, char *_DeviceName, LiquidCrystal *_LCD) {
  Device = _Device;
  Pin = _Pin;
  DeviceName = _DeviceName;
  LCD = _LCD;
}

//---------------------------------------------------------------------------------------------------------------------
// Mode()
//---------------------------------------------------------------------------------------------------------------------
void PinPoint::Mode(uint8_t _Mode) {
  PinMode = _Mode;
  
  switch ( PinMode ) {
    case INPUT:           
      if ( Device ) pinMode(Pin, INPUT);
      IsOnOff = ( Pin < A0 ); 
      break;
    case INPUT_PULLUP:    
      if ( Device ) pinMode(Pin, INPUT_PULLUP);
      IsOnOff = ( Pin < A0 ); 
      break;
    case OUTPUT:          
      if ( Device ) pinMode(Pin, OUTPUT);
      IsOnOff = ( Pin < A0 ); 
      break;
    case INPUT_SONIC:     break;
    case OUTPUT_BUZZER:   
      if ( Device ) pinMode(Pin, OUTPUT);
      break;
    case OUTPUT_PWM:      
      if ( Device ) pinMode(Pin, OUTPUT);
      break;
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

//---------------------------------------------------------------------------------------------------------------------
// ReadValue()
//---------------------------------------------------------------------------------------------------------------------
void PinPoint::ReadValue(bool _ForceBlocking = false) {
  DBL((""));DB((F("PinPoint::ReadValue("))); if ( _ForceBlocking ) { DB((F("Blocking")));DBC; }
  DBF(("Device="));DB((int(Device)));DBC;DB((F("Pin=")));DB((int(Pin)));DB((")"));

  mStatus = ERR; mPacketID = -1;              // Set Default Status
  if ( Device > 16 || Pin > 127 ) return;     // Value Range Check
  mState = WAIT;                              // Set Default State
  
  if ( !Device ) {                                          //--- ThisDevice Pin Read ---
    if ( PinMode == INPUT_SONIC ) {                         // SonicPin Only works on local device
      if ( Sonar != NULL ) {
        mValue = Sonar->ping_in();                          // Store Measured Distance               
        XBee->VirtualPin(Pin, mValue);                      // Record distance on the Virtual Pin
      }
    }
    
    else if ( PinMode == OUTPUT_PWM ) { 
      mValue = XBee->analogReadOutput(Pin);                 // Read PWM Setting through Arduino registers
    }
    else if ( Pin >= A0 ) { mValue = analogRead(Pin); }     // Analog Read
    else { mValue = digitalRead(Pin); }                     // Digital Read
    
    if ( mValue != -1 ) { mStatus = OKAY; }                 // Set Status
    mState = READY;                                         // Set State
    DBF((" - LOCAL="));DBL((mValue));                       // Debug Show value obtained
  }
  
  else if ( _ForceBlocking ) {                                                    //--- Remote Device Pin Read ---
    if ( Device != XBee->TargetArduinoID() ) XBee->TargetArduinoID( Device );     // Set XBee TransceiverID
    if ( Pin >= A0 || PinMode ==OUTPUT_PWM ) { mValue = XBee->analogReadB(Pin); } // Analog Read
    else { mValue = XBee->digitalReadB(Pin); }                                    // Digital Read
    if ( mValue != -1 ) { mStatus = OKAY; }                                       // Set Status
    mState = READY;                                                               // Set State
    DBF((" - REMOTE BLOCKED="));DBL((mValue));                                    // Debug Show Value
  }
  
  else {                                                                          //--- Remote Device Pin Read ---
    if ( Device != XBee->TargetArduinoID() ) XBee->TargetArduinoID( Device );     // Set XBee TransceiverID
    mWaitStart = millis();                                                        // Start WAIT clock
    if ( Pin >= A0 || PinMode ==OUTPUT_PWM || PinMode == CONTROLLER ) { 
      mPacketID = XBee->analogReadNB(Pin); }                                      // Analog Read
    else { mPacketID = XBee->digitalReadNB(Pin); }                                // Digital Read
    DBFL((" - REMOTE REQUESTED"));                                                // Debug Show value REQUESTED
  }
  Display();                                                // Update the Display
}

//---------------------------------------------------------------------------------------------------------------------
// GetRawValue ()
//---------------------------------------------------------------------------------------------------------------------
int PinPoint::GetRawValue() {
  if ( PinMode == CONTROLLER ) { return XBee->VirtualPin(Pin); }
  return mValue;
}

//---------------------------------------------------------------------------------------------------------------------
// AttachValueModifier ()
//---------------------------------------------------------------------------------------------------------------------
void PinPoint::AttachValueModifier(int (*_ValueModifierCallback)(int)) {
  ValueModifierCallback = _ValueModifierCallback;
}

//---------------------------------------------------------------------------------------------------------------------
// GetModifiedValue ()
//---------------------------------------------------------------------------------------------------------------------
int PinPoint::GetModifiedValue() {
  DBF(("PinPoint::GetModifiedValue(Raw="));DB((mValue));DBFL((")"));
  
  if ( ValueModifierCallback == NULL ) return mValue;
  return (*ValueModifierCallback)(mValue);
}

//---------------------------------------------------------------------------------------------------------------------
// ModifyValue (int)
//---------------------------------------------------------------------------------------------------------------------
int PinPoint::ModifyValue(int _Value) {
  DBF(("PinPoint::ModifyValue("));DB((mValue));DBFL((")"));
  
  if ( ValueModifierCallback == NULL ) return _Value;
  return (*ValueModifierCallback)(_Value);
}

//---------------------------------------------------------------------------------------------------------------------
// Get State()
//---------------------------------------------------------------------------------------------------------------------
eState PinPoint::State() {
  XBee->Available();
  if ( mState == WAIT ) {
    
    if ( mPacketID != -1 ) {                        // Check for a GetReply()
      int Ret = XBee->GetReply(mPacketID);
      if ( Ret != -1 ) {
        DBF(("PinPoint::State(WAIT) -> "));         // Debug show WAIT->READY State Change
        if ( Pin > 63 ) {
          mStatus = (Ret & 0x3000) >> 12;           // Parse Status from Virtual Pin Reads
          mValue = Ret & 0x0FFF;                    // Parse Value from Virtual Pin Reads
          DBF(("VirtualPin "));                     // Debug show Virtual Pin
        } else {
          mValue = Ret;                             // Set the Returned Value
          mStatus = OKAY;                           // Set Status as OKAY
        }
        mState = READY;                             // Switch State to READY
        DBF(("READY - Status="));DB((mStatus));     // Debug Show Status
        DBF((" Value="));DBL((mValue));             // Debug Show Value
        Display();                                  // Update the Display with Value
      }
    }
    
    // Timeout
    if ( mState == WAIT && millis() - mWaitStart > XBee->Timeout() ) {
      mPacketID = -1;
      mStatus = ERR; mState = READY;                // State to READY and Status to ERR
      DBFL(("PinPoint::State(WAIT) -> TIMEOUT!"));  // Debug Show Timeout
      Display();                                    // Update Display
    }
  }
  return mState;
}

//---------------------------------------------------------------------------------------------------------------------
// Set State()
//---------------------------------------------------------------------------------------------------------------------
void PinPoint::State(eState _State) {
  mState = _State;
  if ( mState == SETTING && CurrControl != NULL ) { CurrControl->Display(); }
}

//---------------------------------------------------------------------------------------------------------------------
// Get Status()
//---------------------------------------------------------------------------------------------------------------------
PinStatus PinPoint::Status() {
  if ( !Device && Pin > 63 ) mStatus = XBee->VirtualPinStatus(Pin);    // For PinMode CONTROLLER
  return mStatus;
}

//---------------------------------------------------------------------------------------------------------------------
// SetTo()
//---------------------------------------------------------------------------------------------------------------------
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
    if ( Pin > 63 ) { 
      XBee->VirtualPin(Pin, _Value, _Status); 
      DB((F("PinPoint::SetTo - VirtualPin(")));DB((int(Pin)));DBC;DB((_Value));DBC;DB((_Status));DBL((")"));
    }                 // Drive a Virtual Pin
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
    else if ( PinMode == CONTROLLER ) { 
      int ValueWithStatus = ((_Value << 0) & 0x0FFF) + ((_Status << 12) & 0x3000);
      XBee->analogWriteB(Pin,ValueWithStatus);
    }
    else { XBee->digitalWriteB(Pin, _Value); }                                  // Set the Remote Arduino Digital Pin
  }
}

//---------------------------------------------------------------------------------------------------------------------
// Display
//---------------------------------------------------------------------------------------------------------------------
void  PinPoint::Display() {
  DBFL(("PinPoint::Display()"));
  if ( LCD == NULL ) return;
    
  // Display Device ( Top Row Left )
  LCD->clear();LCD->setCursor(0,0);
  LCD->print( DeviceName );

  // Display OnControls
  if ( PinMode != CONTROLLER ) {
    int lastspace = 15 - strlen( DeviceName ); int pos = 15;
    for ( int Idx = 15; Idx >= 0; Idx-- ) {
      if ( UserControl::OnControls[Idx] != NULL ) {
        if ( UserControl::OnControls[Idx] != ' ' )  {
          if ( pos < lastspace ) break;
          LCD->setCursor(pos,0);LCD->print(UserControl::OnControls[Idx]);
        }
      }
    }
  } else {
    if ( mStatus == ISON ) { LCD->setCursor(14,0);LCD->print("On"); }
    else if ( mStatus == ISOFF ) { LCD->setCursor(13,0);LCD->print("Off"); }
    else if ( mStatus == ERR ) { LCD->setCursor(15,0);LCD->print("?"); }
  }
  
  //Display the Pin Name ( Bottom Row )
  LCD->setCursor(0,1);LCD->print( Name );LCD->print("=");

  // Display Value
  if ( mState == WAIT ) { LCD->print("?"); }
  else if ( mState == READY ) {
    if ( mState == ERR ) { LCD->print("ERR"); }
    else if ( IsOnOff ) {
      if ( mValue == 0 ) { LCD->print("Off"); }
      else { LCD->print("On"); }
    } else {
      LCD->print(GetModifiedValue());
    }
  }   
}

