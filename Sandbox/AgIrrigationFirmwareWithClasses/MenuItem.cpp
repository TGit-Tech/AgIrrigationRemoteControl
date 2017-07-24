/******************************************************************************************************************//**
 * @brief  Arduino Sketch firmware to be uploaded onto the AgIrrigationRemoteControl Hand-Remote Device.
 * @see https://github.com/tgit23/AgIrrigationRemoteControl
 * @remarks Version 2017.06.17
 * @todo
 *  - Implement Firmata for Base/Desktop operation
 * @authors 
 *    tgit23        01/2017       Original
 *    tgit23        07/2017       Implemented keypad button interrupts and Non-Blocking functionality
**********************************************************************************************************************/
#include "MenuItem.h"

#if DEBUG>0                             // Activate Debug Messages ( DEBUG defined in PeerRemoteMenu.h )
  #define DBL(x) Serial.println x
  #define DB(x) Serial.print x
  #define DBC Serial.print(", ")
#else                                   // ELSE - Clear Debug Messages
  #define DBL(x)
  #define DB(x)
  #define DBC  
#endif
/******************************************************************************************************************//**
 * @brief  Setup the LCD menu
 * @remarks
 * - Allows a single spot customization to the user interface
 * - Display will show the items in the same order as they are defined here
 * @code
 *   exmaple code
 * @endcode
**********************************************************************************************************************/
MenuItem::MenuItem(PeerIOSerialControl *_XBee, uint8_t _ThisDeviceID, char *_Name, char _ID, uint8_t _Device, uint8_t _Pin, bool _IsOnOff ) {
  XBee = _XBee;
  ThisDeviceID = _ThisDeviceID;
  Name = _Name;
  ID = _ID;
  Device = _Device;
  Pin = _Pin;
  IsOnOff = _IsOnOff;
}
MenuItem::MenuItem() {}; // for pointer usage only

/******************************************************************************************************************//**
 * @brief  Setup the LCD menu
 * @remarks
 * - Allows a single spot customization to the user interface
 * - Display will show the items in the same order as they are defined here
 * @code
 *   exmaple code
 * @endcode
**********************************************************************************************************************/
/*
void MenuItem::AttachSet(uint8_t _DriveDevice = NODEVICE, uint8_t _DrivePin = NOPIN, uint8_t _DriveOnOffVPin = NOPIN ) {
  if ( Set != NULL ) delete Set;
  Set = new uSet;
  if ( _DriveDevice == NODEVICE ) {
    Set->DriveDevice = Device;
    Set->DrivePin = Pin;
  } else {
    Set->DriveDevice = _DriveDevice;
    Set->DrivePin = _DrivePin;
    Set->DriveOnOffVPin = _DriveOnOffVPin;
  }
}
*/
/******************************************************************************************************************//**
 * @brief  Setup the LCD menu
 * @remarks
 * - Allows a single spot customization to the user interface
 * - Display will show the items in the same order as they are defined here
 * @code
 *   exmaple code
 * @endcode
**********************************************************************************************************************/
/*
void MenuItem::AttachPID(MenuItem *_OutputItem, double _Kp, double _Ki, double _Kd, int _POn, int _Direction, uint8_t _ValueOnPin = NOPIN ) {
  if ( SetPID != NULL ) { delete SetPID->OPID; delete SetPID; }       // Clean any previously SetPID on Item
  if ( _OutputItem != NULL ) {
    SetPID = new uSetPID;
    SetPID->OPID = new PID(&SetPID->Input, &SetPID->Output, &SetPID->Setpoint, _Kp, _Ki, _Kd, _POn, _Direction);
    SetPID->OPID->SetOutputLimits(1,1023); // Output limits to Arduino Analog limits
    SetPID->OPID->SetSampleTime(5000);
    SetPID->OPID->SetMode(MANUAL);
    SetPID->OutputItem = _OutputItem;
    SetPID->OutputItem->Set->AttachedPID = SetPID;
    SetPID->ValueOnPin = _ValueOnPin;
  }
}
*/

/******************************************************************************************************************//**
 * @brief Attaches Functions to the Menu-Item and manages the collection of Functions
 * @remarks
 * @code
 *   exmaple code
 * @endcode
**********************************************************************************************************************/
void MenuItem::AttachFunction(eFunctionType _FunctionType, uint8_t _DriveDevice = NODEVICE, uint8_t _DrivePin = NOPIN, 
                int _DriveValue = USER_SELECT_NUMERIC, uint8_t _ValueOnPin = NOPIN, char _ID = NULL, 
                bool _HaltOnAlarm = false, uint8_t _ViolationCount = 1 ) {

  MenuFunction *thisFunction = NULL;

  if ( (_FunctionType == LESS_THAN_ALARM || _FunctionType == EQUAL_TO_ALARM) && _ID == NULL ) {
    _ID = this->ID; bitSet(_ID,5);        // Auto-assign a lower-case ID
  } 
  else if ( (_FunctionType == GREATER_THAN_ALARM || _FunctionType == NOT_EQUAL_TO_ALARM) && _ID == NULL ) {
    _ID = this->ID; bitClear(_ID,5);      // Auto-assign an upper-case ID
  }

  if ( FirstFunction == NULL ) {         // Create first function in Link-List
    FirstFunction = new MenuFunction(_FunctionType, _DriveDevice, _DrivePin, _DriveValue, _ValueOnPin, _ID, _HaltOnAlarm, _ViolationCount ); 
    thisFunction = FirstFunction; 
  }
  else {
    thisFunction = FirstFunction;                                                       // Start at the First function
    while ( thisFunction->Next != NULL ) { thisFunction = thisFunction->Next; }         // Iterate to an empty 'next'
    thisFunction->Next = new MenuFunction(_FunctionType, _DriveDevice, _DrivePin, _DriveValue, _ValueOnPin, _ID, _HaltOnAlarm, _ViolationCount ); 
    thisFunction->Next->Prev = thisFunction;                                            // Assign the 'Prev' function
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
void MenuItem::AttachValueModifier(int (*_ValueModifierCallback)(int)) {
  ValueModifierCallback = _ValueModifierCallback;
}

/******************************************************************************************************************//**
 * @brief Use a user-defined function in the main sketch to modify the raw value read from the arduino
 * @remarks
 * - Raw values are still used behind the scene's only the displayed value is modified
 * @code
 *   exmaple code
 * @endcode
**********************************************************************************************************************/
void MenuItem::Get() {
  DB(("MenuItem::Get("));DB((Name));DBL((")"));
  
  if ( Device < 1 || Device > 15 ) return;
  if ( Pin == NOPIN ) return;

  Value = VALUE_ERR;                                      // Make last Item reading an error
  if ( Device == ThisDeviceID ) {                         //--- Local Pin Read ---
    if ( Pin >= A0 ) {                                    
      Value = analogRead(Pin);
    } else {
      Value = digitalRead(Pin);
    }
    //CheckValue(Item);                                           // Check the new value for Alarms
  }
  else {                                                        //--- Remote Pin Read ---
    if ( Device != XBee->TargetArduinoID() ) XBee->TargetArduinoID( Device ); //Set TransceiverID
#if BLOCKING==0
    Value = VALUE_WAIT;
    wait_reply = millis();
    
    if ( Pin >= A0 ) { PacketID = XBee->analogReadNB(Pin); }
    else { PacketID = XBee->digitalReadNB(Pin); }
    DB(("PacketID="));DBL((PacketID));
/*
    if ( Set != NULL ) {
      if ( Item->Set->DriveOnOffVPin != NOPIN ) {
        wait_reply_ison = millis();
        Item->Set->IsOnStatus = VALUE_WAIT;
        Item->IsOnPacketID = XBee->analogReadNB(Item->Set->DriveOnOffVPin);
        DB(("OnOffPacketID="));DBL((Item->IsOnPacketID));
      }
    }
*/
#else
    if ( Pin >= A0 ) { Value = XBee->analogReadB(Item->Pin); }
    else { Value = XBee->digitalReadB(Pin); }
    /*
    if ( Item->Value >= 0 ) CheckValue(Item);                   // Check the new value for Alarms

    if ( Item->Set != NULL ) {
      if ( Item->Set->DriveOnOffVPin != NOPIN ) {
        Item->Set->OnOffStatus = VALUE_ERR;
        int Ret = XBee->analogReadB(Item->Set->DriveOnOffVPin);
        if ( Ret != -1 ) Item->Set->IsOn = bool(Ret);
      }
    }
    */
    
#endif
  }
/*
  // Update ValueOnPin(s) for the item                             // Retreive Settings on Virtual Pins
  if ( Item->SetPID != NULL ) {
    if ( Item->SetPID->ValueOnPin != NOPIN ) Item->SetPID->Setpoint = XBee->VirtualPin(Item->SetPID->ValueOnPin);
    if ( Item->SetPID->SetOnOffVPin != NOPIN ) Item->SetPID->IsOn = XBee->VirtualPin(Item->SetPID->SetOnOffVPin);
    if (Item->SetPID->IsOn) { Item->SetPID->OPID->SetMode(AUTOMATIC); }
    else { Item->SetPID->OPID->SetMode(MANUAL); }
  }

  uAlarm *Alarm; Alarm = Item->FirstAlarm;
  while ( Alarm != NULL ) {
    if ( Alarm->ValueOnPin != NOPIN ) { Alarm->GetValue() = XBee->VirtualPin(Alarm->ValueOnPin); }
    if ( Alarm->SetOnOffVPin != NOPIN ) { Alarm->IsOn = XBee->VirtualPin(Alarm->SetOnOffVPin); }
    Alarm = Alarm->Next;
  }
  */
}
/******************************************************************************************************************//**
 * @brief Use a user-defined function in the main sketch to modify the raw value read from the arduino
 * @remarks
 * - Raw values are still used behind the scene's only the displayed value is modified
 * @code
 *   exmaple code
 * @endcode
**********************************************************************************************************************/
void MenuItem::Check() {
/*
 * Check if Value has been Returned
 * And check the MenuItems Value against the READ value
//-------------------------------------------- CHECK GOTTEN ITEM -----------------------------------------------
void PeerRemoteMenu::CheckValue(MenuItem *Item) {
  DB(("CheckValue("));DB((Item->Name));DBL((")"));
  
  if ( !bIterating ) return;                                                  // No Alarms unless iterating 
 
  if ( Item->SetPID != NULL ) {                                               // Apply the PID Output
    if ( Item->SetPID->OPID->GetMode() == AUTOMATIC ) {
      if ( Item->Value >= 0 ) {
        Item->SetPID->Input = double(Item->Value);
        Item->SetPID->Setpoint = double(Item->Set->Value);
        
        if ( Item->SetPID->OPID->Compute() ) {
          DB(("SetPID->OPID->Compute ( Input="));DB((Item->SetPID->Input));DBC;
          DB((" Setpoint="));DB((Item->SetPID->Setpoint));DBC;
          DB((" DrivingOutputTo="));DB((Item->SetPID->Output));DBL((" )"));
          SetPin( Item->SetPID->OutputItem->Device, Item->SetPID->OutputItem->Pin, int(Item->SetPID->Output) );
        }
      }
    }
  }
  
  uAlarm *Alarm;                                                              // Check ALARMS
  Alarm = Item->FirstAlarm;
  while ( Alarm != NULL ) {
    
    // Check virtual pin for remote changes
    if ( Alarm->ValueOnPin != NOPIN ) Alarm->GetValue() = XBee->VirtualPin(Alarm->ValueOnPin);
    if ( Alarm->SetOnOffVPin != NOPIN ) Alarm->IsOn = XBee->VirtualPin(Alarm->SetOnOffVPin);

    // Check the value against Alarms
    if ( !Alarm->IsOn ) { Alarm->IsActive = false; }
    else {
      switch ( Alarm->Compare ) {
        case LESS: 
          if ( Item->Value < Alarm->GetValue() ) { Alarm->Violations++; } else { Alarm->Violations = 0; }
          break;
        case GREATER:
          if ( Item->Value > Alarm->GetValue() ) { Alarm->Violations++; } else { Alarm->Violations = 0; }
          break;
        case EQUAL:
          if ( Item->Value = Alarm->GetValue() ) { Alarm->Violations++; } else { Alarm->Violations = 0; }
          break;
        case NOTEQUAL:
          if ( Item->Value != Alarm->GetValue() ) { Alarm->Violations++; } else { Alarm->Violations = 0; }
          break;
      }

      // Activate Alarm is Violations exceed ViolationCount
      Alarm->IsActive = ( Alarm->Violations >= Alarm->ViolationCount );
      if ( Alarm->IsActive ) {
        if ( Alarm->HaltOnAlarm ) AlarmHalt = true;
        SetPin( Alarm->DriveDevice, Alarm->DrivePin, Alarm->DriveValue );
      }
    }
    Alarm = Alarm->Next;
  }
*/  
}

/******************************************************************************************************************//**
 * @brief Use a user-defined function in the main sketch to modify the raw value read from the arduino
 * @remarks
 * - Raw values are still used behind the scene's only the displayed value is modified
 * @code
 *   exmaple code
 * @endcode
**********************************************************************************************************************/
void MenuItem::GetStatusLine(char *Status) {
  // Loop through Functions to determine ON functions
  /*
   * // Display Status ( Top Row Right )
  char Stat[16] = "";uint8_t p = 0;uint8_t i = 0; bool bAlarmActive = false;
  if ( Func == MAIN || Func == SET ) {
    bool bShowAll = true;
    if ( CurrItem->Set != NULL ) {
      if ( CurrItem->Set->DriveOnOffVPin != NULL ) {
        bShowAll = false;
        if ( CurrItem->Set->IsOn ) { LCD->setCursor(13,0);LCD->print("ON"); }
        else { LCD->setCursor(13,0);LCD->print("OFF"); }
      }
    }

    if ( bShowAll ) {
      //---------------------------------------------------------------------------------------
      MenuItem *Item; Item = FirstItem; uAlarm *Alarm;                      // -- Loop Items --
      while ( Item != NULL ) {                                              
        if ( Item->SetPID != NULL ) {                                       // For active PIDs
          if ( Item->SetPID->OPID->GetMode() == AUTOMATIC ) {               // Display PID [IO]
            if (i<16) { Stat[i]='[';i++; }
            if (i<16) { Stat[i]=Item->ID;i++; }
            if (i<16) { Stat[i]=Item->SetPID->OutputItem->ID;i++; }
            if (i<16) { Stat[i]=']';i++; }
            p = i;
          }
        }
        Alarm = Item->FirstAlarm;                                           // Loop Alarms
        while ( Alarm != NULL ) {                                           
          if ( Alarm->IsActive ) {                                          // For active Alarms
            i = p;                                                          // Place active right after PID
            if (i<16) { Stat[i]='!';i++; }                                  // Display active alarm !x!
            if (i<16) { Stat[i]=Alarm->ID;i++; }
            if (i<16) { Stat[i]='!';i++; }
            break;                                                          // Exit on first active alarm
          }
          if ( Alarm->IsOn && i<16 ) { Stat[i]=Alarm->ID;i++; }             // For On Alarms
          Alarm = Alarm->Next;
        }
        Item = Item->Next;                                                  // -- Next Item --
      }
      //--------------------------------------------------------------------------------------
      Stat[i]='\0'; // Terminate
      if ( strlen(Devices[CurrItem->Device]) > 16-i ) { LCD->setCursor(strlen(Devices[CurrItem->Device]),0); }
      else { LCD->setCursor(16-i,0); }
      LCD->print(Stat);
    }
   */
}


