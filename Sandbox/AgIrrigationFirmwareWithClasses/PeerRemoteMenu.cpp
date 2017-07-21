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
#include "PeerRemoteMenu.h"

#if XBEECONFIG>0                        // XBEECONFIG defined in PeerRemoteMenu.h
  #undef BLOCKING                       // Undefine any previously defined BLOCKING
  #define BLOCKING 1                    // Digi-Xbee-XCTU configuration software has problems with active interupts
#endif

#if DEBUG>0                             // Activate Debug Messages ( DEBUG defined in PeerRemoteMenu.h )
  #define DBL(x) Serial.println x
  #define DB(x) Serial.print x
  #define DBC Serial.print(", ")
#else                                   // ELSE - Clear Debug Messages
  #define DBL(x)
  #define DB(x)
  #define DBC  
#endif

//=====================================================================================================================
//------------------------------ STATIC FUNCTIONS ---------------------------------------------------------------------
//=====================================================================================================================
/******************************************************************************************************************//**
 * @brief  Receives button press ( adc_value ), decodes it, and saves it to be processed by loop()
 * @remarks
 * - This function must be static because it is called by an ISR() routine
 * - Button_Debounce_ms defined in 'PeerRemoteMenu.h' only allows a button press to register every ? milliseconds.
 * @code
 *   ButtonCheck(analogRead(0));
 * @endcode
**********************************************************************************************************************/
static void PeerRemoteMenu::ButtonCheck(int adc_value) {
  //DB(("ButtonCheck("));DB((adc_value));DBL((")"));
  
  unsigned long clk = millis();
  if ( clk - last_bpress_millis < BUTTON_DEBOUNCE_MS ) return;         // Debounce button presses

  if (adc_value > 1000) { last_bpress = NONE; }
  else if (adc_value < 50) { last_bpress = RIGHT; }
  else if (adc_value < 195) { last_bpress = UP; }
  else if (adc_value < 380) { last_bpress = DOWN; }
  else if (adc_value < 555) { last_bpress = LEFT; }
  else if (adc_value < 790) { last_bpress = SELECT; }

  if ( prev_bpress == last_bpress ) { ButtonHeld++; } else { ButtonHeld=0; }  // Determine is same button is held
  prev_bpress = last_bpress;                                          // Record button for above
  if ( last_bpress != NONE ) {
    DBL(("Button Pressed"));
    last_bpress_millis = millis();                                    // Clk for Debounce
  }
}

#if BLOCKING==0
/******************************************************************************************************************//**
 * @brief  ISR ( Interrupt Service Routine ) for Keypad Up, Down, and Right arrow buttons.
 * @remarks
 * - PCINT1_vect Pin Change Interrupt will not trigger on Left or Select buttons ( digital threshold? )
 * - The interrupt stores the button pressed by calling ButtonCheck() and processes it when the loop() is called.
 * - The original SoftwareSerial Library calls ALL Interrupts so a modified 'SSoftwareSerial' must be used to compile
**********************************************************************************************************************/
ISR(PCINT1_vect) {
  PeerRemoteMenu::ButtonCheck(analogRead(0));
}
#endif
    
//=====================================================================================================================
//------------------------------ MENU ITEM METHODS --------------------------------------------------------------------
//=====================================================================================================================               
/******************************************************************************************************************//**
 * @brief  Setup the LCD menu
 * @remarks
 * - Allows a single spot customization to the user interface
 * - Display will show the items in the same order as they are defined here
 * @code
 *   exmaple code
 * @endcode
**********************************************************************************************************************/
MenuItem::MenuItem() {
  
}

/******************************************************************************************************************//**
 * @brief  Setup the LCD menu
 * @remarks
 * - Allows a single spot customization to the user interface
 * - Display will show the items in the same order as they are defined here
 * @code
 *   exmaple code
 * @endcode
**********************************************************************************************************************/
void MenuItem::AttachSet(uint8_t _DriveDevice = NODEVICE, uint8_t _DrivePin = NOPIN ) {
  if ( Set != NULL ) delete Set;
  Set = new uSet;
  if ( _DriveDevice == NODEVICE ) {
    Set->DriveDevice = Device;
    Set->DrivePin = Pin;
  } else {
    Set->DriveDevice = _DriveDevice;
    Set->DrivePin = _DrivePin;
  }
}

/******************************************************************************************************************//**
 * @brief  Setup the LCD menu
 * @remarks
 * - Allows a single spot customization to the user interface
 * - Display will show the items in the same order as they are defined here
 * @code
 *   exmaple code
 * @endcode
**********************************************************************************************************************/
void MenuItem::AttachPID(MenuItem *_OutputItem, double _Kp, double _Ki, double _Kd, int _POn, int _Direction, uint8_t _SetVPin = NOPIN ) {
  if ( SetPID != NULL ) { delete SetPID->OPID; delete SetPID; }       // Clean any previously SetPID on Item
  if ( _OutputItem != NULL ) {
    SetPID = new uSetPID;
    SetPID->OPID = new PID(&SetPID->Input, &SetPID->Output, &SetPID->Setpoint, _Kp, _Ki, _Kd, _POn, _Direction);
    SetPID->OPID->SetOutputLimits(1,1023); // Output limits to Arduino Analog limits
    SetPID->OPID->SetSampleTime(5000);
    SetPID->OPID->SetMode(MANUAL);
    SetPID->OutputItem = _OutputItem;
    SetPID->OutputItem->Set->AttachedPID = SetPID;
    SetPID->SetVPin = _SetVPin;
  }
}

/******************************************************************************************************************//**
 * @brief     AttachAlarm() - Allows user to set 'boundaries' on Menu-items that activate an alarm condition.
 * @remarks
 * @code
 *   exmaple code
 * @endcode
**********************************************************************************************************************/
void MenuItem::AttachAlarm(eCompare _Compare, uint8_t _DriveDevice = NODEVICE, uint8_t _DrivePin = NOPIN, int _DriveValue = 0, bool _HaltOnAlarm = false, uint8_t _ViolationCount = 1, char _ID = NULL, uint8_t _SetVPin = NOPIN ) {
  
  uAlarm *thisAlarm = NULL;
  if ( FirstAlarm == NULL ) { FirstAlarm = new uAlarm; thisAlarm = FirstAlarm; }  // Create first alarm in Link-List
  else {
    thisAlarm = FirstAlarm;                                                       // Start at the First alarm
    while ( thisAlarm->Next != NULL ) { thisAlarm = thisAlarm->Next; }            // Iterate to an empty 'next'
    thisAlarm->Next = new uAlarm;                                                 // Create a new alarm
    thisAlarm->Next->Prev = thisAlarm;                                            // Assign the 'Prev' alarm
    thisAlarm = thisAlarm->Next;                                                  // Move into new alarm
  }
  
  if ( _ID != NULL ) { thisAlarm->ID = _ID; }                                     // Assign given Alarm-ID
  else {                                                                          // ELSE; auto assign one
    thisAlarm->ID = this->ID;                                                     // Grab the Menu-Item ID
    if ( _Compare == LESS || _Compare == EQUAL ) bitSet(thisAlarm->ID,5);         // Create lower-case
    if ( _Compare == GREATER || _Compare == NOTEQUAL ) bitClear(thisAlarm->ID,5); // Create upper-case
  }
  
  thisAlarm->Compare = _Compare;                                                  // Assign arguments
  thisAlarm->DriveDevice = _DriveDevice;
  thisAlarm->DrivePin = _DrivePin;
  thisAlarm->DriveValue = _DriveValue;
  thisAlarm->HaltOnAlarm = _HaltOnAlarm;
  thisAlarm->ViolationCount = _ViolationCount;
  thisAlarm->SetVPin = _SetVPin;
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

//=====================================================================================================================
//------------------------------ PEER-REMOTE-MENU METHODS -------------------------------------------------------------
//=====================================================================================================================
/******************************************************************************************************************//**
 * @brief  Setup the LCD menu
 * @remarks
 * - Allows a single spot customization to the user interface
 * - Display will show the items in the same order as they are defined here
 * @code
 *   exmaple code
 * @endcode
**********************************************************************************************************************/
PeerRemoteMenu::PeerRemoteMenu(PeerIOSerialControl *_XBee, LiquidCrystal *_LCD, uint8_t _ThisDeviceID, uint8_t _BuzzerPin = NOPIN ) {
  LCD = _LCD;
  XBee = _XBee;
  BuzzerPin = _BuzzerPin;
#if BLOCKING==1
  noInterrupts();               // switch interrupts off while messing with their settings  
  PCICR =0x02;                  // Enable 'PCIE1' bit of PCICR Pin Change Interrupt the PCINT1 interrupt
  PCMSK1 = 0b00000001;          // Pin Change Interrupt Mask ( NA, RESET, A5, A4, A3, A2, A1, A0 ) - Activate A0              
  interrupts();                 // turn interrupts back on
#endif
  ThisDeviceID = _ThisDeviceID;
}

/******************************************************************************************************************//**
 * @brief  Setup the LCD menu
 * @remarks
 * - Allows a single spot customization to the user interface
 * - Display will show the items in the same order as they are defined here
 * @code
 *   exmaple code
 * @endcode
**********************************************************************************************************************/
MenuItem *PeerRemoteMenu::AddMenuItem( char *_Name, char _ID, uint8_t _Device, uint8_t _Pin, bool _IsOnOff = false ) {
  MenuItem *thisItem = NULL;
  if ( FirstItem == NULL ) {
    FirstItem = new MenuItem;
    thisItem = FirstItem;
  } else {
    thisItem = FirstItem;
    while ( thisItem->Next != NULL ) { thisItem = thisItem->Next; }
    thisItem->Next = new MenuItem;
    thisItem->Next->Prev = thisItem;
    thisItem = thisItem->Next;
  }
  thisItem->Name = _Name;
  thisItem->ID = _ID;
  thisItem->Device = _Device;
  thisItem->Pin = _Pin;
  thisItem->IsOnOff = _IsOnOff;
  return thisItem;
}

/******************************************************************************************************************//**
 * @brief  Setup the LCD menu
 * @remarks
 * - Allows a single spot customization to the user interface
 * - Display will show the items in the same order as they are defined here
 * @code
 *   exmaple code
 * @endcode
**********************************************************************************************************************/
void PeerRemoteMenu::Start( MenuItem *StartItem ) {
  MenuItem *Item = FirstItem; uAlarm *Alarm = NULL; unsigned int Offset = 0;
  
  // Assign an EpromOffset to all items that need to have non-volatile values
  // Then Read those values from Eprom
  do {
    if ( Item->Set != NULL ) {
      Item->Set->EpromOffset = Offset;
      Item->Set->Value = EEPROMGet(Offset);
      Offset = Offset + 2; // Reserve 2-bytes
    }
    if ( Item->SetPID != NULL ) {
      bool bIsAuto = false;
      Item->SetPID->EpromOffset = Offset;
      Item->SetPID->Setpoint = EEPROMGet(Offset,&bIsAuto);
      if ( bIsAuto ) { 
        Item->SetPID->OPID->SetMode(AUTOMATIC);
      } else {
        Item->SetPID->OPID->SetMode(MANUAL);
      }
      Offset = Offset + 3; // Reserve 3-bytes
    }
    Alarm = Item->FirstAlarm;
    while ( Alarm != NULL ) {
      Alarm->EpromOffset = Offset;
      Alarm->Value = EEPROMGet(Offset,&Alarm->IsOn);
      Offset = Offset + 3;  // Reserve 3-bytes
      Alarm = Alarm->Next;
    }
    Item = Item->Next;
  } while ( Item != FirstItem && Item != NULL );

  CurrItem = StartItem;
  LCD_display();
}

/******************************************************************************************************************//**
 * @brief  Setup the LCD menu
 * @remarks
 * - Allows a single spot customization to the user interface
 * - Display will show the items in the same order as they are defined here
 * @code
 *   exmaple code
 * @endcode
**********************************************************************************************************************/
void PeerRemoteMenu::AddDevice ( uint8_t _Device, char *_Name ) {
  if ( _Device > 0 && _Device < 16 ) Devices[_Device] = _Name;
}

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
void PeerRemoteMenu::EEPROMSet(unsigned int Offset, int Value, int IsOn = -1) {
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

/******************************************************************************************************************//**
 * @brief  Read Menu Item Values from Arduino EEPROM non-volitale memory.
 * @see    EEPROMSet for Addressing notation
 * @code
 *   exmaple code
 * @endcode
**********************************************************************************************************************/
int PeerRemoteMenu::EEPROMGet(unsigned int Offset, bool *IsOn = NULL) {
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

/******************************************************************************************************************//**
 * @brief  Obtain menu values
 * @remarks
 * @code
 *   exmaple code
 * @endcode
**********************************************************************************************************************/
void PeerRemoteMenu::GetItem(MenuItem *Item) {
  DB(("GetItem("));DB((Item->Name));DBL((")"));
  if ( Item->Device < 1 || Item->Device > 15 ) return;
  if ( Item->Pin == NOPIN ) return;

  Item->Value = VALUE_ERR;                                      // Make last Item reading an error
  if ( Item->Device == ThisDeviceID ) {                         //--- Local Pin Read ---
    if ( Item->Pin >= A0 ) {                                    
      Item->Value = analogRead(CurrItem->Pin);
    } else {
      Item->Value = digitalRead(CurrItem->Pin);
    }
    CheckValue(Item);                                           // Check the new value for Alarms
  }
  else {                                                        //--- Remote Pin Read ---
    if ( Item->Device != XBee->TargetArduinoID() ) XBee->TargetArduinoID( Item->Device ); //Set TransceiverID
#if BLOCKING==0
    Item->Value = VALUE_WAIT;
    wait_reply = millis();
    
    if ( Item->Pin >= A0 ) { Item->PacketID = XBee->analogReadNB(Item->Pin); }
    else { Item->PacketID = XBee->digitalReadNB(Item->Pin); }
    DB(("PacketID="));DBL((Item->PacketID));

#else
    if ( Item->Pin >= A0 ) { Item->Value = XBee->analogReadB(Item->Pin); }
    else { Item->Value = XBee->digitalReadB(Item->Pin); }
    if ( Item->Value >= 0 ) CheckValue(Item);                   // Check the new value for Alarms
#endif
  }

  // Update SetVPin(s) for the item                             // Retreive Settings on Virtual Pins
  if ( Item->SetPID != NULL ) {
    if ( Item->SetPID->SetVPin != NOPIN ) {
      int PinVal = XBee->VirtualPin(Item->SetPID->SetVPin);
      if ( bitRead(PinVal, 13) ) {
        Item->SetPID->OPID->SetMode(AUTOMATIC);
        bitClear(PinVal, 13);
      } else {
        Item->SetPID->OPID->SetMode(MANUAL);
      }
      Item->SetPID->Setpoint = PinVal;
    }
  }

  uAlarm *Alarm; Alarm = Item->FirstAlarm;
  while ( Alarm != NULL ) {
    if ( Alarm->SetVPin != NOPIN ) {
      int PVal = XBee->VirtualPin(Alarm->SetVPin);
      if ( bitRead(PVal, 13) ) {
        Alarm->IsOn = true;
        bitClear(PVal, 13);
      } else {
        Alarm->IsOn = false;
      }
      Alarm->Value = PVal;
    }
    Alarm = Alarm->Next;
  }
}

/******************************************************************************************************************//**
 * @brief  Checks a Menu items 'Value' with its LOALARM and HIALARM values and trigger the buzzer if needed.
 * @remarks
 * - Only the NOT-EQUAL alarm will trigger during a communications error; all others will be disabled.
 * - Alarms are only checked while the unit is in idle iteration mode.
 * - Battery levels under 550 are disabled to allow USB Power without triggering the alarm.
 * @code
 *   exmaple code
 * @endcode
**********************************************************************************************************************/
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
    if ( Item->Device == ThisDeviceID && Alarm->SetVPin != NOPIN ) {         // Check Store Pin Remote changes
      bool bIsOn = false;
      int PinValue = XBee->VirtualPin(Alarm->SetVPin);
      if ( bitRead(PinValue, ISONBIT) ) { bIsOn = true; bitClear(PinValue, ISONBIT); }   // Check if IsOn bit-13 is set
      if ( Alarm->IsOn != bIsOn ) Alarm->IsOn = bIsOn;
      if ( Alarm->Value != PinValue ) Alarm->Value = PinValue;
    }

    if ( !Alarm->IsOn ) { Alarm->IsActive = false; }
    else {
      switch ( Alarm->Compare ) {
        case LESS: 
          if ( Item->Value < Alarm->Value ) { Alarm->Violations++; } else { Alarm->Violations = 0; }
          break;
        case GREATER:
          if ( Item->Value > Alarm->Value ) { Alarm->Violations++; } else { Alarm->Violations = 0; }
          break;
        case EQUAL:
          if ( Item->Value = Alarm->Value ) { Alarm->Violations++; } else { Alarm->Violations = 0; }
          break;
        case NOTEQUAL:
          if ( Item->Value != Alarm->Value ) { Alarm->Violations++; } else { Alarm->Violations = 0; }
          break;
      }
      Alarm->IsActive = ( Alarm->Violations >= Alarm->ViolationCount );
      if ( Alarm->IsActive ) {
        if ( Alarm->HaltOnAlarm ) AlarmHalt = true;
        SetPin( Alarm->DriveDevice, Alarm->DrivePin, Alarm->DriveValue );
      }
    }
    Alarm = Alarm->Next;
  }
}

/******************************************************************************************************************//**
 * @brief  Preform value setting and recording when the 'Select' button is pressed.
 * @remarks
 * - Setting values implements blocking; without it the read-back 'GetItem()' doesn't function correctly.
 * @code
 *   exmaple code
 * @endcode
**********************************************************************************************************************/
void PeerRemoteMenu::SetPin(uint8_t _DriveDevice, uint8_t _DrivePin, int _Value ) {
  DB(("SetPin("));DB((_DriveDevice));DBC;DB((_DrivePin));DBC;DB((_Value));DBL((")"));

  if ( _DriveDevice == BUZZER && BuzzerPin != NOPIN ) {
    if ( _Value != 0 ) { tone(BuzzerPin,_Value); } else { noTone(BuzzerPin); }
    return;
  }
  if ( _DriveDevice < 0 || _DriveDevice > 16 || _DrivePin == NOPIN ) return;    // Value Check

  if ( _DriveDevice == ThisDeviceID ) {                                         // Drive a LOCAL Pin
    if ( _DrivePin >= A0 ) {
      analogWrite(_DrivePin, _Value);
    } else {
      digitalWrite(_DrivePin, _Value);
    }
    
  } else {                                                                      // Drive a REMOTE Pin
    if ( _DriveDevice != XBee->TargetArduinoID() ) XBee->TargetArduinoID( _DriveDevice );
    if ( _DrivePin >= A0 ) {
      XBee->analogWriteB(_DrivePin, _Value);                  // Set the Remote Arduino Analog Pin
    } else {
      XBee->digitalWriteB(_DrivePin, _Value);                 // Set the Remote Arduino Digital Pin
    }
  }
}

/******************************************************************************************************************//**
 * @brief  Properly display a Menu Item on the LCD screen
 * @code
 *   exmaple code
 * @endcode
**********************************************************************************************************************/
void PeerRemoteMenu::LCD_display() {
  DB(("LCD_display("));DB((CurrItem->Name));DBL((")"));

  // Display Device ( Top Row Left )
  LCD->clear();LCD->setCursor(0,0);
  LCD->print( Devices[CurrItem->Device] );
  
  // Display Status ( Top Row Right )
  char Stat[16] = "";uint8_t p = 0;uint8_t i = 0; bool bAlarmActive = false;
  if ( Func == MAIN || (Func == SET && !CurrItem->Pin > 63) ) {
    
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
    Stat[i]='\0'; // Terminate
    if ( strlen(Devices[CurrItem->Device]) > 16-i ) { LCD->setCursor(strlen(Devices[CurrItem->Device]),0); }
    else { LCD->setCursor(16-i,0); }
    LCD->print(Stat);
    
  } else if ( Func == SET && CurrItem->Pin > 63 ) {
    // Show what in top-right if the item is ON/OFF and do not return to MAIN when set
    
  } else if ( Func == SETPID ) {
    if ( CurrItem->SetPID->OPID->GetMode() == AUTOMATIC ) { LCD->setCursor(12,0); LCD->print("AUTO"); }
    else { LCD->setCursor(13,0); LCD->print("OFF"); }

  } else if ( Func == ALARM ) {
    if ( CurrItem->CurrAlarm->IsOn ) { LCD->setCursor(14,0); LCD->print("ON"); }
    else { LCD->setCursor(13,0); LCD->print("OFF"); }
  }
  // Display Menu-Item and ID ( Bottom Row Left )
  LCD->setCursor(0,1);LCD->print(CurrItem->Name);
  LCD->print("(");LCD->print(CurrItem->ID);LCD->print(")");
  LCD->setCursor(strlen(CurrItem->Name) + 3, 1);

  // Display Current Function Symbol ( Bottom Row Middle )
  if ( Func == MAIN ) { LCD->print(" ="); }
  else if ( Func ==SETPID ) { LCD->print(" PID"); LCD->print((char)126); }//PID->
  else if ( Func == SET ) { LCD->print(" SET"); LCD->print((char)126); }//SET->
  else if ( Func == ALARM ) { LCD->print(" ");LCD->print((char)225); // (a)larm indicator
    switch ( CurrItem->CurrAlarm->Compare ) {
      case LESS:      LCD->print("<"); break;
      case GREATER:   LCD->print(">"); break;
      case EQUAL:     LCD->print("="); break;
      case NOTEQUAL:  LCD->print((char)183);break; // slashed equal
      case EMPTY:     LCD->print("ERR");break;
      default: LCD->print("ERR");
    }
  }
  
  // Display the Value ( Bottom Row Right )
  int RawVal = VALUE_ERR;
  switch ( Func ) {
    case MAIN:    RawVal = CurrItem->Value; break;
    case SETPID:  RawVal = int(CurrItem->SetPID->Setpoint); break;
    case SET:     RawVal = CurrItem->Set->Value; break;
    case ALARM:   RawVal = CurrItem->CurrAlarm->Value; break;
  }

  if ( CurrItem->IsOnOff ) {
    if ( RawVal == LOW ) { LCD->print("Off"); }
    else if ( RawVal == HIGH ) { LCD->print("On"); }
    else { LCD->print("ERR"); }
    
  } else {
    if ( RawVal == VALUE_ERR ) { LCD->print("ERR"); }
    else if ( RawVal == VALUE_WAIT ) { LCD->print("?"); }
    else {
      //(*ValueModifierCallback)(int)
      if ( CurrItem->ValueModifierCallback != NULL ) {
        LCD->print( CurrItem->ValueModifierCallback(RawVal) );
      } else {
        LCD->print( RawVal );
      } 
    }
  }
}

/******************************************************************************************************************//**
 * @brief  Arduino Sketch Loop() routine
 * @remarks
 * - This function is called automatically over-and-over again by the Arduino
 * - Handles incoming XBee communications
 * - Handles button presses and LCD response updates
 * - Handles Menu iteratation during idle.
**********************************************************************************************************************/
void PeerRemoteMenu::loop(){
  //DBL(("loop()"));
  int IsOn = -1;

  XBee->Available();                                            // Check communications 
  
  #if BLOCKING==0                                               // Check for Non-Blocked Replies
    if ( CurrItem->PacketID != -1 ) {                           // Assign Non-Blocking Get Items
      int Ret = XBee->GetReply(CurrItem->PacketID);
      if ( Ret != -1 ) {                                        // -- If reply value was available
        if ( CurrItem->Pin > 63 ) {                             // When Item 'IsVirtual'
          if ( CurrItem->Set != NULL ) CurrItem->Set->IsOn = bitRead( Ret, ISONBIT );
          bitClear( Ret, ISONBIT );
        }
        CurrItem->Value = Ret;                                  // Assign received value
        CheckValue(CurrItem);                                   // Check the value for alarms and Update PID
        CurrItem->PacketID = -1;                                // Reset non-blocking packet
        LCD_display();                                          // Update Display with new value
      } else {
        CurrItem->Value = VALUE_WAIT;
      }
      if ( millis() - wait_reply > XBee->Timeout() ) {
        CurrItem->PacketID = -1;                                // Time-out and ERR
        CurrItem->Value = VALUE_ERR;
        LCD_display();                                          // Display the ERR
      }
    }
  #endif

  unsigned long clk = millis(); 
  Button bpress = last_bpress;                                  // Determine button pressed
  last_bpress = NONE;                                           // Release button press
  
  //--- Iterate Menu when Idle to check Alarms ------------------------------------------------------
  if( bpress == NONE ) {
    if ( clk - last_bpress_millis > START_STATUS_ITERATE) bIterating = true;
    if ( bIterating && (( clk - last_iter_millis ) > ITERATE_EVERY) && !AlarmHalt ) {
          Func = MAIN;                                        // Switch to 'MAIN' menu items
          if ( CurrItem->Next == NULL ) {
            CurrItem = FirstItem;
          } else {
            CurrItem = CurrItem->Next;
          }
          GetItem(CurrItem);                                // Get the new value
          last_iter_millis=clk;                             // Record time for next iteration
          LCD_display();                                    // Update the display
    }

    ButtonCheck(analogRead(0));                            // Check for Left, Select button press
    return;
  }
  
  //--- Process Button Press ------------------------------------------------------------------------
  SetPin(BUZZER,NOPIN,0);                                 // Turn off any alarms at button press
  bIterating = false;                                     // Stop Iterating Menu Items
  
  //------- ( SELECT ) -------
  if (bpress == SELECT) {

    if ( Func == MAIN ) { GetItem(CurrItem); }
    
    // Toggle PID (OFF<->ON) when Select is pressed
    else if ( Func == SETPID ) {
      int PinValue = CurrItem->SetPID->Setpoint;
      if ( CurrItem->SetPID->OPID->GetMode() == AUTOMATIC ) {
        CurrItem->SetPID->OPID->SetMode(MANUAL);IsOn = MANUAL;
        bitClear( PinValue, ISONBIT );
      } else {
        CurrItem->SetPID->OPID->SetMode(AUTOMATIC);IsOn = AUTOMATIC;
        bitSet( PinValue, ISONBIT );
      }
      // Set the 'SetVPin' and Save changes to EEPROM
      if ( CurrItem->SetPID->SetVPin != NOPIN ) { XBee->VirtualPin( CurrItem->SetPID->SetVPin, PinValue ); }
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

      // Drive the Device-Pin
      SetPin( CurrItem->Set->DriveDevice, CurrItem->Set->DrivePin, CurrItem->Set->Value );
      EEPROMSet(CurrItem->Set->EpromOffset, CurrItem->Set->Value);        // Just a convenience save
      Func = MAIN;                                                        // Return to MAIN for convenience
      GetItem(CurrItem);                                                  // Get item again after SET
    }    

    // Save the Value and Toggle Alarm (OFF<->ON) when Select is pressed
    else if ( Func == ALARM ) {
      int PinValue = CurrItem->CurrAlarm->Value;
      if ( CurrItem->CurrAlarm->IsOn ) { 
        CurrItem->CurrAlarm->IsOn = false;
        bitClear( PinValue, ISONBIT ); 
      }
      else { 
        CurrItem->CurrAlarm->IsOn = true;
        bitSet( PinValue, ISONBIT );
      }
      // Set the 'SetVPin' and Save changes to EEPROM
      if ( CurrItem->CurrAlarm->SetVPin != NOPIN ) { XBee->VirtualPin( CurrItem->CurrAlarm->SetVPin, PinValue ); }
      EEPROMSet(CurrItem->CurrAlarm->EpromOffset, CurrItem->CurrAlarm->Value, (int)CurrItem->CurrAlarm->IsOn ); 
    }
  }
  //------- (   UP/DOWN   ) -------
  else if (bpress == UP || bpress == DOWN ) {
    if ( Func == MAIN ) {
      
      if ( bpress == DOWN ) {
        if ( CurrItem->Next != NULL ) { CurrItem = CurrItem->Next; } 
        else { CurrItem = FirstItem; }
      }  
      else if ( bpress == UP ) {
        if ( CurrItem->Prev != NULL ) { CurrItem = CurrItem->Prev; }
        else {  
          CurrItem = FirstItem;
          while ( CurrItem->Next != NULL ) { CurrItem = CurrItem->Next; } // Find Last entry
        }
      }
      GetItem(CurrItem);
    }
    else if ( Func == SET ) {
      if ( CurrItem->IsOnOff ) {
        if ( CurrItem->Set->Value == LOW ) { CurrItem->Set->Value = HIGH; } 
        else { CurrItem->Set->Value = LOW; }
      } else {
        if ( bpress == UP ) { CurrItem->Set->Value++; }
        else { CurrItem->Set->Value--; }
      }
    }    
    else if ( Func == SETPID ) {
      if ( CurrItem->IsOnOff ) {
        if ( CurrItem->SetPID->Setpoint == LOW ) { CurrItem->SetPID->Setpoint = HIGH; } 
        else { CurrItem->SetPID->Setpoint = LOW; }
      } else {
        if ( bpress == UP ) { CurrItem->SetPID->Setpoint++; } 
        else { CurrItem->SetPID->Setpoint--; }
      }
    }    
    else if ( Func == ALARM ) {
      if ( CurrItem->IsOnOff ) {
        if ( CurrItem->CurrAlarm->Value == LOW ) { CurrItem->CurrAlarm->Value = HIGH; } 
        else { CurrItem->CurrAlarm->Value = LOW; }
      } else {
        if ( bpress == UP ) { CurrItem->CurrAlarm->Value++; }
        else { CurrItem->CurrAlarm->Value--; }
      }
    }
  }
  //------- (   RIGHT   ) -------
  else if (bpress == RIGHT) {                                             // Set the Function to the Next Function
    
    if ( Func == MAIN ) {
      if ( CurrItem->SetPID != NULL ) { Func = SETPID; }
      else if ( CurrItem->Set != NULL ) { Func = SET; }
      else if ( CurrItem->FirstAlarm != NULL ) { CurrItem->CurrAlarm = CurrItem->FirstAlarm; Func = ALARM; }
    } 
    else if ( Func == SETPID ) {
      if ( CurrItem->Set != NULL ) { Func = SET; }
      else if ( CurrItem->FirstAlarm != NULL ) { CurrItem->CurrAlarm = CurrItem->FirstAlarm; Func = ALARM; }
    }  
    else if ( Func == SET ) {
      if ( CurrItem->FirstAlarm != NULL ) { CurrItem->CurrAlarm = CurrItem->FirstAlarm; Func = ALARM; }
    }  
    else if ( Func == ALARM ) {
      if ( CurrItem->CurrAlarm->Next != NULL ) { CurrItem->CurrAlarm = CurrItem->CurrAlarm->Next; }
    }
  }
  //------- (   LEFT   ) -------
  else if (bpress == LEFT) {            // Set the Function to the Previous Function
    
    if ( Func == ALARM ) {
      if ( CurrItem->CurrAlarm->Prev != NULL ) { CurrItem->CurrAlarm = CurrItem->CurrAlarm->Prev; }
      else if ( CurrItem->Set != NULL ) { Func = SET; }
      else if ( CurrItem->SetPID != NULL ) { Func = SETPID; } 
      else { Func = MAIN; }
    }
    else if ( Func == SET ) {
      if ( CurrItem->SetPID != NULL ) { Func == SETPID; } 
      else { Func = MAIN; }
    }    
    else if ( Func == SETPID ) { Func = MAIN; }
  }
  LCD_display();                           // Update Display after every button press
}


