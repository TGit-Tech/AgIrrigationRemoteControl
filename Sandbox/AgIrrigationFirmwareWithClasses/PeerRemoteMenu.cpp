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

  if ( prev_bpress == last_bpress ) { ButtonHeld++; } else { ButtonHeld=0; }
  prev_bpress = last_bpress;
  if ( last_bpress != NONE ) {
    DBL(("Button Pressed"));
    last_bpress_millis = millis();
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

void MenuItem::AttachSet(uint8_t _DriveDevice = NODEVICE, uint8_t _DrivePin = NOPIN, uint8_t _ValueStorePin = NOPIN ) {
  if ( Set != NULL ) delete Set;
  Set = new uSet;
  if ( _DriveDevice == NODEVICE ) {
    Set->DriveDevice = Device;
    Set->DrivePin = Pin;
  } else {
    Set->DriveDevice = _DriveDevice;
    Set->DrivePin = _DrivePin;
  }
  Set->ValueStorePin = _ValueStorePin;
}

void MenuItem::AttachPID(MenuItem *_OutputItem, double _Kp, double _Ki, double _Kd, int _POn, int _Direction ) {
  if ( SetPID != NULL ) {
    delete SetPID->OPID;
    delete SetPID;
  }
  if ( _OutputItem != NULL ) {
    SetPID = new uSetPID;
    SetPID->OutputItem = _OutputItem;
    SetPID->OutputItem->Set->AttachedPID = SetPID;
    SetPID->OPID = new PID(&SetPID->Input, &SetPID->Output, &SetPID->Setpoint, _Kp, _Ki, _Kd, _POn, _Direction);
    SetPID->OPID->SetOutputLimits(1,1023); // Output limits to Arduino Analog limits
    SetPID->OPID->SetSampleTime(5000);
    SetPID->OPID->SetMode(MANUAL);
  }
}

void MenuItem::AttachAlarm(char _ID, eCompare _Compare, uint8_t _DriveDevice = 0, uint8_t _DrivePin = 0, int _DriveValue = 0, bool _HaltOnAlarm = false, uint8_t _ViolationCount = 1, uint8_t _StorePin = NOPIN ) {
  uAlarm *thisAlarm = NULL;
  if ( FirstAlarm == NULL ) {
    FirstAlarm = new uAlarm;
    thisAlarm = FirstAlarm;
  } else {
    thisAlarm = FirstAlarm;
    while ( thisAlarm->Next != NULL ) { thisAlarm = thisAlarm->Next; }
    thisAlarm->Next = new uAlarm;
    thisAlarm->Next->Prev = thisAlarm;
    thisAlarm = thisAlarm->Next;
  }
  thisAlarm->ID = _ID;
  thisAlarm->Compare = _Compare;
  thisAlarm->DriveDevice = _DriveDevice;
  thisAlarm->DrivePin = _DrivePin;
  thisAlarm->DriveValue = _DriveValue;
  thisAlarm->HaltOnAlarm = _HaltOnAlarm;
  thisAlarm->ViolationCount = _ViolationCount;
  thisAlarm->StorePin = _StorePin;
}

void MenuItem::AttachValueModifier(int (*_ValueModifierCallback)(int)) {
  ValueModifierCallback = _ValueModifierCallback;
}
//=====================================================================================================================
//------------------------------ PEER-REMOTE-MENU METHODS -------------------------------------------------------------
//=====================================================================================================================
PeerRemoteMenu::PeerRemoteMenu(PeerIOSerialControl *_XBee, LiquidCrystal *_LCD, uint8_t _BuzzerPin = 0  ) {
  LCD = _LCD;
  XBee = _XBee;
  BuzzerPin = _BuzzerPin;
#if BLOCKING==1
  noInterrupts();               // switch interrupts off while messing with their settings  
  PCICR =0x02;                  // Enable 'PCIE1' bit of PCICR Pin Change Interrupt the PCINT1 interrupt
  PCMSK1 = 0b00000001;          // Pin Change Interrupt Mask ( NA, RESET, A5, A4, A3, A2, A1, A0 ) - Activate A0              
  interrupts();                 // turn interrupts back on
#endif

}

MenuItem *PeerRemoteMenu::AddMenuItem( char *_Text, uint8_t _Device, uint8_t _Pin, bool _IsOnOff ) {
  MenuItem *thisItem = NULL;
  if ( FirstItem == NULL ) {
    FirstItem = new MenuItem;
    thisItem = FirstItem;
  } else {
    thisItem = FirstItem;
    while ( thisItem->Next != NULL ) { thisItem = thisItem->Next; }
    thisItem->Next = new MenuItem;
    thisItem->Next->Prev = thisItem;
    thisItem->Next->EpromOffset = thisItem->EpromOffset + 7;
    thisItem = thisItem->Next;
  }
  thisItem->Text = _Text;
  thisItem->Device = _Device;
  thisItem->Pin = _Pin;
  thisItem->IsOnOff = _IsOnOff;
  return thisItem;
}

void PeerRemoteMenu::SetStartingItem ( MenuItem *Item ) {
  CurrItem = Item;
  LCD_display();
  // NEED TO SET EEPROM OFFSETS ACCORDING TO MENU ITEMS
}

void PeerRemoteMenu::AddDevice ( uint8_t _Device, char *_Name ) {
  if ( _Device > 0 && _Device < 16 ) Devices[_Device] = _Name;
}

void PeerRemoteMenu::ThisDevicesID ( uint8_t _DeviceID ) {
  ThisDeviceID = _DeviceID;
}

void PeerRemoteMenu::ThisDevicesID () {
  return ThisDeviceID;
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
/*
void PeerRemoteMenu::EEPROMSet(int i = -1) {
  if ( i == -1 ) i = idx;DB(("EEPROMSet("));DB((i));DBL((")"));
  int iOffset = (i*5);      // (5)Bytes per Menu Item
    
  // Store Alarm Values in EEPROM
  byte loAlarmLoByte = ((CurrItem->LoAlarm->Value >> 0) & 0xFF);
  byte loAlarmHiByte = ((CurrItem->LoAlarm->Value >> 8) & 0xFF);
  byte hiAlarmLoByte = ((CurrItem->HiAlarm->Value >> 0) & 0xFF);
  byte hiAlarmHiByte = ((CurrItem->HiAlarm->Value >> 8) & 0xFF);
  byte AlarmSet = 0x22;   // 2=OFF, A=ON
  if ( CurrItem->LoAlarm->IsOn ) bitSet(AlarmSet,3);
  if ( CurrItem->HiAlarm->IsOn ) bitSet(AlarmSet,7);
    
  EEPROM.update( iOffset, loAlarmLoByte);iOffset++;
  DB(("EEPROM.update( "));DB((iOffset));DBC;DB((loAlarmLoByte, HEX));DBL((")"));
  EEPROM.update( iOffset, loAlarmHiByte);iOffset++;
  DB(("EEPROM.update( "));DB((iOffset));DBC;DB((loAlarmHiByte, HEX));DBL((")"));
  EEPROM.update( iOffset, hiAlarmLoByte);iOffset++;
  DB(("EEPROM.update( "));DB((iOffset));DBC;DB((hiAlarmLoByte, HEX));DBL((")"));
  EEPROM.update( iOffset, hiAlarmHiByte);iOffset++;
  DB(("EEPROM.update( "));DB((iOffset));DBC;DB((hiAlarmHiByte, HEX));DBL((")"));
  EEPROM.update( iOffset, AlarmSet);iOffset++;
  DB(("EEPROM.update( "));DB((iOffset));DBC;DB((AlarmSet, HEX));DBL((")"));
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
void PeerRemoteMenu::EEPROMGet(int i = -1) {
  byte StatusByte = 0;
  if ( i == -1 ) i = idx;DB(("EEPROMGet("));DB((i));DB((")"));
  int iOffset = (i*5);                                          // (5)Bytes per Menu Item

  // Get Alarm Values from EEPROM
  DB(("EEPROMGet() - LoAlarmByte Offset = "));DB((iOffset));DBL((")"));
  byte loAlarmLoByte = EEPROM.read( iOffset );iOffset++;
  byte loAlarmHiByte = EEPROM.read( iOffset );iOffset++;
  byte hiAlarmLoByte = EEPROM.read( iOffset );iOffset++;
  byte hiAlarmHiByte = EEPROM.read( iOffset );iOffset++;
  byte AlarmSet = EEPROM.read( iOffset );iOffset++;
  
  CurrItem->LoAlarm->Value = (int)((loAlarmLoByte << 0) & 0xFF) + ((loAlarmHiByte << 8) & 0xFF00);
  CurrItem->HiAlarm->Value = (int)((hiAlarmLoByte << 0) & 0xFF) + ((hiAlarmHiByte << 8) & 0xFF00);
  CurrItem->LoAlarm->State = bitRead(AlarmSet,3);
  CurrItem->HiAlarm->State = bitRead(AlarmSet,7);

  // NOPIN Values are 'true' if the AlarmSet Bit Check passes ( Value was set/not just garbage )
  if ( CurrItem->Pin == NOPIN ) {
    if ( (AlarmSet & 0x77) == 0x22 ) CurrItem->ValueValid = true;
  }
}
*/
/******************************************************************************************************************//**
 * @brief  Obtain menu values
 * @remarks
 *  - Battery read 458 when running off USB and 856 when running from 9VDC Battery.
 * @code
 *   exmaple code
 * @endcode
**********************************************************************************************************************/
void PeerRemoteMenu::GetItem(MenuItem *Item) {
  DB(("GetItem("));DB((Item->Text));DBL((")"));
  if ( Item->Device < 1 || Item->Device > 15 ) return;
  if ( Item->Pin == NOPIN ) return;

  Item->Value = VALUE_ERR;                                      // Make last Item reading an error
  if ( Item->Device == ThisDeviceID ) {                         // Local Pin Read
    if ( Item->Pin >= A0 ) {                                    
      Item->Value = analogRead(CurrItem->Pin);
    } else {
      Item->Value = digitalRead(CurrItem->Pin);
    }
    CheckAlarmsUpdatePID(Item);                                 // Check the value for Alarms
  } else {                                                      // Remote Pin Read
    if ( Item->Device != XBee->TargetArduinoID() ) XBee->TargetArduinoID( Item->Device ); //Set TransceiverID
#if BLOCKING==0
    Item->Value = VALUE_WAIT;
    wait_reply = millis();
    if ( Item->Pin >= A0 ) {
        Item->PacketID = XBee->analogReadNB(Item->Pin);
    } else {
        Item->PacketID = XBee->digitalReadNB(Item->Pin);;
    }
    DB(("PacketID="));DBL((Item->PacketID));
#else
    if ( Item->Pin >= A0 ) {
      Item->Value = XBee->analogReadB(Item->Pin);
    } else {
      Item->Value = XBee->digitalReadB(Item->Pin);
    }
    if ( Item->Value >= 0 ) CheckAlarmsUpdatePID(Item);              // Check the value for Alarms
#endif
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
void PeerRemoteMenu::CheckAlarmsUpdatePID(MenuItem *Item) {
  DB(("CheckAlarmsUpdatePID("));DB((Item->Text));DBL((")"));
  
  if ( !bIterating ) return;                                                      // No Alarms unless iterating 

  // Check if SET has an automatic PID to be applied
  if ( Item->SetPID != NULL ) {
    if ( Item->SetPID->OPID->GetMode() == AUTOMATIC ) {
      if ( Item->Value >= 0 ) {
        Item->SetPID->Input = double(Item->Value);
        Item->SetPID->Setpoint = double(Item->Set->Value);
        
        if ( Item->SetPID->OPID->Compute() ) {
          SetPin( Item->SetPID->OutputItem->Device, Item->SetPID->OutputItem->Pin, int(Item->SetPID->Output) );
          DB(("SetPID->OPID->Compute ( Input="));DB((Item->SetPID->Input));DBC;
          DB((" Setpoint="));DB((Item->SetPID->Setpoint));DBC;
          DB((" DrivingOutputTo="));DB((Item->SetPID->Output));DBL((" )"));
        }
      }
    }
  }
  
  uAlarm *Alarm;
  Alarm = Item->FirstAlarm;
  while ( Alarm != NULL ) {
    if ( Alarm->IsOn ) {
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
      if ( Alarm->Violations >= Alarm->ViolationCount ) {
        if ( Alarm->HaltOnAlarm ) AlarmHalt = true;
        ActiveAlarm = Alarm->ID;
        //SetPin( Alarm->DriveDevice, Alarm->DrivePin, Alarm->DriveValue, Alarm->StorePin );
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
void PeerRemoteMenu::SetPin(uint8_t _DriveDevice, uint8_t _DrivePin, int _Value, uint8_t _ValueStorePin = NOPIN ) {
  DB(("SetPin("));DB((_DriveDevice));DBC;DB((_DrivePin));DBC;DB((_Value));DBC;DB((_ValueStorePin));DBL((")"));

  if ( _DriveDevice == BUZZER ) {
    // NEED TO BLOCK THIS on ULTRASONIC
    if ( _Value != 0 ) {
      tone(_DrivePin,_Value);
    } else {
      noTone(_DrivePin);
    }
    
  } else if ( _DriveDevice > 0 && _DriveDevice < 16 && _DrivePin != NOPIN ) {
    if ( _DriveDevice == ThisDeviceID ) {
      if ( _DrivePin >= A0 ) {
        analogWrite(_DrivePin, _Value);
      } else {
        digitalWrite(_DrivePin, _Value);
      }
    } else {
      if ( _DriveDevice != XBee->TargetArduinoID() ) XBee->TargetArduinoID( _DriveDevice );
      if ( _DrivePin >= A0 ) {
        XBee->analogWriteB(_DrivePin, _Value);                  // Set the Remote Arduino Analog Pin
      } else {
        XBee->digitalWriteB(_DrivePin, _Value);                 // Set the Remote Arduino Digital Pin
      }
      if ( _ValueStorePin != NOPIN ) XBee->analogWriteB(_DrivePin, _Value);
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
  DB(("LCD_display("));DB((CurrItem->Text));DBL((")"));

  // Top Row (Display Device)
  LCD->clear();LCD->setCursor(0,0);
  LCD->print( Devices[CurrItem->Device] );
  
  // Top Row - Right Side Alarm Identifiers
  if ( Func == MAIN || Func == SET ) {
    if ( ActiveAlarm != NULL ) {
      LCD->setCursor(0,13);LCD->print("!");LCD->print(ActiveAlarm);LCD->print("!");
    } else {
      unsigned int lastpos = ( 15 - strlen(Devices[CurrItem->Device]));     // Find last position before Device text cuts
      uint8_t currPos = 15;                                                 // Track position Right -> Left
      uAlarm *Alarm; MenuItem *Item;
      Item = FirstItem;
      while ( Item != NULL ) {                                              // Loop Menu Items
        Alarm = Item->FirstAlarm;
        while ( Alarm != NULL ) {                                           // Loop Alarms
          if ( Alarm->IsOn ) {                                                // If Alarm is ON
            if ( currPos > lastpos ) {                                        // Do we have room to show it
              LCD->setCursor(0,currPos);LCD->print(Alarm->ID);currPos--;      // Show it and move cursor left
            } else {
              LCD->setCursor(0,lastpos);LCD->print(".");break;                // If no more room; show '.' and stop checking
            }
          }
          Alarm = Alarm->Next;
        }
        if ( currPos > lastpos ) break;                                       // Break Item loop if no more room
        Item = Item->Next;
      }
    }
  } else if ( Func == SETPID ) {
    if ( CurrItem->SetPID->OPID->GetMode() == AUTOMATIC ) {
      LCD->setCursor(12,0); LCD->print("AUTO");
    } else {
      LCD->setCursor(13,0); LCD->print("OFF");
    }
  } else if ( Func == ALARM ) {
    if ( CurrItem->CurrAlarm->IsOn ) {
      LCD->setCursor(14,0); LCD->print("ON");
    } else {
      LCD->setCursor(13,0); LCD->print("OFF");
    }
  }
  
  // Bottom Row ( Menu Item Text ) and Function
  LCD->setCursor(0,1);LCD->print(CurrItem->Text);
  LCD->setCursor(strlen(CurrItem->Text), 1);
  
  if ( Func == MAIN ) {
    LCD->print(" =");
    LCD->setCursor( strlen(CurrItem->Text) + 2, 1);

  } else if ( Func ==SETPID ) {
    LCD->print(" PID");
    LCD->print((char)126); //Character '->'
    LCD->setCursor( strlen(CurrItem->Text) + 5, 1);
    
  } else if ( Func == SET ) {
    LCD->print(" SET");
    LCD->print((char)126); //Character '->'
    LCD->setCursor( strlen(CurrItem->Text) + 5, 1);
  
  } else if ( Func == ALARM ) {
    switch ( CurrItem->CurrAlarm->Compare ) {
      case LESS:      LCD->print("<!"); break;
      case GREATER:   LCD->print(">!"); break;
      case EQUAL:     LCD->print("=!"); break;
      case NOTEQUAL:  LCD->print((char)183);LCD->print("!");break; // slashed equal
      case EMPTY:     LCD->print("E");break;
      default: LCD->print("!!");
    }
    LCD->setCursor( strlen(CurrItem->Text) + 3, 1);
  }
  
  // Bottom Row ( Display Value )
  if ( Func == MAIN ) {
    if ( CurrItem->Value == VALUE_ERR ) {
      LCD->print("ERR");
    } else if ( CurrItem->Value == VALUE_WAIT ) {
      LCD->print("?");
    }
  } else {
    if ( CurrItem->IsOnOff ) {
      if ( Func == MAIN ) {
        if ( CurrItem->Value == LOW ) {
          LCD->print("Off");
        } else if ( CurrItem->Value == HIGH ) {
          LCD->print("On");
        } else {
          LCD->print("ERR");
        }
      } else if ( Func == SETPID ) {
        if ( int(CurrItem->SetPID->Setpoint) == LOW ) {
          LCD->print("Off");
        } else if ( int(CurrItem->SetPID->Setpoint) == HIGH ) {
          LCD->print("On");
        } else {
          LCD->print("ERR");
        }
      } else if ( Func == SET ) {
        if ( CurrItem->Set->Value == LOW ) {
          LCD->print("Off");
        } else if ( CurrItem->Set->Value == HIGH ) {
          LCD->print("On");
        } else {
          LCD->print("ERR");
        }
      } else if ( Func == ALARM ) {
        if ( CurrItem->CurrAlarm->Value == LOW ) {
          LCD->print("Off");
        } else if ( CurrItem->CurrAlarm->Value == HIGH ) {
          LCD->print("On");
        } else {
          LCD->print("ERR");
        }
      }
    } else {
      if ( Func == MAIN ) {
        if ( CurrItem->Value == VALUE_ERR) {
          LCD->print("ERR");
        } else {
          //(*ValueModifierCallback)(int)
          if ( CurrItem->ValueModifierCallback != NULL ) {
            LCD->print( CurrItem->ValueModifierCallback(CurrItem->Value) );
          } else {
            LCD->print( CurrItem->Value );
          }
        }
      } else if ( Func == SETPID ) {
        if ( int(CurrItem->SetPID->Setpoint) == VALUE_ERR ) {
          LCD->print("ERR");
        } else {
          LCD->print( int(CurrItem->SetPID->Setpoint) );
        }
      } else if ( Func == SET ) {
        if ( CurrItem->Set->Value == VALUE_ERR ) {
          LCD->print("ERR");
        } else {
          LCD->print( CurrItem->Set->Value );
        }
      } else if ( Func == ALARM ) {
        LCD->print( CurrItem->CurrAlarm->Value );
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

  XBee->Available();                                            // Check communications 
  
  #if BLOCKING==0                                               // Check for Non-Blocked Replies
    if ( CurrItem->PacketID != -1 ) {                           // Assign Non-Blocking Get Items
      int Ret = XBee->GetReply(CurrItem->PacketID);
      if ( Ret != -1 ) {                                        // -- If reply value was available
        CurrItem->Value = Ret;                                  // Assign received value
        CheckAlarmsUpdatePID(CurrItem);                         // Check the value for alarms and Update PID
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
          MenuItem *CurrentItem = CurrItem;                     // Mark the current item
          MenuItem *Item;
          do {                                                  // Find the next menu item with an Alarm
            if ( CurrItem->Next != NULL ) { Item = CurrItem->Next; } else { Item = FirstItem; }
            if ( Item->FirstAlarm != NULL ) break;              // Check if it has any alarms
          } while ( Item != CurrentItem );                      // Stop checking on full rotation

          CurrItem = Item;                                  // Set CurrItem to next item with an Alarm
          GetItem(Item);                                    // Get the new value
          last_iter_millis=clk;                             // Record time for next iteration
          LCD_display();                                    // Update the display
    }

    ButtonCheck(analogRead(0));                            // Check for Left, Select button press

  //--- Process Button Press ------------------------------------------------------------------------
  } else {
    noTone(BuzzerPin);                                      // Turn off any alarms at button press
    bIterating = false;                                     // Stop Iterating Menu Items
    
    //------- ( SELECT ) -------
    if (bpress == SELECT) {

      if ( Func == MAIN ) {
        GetItem(CurrItem);
        
      } else if ( Func == SET ) {
        if ( CurrItem->Set != NULL ) {
          // Setting an Item controlled by a PID MUST disable the PID
          if ( CurrItem->Set->AttachedPID != NULL ) {
            CurrItem->Set->AttachedPID->OPID->SetMode(MANUAL);
            CurrItem->Set->AttachedPID->Output = CurrItem->Set->Value; // Keep Output insync
          } else {
            SetPin( CurrItem->Set->DriveDevice, CurrItem->Set->DrivePin, CurrItem->Set->Value, CurrItem->Set->ValueStorePin );
          }
        }
      }
      //SetItem();                                          // SET the Menu Item to its new value
      Func = MAIN;                                          // Return to the MAIN items once a SET is done
      //GetItem();                                            // Get the Items Value
      
      // NEED TO SET THIS UP
      //if ( Func == LOALARM || Func == HIALARM ) {                         // ----------- ALARMS -----------------
//    if (CurrItem->Sub[Func].State == ON) {CurrItem->Sub[Func].State = !ON;} else {CurrItem->Sub[Func].State = ON;}
//    EEPROMSet();                                                          // Save Alarm ON/OFF Status in EEPROM

//  } else if ( Func == MAIN ) {                                          // ----------- MAIN -------------------
    //GetItem(i);                                                           // Select will Refresh item

    
    //------- (   UP   ) -------
    } else if (bpress == UP ) {
      
      if ( Func == MAIN ) {
        if ( CurrItem->Prev == NULL ) {
          if ( CurrItem->Next != NULL ) {
            while ( CurrItem->Next != NULL ) { CurrItem = CurrItem->Next; } // Find Last entry
          }
        } else {
          CurrItem = CurrItem->Prev;
        }
        GetItem(CurrItem);

      } else if ( CurrItem->IsOnOff ) {
        if ( Func == SET ) {
          if ( CurrItem->Set->Value == LOW ) { CurrItem->Set->Value = HIGH; } else { CurrItem->Set->Value = LOW; }
        } else if ( Func == SETPID ) {
          if ( CurrItem->SetPID->Setpoint == LOW ) { CurrItem->SetPID->Setpoint = HIGH; } else { CurrItem->SetPID->Setpoint = LOW; }
        } else if ( Func == ALARM ) {
          if ( CurrItem->CurrAlarm->Value == LOW ) { CurrItem->CurrAlarm->Value = HIGH; } else { CurrItem->CurrAlarm->Value = LOW; }
        }
        
      } else {
        if ( Func == SET ) {
          CurrItem->Set->Value++;
        } else if ( Func == SETPID ) {
          CurrItem->SetPID->Setpoint++;
        } else if ( Func == ALARM ) {
          CurrItem->CurrAlarm->Value++;
        }
      }

    //------- (  DOWN  ) -------
    } else if (bpress == DOWN) {                                          
      
      if ( Func == MAIN ) {
        if ( CurrItem->Next != NULL ) { CurrItem = CurrItem->Next; } else { CurrItem = FirstItem; }
        GetItem(CurrItem);
      } else if ( CurrItem->IsOnOff ) {
        if ( Func == SET ) {
          if ( CurrItem->Set->Value == LOW ) { CurrItem->Set->Value = HIGH; } else { CurrItem->Set->Value = LOW; }
        } else if ( Func == SETPID ) {
          if ( CurrItem->SetPID->Setpoint == LOW ) { CurrItem->SetPID->Setpoint = HIGH; } else { CurrItem->SetPID->Setpoint = LOW; }
        } else if ( Func == ALARM ) {
          if ( CurrItem->CurrAlarm->Value = LOW ) { CurrItem->CurrAlarm->Value = HIGH; } else { CurrItem->CurrAlarm->Value = LOW; }
        }
      } else {
        if ( Func == SET ) {
          CurrItem->Set->Value--;
        } else if ( Func == SETPID ) {
          CurrItem->SetPID->Setpoint--;
        } else if ( Func == ALARM ) {
          CurrItem->CurrAlarm->Value--;
        }
      }
    
    
    } else if (bpress == RIGHT) {                                             // Set the Function to the Next Function
      if ( Func == MAIN ) {
        if ( CurrItem->SetPID != NULL ) {
          Func = SETPID;
        } else if ( CurrItem->Set != NULL ) {
          Func = SET;
        } else if ( CurrItem->FirstAlarm != NULL ) {
          Func = ALARM;
          CurrItem->CurrAlarm = CurrItem->FirstAlarm;
        }
      } else if ( Func == SETPID ) {
        if ( CurrItem->Set != NULL ) {
          Func = SET;
        } else if ( CurrItem->FirstAlarm != NULL ) {
          Func = ALARM;
          CurrItem->CurrAlarm = CurrItem->FirstAlarm;
        }
      } else if ( Func == SET ) {
        if ( CurrItem->FirstAlarm != NULL ) {
          Func = ALARM;
          CurrItem->CurrAlarm = CurrItem->FirstAlarm;
        }
      } else if ( Func == ALARM ) {
        if ( CurrItem->CurrAlarm->Next != NULL ) { CurrItem->CurrAlarm = CurrItem->CurrAlarm->Next; }
      }
      
    } else if (bpress == LEFT) {                                                // Set the Function to the Previous Function
      if ( Func == ALARM ) {
        if ( CurrItem->CurrAlarm->Prev != NULL ) {
          CurrItem->CurrAlarm = CurrItem->CurrAlarm->Prev;
        } else if ( CurrItem->Set != NULL ) { 
          Func = SET; 
        } else if ( CurrItem->SetPID != NULL ) {
          Func = SETPID;
        } else { 
          Func = MAIN; 
        }

      } else if ( Func == SET ) {
        if ( CurrItem->SetPID != NULL ) {
          Func == SETPID;
        } else {
          Func = MAIN;
        }
      } else if ( Func == SETPID ) {
        Func = MAIN;
      }
    }
    
    LCD_display();                                      // Update Display after button press
  }
}


