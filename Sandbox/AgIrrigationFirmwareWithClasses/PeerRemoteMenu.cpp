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
 * @brief  Manage the Collection of Items
 * @remarks
 * - Allows a single spot customization to the user interface
 * - Display will show the items in the same order as they are defined here
 * @code
 *   exmaple code
 * @endcode
**********************************************************************************************************************/
MenuItem *PeerRemoteMenu::AddMenuItem( char *_Name, char _ID, uint8_t _Device, uint8_t _Pin, bool _IsOnOff = false ) {
  MenuItem *thisItem = NULL;
//MenuItem(PeerIOSerialControl *_XBee, uint8_t _ThisDeviceID, char *_Name, char _ID, uint8_t _Device, uint8_t _Pin, bool _IsOnOff );
  
  if ( FirstItem == NULL ) {
    FirstItem = new MenuItem(XBee, ThisDeviceID, _Name, _ID, _Device, _Pin, _IsOnOff);
    thisItem = FirstItem;
  } else {
    thisItem = FirstItem;
    while ( thisItem->Next != NULL ) { thisItem = thisItem->Next; }
    thisItem->Next = new MenuItem(XBee, ThisDeviceID, _Name, _ID, _Device, _Pin, _IsOnOff);
    thisItem->Next->Prev = thisItem;
    thisItem = thisItem->Next;
  }
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
      Alarm->SetEpromOffset(Offset);
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
 * @brief  Obtain menu values
 * @remarks
 * @code
 *   exmaple code
 * @endcode
**********************************************************************************************************************/


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
  
  char Stat[16] = "";uint8_t p = 0;uint8_t i = 0; bool bAlarmActive = false;
  if ( Func == MAIN || Func == SET ) {
    MenuItem *Item; Item = FirstItem;                         // -- Loop Items --
    while ( Item != NULL ) { Item->GetStatusLine(Stat); }   // Append Status letters
    Item = Item->Next;                                      // -- Next Item --
    if ( strlen(Devices[CurrItem->Device]) > 16-i ) { LCD->setCursor(strlen(Devices[CurrItem->Device]),0); }
    else { LCD->setCursor(16-i,0); }
    LCD->print(Stat);
  } 
  else if ( Func == SETPID ) {
    if ( CurrItem->SetPID->OPID->GetMode() == AUTOMATIC ) { LCD->setCursor(12,0); LCD->print("AUTO"); }
    else { LCD->setCursor(13,0); LCD->print("OFF"); }
  } 
  else if ( Func == ALARM ) {
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
    case ALARM:   RawVal = CurrItem->CurrAlarm->GetValue(); break;
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
        CurrItem->Value = Ret;                                  // Assign received value
        CheckValue(CurrItem);                                   // Check the value for alarms and Update PID
        CurrItem->PacketID = -1;                                // Reset non-blocking packet
        LCD_display();                                          // Update Display with new value
      }
      if ( millis() - wait_reply > XBee->Timeout() ) {
        CurrItem->PacketID = -1;                                // Time-out and ERR
        CurrItem->Value = VALUE_ERR;
        if ( CurrItem->Set != NULL ) CurrItem->Set->IsOnStatus = VALUE_ERR;
        LCD_display();                                          // Display the ERR
      }
    }
    if ( CurrItem->IsOnPacketID != -1 ) {
      int RetIsOn = XBee->GetReply(CurrItem->IsOnPacketID);
      if ( RetIsOn != -1 ) {                                        // -- If reply value was available
        CurrItem->Set->IsOn = RetIsOn;                                  // Assign received value
        CurrItem->Set->IsOnStatus = 0;
        CurrItem->IsOnPacketID = -1;                                // Reset non-blocking packet
        LCD_display();                                          // Update Display with new status
      }
      if ( millis() - wait_reply_ison > XBee->Timeout() ) {
        CurrItem->IsOnPacketID = -1;                            // Time-out and ERR
        CurrItem->Set->IsOnStatus = VALUE_ERR;
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

    if ( Func == MAIN ) { CurrItem->Get(); }
    
    // Toggle PID (OFF<->ON) when Select is pressed
    else {
      CurrItem->CurrFunction->Select();
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
    else if ( Func == SETPID ) {
      if ( CurrItem->IsOnOff ) {
        if ( CurrItem->SetPID->Setpoint == LOW ) { CurrItem->SetPID->Setpoint = HIGH; }
        else { CurrItem->SetPID->Setpoint = LOW; }
      } 
      else {
        if ( bpress == UP && CurrItem->SetPID->Setpoint < 1023 ) { CurrItem->SetPID->Setpoint++; }
        else if ( bpress == DOWN && CurrItem->SetPID->Setpoint > 0 ) { CurrItem->SetPID->Setpoint--; }
      }
    } else if ( Func == SET ) {
      if ( CurrItem->IsOnOff ) {
        if ( CurrItem->Set->Value == LOW ) { CurrItem->Set->Value = HIGH; }
        else { CurrItem->Set->Value = LOW; }
      } 
      else {
        if ( bpress == UP && CurrItem->Set->Value < 1023 ) { CurrItem->Set->Value++; }
        else if ( bpress == DOWN && CurrItem->Set->Value > 0 ) { CurrItem->Set->Value--; }
      }
    } else if ( Func == ALARM ) {
      if ( CurrItem->IsOnOff ) {
        if ( CurrItem->CurrAlarm->GetValue() == LOW ) { CurrItem->CurrAlarm->GetValue() = HIGH; }
        else { CurrItem->CurrAlarm->GetValue() = LOW; }
      } 
      else {
        if ( bpress == UP ) { CurrItem->CurrAlarm->AddToValue(1); }
        else if ( bpress == DOWN ) { CurrItem->CurrAlarm->AddToValue(-1); }
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


