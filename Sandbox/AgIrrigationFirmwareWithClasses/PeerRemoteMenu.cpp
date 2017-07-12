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
  DB(("ButtonCheck("));DB((adc_value));DBL((")"));
  
  unsigned long clk = millis();
  if ( clk - last_bpress_millis < Button_Debounce_ms ) return;         // Debounce button presses

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

void MenuItem::AttachSet(uint8_t _DriveDevice = 0, uint8_t _DrivePin = NOPIN, uint8_t _ValueStorePin = NOPIN, PID *_SetPID = NULL ) {
  if ( Set != NULL ) delete Set;
  Set = new uSet;
  if ( _DriveDevice == READ_DEVICE_AND_PIN ) {
    Set->DriveDevice = Device;
    Set->DrivePin = Pin;
  } else {
    Set->DriveDevice = _DriveDevice;
    Set->DrivePin = _DrivePin;
    Set->SetPID = _SetPID;
  }
  Set->ValueStorePin = _ValueStorePin;
}

void MenuItem::AttachAlarm(char _ID, eCompare _Compare, uint8_t _StorePin = NOPIN, uint8_t _DriveDevice = 0, uint8_t _DrivePin = 0, int _DriveValue = 0) {
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
  thisAlarm->StorePin = _StorePin;
  thisAlarm->DriveDevice = _DriveDevice;
  thisAlarm->DrivePin = _DrivePin;
  thisAlarm->DriveValue = _DriveValue;
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
}

void PeerRemoteMenu::AddDeviceName ( uint8_t _Device, char *_Name ) {
  if ( _Device > 0 && _Device < 16 ) Devices[_Device] = _Name;
}

/*
  //------------[ Start-Up the Display ( DO NOT CHANGE! )]-------------  
  for ( int i = 0; i <= MenuItemsIdx; i++ ) {
    if ( CurrItem->LoAlarm->ID != NULL || CurrItem->HiAlarm->ID != NULL ) {
      EEPROMGet(i);                                         // Load Alarm values from EEPROM
    }
  }
  GetItem();                                                // Get starting Menu item
  LCD_display();                                            // Update the display
}
*/


/******************************************************************************************************************//**
 * @brief  Function to modify raw values into meaningful information
 * @code
 *   exmaple code
 * @endcode
**********************************************************************************************************************/
/*
int PeerRemoteMenu::ValueModify(int RawValue, eValModifier Modifier, int AddBy = 0) {
  int RetVal = 0;       // Return value
  int iVal = 0;         // Initial value before adding or subtracting
  bool bOnce = true;
  
  do {  //------------ Modify RAW values to meaningful Values ---------------------------------------------
    
    // Change Pressure MPa Raw value to PSI ( pounds per square inch )
    if ( Menu[Index].ValueModifier == PRESSURE ) {
      RetVal = (int) ((Menu[Index].Sub[SubIndex].Value - 97) * 0.2137);

    // Calculate voltage from Resistors used in Voltage Divider
    } else if ( Menu[Index].ValueModifier == BATTVOLTS ) {
      RetVal = (long(Menu[Index].Sub[SubIndex].Value)*BATT_R1_VINTOA1)/long(1.6*BATT_R2_A1TOGND);

    // No Modification - just return raw value
    } else {
      RetVal = Menu[Index].Sub[SubIndex].Value;
    }

    // -------------- Change meaningful Value by 'AddBy' --------------------------------------------------
    if ( AddBy == 0 ) break;
    if (bOnce) {
      iVal = RetVal; bOnce = false; continue;
    } else {
      if( AddBy>0 ) { Menu[Index].Sub[SubIndex].Value++; } else { Menu[Index].Sub[SubIndex].Value--; }
    }

    // If adding - keep adding till the return value is no longer less than 'AddBy'
    // If subtracting - keep subtracting till the return value is no longer greater than 'AddBy'
  } while ( (AddBy > 0 && (RetVal < iVal + AddBy)) || ( AddBy < 0 && (RetVal > iVal + AddBy)) );
  

  return RetVal;
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
/*
void PeerRemoteMenu::GetItem(int i = -1) {
  
  if ( i == -1 ) i = idx;                                                         // Default index = global 'idx'
  DB(("GetItem("));DB((i));DBL((")"));                
  if ( CurrItem->Pin == NOPIN ) return;

  CurrItem->ValueValid = !true;                                               // Invalidate last reading
  if ( CurrItem->Device == TRANSCEIVER_ID ) {                         // Local Pin Read
    if ( CurrItem->Pin >= A0 ) {                                                    // Read value from Hand-Remote Pin
      CurrItem->Value = analogRead(CurrItem->Pin);
    } else {
      CurrItem->Value = digitalRead(CurrItem->Pin);
    }
    CurrItem->ValueValid = true;            // Validate local Pin Values
    CheckAlarms(i);                             // Check the value for Alarms
  } else {                                                                        // Remote Pin Read; Set TransceiverID
    if ( CurrItem->Device != XBee->TargetArduinoID() ) XBee->TargetArduinoID( CurrItem->Device );
#if BLOCKING==1
    if ( CurrItem->Pin >= A0 ) {
        wait_reply = millis();
        PacketID = XBee->analogReadNB(CurrItem->Pin);
        DB(("PacketID="));DBL((PacketID));
    } else {
        wait_reply = millis();
        PacketID = XBee->digitalReadNB(CurrItem->Pin);
        DB(("PacketID="));DBL((PacketID));
    }
#else
    if ( CurrItem->Pin >= A0 ) {
      CurrItem->Value = XBee->analogReadB(CurrItem->Pin);
    } else {
      CurrItem->Value = XBee->digitalReadB(CurrItem->Pin);
    }
    if ( CurrItem->Value != -1 ) {
      CurrItem->ValueValid = true;            // Validate Pin Values that return a legitimate value
      CheckAlarms(i);                             // Check the value for Alarms
    }
#endif
  }
}
*/
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
/*
void PeerRemoteMenu::CheckAlarms(int i = -1) {
  if ( i == -1 ) i = idx;                                                         // Default index = global 'idx'
  DB(("AlarmCheck("));DB((i));DBL((")"));
  
  if ( !bIterating ) return;                                                      // No Alarms unless iterating
  bool bOption = ( CurrItem->LastOptionIdx > 0 );
  bool bLoAlarmOn = ( CurrItem->LoAlarm->IsOn && CurrItem->LoAlarm->ID != NULL );
  bool bHiAlarmOn = ( CurrItem->HiAlarm->IsOn && CurrItem->HiAlarm->ID != NULL );

  if ( bOption && bHiAlarmOn ) AlarmActive = ( CurrItem->Value != CurrItem->HiAlarm->Value ); // NOT-EQUAL
  
  if ( CurrItem->ValueValid == true && !AlarmActive ) {                       // All Other Alarms are OFF if ERR
    if ( !bOption ) {                                                             // Compare numeric Values
      if ( bHiAlarmOn ) AlarmActive = ( CurrItem->Value > CurrItem->HiAlarm->Value );         // GREATER-THAN
      if ( bLoAlarmOn && !AlarmActive ) AlarmActive = ( CurrItem->Value < CurrItem->LoAlarm->Value ); // LESS-THAN
      if ( i == BATT && CurrItem->Value < 550 ) AlarmActive = false;      // Disable Low BATT for USB Plug-In
    } else {
      if ( bLoAlarmOn) AlarmActive = ( CurrItem->Value == CurrItem->LoAlarm->Value ); // Option Compare EQUALS
    }
  }
  //if ( AlarmActive ) tone(SBUZZ,CurrItem->LoAlarm->ToneHz);                     // Sound Buzzer

}
*/
/******************************************************************************************************************//**
 * @brief  Preform value setting and recording when the 'Select' button is pressed.
 * @remarks
 * - Setting values implements blocking; without it the read-back 'GetItem()' doesn't function correctly.
 * @code
 *   exmaple code
 * @endcode
**********************************************************************************************************************/
/*
void PeerRemoteMenu::SetItem(int i = -1) {
  if ( i == -1 ) i = idx;int iSetValue = 0;DBL(("SetItem()"));

  if ( Func == LOALARM || Func == HIALARM ) {                         // ----------- ALARMS -----------------
    if (CurrItem->Sub[Func].State == ON) {CurrItem->Sub[Func].State = !ON;} else {CurrItem->Sub[Func].State = ON;}
    EEPROMSet();                                                          // Save Alarm ON/OFF Status in EEPROM

  } else if ( Func == MAIN ) {                                          // ----------- MAIN -------------------
    GetItem(i);                                                           // Select will Refresh item

  } else if ( Func == SET ) {                                           // ----------- SET --------------------
    iSetValue = CurrItem->Set->Value;                                   // Record the current Set Value
    if ( CurrItem->LastOptionIdx > 0 ) {                                                // IF [SET] value is an OPTION
      if (CurrItem->Set->Value < 0 || CurrItem->Set->Value > MAXOPTIONS) return;  // Boundary Check the OPTION
      iSetValue = CurrItem->Option[CurrItem->Set->Value].Value;                       // Record the OPTIONS [SET] Value
    }
    
    if ( CurrItem->Pin != NOPIN ) {                                         // ----- HARDWARE SET ------------------
      CurrItem->ValueValid = !true;                                   // UN-trueATE the [MAIN] Value; its changing
      if ( CurrItem->Device == TRANSCEIVER_ID ) {
        if ( CurrItem->Pin >= A0 ) {
          analogWrite(CurrItem->Pin, iSetValue);
        } else {
          digitalWrite(CurrItem->Pin, iSetValue);
        }
      } else {
        if ( CurrItem->Device != XBee->TargetArduinoID() ) XBee->TargetArduinoID( CurrItem->Device );
        if ( CurrItem->Pin >= A0 ) {
          XBee->analogWriteB(CurrItem->Pin, iSetValue);                  // Set the Remote Arduino Analog Pin
        } else {
          XBee->digitalWriteB(CurrItem->Pin, iSetValue);                 // Set the Remote Arduino Digital Pin
        }
      }
    }
    
  }
}
*/
/******************************************************************************************************************//**
 * @brief  Properly display a Menu Item on the LCD screen
 * @code
 *   exmaple code
 * @endcode
**********************************************************************************************************************/
void PeerRemoteMenu::LCD_display() {
  DBL(("LCD_display()"));

  // Top Row (Display Device)
  LCD->clear();LCD->setCursor(0,0);
  LCD->print( Devices[CurrItem->Device] );
  
  // Right Side Alarm Identifiers
  /*int pos = 15;
  MenuItem *CheckItem = FirstItem;
  do {
    if ( CheckItem->FirstAlarm != NULL ) {
      uAlarm *alarm = CheckItem->FirstAlarm;
      do {
        if ( alarm->IsOn ) { LCD->setCursor(pos,0);LCD->print(alarm->ID);pos--; }
      } while ( alarm != CheckItem->LastAlarm );
    }
    CheckItem = CheckItem->Next;    
  } while ( CheckItem != FirstItem );
    */    
  // Bottom Row ( Menu Item Text ) and Function
  LCD->setCursor(0,1);LCD->print(CurrItem->Text);
  LCD->setCursor(strlen(CurrItem->Text), 1);
  
  if ( Func == MAIN ) {
    LCD->print(" =");
    LCD->setCursor( strlen(CurrItem->Text) + 2, 1);
  
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
  if ( Func == MAIN && CurrItem->ValueValid != true ) {
#if BLOCKING==0
    if ( PacketID != -1 ) { LCD->print("?"); } else { LCD->print("ERR"); }
#else
    LCD->print("ERR");
#endif
  } else {
    if ( CurrItem->IsOnOff ) {
      if ( Func == MAIN ) {
        if ( CurrItem->Value != LOW ) { LCD->print("On"); } else { LCD->print("Off"); }
      } else if ( Func == SET ) {
        if ( CurrItem->Set->Value != LOW ) { LCD->print("On"); } else { LCD->print("Off"); }
      } else if ( Func == ALARM ) {
        if ( CurrItem->CurrAlarm->Value != LOW ) { LCD->print("On"); } else { LCD->print("Off"); }
      }
    } else {
      if ( Func == MAIN ) {
        LCD->print( CurrItem->Value );
      } else if ( Func == SET ) {
        LCD->print( CurrItem->Set->Value );
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

  XBee->Available();                                             // Check communications 
  
  #if BLOCKING==0                                             // Check for Non-Blocked Replies
    if ( PacketID != -1 ) {                                     // Assign Non-Blocking Get Items
      int Ret = XBee->GetReply(PacketID);
      if ( Ret != -1 ) {                                        // -- If reply value was available
        CurrItem->Value = Ret;                        // Assign received value
        CurrItem->ValueValid = true;                      // Validate received value
        //CheckAlarms(idx);                                       // Check value for alarms
        PacketID = -1;                                          // Reset non-blocking packet
        LCD_display();                                          // Update Display with new value
      } 
      if ( millis() - wait_reply > XBee->Timeout() ) {
        PacketID = -1;                                          // Time-out and ERR  
        LCD_display();                                          // Display the ERR
      }
    }
  #endif

  unsigned long clk = millis(); 
  Button bpress = last_bpress;                                  // Determine button pressed
  last_bpress = NONE;                                           // Release button press
  
  //--- Iterate Menu while Idle ---------------------------------------------------------------------
  if( bpress == NONE ) {
    if ( clk - last_bpress_millis > START_STATUS_ITERATE) bIterating = true;
    if ( bIterating && (( clk - last_iter_millis ) > ITERATE_EVERY) && !AlarmActive ) {
          Func = MAIN;                                        // Switch to 'MAIN' menu items
          MenuItem *CurrentItem = CurrItem;                     // Mark the current item
          MenuItem *Item;
          do {                                                  // Find the next menu item with an Alarm
            Item = CurrItem->Next;                              // Get Next MenuItem
            if ( Item->FirstAlarm != NULL ) break;              // Check if it has any alarms
          } while ( Item != CurrentItem );                      // Stop checking on full rotation
          
          PacketID = -1;                                    // Clear the previous PacketID; We're moving
          //GetItem();                                        // Get the new value
          last_iter_millis=clk;                             // Record time for next iteration
          LCD_display();                                    // Update the display
    }

    ButtonCheck(analogRead(0));                            // Check for Left, Select button press

  //--- Process Button Press ------------------------------------------------------------------------
  } else {
    PacketID = -1;                                          // Clear Non-Blocked Packets
    noTone(BuzzerPin);AlarmActive = false;                  // Turn off any alarms at button press
    bIterating = false;                                     // Stop Iterating Menu Items
    
    //------- ( SELECT ) -------
    if (bpress == SELECT) {
      //SetItem();                                          // SET the Menu Item to its new value
      Func = MAIN;                                          // Return to the MAIN items once a SET is done
      //GetItem();                                            // Get the Items Value
    
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
        
      } else if ( CurrItem->IsOnOff ) {
        if ( Func == SET ) {
          if ( CurrItem->Set->Value == LOW ) { CurrItem->Set->Value = HIGH; } else { CurrItem->Set->Value = LOW; }
        } else if ( Func == ALARM ) {
          if ( CurrItem->CurrAlarm->Value = LOW ) { CurrItem->CurrAlarm->Value = HIGH; } else { CurrItem->CurrAlarm->Value = LOW; }
        }
        
      } else {
        if ( Func == SET ) {
          CurrItem->Set->Value++;
        } else if ( Func == ALARM ) {
          CurrItem->CurrAlarm->Value++;
        }
      }

    //------- (  DOWN  ) -------
    } else if (bpress == DOWN) {                                          
      
      if ( Func == MAIN ) {
        if ( CurrItem->Next != NULL ) { CurrItem = CurrItem->Next; } else { CurrItem = FirstItem; }
        
      } else if ( CurrItem->IsOnOff ) {
        if ( Func == SET ) {
          if ( CurrItem->Set->Value == LOW ) { CurrItem->Set->Value = HIGH; } else { CurrItem->Set->Value = LOW; }
        } else if ( Func == ALARM ) {
          if ( CurrItem->CurrAlarm->Value = LOW ) { CurrItem->CurrAlarm->Value = HIGH; } else { CurrItem->CurrAlarm->Value = LOW; }
        }
      } else {
        if ( Func == SET ) {
          CurrItem->Set->Value--;
        } else if ( Func == ALARM ) {
          CurrItem->CurrAlarm->Value--;
        }
      }
    
    
    } else if (bpress == RIGHT) {                                             // Set the Function to the Next Function
      if ( Func == MAIN ) {
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
        } else {
          if ( CurrItem->Set != NULL ) { Func = SET; } else { Func = MAIN; }
        }
      } else if ( Func == SET ) {
        Func = MAIN;
      }
    }
    
    LCD_display();                                      // Update Display after button press
  }
}


