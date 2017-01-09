/************************************************************************//**
 *  @brief  Arduino Sketch to be loaded onto the Irrigation Pump Remote Hand-Held unit.
 *    see:
  note: 2x16 characters on display
  Arduino/Genuino Uno is a microcontroller board based on the ATmega328P
  https://www.arduino.cc/en/Main/ArduinoBoardUno
  1KB of EEPROM memory
 *  @code
 *    exmaple code
 *  @endcode
 *  @authors 
 *    tgit23        12/2016       Original
******************************************************************************/
#include <LiquidCrystal.h>
#include <EEPROM.h>
#include "PeerIOSerialControl.h"

#define SBUZZ 2                 // Signal-Pin on Buzzer
#define SS_TX_PIN 11            // Pin 2 is used by Buzzer
#define SS_RX_PIN 12            // Pin 10 is input backlight LCD; pin 8,9 are LCD
#define BATT_PIN 1

#define WAIT_REPLY_MS 1000      // How long to wait for a reply
#define DIGITAL true
#define ANALOG false
#define READ true
#define WRITE false

// Setup a Software Serial for XBEE (Allows Debug)
#include <SoftwareSerial.h>
SoftwareSerial IOSerial(SS_RX_PIN,SS_TX_PIN);   // ( rxPin, txPin )
PeerIOSerialControl XBee(1,IOSerial,Serial);    // ArduinoID, IOSerial, DebugSerial

//---[ STATIC SETTINGS ]---------------------------------------------------------------
#define ALARMOFF 0x9B
#define ALARMON 0x9A
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);          // select the pins used on the LCD panel
char pointer = (char)126;                     // Shows menu pointer on LCD
enum Button { UP, DOWN, RIGHT, LEFT, SELECT, NONE };
enum eValueLocation { PROG, LOCALPIN, REMOTEPIN, EPROM };
enum eAlarmCompare { LESS, GREATER, EQUAL };
unsigned long last_bpress = 0;                // Track last button press time
unsigned long last_change = 0;                // Track last status iteration time
int idx = 0;
int last_idx = 0;
bool ActivateAlarm = false;
int CmdID = 0;
int WaitForReply = 0;
int Waitloops = 0;

//---[ Behavior Settings ]-------------------------------------------------------------
#define START_STATUS_ITERATE  15000           // Start iterating statuses after (ms)
#define ITERATE_EVERY         3000            // Iterate Status every (ms)

//---[ MENU STRUCTURE ]----------------------------------------------------------------
#define MENUITEMS 9                           // # of Menu Items ( 1 + index )
#define MAXOPTIONS 2                          // Maximum # of Options allowed
#define PUMPIDX 0                             // Menu-idx of Select Remote Pump Location
#define PUMPADDR 1020                         // EEPROM address to store Selected Pump
#define STARTIDX 1                            // First Menu item displayed after reset

struct uItemOption {
  char *Text;
  uint8_t DrivePin = LOW;
  uint8_t PumpID = 0;
};

struct MenuItems {
  char *Text;

  // Value Variables
  int Value = 0;                            // Value selects the Option if Options defined
  int ValueSetTo = 0;
  bool ValueValid=false;
  bool ValueSettable = false;
  bool ValueSettingNow = false;
  
  eValueLocation ValueLocation = PROG;      // PROG = Internally in Program
  byte Pin = 0;                             // Arduino Pin#, Analog/Digital Set below
  bool Analog = false;                      // Default operation is Digital
  
  // Alarm Variables
  eAlarmCompare AlarmCompare = EQUAL;
  int AlarmValueIdx = 0;
  bool Alarm = false;
  
  // Options
  uItemOption Option[MAXOPTIONS];
  byte LastOption = 0;            // If entry doesn't have options its numeric
  bool Poll=true;
  
} Menu[MENUITEMS];


//vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
// EEPROMSet()
//-----------------------------------------------------------------------------------------
  // STORE 'EPROM' SET VALUES IN EEPROM
  //  Address = (Menu index * 2-byte-integer) + Pump Offset
  //  Address = ((Menu-idx * (int)2-bytes) + (Menu[Pump].Value * 200)
  //  Example; Pump#0 Addresses @(0-199) - allowing 66 Menu Items (int value & status byte)
  //  Example; Pump#1 Addresses @(200-399) - allowing 66 (3-byte) values
  //  Therefore; Addressing support 99-Menu items on 4-pumps (4*200 = 800 + 200values = 1000B)
void EEPROMSet(int i = -1) {
  byte StatusByte;
  if ( i == -1 ) i = idx;
  if ( Menu[i].Alarm ) { StatusByte = ALARMON; } else { StatusByte = ALARMOFF; }
  if ( i == PUMPIDX ) {
    EEPROM.update(PUMPADDR,(char)Menu[i].Value);    // Write
    EEPROM.update(PUMPADDR+1,StatusByte);                 // Validate in EEPROM
  } else {
    byte lowByte = ((Menu[i].Value >> 0) & 0xFF);
    byte highByte = ((Menu[i].Value >> 8) & 0xFF);  
    EEPROM.update(((i*3)+(Menu[i].Value*200)), lowByte);
    EEPROM.update(((i*3)+(Menu[i].Value*200) + 1), highByte);
    EEPROM.update(((i*3)+(Menu[i].Value*200) + 2), StatusByte);
  }
}
//vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
// EEPROMGet()
//-----------------------------------------------------------------------------------------
//This function will read a 2 byte integer from the eeprom at the specified address and address + 1
void EEPROMGet(int i = -1) {
  byte StatusByte = 0;
  if ( i == -1 ) i = idx;
  
  if ( i == PUMPIDX ) {
    Menu[i].Value = (int)EEPROM.read(PUMPADDR);
    StatusByte = EEPROM.read(PUMPADDR+1);
  } else {
    byte lowByte = EEPROM.read((i*3)+(Menu[i].Value*200));
    byte highByte = EEPROM.read((i*3)+(Menu[i].Value*200)+1);
    byte Status = EEPROM.read((i*3)+(Menu[i].Value*200)+2);
    Menu[i].Value = (int)((lowByte << 0) & 0xFF) + ((highByte << 8) & 0xFF00);
  }

  // Validate the reading was from a previous setting using the StatusByte
  if ( StatusByte == ALARMON ) {
    Menu[i].Alarm = true;
    Menu[i].ValueValid = true;
  } else if ( StatusByte = ALARMOFF ) {
    Menu[i].Alarm = false;
    Menu[i].ValueValid = true;
  }
}

//vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
// SetItem()
//-----------------------------------------------------------------------------------------
void SetItem(int i = -1) {
  if ( i == -1 ) i = idx;
  if ( i != PUMPIDX && !Menu[PUMPIDX].ValueValid ) return;  // No control till pump selected
  
  switch ( Menu[i].ValueLocation ) {

    case EPROM:
      if ( Menu[i].ValueSettingNow ) {
        if ( Menu[i].LastOption > 0 && (Menu[i].ValueSetTo < 0 || Menu[i].ValueSetTo > MAXOPTIONS)) break;
        Menu[i].Value = Menu[i].ValueSetTo;   // Assign the setting
        EEPROMSet();                          // Write Value to EEPROM
        Menu[i].ValueValid = true;            // Validate
        if ( i == PUMPIDX ) XBee.TargetArduinoID(Menu[i].Option[Menu[i].Value].PumpID);
        
      } else {
        if ( strncmp(Menu[i].Text,"!",1)==0 ) {
          Menu[i].Alarm = !Menu[i].Alarm;     // Toggle Alarm
          EEPROMSet();
        }
      }
      break;

    case LOCALPIN:
      if ( Menu[i].ValueSettingNow ) {
        if ( Menu[i].LastOption > 0 && (Menu[i].ValueSetTo < 0 || Menu[i].ValueSetTo > MAXOPTIONS)) break;
        Menu[i].Value = Menu[i].ValueSetTo;   // Assign the setting
        Menu[i].ValueValid = true;

        // Write Value to local pin
        if( !Menu[i].Analog ) {
          if ( Menu[i].LastOption > 0 ) {
            digitalWrite(Menu[i].Pin, Menu[i].Option[Menu[i].Value].DrivePin);
          } else {
            digitalWrite(Menu[i].Pin, Menu[i].Value);
          }
        } else {
          analogWrite(Menu[i].Pin,Menu[i].Value);
        }
      }
      break;
      
    case REMOTEPIN:
      if ( Menu[i].ValueSettingNow ) {
        if ( Menu[i].LastOption > 0 && (Menu[i].ValueSetTo < 0 || Menu[i].ValueSetTo > MAXOPTIONS)) break;
        if( Menu[i].Analog ) {
            XBee.analogWriteNB(Menu[i].Pin,Menu[i].ValueSetTo);
        } else {
            XBee.digitalWriteNB(Menu[i].Pin,Menu[i].ValueSetTo);
        }
      }
      break;
  }

  LCD_display();
}
//vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
// GetItem()
// Baterry reads 458 when running off USB and 856 when running from 9VDC battery
//
//-----------------------------------------------------------------------------------------
void GetItem(int i = -1) {
  if ( i == -1 ) i = idx;
  int iReply = -1;
  // Make sure we have a valid Pump Selected
  if ( i != PUMPIDX && !Menu[PUMPIDX].ValueValid ) return;

  switch(Menu[i].ValueLocation) {
    
    case EPROM:
      // Read Value from EEPROM
      EEPROMGet();
      break;
        
    case REMOTEPIN:
      if( Menu[i].Analog ) {
        iReply = XBee.analogReadB(Menu[i].Pin);
      } else {
        iReply = XBee.digitalReadB(Menu[i].Pin);
      }
      if ( iReply != -1 ) {
        Menu[i].Value = iReply;
        Menu[i].ValueValid = true;
      } else {
        Menu[i].ValueValid = false;
      }
      break;
      
    case LOCALPIN:
      // Read Value from local pin
      if( !Menu[i].Analog ) {
        Menu[i].Value = digitalRead(Menu[i].Pin);
        Menu[i].ValueValid = true;
      } else {
        Menu[i].Value = analogRead(Menu[i].Pin);
        Menu[i].ValueValid = true;
      }
      break;
  }

  // Check this Value for an Active Alarm Menu Item
  if ( Menu[i].ValueValid ) {
    for (int AVI = 0;AVI<MENUITEMS-1;AVI++) {
      if ( Menu[AVI].AlarmValueIdx == i ) {
        if ( Menu[AVI].ValueValid && Menu[AVI].Alarm ) {
          
          // Check Value is within Alarm Boundaries
          switch ( Menu[AVI].AlarmCompare ) {
            case LESS:
              ActivateAlarm = ( Menu[AVI].Value < Menu[i].Value );break;
            case GREATER:
              ActivateAlarm = ( Menu[AVI].Value > Menu[i].Value );break;
            case EQUAL:
              ActivateAlarm = ( Menu[AVI].Value == Menu[i].Value );break;
          }
  }}}}
  
}

//vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
// SetupMenu()
//-----------------------------------------------------------------------------------------
void SetupMenu() {

  // Be sure to change 'MENUITEMS' when adding/deleting items
  Menu[PUMPIDX].Text = "Set Pump";
  Menu[PUMPIDX].ValueLocation = EPROM;
  Menu[PUMPIDX].Option[0].Text = "Canal";
  Menu[PUMPIDX].Option[0].PumpID = 10;           // Target ArduinoID
  Menu[PUMPIDX].Option[1].Text = "Ditch";
  Menu[PUMPIDX].Option[1].PumpID = 11;           // Target ArduinoID
  Menu[PUMPIDX].LastOption = 1;
  Menu[PUMPIDX].Poll=false;                     // Poll for Updates
  Menu[PUMPIDX].ValueSettable = true;
  
  Menu[1].Text = "Power";
  Menu[1].ValueLocation = REMOTEPIN; 
  Menu[1].Option[0].Text = "Off";   
  Menu[1].Option[0].DrivePin = LOW;  
  Menu[1].Option[1].Text = "On";
  Menu[1].Option[1].DrivePin = HIGH;
  Menu[1].LastOption = 1;
  Menu[1].Pin = 5;
  Menu[1].ValueSettable = true;

  Menu[2].Text = "Water Level";
  Menu[2].ValueLocation = REMOTEPIN; 
  Menu[2].Pin = 5;
  Menu[2].Analog = true;

  Menu[3].Text = "Battery";
  Menu[3].ValueLocation = LOCALPIN;
  Menu[3].Pin = BATT_PIN; // Analog A2
  Menu[3].Analog = true;
  
  Menu[4].Text = "Pressure";
  Menu[4].ValueLocation = REMOTEPIN;
  Menu[4].Pin = 7;
  Menu[3].Analog = true;
  
  // --- ALARMS --- ( Select on item toggles On/Off )
  Menu[5].Text = "!P-Power";
  Menu[5].ValueLocation = EPROM; 
  Menu[5].Option[0].Text = "Off";     
  Menu[5].Option[1].Text = "On";
  Menu[5].LastOption = 1;
  Menu[5].Poll=false;                   // Poll the channel for Updates
  Menu[5].ValueSettable = true;
  Menu[5].AlarmValueIdx = 1;
  Menu[5].AlarmCompare = EQUAL;
  
  Menu[6].Text = "!R-PressureL";
  Menu[6].ValueLocation = EPROM;
  Menu[6].Poll=false;                   // Poll the channel for Updates
  Menu[6].ValueSettable = true;
  Menu[6].AlarmValueIdx = 4;
  Menu[6].AlarmCompare = LESS;

  Menu[7].Text = "!H-WaterLevH";
  Menu[7].ValueLocation = EPROM;
  Menu[7].Poll=false;                   // Poll the channel for Updates
  Menu[7].ValueSettable = true;
  Menu[7].AlarmValueIdx = 2;
  Menu[7].AlarmCompare = GREATER;

  Menu[8].Text = "!L-WaterLevL";
  Menu[8].ValueLocation = EPROM;
  Menu[8].Poll=false;                   // Poll the channel for Updates
  Menu[8].ValueSettable = true;
  Menu[8].AlarmValueIdx = 2;
  Menu[8].AlarmCompare = LESS;
    
  // Read all values at start-up
  idx = PUMPIDX;GetItem();
  for (idx=0;idx<MENUITEMS-1;idx++) { GetItem(); } 
  idx = STARTIDX;
  XBee.TargetArduinoID(Menu[PUMPIDX].Option[Menu[PUMPIDX].Value].PumpID);
}

//vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
// read_LCD_Buttons()
//-----------------------------------------------------------------------------------------
Button read_LCD_buttons(){               // read the buttons
    int adc_key_in = analogRead(0);       // read the value from the sensor 
    if (adc_key_in > 1000) return NONE; 
    if (adc_key_in < 50)   return RIGHT;  
    if (adc_key_in < 195)  return UP; 
    if (adc_key_in < 380)  return DOWN; 
    if (adc_key_in < 555)  return LEFT; 
    if (adc_key_in < 790)  return SELECT;   
    return NONE;                // when all others fail, return this.
}

//vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
// LCD_display()
//-----------------------------------------------------------------------------------------
void LCD_display() {
  int val = 0;
  
  // Top Row (Status)
  lcd.clear();lcd.setCursor(0,0);
  if(Menu[PUMPIDX].ValueValid) { 
    lcd.print(Menu[PUMPIDX].Option[Menu[PUMPIDX].Value].Text);
    lcd.setCursor(strlen(Menu[PUMPIDX].Option[Menu[PUMPIDX].Value].Text)+1,0);lcd.print("Pump");
  } else {
    lcd.print("ERR Pump");
  }
  
  // Alarm Statuses
  int pos = 15;
  for (int i=0;i<MENUITEMS-1;i++) {
    if(Menu[i].ValueValid && Menu[i].Alarm) {
      lcd.setCursor(pos,0);lcd.print(Menu[i].Text[1]);pos--;
    }
  }
  
  // Menu Item Text on Bottom Row
  lcd.setCursor(0,1);lcd.print(Menu[idx].Text);
  lcd.setCursor(strlen(Menu[idx].Text),1);lcd.print(" =");

  // Value
  lcd.setCursor(strlen(Menu[idx].Text)+2,1);
  if ( Menu[idx].ValueValid || Menu[idx].ValueSettingNow ) {
    // Options Display
    if ( Menu[idx].LastOption > 0 ) {
      if(Menu[idx].ValueSettingNow) { val = Menu[idx].ValueSetTo; } else { val = Menu[idx].Value; }
      if(val<0||val>Menu[idx].LastOption) { lcd.print("ERR-");lcd.print(val); } else { lcd.print(Menu[idx].Option[val].Text); }
    } else {
    // Numeric Display
      if(Menu[idx].ValueSettingNow) {
        lcd.print(Menu[idx].ValueSetTo);
      } else {
        lcd.print(Menu[idx].Value);
      }
    }  
  } else {
    lcd.print("ERR");
  }
  delay(200);
}



//vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
// Setup()
// See:     Q1-Backlit shorting issue; http://forum.arduino.cc/index.php?topic=96747.0
//-----------------------------------------------------------------------------------------
void setup(){
    pinMode(10,INPUT);            // Fix for Q1-LCD Backlit shorting issue
    pinMode(SBUZZ,OUTPUT);
    pinMode(SS_RX_PIN, INPUT);
    pinMode(SS_TX_PIN, OUTPUT);
    
    IOSerial.begin(9600);
    Serial.begin(9600);           // Start UART Communications with the XBee->Module
    lcd.begin(16, 2);             // Start the LCD library
    SetupMenu();
    XBee.Timeout(WAIT_REPLY_MS);  // Set the Timeout
    LCD_display();                // Display the main menu
}

//vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
// Loop()
//-----------------------------------------------------------------------------------------
void loop(){
  Button bpress = read_LCD_buttons();
  XBee.Available();
  
  if(bpress == NONE) {
    if ( ActivateAlarm ) {
      for (unsigned long i = 5000;i>100;i=i-100) {
        digitalWrite(SBUZZ,HIGH);delayMicroseconds(i);
        digitalWrite(SBUZZ,LOW);delayMicroseconds(i);
      }
      
    } else if ( millis() - last_bpress > START_STATUS_ITERATE && 
                millis() - last_change > ITERATE_EVERY ) {
          
          Menu[idx].ValueSettingNow =false;
          int i = idx;                  // Mark current idx
          do {
            i++;if(i>MENUITEMS-1) i=0;  // reset idx to beginning
            if(i == idx) break;         // break if made a full rotation
          } while (!Menu[i].Poll);      // stop if item is Poll
          idx=i;                        // set new 'idx' to next with a Poll
          GetItem();
          last_change=millis();
          LCD_display();
    }
  } else { last_bpress = millis(); }
  
  if(bpress !=NONE) {
    ActivateAlarm = false;
    if(bpress == SELECT) {
      SetItem();
    } else if(bpress == UP) {
      idx--;if(idx<0) idx=MENUITEMS-1;
    } else if(bpress == DOWN) {
      idx++;if(idx>MENUITEMS-1) idx=0;
    } else if(bpress == RIGHT && Menu[idx].ValueSettable) {
      Menu[idx].ValueSettingNow = true;
      Menu[idx].ValueSetTo++;
      if(Menu[idx].LastOption>0 && Menu[idx].ValueSetTo>Menu[idx].LastOption) Menu[idx].ValueSetTo = 0;
    } else if(bpress == LEFT && Menu[idx].ValueSettable) {
      Menu[idx].ValueSettingNow = true;
      Menu[idx].ValueSetTo--;
      if(Menu[idx].LastOption>0 && Menu[idx].ValueSetTo<0) Menu[idx].ValueSetTo = Menu[idx].LastOption;
    }
    // Update Display
    LCD_display();
  }

  // Disable Setting anytime Menu Changes
  if(last_idx != idx) Menu[last_idx].ValueSettingNow = false;
  last_idx = idx;
}
