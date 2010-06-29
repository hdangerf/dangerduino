//------------------------------------------------------------------------------
//  
// 
//
//
// Purpose: Krusduino   Fish Tank LED Lighting Controller
//
//
//
// Initial version 1.0   15.09.2009
//
//  Changes      Date              Details
//               16.09.09         Added Extras to menu,
//               20.11.09         1 sec interrupt correction
//               24.11.09         Added temperature to LEDTest
//               18.12.09         Changed "check" function to make more accuate using floating point, 
//                                + added serial case S to toggle speedup
//                                + in serial case D added display clock after setting it
//               3.2.10            Changed dimming values to be an actual useable set
//               4.3.10            R13 - Added Mean Well support, tidied up booleans
//                                 + corrected reading temperature in LED Test Screen
//                17.3.10          R13c  testing the Clock Halt Bit resetting and tidy up screen
//                29.6.10          R14   adding Farenheit support
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation version 3
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
//------------------------------------------------------------------------------

 
#include <EEPROM.h>       // EEPROM for LED patterns
#include <Wire.h>         // specify use of Wire.h library.
#include <WString.h>
#include <Messenger.h>    // This example executes serial commands through a callback function
#include <avr/interrupt.h>

#include "nokia_3310_lcd.h"

#include "whiteblue_bmp.h"
#include "time_bmp.h"
#include "degC_bmp.h"
#include "degF_bmp.h"

String dataString;

const int TIMER1_F = 5000;  //was 5000
boolean debugon = false;
boolean load_from_EEPROM  = false;
boolean speedup  = false;

// for use in setting via LCD keypanel
boolean temp_debugon = debugon;
boolean temp_load_from_EEPROM  = load_from_EEPROM;
boolean temp_speedup  = speedup;


#define SS 15  //Sector size

int ledPinBlue = 6;   // select the pin for the LED
int ledPinWhite = 5;

int ticks ;
int ltick;
int min_cnt ;

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

boolean BUCKPUCK  = true;  //For Mean Well displays change "true" to "false"
boolean CELSIUS  = true;  //For Fahrenheit change "true" to "false"

//Dimming vales can be changed below
//Dimming values 8am_to_10pm_2hrDawn_2hrDusk_moonlight_75percent
byte bled[96] = {
  1, 1, 1, 1, 1, 1, 1, 1,  //0 - 1
  1, 1, 1, 1, 1, 1, 1, 1,  //2 - 3
  1, 1, 1, 1, 1, 1, 1, 1,  //4 - 5
  1, 1, 1, 1, 1, 1, 1, 1,  //6 - 7
  1, 16, 32, 55, 80, 110, 140, 190,  //8 - 9
  190, 190, 190, 190, 190, 190, 190, 190,  //10 - 11
  190, 190, 190, 190, 190, 190, 190, 190,  //12 - 13
  190, 190, 190, 190, 190, 190, 190, 190,  //14 - 15
  190, 190, 190, 190, 190, 190, 190, 190,  //16 - 17
  190, 190, 190, 190, 190, 190, 190, 190,  //18 - 19
  190, 110, 90, 70, 55, 32, 16, 1,  //20 - 22
  1, 1, 1, 1, 1, 1, 1, 1    //22 - 23
};  
byte wled[96] = {
  0, 0, 0, 0, 0, 0, 0, 0,  //0 - 1
  0, 0, 0, 0, 0, 0, 0, 0,  //2 - 3
  0, 0, 0, 0, 0, 0, 0, 0,  //4 - 5
  0, 0, 0, 0, 0, 0, 0, 0,  //6 - 7
  0, 16, 32, 55, 80, 110, 140, 190,  //8 - 9
  190, 190, 190, 190, 190, 190, 190, 190,  //10 - 11
  190, 190, 190, 190, 190, 190, 190, 190,  //12 - 13
  190, 190, 190, 190, 190, 190, 190, 190,  //14 - 15
  190, 190, 190, 190, 190, 190, 190, 190,  //16 - 17
  190, 190, 190, 190, 190, 190, 190, 190,  //18 - 19
  190, 110, 90, 70, 55, 32, 16, 0,  //20 - 22
  0, 0, 0, 0, 0, 0, 0, 0    //22 - 23
};  //White LED array in RAM

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
byte bled_out ;
byte wled_out ;

#define TEMP_PIN  4  //Temperature measurement pin
void OneWireReset(int Pin);
void OneWireOutByte(int Pin, byte d);
byte OneWireInByte(int Pin);

int SignBit,  Whole, Fract;

//Real Time Clock variables
int secs, mins, hrs, day, date, month, year;

int set_secs = (0); //seconds;
int set_mins = (0) ; //minutes
int set_hrs = (10); //hours (24hr time)
int set_day = (6); // Day 01-07
int set_date = (1); // Date 0-31
int set_month = (10); // month 0-12
int set_year = (9); // Year 00-99

byte CST;

//*******Nokia 3310
//keypad debounce parameter
#define DEBOUNCE_MAX 15
#define DEBOUNCE_ON  10
#define DEBOUNCE_OFF 3 

#define NUM_KEYS 5

#define NUM_MENU_ITEM	5  //Main, Set Time, LED Test, Extras, About

// joystick number
#define UP_KEY 3
#define LEFT_KEY 0
#define CENTER_KEY 1
#define DOWN_KEY 2
#define RIGHT_KEY 4

// menu starting points

#define MENU_X	0		// 0-83
#define MENU_Y	5		// 0-5

// adc preset value, represent top value,incl. noise & margin,that the adc reads, when a key is pressed
// set noise & margin = 30 (0.15V@5V)
int  adc_key_val[5] ={30, 150, 360, 535, 760 };

// debounce counters
byte button_count[NUM_KEYS];
// button status - pressed/released
byte button_status[NUM_KEYS];
// button on flags for user program 
byte button_flag[NUM_KEYS];

// menu definition  - note Pad to eight characters
char menu_items[NUM_MENU_ITEM][12]={
	"MAIN    ",
        "SET TIME",
        "LED TEST",
        "EXTRAS  ",
        "ABOUT   "
};

void (*menu_funcs[NUM_MENU_ITEM])(void) = {
	menu_main,
        menu_set_time,
        menu_led_test,
        menu_extras,
        menu_about
};

char current_menu_item;

const char RTC_MENU_ITEMS = 9;
const char LED_MENU_ITEMS = 7;
const char EXTRAS_MENU_ITEMS = 5;

byte LCD_backlight_pinState = 1;
char tstr[5];  // temporary store for manipulating strings for LCD output



Nokia_3310_lcd lcd=Nokia_3310_lcd();


//******** Instantiate Messenger object with the default separator (the space character)

Messenger message = Messenger(); 


//*****************************************
// START of INITIALIZATION

void setup() {
    Serial.begin(19200);
// Initiate Serial Communication
// Attach the callback function to the Messenger
  message.attach(messageReady);
 // Serial.println("Messenger Attached Ready");
  pinMode(2, INPUT);  //interupt
  Wire.begin();  //join I2C bus (address optional for master)-RTC on I2C
  
//  // Setup timer1 -- Prescaler/64
//  TCCR1A = 0;
//  TCCR1B = _BV(WGM12) | _BV(CS11) | _BV(CS10);
// OCR1A  = TIMER1_F;
 // TCNT1  = 0;
//  TIMSK1 = _BV(OCIE1A);
  
//  SREG |= 1 << SREG_I;


//* Connect to digital port 2  aka external interrupt 0

attachInterrupt(0,onesecint, RISING);

//********

// reset button arrays
   for(byte i=0; i<NUM_KEYS; i++){
     button_count[i]=0;
     button_status[i]=0;
     button_flag[i]=0;
   }
  
  // Setup timer2 -- Prescaler/256
  TCCR2A &= ~((1<<WGM21) | (1<<WGM20));
  TCCR2B &= ~(1<<WGM22);
  TCCR2B = (1<<CS22)|(1<<CS21);      
  
  ASSR |=(0<<AS2);

   // Use normal mode  
   TCCR2A =0;    
     //Timer2 Overflow Interrupt Enable  
     TIMSK2 |= (0<<OCIE2A);
     TCNT2=0x6;  // counting starts from 6;  
   TIMSK2 = (1<<TOIE2);    
   
                  
 
  SREG|=1<<SREG_I;
  
  lcd.LCD_3310_init();
  lcd.LCD_3310_clear();
  
   //menu initialization
   init_MENU();
   current_menu_item = 0;

//**********


  Serial.println("Aquarium Lighting Control with display  r14 29-Jun-10");
  if (BUCKPUCK) {Serial.println("Buck Puck configured");}
  else {Serial.println("Mean Well configured");}
  //Serial.print(" Clock Status is ->   ");
  //CST = ReadClockStatus();
  //Serial.println(CST ,HEX);
  //------Now Temp stuff
  digitalWrite(TEMP_PIN, LOW);
    pinMode(TEMP_PIN, INPUT);      // sets the digital pin as input (logic 1) 
 
                                   //  for 1 wire interface to temperature sensor
  
  ltick =0;
  ticks = 0;
  
  SetClockStatus();                   
  
  
  //LoadLEDArray();      //Load LED pattern from EEPROM to LED arrays
  
 
  ReadTemperature();    // First read always seems to be about 80 deg!!
  ReadTemperature();
  SerialPrintTemperature();
                        //Enable interrupts  ****************
  SyncClock();  // synchronize with RTC
  SerialPrintClock();
  LED_levels_output();  
  LCD_Main_Draw();                      
  
}


//*****************************************
// START of MAINLOOP

void loop() {
  // The following line is the most effective way of using Serial and Messenger's callback
  while ( Serial.available() ) {
    //Serial.print("Characters In ");
    message.process(Serial.read () );
  }
  
  if( ticks >=1)   // 20 msec to a minute 30000 but sec for testing thus
  {  
    min_cnt = min_cnt+ticks;
    ticks = 0;
    //flash LED 13
    LED_levels_output();
    
    LCD_Main_Draw();
    
       if (debugon) {Serial.print(" >W ");
       Serial.print(int(wled_out));
       Serial.print(" -B ");
       Serial.print(int(bled_out));
       Serial.print(" .. ");
       Serial.println(min_cnt); }

  }
  
  // Here some serial instrumentation stuff
   // delay(2000);
   //ticks=501;
   byte i;
    for(i=0; i<NUM_KEYS; i++){
       if(button_flag[i] !=0){
            
        button_flag[i]=0;  // reset button flag
	switch(i){

		case LEFT_KEY:
			// current item to normal display
		  //lcd.LCD_3310_write_string(MENU_X, MENU_Y + current_menu_item, menu_items[current_menu_item], MENU_NORMAL );
		  current_menu_item -=1;
		  if(current_menu_item <0)  current_menu_item = NUM_MENU_ITEM -1;
			// next item to highlight display
		  lcd.LCD_3310_write_string(MENU_X, MENU_Y, menu_items[current_menu_item], MENU_HIGHLIGHT );
		  
                  break;

		case RIGHT_KEY:
				// current item to normal display
		  //lcd.LCD_3310_write_string(MENU_X, MENU_Y + current_menu_item, menu_items[current_menu_item], MENU_NORMAL );
		  current_menu_item +=1;
		  if(current_menu_item >(NUM_MENU_ITEM-1))  current_menu_item = 0;
					// next item to highlight display
		  lcd.LCD_3310_write_string(MENU_X, MENU_Y, menu_items[current_menu_item], MENU_HIGHLIGHT );
		  break;

		 case UP_KEY:
                  init_MENU();
                  LED_levels_output();  
                  LCD_Main_Draw(); 
		  current_menu_item = 0;
                              // Toggle LCD Backlight state
                  LCD_backlight_pinState = !LCD_backlight_pinState;
                  digitalWrite(7, LCD_backlight_pinState); 
                                // if _pinState = 0, set it to 1, and vice versa:
		  break;

		 case DOWN_KEY:
        	     lcd.LCD_3310_clear();
		    (*menu_funcs[current_menu_item])();
                     lcd.LCD_3310_clear();
		    init_MENU();
                    LED_levels_output();  
                    LCD_Main_Draw(); 
		    current_menu_item = 0;           
		    break;

                  case CENTER_KEY:
        	    lcd.LCD_3310_clear();
		    (*menu_funcs[current_menu_item])();
                     lcd.LCD_3310_clear();
		    init_MENU();
                    LED_levels_output();  
                    LCD_Main_Draw(); 
		    current_menu_item = 0;           
		    break;	
		}
				
	}
    }
   

}
// END of MAINLOOP

// Create the callback function
void messageReady() {
    int type_int;
    char type_char;
    char mtype;
    int E_start_addr, E_length, E_value;
       // Loop through all the available elements of the message
       while ( message.available() ) {
	// Set the pin as determined by the message
         //digitalWrite( pin, message.readInt() );
         //pin=pin+1;
        
          mtype = message.readChar();
          //if (debugon) {Serial.print("Message Available Type -> ");
          //Serial.println(mtype);}
          switch (mtype) {
            case 'C':
// C   Read clock
            if (debugon) {Serial.print("Case C Char  --> ");}
            type_char = message.readChar();
            Serial.println(type_char);
            ReadClock();
            SerialPrintClock();
            break;
            
            case 'D':
// D Set clock
            if (debugon) {Serial.print("Case D Char  --> ");}
            set_secs = message.readInt();
            set_mins = message.readInt();
            set_hrs = message.readInt();
            set_day = message.readInt();
            set_date = message.readInt();
            set_month = message.readInt();
            set_year = message.readInt();      
            SetClock();
            ReadClock();
            SerialPrintClock();
            LCD_Main_Draw();
            break;
            
             
            case 'E':
//E  Read from EEPROM
            // E <start_addr> <length>     space delimiter, end with CR
             E_start_addr = message.readInt();  // get EEPROM start address
             E_length = message.readInt();  // get EEPROM start address
             for (int i = E_start_addr; i <= (E_start_addr + E_length); i++){
             int val = EEPROM.read(i);
             Serial.print(i);
             Serial.print(" EEPROM --> ");
             Serial.println(val,DEC);  
             }
            break;
            
            case 'F':
// F  Write to EEPROM
            // F < addr> <value>     space delimiter, end with CR
             E_start_addr = message.readInt();  // get EEPROM start address
             E_value = message.readInt();  // get EEPROM start address 
             EEPROM.write(E_start_addr, E_value);
             if (debugon) {Serial.print(E_start_addr);
             Serial.print(" EEPROM --> ");
             Serial.println(E_value,DEC);}
            break;
            
            case 'G':
// G  Load EEPROM to LED arrays
            // G         end with CR
            LoadLEDArray();
             Serial.println(" LED values loaded ");
            break;
 
             case 'S':
// S Speedup status toggle
            speedup =  !speedup;
            Serial.print("Speedup Status is  ");
            Serial.println(speedup, BIN);
            break;
                     
            
            case 'T': 
// T Read Temperature
            type_int = message.readInt();
            if (debugon) {Serial.print("Case T Integer --> ");
            Serial.println(type_int);}
            ReadTemperature();
            SerialPrintTemperature();
            
            break;

            case 'U': 
// U Toggle Temperature Units
            type_int = message.readInt();
            CELSIUS = !CELSIUS;
            ReadTemperature();
            SerialPrintTemperature();
            
            break;            
            case 'Z':
// Z Debug status toggle
            debugon = !debugon;
            Serial.print("Debug Status is  ");
            Serial.println(debugon, BIN);
            break;
            
            default:
            Serial.println("No match");
            break;
          }
      }
}

//**************************
//LED Levels output
//
void LED_levels_output()
{
int sector, sstep, t1, t2 ;
byte b_out, w_out;

if (min_cnt>=1440) {min_cnt=1;}   // 24 hours of minutes
    sector = min_cnt/SS;    // divided by gives sector -- 15 minute
    sstep = min_cnt%SS;     // remainder gives add on to sector value
  
   
    t1 =sector;
    if (t1==95) {t2=0;}
    else {t2 = t1+1;}
       if (debugon) {Serial.print(t1);
       Serial.print("__");
       Serial.print(sstep);}
    if (sstep==0)
    { bled_out = bled[t1];
    wled_out = wled[t1];
    //Serial.println("********>border");
    }
    else { bled_out = check(&bled[t1], &bled[t2], sstep);
          wled_out = check(&wled[t1], &wled[t2], sstep);
    }
    
    if (BUCKPUCK) {b_out = bled_out;
                  w_out = wled_out;}
    
    else {b_out = 255 - bled_out;
          w_out = 255 - wled_out;}
    
    analogWrite(ledPinBlue, b_out);
    analogWrite(ledPinWhite, w_out);
    
  }
  
  
  byte check( byte *pt1, byte *pt2, int lstep)
  {
    byte result;
    float fresult;
   
    if (*pt1==*pt2) {result = *pt1;}   // No change
    else if (*pt1<*pt2)                //Increasing brightness
    { fresult = ((float(*pt2-*pt1)/15.0) * float(lstep))+float(*pt1);
     result = byte(fresult);
     }
     //Decreasing brightness
    else {fresult = -((float(*pt1-*pt2)/15.0) * float(lstep))+float(*pt1);
     result = byte(fresult);
                     
  }
 
    return result;
  }

//********************
//One wire subroutines  for Temperature reading - only ONE device possible with this code

void OneWireReset(int Pin) // reset.  Should improve to act as a presence pulse
{
     digitalWrite(Pin, LOW);
     pinMode(Pin, OUTPUT); // bring low for 500 us
     delayMicroseconds(500);
     pinMode(Pin, INPUT);
     delayMicroseconds(500);
}

void OneWireOutByte(int Pin, byte d) // output byte d (least sig bit first).
{
   byte n;
   
   for(n=8; n!=0; n--)
   {
      if ((d & 0x01) == 1)  // test least sig bit
      {
         digitalWrite(Pin, LOW);
         pinMode(Pin, OUTPUT);
         delayMicroseconds(5);
         pinMode(Pin, INPUT);
         delayMicroseconds(60);
      }
      else
      {
         digitalWrite(Pin, LOW);
         pinMode(Pin, OUTPUT);
         delayMicroseconds(60);
         pinMode(Pin, INPUT);
      }

      d=d>>1; // now the next bit is in the least sig bit position.
   }
   
}

byte OneWireInByte(int Pin) // read byte, least sig byte first
{
    byte d, n, b;
    
    for (n=0; n<8; n++)
    {
        digitalWrite(Pin, LOW);
        pinMode(Pin, OUTPUT);
        delayMicroseconds(5);
        pinMode(Pin, INPUT);
        delayMicroseconds(5);
        b = digitalRead(Pin);
        delayMicroseconds(50);
        d = (d >> 1) | (b<<7); // shift d to right and insert b in most sig bit position
    }
    return(d);
}


void ReadTemperature()
{
 int HighByte, LowByte, TReading,  Tc_100 ;

            OneWireReset(TEMP_PIN);
            OneWireOutByte(TEMP_PIN, 0xcc);
            OneWireOutByte(TEMP_PIN, 0x44); // perform temperature conversion, strong pullup for one sec

            OneWireReset(TEMP_PIN);
            OneWireOutByte(TEMP_PIN, 0xcc);
            OneWireOutByte(TEMP_PIN, 0xbe);

            LowByte = OneWireInByte(TEMP_PIN);
            HighByte = OneWireInByte(TEMP_PIN);
//From the datasheet, you basically get the result in 2 bytes:
//
//[  7  6  5  4  3   2  1  0 ] low byte
//[  s  s  s  s  11 10  9  8 ] high byte
//
//which is a 2's complement 12-bit number sign-extended to 16-bits. 
//That means if we tell the compiler it is a 16-bit signed variable (int), it'll treat it like one.
//
//Secondly, the temperature in degrees is represented as a fractional number, with the bottom 4-bits being the fraction. Think of it like:
//
//[ s s s s v v v v v v v v . f f f f ]
//
//So to get it in degrees, we would divide by 16 (2 ^ 4). 
//If we do that as an integer division, we'd obviously only get the result in whole degrees, 
//so in this case, we'll divide to create a floating point number.           
		   
		   
            TReading = (HighByte << 8) + LowByte;  //construct sign extended 12-bit value
            
           float temperature = float(TReading) / 16.0;  //convert 12.4 fractional result to float
		   
		if (!CELSIUS) { temperature = (temperature*1.8) + 32.0;}		// Convert to Farenheit if required
		// For use in displaying on the LCD screen
		// Get sign bit
        if (temperature<0) {SignBit = 1;}
          else {SignBit = 0;}
		  // separate off the whole and fractional portions
          Whole = int(temperature);
          Fract = int((temperature - float(Whole))*100);       
} 

void SerialPrintTemperature ()
{ 
  if (CELSIUS) {Serial.print("Temperature in Deg. C  --> ");}
  else {Serial.print("Temperature in Deg. F  --> ");}
            if (SignBit) // If its negative
            {
             Serial.print("-");
            }
            Serial.print(Whole);
            Serial.print(".");
            if (Fract < 10)
            {
               Serial.print("0");
            }
            Serial.println(Fract);
}

//********************
//
void LoadLEDArray ()   // Load LED array from EEPROM
{
  Serial.println("Load LED Array from EEPROM ");
  for (int i = 0; i <= 95; i++){
            wled[i] = EEPROM.read(i);
            bled[i] = EEPROM.read(i+96);
            }
}

//********************
//

void ReadClock()
{
  Wire.beginTransmission(0x68);
  Wire.send(0);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 7);
  secs = BCDtoDec(Wire.receive());
  mins = BCDtoDec(Wire.receive());
  hrs = BCDtoDec(Wire.receive());
  day = BCDtoDec(Wire.receive());
  date = BCDtoDec(Wire.receive());
  month = BCDtoDec(Wire.receive());
  year = BCDtoDec(Wire.receive());

  
} 

void SerialPrintClock()
{
// hours, minutes, seconds
  Serial.print("The time is ");
  if (hrs < 10) Serial.print("0");
  Serial.print(hrs);
  Serial.print(":");
  if (mins < 10) Serial.print("0");
  Serial.print(mins);
  Serial.print(":");
  if (secs < 10) Serial.print("0");
  Serial.println(secs);
  // use DD-MM-YYYY
  Serial.print("The date is ");
  if (date < 10) Serial.print("0");
  Serial.print(date);
  Serial.print("-");
  if (month < 10) Serial.print("0");
  Serial.print(month);
  Serial.print("-");
  Serial.print("20");
  if (year < 10) Serial.print("0");
  Serial.println(year);
  Serial.print(" Clock Status is ->   ");
  CST = ReadClockStatus();
  Serial.println(CST, HEX );
}

byte ReadClockStatus()
{
  byte clock_status;
  Wire.beginTransmission(0x68);
  Wire.send(7);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 1);
  clock_status = Wire.receive();
 return (clock_status);
} 

void SetClockStatus ()
{
  byte ch_secs;
  
  // now clear CH bit - bit 7 in seconds register
  
  Wire.beginTransmission(0x68);
  Wire.send(0);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 1);
  ch_secs = Wire.receive();
  ch_secs = ch_secs & B01111111;  // mask bit 7 in seconds register
  Wire.beginTransmission(0x68); // activate DS1307
  Wire.send(0);
  Wire.send(ch_secs); // clear CH bit in DS1307
  Wire.endTransmission();
  delay(10);
  Wire.beginTransmission(0x68); // activate DS1307
  Wire.send(7); // where to begin
  Wire.send(0x10); // Control 0x10 produces a 1 HZ square wave on pin 7.
  Wire.endTransmission();
}

void SetClock()
{
Wire.beginTransmission(0x68); // activate DS1307
Wire.send(0); // where to begin
Wire.send(DectoBCD(set_secs)); //seconds
Wire.send(DectoBCD(set_mins)); //minutes
Wire.send(DectoBCD(set_hrs)); //hours (24hr time)
Wire.send(DectoBCD(set_day)); // Day 01-07
Wire.send(DectoBCD(set_date)); // Date 0-31
Wire.send(DectoBCD(set_month)); // month 0-12
Wire.send(DectoBCD(set_year)); // Year 00-99

Wire.send(0x10); // Control 0x10 produces a 1 HZ square wave on pin 7.
Wire.endTransmission();


secs = set_secs;
mins = set_mins; //minutes
hrs = set_hrs; //hours (24hr time)
day = set_day; // Day 01-07
date = set_date; // Date 0-31
month = set_month; // month 0-12
year = set_year;
min_cnt= (hrs*60)+ mins;  // synchronize with RTC
}

void SyncClock()
{
  ReadClock();
  min_cnt= ( hrs*60)+ mins;  // synchronize with RTC
}
//**************************
void init_MENU(void){

  byte i;
  
  lcd.LCD_3310_clear();

    lcd.LCD_3310_write_string(MENU_X, MENU_Y, menu_items[0], MENU_HIGHLIGHT );
	
  //for (i=1; i<NUM_MENU_ITEM; i++){
   // lcd.LCD_3310_write_string(MENU_X*i*6, MENU_Y, menu_items[i], MENU_NORMAL);      
  //}
	
	
}

// waiting for center key press
void waitfor_OKkey(){
  byte i;
  byte key = 0xFF;
	while (key!= CENTER_KEY){
    for(i=0; i<NUM_KEYS; i++){
       if(button_flag[i] !=0){
           button_flag[i]=0;  // reset button flag
           if(i== CENTER_KEY) key=CENTER_KEY;
        }
     }
   }
		
}

void menu_main()
{
        
	//waitfor_OKkey();
}


void menu_set_time(){
 
 LCDSetupRTClock();
}

void menu_led_test()
{
  LCDSetupLEDTest();       
//	waitfor_OKkey();
}

void menu_extras()
{
  LCDSetupExtras();
}

void menu_about(){
  
  lcd.LCD_3310_write_string(0, 0, "LED Array by", MENU_NORMAL);
  lcd.LCD_3310_write_string(0, 1, "Dave Rosser", MENU_NORMAL);
  lcd.LCD_3310_write_string(0, 2, "Arduino by", MENU_NORMAL);
  lcd.LCD_3310_write_string(0, 3, "Hugh", MENU_NORMAL);
  lcd.LCD_3310_write_string(0, 4, "Dangerfield", MENU_NORMAL);
  lcd.LCD_3310_write_string(0, 5, "r14", MENU_NORMAL);
  lcd.LCD_3310_write_string(54, 5, "06-10", MENU_NORMAL);
  
  lcd.LCD_3310_write_string(30, 5, "OK", MENU_HIGHLIGHT );
	waitfor_OKkey();   
}

// The followinging are interrupt-driven keypad reading functions
//  which includes DEBOUNCE ON/OFF mechanism, and continuous pressing detection


// Convert ADC value to key number
char get_key(unsigned int input)
{
	char k;
    
	for (k = 0; k < NUM_KEYS; k++)
	{
		if (input < adc_key_val[k])
		{
           
    return k;
        }
	}
    
    if (k >= NUM_KEYS)
        k = -1;     // No valid key pressed
    
    return k;
}

void update_adc_key(){
  int adc_key_in;
  char key_in;
  byte i;
  
  adc_key_in = analogRead(0);
  key_in = get_key(adc_key_in);
  for(i=0; i<NUM_KEYS; i++)
  {
    if(key_in==i)  //one key is pressed 
    { 
      if(button_count[i]<DEBOUNCE_MAX)
      {
        button_count[i]++;
        if(button_count[i]>DEBOUNCE_ON)
        {
          if(button_status[i] == 0)
          {
            button_flag[i] = 1;
            button_status[i] = 1; //button debounced to 'pressed' status
          }
		  
        }
      }
	
    }
    else // no button pressed
    {
      if (button_count[i] >0)
      {  
		button_flag[i] = 0;	
		button_count[i]--;
        if(button_count[i]<DEBOUNCE_OFF){
          button_status[i]=0;   //button debounced to 'released' status
        }
      }
    }
    
  }
}

// Timer2 interrupt routine -
// 1/(160000000/256/(256-6)) = 4ms interval

ISR(TIMER2_OVF_vect) {  
  TCNT2  = 6;
  update_adc_key();
}


// Interrupt routine - 1 sec
void onesecint()  {
 //ticks++; // every second
 int tickrate;
  if (speedup==1) { tickrate = 3;}
  else {tickrate = 60;}
 ltick++;
if (ltick>= tickrate) {  //change Here was 20 ********
   ltick = 0;
   ticks++;}
}

// Timer1 interrupt routine -
// (64*5000)/160000000 = 20ms interval

//ISR(TIMER1_COMPA_vect)
//{
  
 // ticks++; // every 20 msec
//  Serial.println(ticks);
//   ltick++;
 //if (ltick>= 50) {
//   ltick = 0;
//   ticks++;
// }
//}


void draw_barchart(byte led1, byte led2){
unsigned int dot_1_count, dot_2_count;

  unsigned int i;
  byte j;
 
  //lcd.LCD_3310_write_byte(0x80, 0); //x
  //lcd.LCD_3310_write_byte(0x40, 0); //y
  lcd.LCD_3310_set_XY(0, 0);
  //for (i=0; i<80; i++) {lcd.LCD_3310_write_byte(0x03, 1);}
  lcd.LCD_3310_write_byte(0x7f , 1);
  dot_1_count = led1/4;
  dot_2_count = led2/4;
  //Serial.print(dot_white_count);
  //Serial.print("  --> ");
  //Serial.println(dot_blue_count);
  for (i=0; i <=64; i++) {
    j = 0;
    if (i<dot_1_count) {j = j | 0x03;}
    if (i<dot_2_count){ j = j | 0x30;}
    lcd.LCD_3310_write_byte(j , 1);
  }
  lcd.LCD_3310_write_byte(0x7f , 1);
}

void LCD_write_chars (unsigned char X,unsigned char Y,unsigned char *bmp, int char_cnt)
{
  unsigned int i;

  lcd.LCD_3310_set_XY(X, Y);
  for (i=0; i<char_cnt*6; i++) {
    lcd.LCD_3310_write_byte(bmp[i] , 1);
  }
}


void LCD_ReadTemperature(unsigned int tx, unsigned int ty){
  ReadTemperature();
dataString="";  //clear string
            if (SignBit) // If its negative
            {
             dataString.append("-");
            }
            itoa (Whole,tstr, 10);
            dataString.append(tstr);
            dataString.append(".");
           // if (Fract < 10)
           // {
           //   dataString.append("0");
           // }
           Fract = Fract/10;
            itoa (Fract,tstr, 10);
            dataString.append(tstr);
             lcd.LCD_3310_write_string(tx,ty, dataString, MENU_NORMAL );
}
 
 
void LCD_Main_Draw() {
  
   draw_barchart(wled_out, bled_out);
   LCD_write_chars(68,0,whiteblue_bmp,2);
   if (CELSIUS) {LCD_write_chars(78,2, degC_bmp,1);}
   else {LCD_write_chars(78,2, degF_bmp,1);}
  //lcd.LCD_3310_write_byte(0x80, 0);
 // lcd.LCD_3310_write_byte(0x40, 0); //y
 //lcd.LCD_3310_write_byte(0xff, 1);
 //lcd.LCD_3310_write_byte(0xff, 1);
 
 ReadClock();
 LCD_write_chars(0,3,time_bmp,14);
 dataString="";  //clear string

  //Hours.Minutes
 if (hrs<10) { dataString.append("0");}
 itoa (hrs,tstr, 10);   // note data in HEX thus BASE 16 conversion
 dataString.append(tstr);
 dataString.append(".");
 if (mins<10) { dataString.append("0");}
 itoa (mins,tstr,10);
 dataString.append(tstr);
 lcd.LCD_3310_write_string(0, 4, dataString, MENU_NORMAL );
 
 dataString="";  //clear string
 
  //Date-Month-Year
 if (date<10) { dataString.append("0");}
 itoa (date,tstr,10);
 dataString.append(tstr);
  dataString.append("-");
 if (month<10) { dataString.append("0");}
 itoa (month,tstr,10);
 dataString.append(tstr);
dataString.append("-");
 if (year<10) { dataString.append("0");}
 itoa (year,tstr,10);
  dataString.append(tstr);
 lcd.LCD_3310_write_string(36, 4, dataString, MENU_NORMAL );
 
 
LCD_ReadTemperature(48,2);
 		
}

void RTCsetDrawMenu( const char rtcmenu_item)
{
  char rtcstr[ 6 ];
     dataString="";  //clear string
    if (set_hrs<10) { dataString.append("0");}
        itoa (set_hrs,rtcstr, 10);   // note data in HEX thus BASE 16 conversion
        dataString.append(rtcstr);
  if( rtcmenu_item == 0 ) {lcd.LCD_3310_write_string(20, 0,  dataString, MENU_HIGHLIGHT );}
  else {lcd.LCD_3310_write_string( 20, 0, dataString, MENU_NORMAL );}
  
       dataString="";
       if (set_mins<10) { dataString.append("0");}
       itoa (set_mins,rtcstr,10);
       dataString.append(rtcstr);
  if( rtcmenu_item == 1 ) {lcd.LCD_3310_write_string(56, 0,  dataString, MENU_HIGHLIGHT );}
  else {lcd.LCD_3310_write_string(56, 0,  dataString, MENU_NORMAL );}
  
       dataString="";
       if (set_secs<10) { dataString.append("0");}
       itoa (set_secs,rtcstr,10);
       dataString.append(rtcstr);
  if( rtcmenu_item == 2 ) {lcd.LCD_3310_write_string(20, 1,  dataString, MENU_HIGHLIGHT );}
  else {lcd.LCD_3310_write_string(20, 1,  dataString, MENU_NORMAL );}
    
         dataString="";
         itoa (set_day,rtcstr,10);
          dataString.append(rtcstr);
  if( rtcmenu_item == 3 ) {lcd.LCD_3310_write_string(20, 3,  dataString, MENU_HIGHLIGHT );}
  else {lcd.LCD_3310_write_string(20, 3,  dataString, MENU_NORMAL );}
    
       dataString="";
       if (set_date<10) { dataString.append("0");}
       itoa (set_date,rtcstr,10);
       dataString.append(rtcstr);
  if( rtcmenu_item == 4 ) 
    {lcd.LCD_3310_write_string( 68, 3, dataString, MENU_HIGHLIGHT );}
  else {lcd.LCD_3310_write_string( 68, 3, dataString, MENU_NORMAL );}

  dataString="";
       if (set_month<10) { dataString.append("0");}
       itoa (set_month,rtcstr,10);
       dataString.append(rtcstr);
  if( rtcmenu_item == 5 ) {lcd.LCD_3310_write_string( 20, 4, dataString, MENU_HIGHLIGHT );}
  else {lcd.LCD_3310_write_string( 20, 4, dataString, MENU_NORMAL );}
  
 dataString="";
       if (set_year<10) { dataString.append("0");}
       itoa (set_year,rtcstr,10);
       dataString.append(rtcstr);
  if( rtcmenu_item == 6 ) {lcd.LCD_3310_write_string( 68, 4, dataString, MENU_HIGHLIGHT );}
  else {lcd.LCD_3310_write_string( 68, 4, dataString, MENU_NORMAL );}
  

  if( rtcmenu_item == 7 ) {lcd.LCD_3310_write_string(0, 5,  "OK", MENU_HIGHLIGHT );}
  else {lcd.LCD_3310_write_string(0, 5, "OK",  MENU_NORMAL );}

  if( rtcmenu_item == 8 ) {lcd.LCD_3310_write_string( 48, 5, "Cancel", MENU_HIGHLIGHT );}
  else {lcd.LCD_3310_write_string( 48, 5, "Cancel", MENU_NORMAL );}
}


void RTCSet_Down_Key(char Lmenu_item)
{
          switch( Lmenu_item ) {
            case 0:
            set_hrs--;
            if( set_hrs < 0 ) {set_hrs = 23;}
              break;
            case 1:
            set_mins--;
            if( set_mins < 0 ) {set_mins = 59;}
            break;
            case 2:
            set_secs--;
            if( set_secs < 0 ) {set_secs = 59; }
            break;
            case 3:
            set_day--;
            if( set_day < 1 ) {set_day = 7;}
            break;
            case 4:
            set_date--;
            if( set_date < 1 ) {set_date = 31;}
            break;
            case 5:
            set_month--;
            if( set_month < 0 ) {set_month = 11; }
            break;
            case 6:
            set_year--;
            if( set_year < 8 ) {set_year = 20;}
            break;
            default:;
          }
}

void RTCSet_Up_Key(char Rmenu_item)
{
  switch( Rmenu_item ) {
          case 0:
            set_hrs++;
            if( set_hrs >= 24 ) {set_hrs = 0;}
          break;
          case 1:
            set_mins++;
            if( set_mins >= 60 ) {set_mins = 0; }
          break;
          case 2:
            set_secs++;
            if( set_secs >= 59 ) {set_secs = 0;}
          break;
          case 3:
            set_day++;
            if( set_day >= 8 ) {set_day = 1; }
          break;
          case 4:
            set_date++;
            if( set_date >= 32 ) {set_date = 1; }
          break;

          case 5:
            set_month++;
            if( set_month >= 12 ) {set_month = 0; }
          break;
          case 6:
            set_year++;
            if( set_year >= 21 ) {set_year = 9;}
          break;
          default:;
        }
}

void LCDSetupRTClock()
{
  set_secs = secs;
  set_mins = mins; //minutes
  set_hrs = hrs; //hours (24hr time)
  set_day = day; // Day 01-07
  set_date = date; // Date 0-31
  set_month = month; // month 0-12
  set_year = year;

  lcd.LCD_3310_write_string( 0, 0,  "hrs",MENU_NORMAL );
  lcd.LCD_3310_write_string( 36, 0,  "min",MENU_NORMAL );
  lcd.LCD_3310_write_string( 0, 1,  "sec",MENU_NORMAL );
  lcd.LCD_3310_write_string( 0, 3,  "day",MENU_NORMAL );
  lcd.LCD_3310_write_string( 42, 3,  "date",MENU_NORMAL );
  lcd.LCD_3310_write_string( 0, 4,  "mth",MENU_NORMAL );
  lcd.LCD_3310_write_string( 42, 4,  "yr",MENU_NORMAL );
  

char rtcmenu_item = 0;

 while( 1 ) {
    RTCsetDrawMenu( rtcmenu_item);

  byte i;
  for(i=0; i<NUM_KEYS; i++){
       if(button_flag[i] !=0){
        button_flag[i]=0;  // reset button flag
        switch(i){
        case DOWN_KEY:
        RTCSet_Down_Key(rtcmenu_item); 
          
            break;
        case UP_KEY:
        RTCSet_Up_Key(rtcmenu_item);
          
            break;
        case LEFT_KEY:
         rtcmenu_item--;
          if( rtcmenu_item < 0 ) {
          rtcmenu_item = RTC_MENU_ITEMS - 1;
            }
          break;
       case RIGHT_KEY:
           rtcmenu_item++;
          if( rtcmenu_item >= RTC_MENU_ITEMS ) {
            rtcmenu_item = 0;
            }   
         break;
        case CENTER_KEY:
          if( rtcmenu_item == 7 ) {
            SetClock();
            return;
            }
            if( rtcmenu_item == 8 ) {
            return;
            }
            break;
            default:;
          }  // end switch
       }
    
     }
    } //end while loop
}

int BCDtoDec(byte bcd)
{
return ((bcd>>4)*10) + (bcd%16);
}

byte DectoBCD(int dec)
{
 return ((dec/10)<<4) + (dec%10);
}

//******************************

int led1test_out;
int led2test_out;

void DisplayLEDTest_LEDValues()
{
char lcdstr[ 6 ];
     dataString="";  //clear string
    if (led1test_out<10) { dataString.append("0");}
    if (led1test_out<100) { dataString.append("0");}
        itoa (led1test_out,lcdstr, 10);   
        dataString.append(lcdstr);
 lcd.LCD_3310_write_string( 36, 1,  dataString,MENU_NORMAL );
 dataString="";  //clear string
    if (led2test_out<10) { dataString.append("0");}
    if (led2test_out<100) { dataString.append("0");}
        itoa (led2test_out,lcdstr, 10);   
        dataString.append(lcdstr);
 lcd.LCD_3310_write_string( 36, 3,  dataString,MENU_NORMAL );
 
}


void LCDSetupLEDTest()
{
//led1test_out = wled_out;
//led2test_out = bled_out;
byte l1test_out, l2test_out;

led1test_out = 0;
led2test_out = 0;
draw_barchart(led1test_out, led2test_out);
LCD_write_chars(68,0,whiteblue_bmp,2);

  lcd.LCD_3310_write_string( 0, 1,  "white",MENU_NORMAL );
  lcd.LCD_3310_write_string( 0, 3,  "blue",MENU_NORMAL );
  LCD_ReadTemperature(0,5);
  DisplayLEDTest_LEDValues();
  if (CELSIUS) {LCD_write_chars(30,5, degC_bmp,1);}
  else {LCD_write_chars(30,5, degF_bmp,1);}
  
  
    byte i;
    char ledmenu_item = 0;
    unsigned long start_time, time_now, time_elapsed;
  start_time = millis();
 while( 1 ) {  
  LEDTestDrawMenu( ledmenu_item);
  
  for(i=0; i<NUM_KEYS; i++){
       if(button_flag[i] !=0){
        button_flag[i]=0;  // reset button flag
        switch(i){
        case DOWN_KEY:
          ledSet_Down_Key(ledmenu_item);
            break;
            
        case UP_KEY:
          ledSet_Up_Key(ledmenu_item); 
            break;
            
        case LEFT_KEY:
        ledmenu_item--;
          if( ledmenu_item < 0 ) {ledmenu_item = LED_MENU_ITEMS - 1;}
          break;
          
       case RIGHT_KEY:     
        ledmenu_item++;
          if( ledmenu_item >= LED_MENU_ITEMS ) {
            ledmenu_item = 0;}  
          break;
          
        case CENTER_KEY:
          if( ledmenu_item == 6 ) { 
            SyncClock();
            return; }
            break;
            
            default:;
          }  // end switch
          draw_barchart(led1test_out, led2test_out);
          DisplayLEDTest_LEDValues();
          
          // write PWM vales         
           if (BUCKPUCK) {l1test_out = led1test_out;
                        l2test_out = led2test_out;}
          else {l1test_out = 255 - led1test_out;   // Mean Well Test Values
                l2test_out = 255 - led2test_out;}

           analogWrite(ledPinWhite, byte(l1test_out));
           analogWrite(ledPinBlue,  byte(l2test_out));
       }
     
      }
     time_now = millis();
     time_elapsed = time_now - start_time;
     if (time_elapsed > 4000)  { LCD_ReadTemperature(0,5);
                                start_time = millis();
                              }
       //to update temperature whilst in LED Test Mode
       // don't want to read temperature sensor too quickly
    } //end while loop
}


void LEDTestDrawMenu( const char ledmenu_item)
{
//   char ledstr[ 6 ];
     dataString="^";  
 
  if( ledmenu_item == 0 ) {lcd.LCD_3310_write_string(36, 2,  dataString, MENU_HIGHLIGHT );}
  else {lcd.LCD_3310_write_string( 36, 2, dataString, MENU_NORMAL );}
       
  if( ledmenu_item == 1 ) {lcd.LCD_3310_write_string(42, 2,  dataString, MENU_HIGHLIGHT );}
  else {lcd.LCD_3310_write_string(42, 2,  dataString, MENU_NORMAL );}
      
  if( ledmenu_item == 2 ) {lcd.LCD_3310_write_string(48, 2,  dataString, MENU_HIGHLIGHT );}
  else {lcd.LCD_3310_write_string(48, 2,  dataString, MENU_NORMAL );}
         
  if( ledmenu_item == 3 ) {lcd.LCD_3310_write_string(36, 4,  dataString, MENU_HIGHLIGHT );}
  else {lcd.LCD_3310_write_string(36, 4,  dataString, MENU_NORMAL );}
      
  if( ledmenu_item == 4 ) {lcd.LCD_3310_write_string( 42, 4, dataString, MENU_HIGHLIGHT );}
  else {lcd.LCD_3310_write_string( 42, 4, dataString, MENU_NORMAL );}

  
  if( ledmenu_item == 5 ) {lcd.LCD_3310_write_string( 48, 4, dataString, MENU_HIGHLIGHT );}
  else {lcd.LCD_3310_write_string( 48, 4, dataString, MENU_NORMAL );}
  
 
  if( ledmenu_item == 6 ) {
    lcd.LCD_3310_write_string( 48, 5, "Cancel", MENU_HIGHLIGHT );}
  else {
    lcd.LCD_3310_write_string( 48, 5, "Cancel", MENU_NORMAL );}
}




 void ledSet_Down_Key(char Lmenu_item)
{
  switch( Lmenu_item ) {
           case 0:
            led1test_out = led1test_out-100;
            if( led1test_out <= 0 ) {
              led1test_out = 255;}
          break;
          case 1:
            led1test_out = led1test_out-10;
            if( led1test_out <= 0 ) {
              led1test_out = 255;}
          break;
          case 2:
            led1test_out = led1test_out-1;
            if( led1test_out <= 0 ) {
              led1test_out = 255;}
          break;
          case 3:
            led2test_out = led2test_out-100;
            if( led2test_out <= 0 ) {
              led2test_out = 255;}
          break;
          case 4:
            led2test_out = led2test_out-10;
            if( led2test_out <= 0 ) {
              led2test_out = 255;}
          break;

          case 5:
            led2test_out = led2test_out-1;
            if( led2test_out <= 0 ) {
              led2test_out = 255;}
          break;
          default:;
        }
 
}

 void ledSet_Up_Key(char Rmenu_item)
{
 

  switch( Rmenu_item ) {
          case 0:
            led1test_out = led1test_out+100;
            if( led1test_out >= 255 ) {
              led1test_out = 0;}
          break;
          case 1:
            led1test_out = led1test_out+10;
            if( led1test_out >= 255 ) {
              led1test_out = 0;}
          break;
          case 2:
            led1test_out = led1test_out+1;
            if( led1test_out >= 255 ) {
              led1test_out = 0;}
          break;
          case 3:
            led2test_out = led2test_out+100;
            if( led2test_out >= 255 ) {
              led2test_out = 0;}
          break;
          case 4:
            led2test_out = led2test_out+10;
            if( led2test_out >= 255 ) {
              led2test_out = 0;}
          break;
          case 5:
            led2test_out = led2test_out+1;
            if( led2test_out >= 255 ) {
              led2test_out = 0;}
          break;
          default:;
        }
 
  
}

//--------------------------------------------------
void LCDSetupExtras()
{
temp_debugon = debugon;
temp_load_from_EEPROM  = load_from_EEPROM;
temp_speedup  = speedup;

lcd.LCD_3310_write_string( 0, 0,  "speed up",MENU_NORMAL );
lcd.LCD_3310_write_string( 0, 1,  "from EEPROM",MENU_NORMAL );
lcd.LCD_3310_write_string( 0, 2,  "debug on",MENU_NORMAL );
//  DisplayLEDTest_LEDValues();


char extrasmenu_item = 0;

 while( 1 ) {
    extrasTestDrawMenu( extrasmenu_item);

  byte i;
  for(i=0; i<NUM_KEYS; i++){
       if(button_flag[i] !=0){
        button_flag[i]=0;  // reset button flag
        switch(i){
        case DOWN_KEY:
          extrasSet_Down_Key(extrasmenu_item);
            break;
            
        case UP_KEY:
          extrasSet_Up_Key(extrasmenu_item); 
            break;
            
        case LEFT_KEY:
        extrasmenu_item--;
          if( extrasmenu_item < 0 ) {extrasmenu_item = EXTRAS_MENU_ITEMS - 1;}
          break;
          
       case RIGHT_KEY:     
        extrasmenu_item++;
          if( extrasmenu_item >= EXTRAS_MENU_ITEMS ) {
            extrasmenu_item = 0;}  
          break;
          
        case CENTER_KEY:
         if( extrasmenu_item == 3 ) { 
           debugon = temp_debugon;
           load_from_EEPROM = temp_load_from_EEPROM;
           if (load_from_EEPROM) {LoadLEDArray();
               load_from_EEPROM = !load_from_EEPROM;
               temp_load_from_EEPROM = !temp_load_from_EEPROM;}
           speedup = temp_speedup;
           SyncClock();
           
            return; }
          if( extrasmenu_item == 4 ) { 
            return; }
            break;
            
            default:;
          }  // end switch
          
          //Might do something here
   
       }
    
     }
    } //end while loop
}


void extrasTestDrawMenu( const char extrasmenu_item)
{
//   char extrasstr[ 6 ];
     if (temp_speedup) {dataString="Y";}
     else   {dataString="N";}
  if( extrasmenu_item == 0 ) {lcd.LCD_3310_write_string(72, 0,  dataString, MENU_HIGHLIGHT );}
  else {lcd.LCD_3310_write_string( 72, 0, dataString, MENU_NORMAL );}

     if (temp_load_from_EEPROM) {dataString="Y";}
     else   {dataString="N";}
  if( extrasmenu_item == 1 ) {lcd.LCD_3310_write_string(72, 1,  dataString, MENU_HIGHLIGHT );}
  else {lcd.LCD_3310_write_string(72, 1,  dataString, MENU_NORMAL );}
 
       if (temp_debugon) {dataString="Y";}
       else   {dataString="N";}
  if( extrasmenu_item == 2 ) {lcd.LCD_3310_write_string(72, 2,  dataString, MENU_HIGHLIGHT );}
  else {lcd.LCD_3310_write_string(72, 2,  dataString, MENU_NORMAL );}

  if( extrasmenu_item == 3 ) {
    lcd.LCD_3310_write_string( 0, 5, "Apply", MENU_HIGHLIGHT );}
  else {
    lcd.LCD_3310_write_string( 0, 5, "Apply", MENU_NORMAL );}

  if( extrasmenu_item == 4 ) {
    lcd.LCD_3310_write_string( 48, 5, "Cancel", MENU_HIGHLIGHT );}
  else {
    lcd.LCD_3310_write_string( 48, 5, "Cancel", MENU_NORMAL );}
}




 void extrasSet_Down_Key(char Lmenu_item)
{
  switch( Lmenu_item ) {
           case 0:
             temp_speedup = !temp_speedup;
          break;
          
          case 1:
            temp_load_from_EEPROM = !temp_load_from_EEPROM;
          break;
          
          case 2:
            temp_debugon = !temp_debugon;
          break;

          default:;
        }
 
}

 void extrasSet_Up_Key(char Rmenu_item)
{
  switch( Rmenu_item ) {
         case 0:
         temp_speedup = !temp_speedup;
          break;
          
          case 1:
          temp_load_from_EEPROM = !temp_load_from_EEPROM;
          break;
          
          case 2:
          temp_debugon = !temp_debugon;
          break;

          default:;
        }
 
  
}

