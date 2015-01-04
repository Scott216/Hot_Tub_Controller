/*

Arduino Pro Mini 3.3 v
I/0
0 -  TX
1 -  RX
2 -  Encoder A
3 -  Encocer B
4 -  LCD CS
5 -  LCD RST
6 -  LCD A0
7 -  LCD SCLK
8 -  LCD SID
9 -  LCD Red (PWM), 82Ω ohm 1/4 watt resitor
10 - LCD Green (PWM), don't need resistor
11 - LCD Blue (PWM), don't need resistor
12 - Encoder pushbutton (10kΩ pullup resistor)
13 - Bubbler LED 

A0 - On/Off LED
A1 - Pump LED
A2 - Jets pushbutton (10kΩ pullup resistor)
A3 - N/C
A4 - I2C SDA
A5 - I2C SCL
A6 - Bubbler pushbutton (10kΩ pullup resistor)
A7 - On/Off pushbutton (10kΩ pullup resistor)

Pushbuttons have built-in resistor for LED
Use 10k pull-up resistors on all buttons inputs

I2C data flow
--------------------
From user panel to main board
 On/Off pushbutton state
 Jets pushbutton state
 Bubbler pushbutton state
 Temperature setpoint

From Main board to user panel
 Hot tub on/off status
 pump on/off status
 bubbler on/off status
 heater on/off status
 water temperature

 Change Log
 09/23/14 v2.00 - Added version
 12/18/14 v2.01 - Used strcpy() to get rid of warning: "deprecated conversion from string to char"
 
*/

#define VERSION "v2.01"

// === Libraries ===
#include <Arduino.h>
#include <ST7565.h>      // LCD Library http://github.com/adafruit/ST7565-LCD  Tutorial: http://bit.ly/geIqhP, Data Sheet: http://bit.ly/19INf0S
#include <Wire.h>
#include "Hot_Tub_LCD_Library.h"  // local library

//=== Output for LEDs in pushbuttons ===
#define HOTTUB_ON_OFF_INDICATOR_OUTPUT_PIN   A0
#define PUMP_INDICATOR_OUTPUT_PIN            A1
#define BUBBLER_INDICATOR_OUTPUT_PIN         13

// Pushbutton inputs are in LocalLibrary.h

//=== Encoder Inputs ===
#define ENCODERA    2
#define ENCODERB    3


// LCD I/O
#define LCD_SID         4
#define LCD_SCLK        5
#define LCD_A0          6
#define LCD_RST         7
#define LCD_CS          8
#define BACKLIGHT_RED   9 // sets LCD backlight color
#define BACKLIGHT_GRN  10
#define BACKLIGHT_BLU  11

ST7565 glcd( LCD_SID, LCD_SCLK, LCD_A0, LCD_RST, LCD_CS);   // create LCD object
const uint8_t CONTRAST = 0x18; 
const int32_t MINUTE = 60000;

// Set Max and Min water temp settings with encoder
const int WATER_TEMP_MAX = 105;
const int WATER_TEMP_MIN  = 70;


// Encoder setup
enum PinAssignments
{
  encoderPinA = ENCODERA,     // right
  encoderPinB = ENCODERB,     // left
  interruptA  = 0,            // Interrupt number (not pin #) for encoderPinA
  interruptB  = 1             // Interrupt number for encoderPinB
};

volatile float encoderTemp;   // counter for the dial  http://arduino.cc/en/Reference/Volatile
float oldEncoderTemp;         // change management
boolean rotating = false;     // debounce management
uint32_t encoderChangeTime;   // timestamp encoder value was last changed

// interrupt service routine variables
boolean A_set = false;
boolean B_set = false;

// Create hottub object
HotTub hottub;
  
// Define Function Prototypes
void ReadInputButtons();
void EncoderSetup();
bool refreshTempSetpoint();
void UpdateDisplay(uint8_t temperature, char statusMsg[] );
void doEncoderA();
void doEncoderB();


void setup()
{
  Serial.begin(9600);
  
  // Define I/O pins.  Some I/O is defined in libraries
  pinMode(HOTTUB_ON_OFF_INDICATOR_OUTPUT_PIN,  OUTPUT);
  pinMode(PUMP_INDICATOR_OUTPUT_PIN,    OUTPUT);
  pinMode(BUBBLER_INDICATOR_OUTPUT_PIN, OUTPUT);
  pinMode(BACKLIGHT_RED,   OUTPUT);
  pinMode(BACKLIGHT_GRN,   OUTPUT);
  pinMode(BACKLIGHT_BLU,   OUTPUT);
  pinMode(ENCODERPB, INPUT_PULLUP);

  // LED Color, 0=on, 255=off
  analogWrite(BACKLIGHT_RED, 0); 
  analogWrite(BACKLIGHT_GRN, 0);
  analogWrite(BACKLIGHT_BLU, 0);

  // Initialize Encoder pins
  EncoderSetup();
  
  // initialize LCD and set the contrast to 0x18
  glcd.begin(CONTRAST);
  glcd.st7565_set_brightness(32);
  glcd.clear();

  hottub.begin(); // configures pushbutton pins and I2C communication
  
  // Initialize encoder default temperature values
  encoderTemp = hottub.getWaterTempDefault();                
  oldEncoderTemp = hottub.getWaterTempDefault();              
  
  Serial.print("Hot tub panel setup ");
  Serial.println(VERSION);
} // setup()


void loop()
{
  byte btnStatus = hottub.processButtons();   // Read button state and update on/off status of Hot tub, pump, bubbler lights inside the pushbuttons
  if ( btnStatus == HOT_TUB_TURNED_OFF )
  {
    // If hot tub is turned off hard reset the LCD display.  Sometime display gets messed up, this is a way to clear it
    glcd.begin(CONTRAST);
    glcd.st7565_set_brightness(32);
  }
  
  // Turn bubbles off after 15 minutes
  if( (long)( millis() - hottub.getBubblerOnTime() ) > 15L * MINUTE )
  { hottub.setBubblerBtnOff(); }
  
  // Turn pump off after 30 minutes
  if( (long)(millis() - hottub.getPumpOnTime() ) > 30L * MINUTE)
  { hottub.setPumpBtnOff(); }
  
  // Update pushbutton LEDs
  digitalWrite( HOTTUB_ON_OFF_INDICATOR_OUTPUT_PIN,  hottub.isHotTubBtnOn() );
  digitalWrite( PUMP_INDICATOR_OUTPUT_PIN,    hottub.isPumpBtnOn() );
  digitalWrite( BUBBLER_INDICATOR_OUTPUT_PIN, hottub.isBubblerBtnOn() );

  // See if there is a new setpoint temperature from encoder.  Returns true if there is a new temperature setpoint
  bool isNewTempSetpoint = refreshTempSetpoint();

  // update LCD display
  char LCDMsg[25];
  if ( isNewTempSetpoint == true )
  {
    strcpy(LCDMsg, "New Setpoint");
    UpdateDisplay(hottub.getWaterTempSetpoint(), LCDMsg );  // Display new temperature setpoint
  }
  else if ( hottub.isHotTubBtnOn() == false )
  {
    strcpy(LCDMsg, "HOT TUB IS OFF");
    UpdateDisplay(hottub.getWaterTemp(), LCDMsg );        // hot tub is off, send actual water temperature to display, but don't display setpoint
  }
  else
  {
    // Hot tub is on, send actual water temperature to display
    bool I2C_is_okay = !hottub.isI2cTimeout(60);
    if ( I2C_is_okay )
    { sprintf(LCDMsg, "Setpoint = %d",  hottub.getWaterTempSetpoint()); }
    else
    { strcpy(LCDMsg, "Lost Communication"); }
    
    // UpdateDisplay() will check to see if anything has changed before it sends new data to the display
    UpdateDisplay(hottub.getWaterTemp(), LCDMsg );
  }
} // loop()


// Update display with temperature and status messages
// Check to see if any info has changed, if not, don't update
void UpdateDisplay(uint8_t temperature, char statusMsg[] )
{
  // prev_... varibles saves setting from last display update
  // if no change, then don't update the display
  static bool    prev_dispInverted;
  static uint8_t prev_temperature;
  static char    prev_statusMsg[25];
  static bool    prev_Bubbler;
  static bool    prev_Heater;
  static bool    prev_Pump;
  
  // get current heater, pump and bubbler status 
  bool new_Bubbler  = hottub.isBubblerOn();
  bool new_Heater =   hottub.isHeaterOn();
  bool new_Pump =     hottub.isPumpOn();
  bool new_Inverted = hottub.isDisplayInverted();
  
  // If any display data has changed, then refresh display
  if( new_Bubbler  != prev_Bubbler ||
      new_Heater   != prev_Heater  ||
      new_Pump     != prev_Pump    ||
      new_Inverted != prev_dispInverted ||
      temperature  != prev_temperature ||
      strcmp(statusMsg, prev_statusMsg) != 0 )
  {
    int digit1 = 0;
    int digit2 = 0;
    int digit3 = 0;
    
    // Variables used to set position of text in display
    int w,h,x,y;
    
    uint8_t tempval = temperature;  // save temperature
    
    if(new_Inverted == true)
    {
      // Make display inverted
      glcd.st7565_command(CMD_SET_ADC_REVERSE);
      glcd.st7565_command(CMD_SET_COM_REVERSE);
    }
    else
    {
      // Make display normal
      glcd.st7565_command(CMD_SET_ADC_NORMAL);
      glcd.st7565_command(CMD_SET_COM_NORMAL);
    }
    
    glcd.clear();
    
    w = 32;
    h = 63;
    x = 20;
    y = 0;
    
    // split the temperature into three separate digits. Get rightmost (digit3) digit first
    digit3 = tempval  % 10;  // gets last digit
    tempval = tempval / 10;  // divide by 10 to get rid of 3rd digit
    digit2 = tempval  % 10;  // gets middle digit (assuming a 3 digit number)
    if (temperature >= 100)
    { digit1 = 1;}
    else
    { digit1 = 0;}
    
    glcd.drawbitmap(x, y,       &bigDigits[digit1][0], w, h, BLACK);
    glcd.drawbitmap(w+x-5, y,   &bigDigits[digit2][0], w, h, BLACK);
    glcd.drawbitmap(w+w+x-8, y, &bigDigits[digit3][0], w, h, BLACK);
    
      // Display status of heater, bubbler and jets
    char dispTxt[7];
    if( new_Bubbler == true )
    {
      strcpy(dispTxt, "Bubble");
      glcd.drawstring(3, 0, dispTxt);
    }
    if( new_Heater == true )
    {
      strcpy(dispTxt, "Heat");
      glcd.drawstring(57, 0, dispTxt);
    }
    if( new_Pump == true )
    {
      strcpy(dispTxt, "Jets");
      glcd.drawstring(100, 0, dispTxt);
    }

    // display status message at bottom of screen
    int leftPos = (128 - strlen(statusMsg) * 6) / 2;
    glcd.drawstring(leftPos, 7, statusMsg);
    
    /* // Set LCD backlight color
    Don't change backlight color based on temperature, this didn't look very good
    int red = -7.2857 * (float) temperature + 765;
    if (temperature >= 105)
    { red = 0; }
    if (temperature <= 70 )
    { red = 255; }
    analogWrite(BACKLIGHT_RED, red);  // 0=on, 255=off
    analogWrite(BACKLIGHT_GRN, 255);
    analogWrite(BACKLIGHT_BLU, 255 - red);  */
    
    glcd.display();
  }  // end something changed
  
  // save display setting and compare on next loop
  prev_dispInverted = new_Inverted;
  prev_temperature =  temperature;
  prev_Bubbler =      new_Bubbler;
  prev_Heater =       new_Heater;
  prev_Pump =         new_Pump;
  strcpy(prev_statusMsg, statusMsg);
  
} // end UpdateDisplay()


//*********************************************************************************
// Setup Encoder pins
//*********************************************************************************
void EncoderSetup()
{
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);
  
  // Turn on pullup resistors
  digitalWrite(encoderPinA, HIGH);
  digitalWrite(encoderPinB, HIGH);
  
  // Encoder pin on interrupt 0 (pin 2)
  attachInterrupt(interruptA, doEncoderA, CHANGE);
  
  // Encoder pin on interrupt 1 (pin 3)
  attachInterrupt(interruptB, doEncoderB, CHANGE);
  
} // EncoderSetup()



//*********************************************************************************
//  Refresh the temperature setupoint from encoder
//  Returns true if there is a new temperature
//*********************************************************************************
bool refreshTempSetpoint()
{
  static uint32_t newSetpointDisplayDelay;  // keeps setpoint temp displayed for a couple seconds
  bool isNewTempSetpoint;

  rotating = true;  // reset the debouncer
  
  // If hot tub is off don't change encoder position
  if( hottub.isHotTubBtnOn() == false )
  { encoderTemp = oldEncoderTemp;  }
  
  // check to see if encoder was moved
  // If so, reset heater on delay interval so heater can come on right away
  if (oldEncoderTemp != encoderTemp)
  {
    oldEncoderTemp = encoderTemp;
    isNewTempSetpoint = true; // Used by LCD display
    newSetpointDisplayDelay = millis();

    // Set Max and Min temperatures
    if (encoderTemp > WATER_TEMP_MAX)
    { encoderTemp = WATER_TEMP_MAX; }
    if (encoderTemp < WATER_TEMP_MIN)
    { encoderTemp = WATER_TEMP_MIN; }

    hottub.setWaterTemp(encoderTemp); // Save new water temp setpoint
  }
  else
  { 
    // after a 2 second delay, reset isNewTempSetpoint flag  
    // this is used so display doesn't revert back to showing actual temperature while user is 
    // rotating encoder 
    if(millis() >= newSetpointDisplayDelay + 2000)
    { isNewTempSetpoint = false; }
    else
    { isNewTempSetpoint = true; }
  }
 
  return isNewTempSetpoint;
  
} // refreshTempSetpoint()


//*********************************************************************************
// Interrupt on A changing state
//*********************************************************************************
void doEncoderA()
{
  // debounce
  if ( rotating ) 
  { delay(1); } // wait a little until the bouncing is done
  
  // Test transition, did things really change?
  if( digitalRead(encoderPinA) != A_set )
  {  // debounce once more
    A_set = !A_set;
    
    // adjust counter + if A leads B
    // and if 200mS since last encoder change has passed
    if ( A_set && !B_set && (long)(millis() - encoderChangeTime) > 200 )
    { 
      encoderTemp = encoderTemp + 0.5;
      encoderChangeTime = millis(); // record time encoder value changed
    }
    rotating = false;  // no more debouncing until loop() hits again
  }
} //doEncoderA()


//*********************************************************************************
// Interrupt on B changing state, same as A above
//*********************************************************************************
void doEncoderB()
{
  if ( rotating ) 
  { delay(1); } // wait a little until the bouncing is done
  
  // adjust counter + if A leads B
  // and if 200mS since last encoder change has passed
  if( digitalRead(encoderPinB) != B_set )
  {
    B_set = !B_set;
    //  adjust counter - 1 if B leads A
    // and if 200mS since last encoder change has passed
    if( B_set && !A_set && (long)(millis() - encoderChangeTime) > 200 )
    {
      encoderTemp = encoderTemp - 0.25;
      encoderChangeTime = millis(); // record time encoder value changed
    }
    rotating = false;
  }
} //doEncoderB()



