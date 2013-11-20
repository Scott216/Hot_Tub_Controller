/*

directory: cd Dropbox/Arduino/Hot_Tub_Controller/ 

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
13 - On/Off LED 


A0 - Jets LED
A1 - Bubble LED
A2 - Bubble pushbutton (10kΩ pullup resistor)
A3 - N/C
A4 - I2C SDA (330Ω inline resistor to reduce volts)
A5 - I2C SCL (330Ω inline resistor to reduce volts)
A6 - On/off pushbutton (10kΩ pullup resistor)
A7 - Jets pushbutton (10kΩ pullup resistor)

Pushbottons have built-in resistor
Use 10kΩ pull-up resistors on all buttons 

Data I2C backpack (slave) will pass back to master
pushbutton status, 3 LED buttons + encoder button.  This can be done in 1 byte by setting the bits
Encoder temperature setting -  1 byte

Data that will be sent to I2C backpack
Pushbutton LED status - this can be one byte, just set bits
On/Off status of Jets, heater, bubbler - can be one byte
Current temperature
Setpoint temperature

I2C data flow
--------------------
From backpack to main board
 On/Off pushbutton state
 Jets pushbutton state
 Bubbler pushbutton state
 Encoder pushbutton state
 temperature setpoint 

From Main board to backback
 Hot tub on/off status
 pump on/off status
 bubbler on/off status
 heater on/off status
 water temperature
 default water temp setpoint

*/


// === Libraries ===
#include "Arduino.h"
// #include <Button.h>       // For pushbuttons http://github.com/carlynorama/Arduino-Library-Button
                                        // I modified button.h library so it would reset the pushbuttons.  Added onPressAsToggleRst() function
#include <ST7565.h>      // LCD Library http://github.com/adafruit/ST7565-LCD  Tutorial: http://bit.ly/geIqhP, Data Sheet: http://bit.ly/19INf0S
#include <Wire.h>


// Include application, user and local libraries
#include "LocalLibrary.h"

//=== Output for LEDs in pushbuttons ===
#define ON_OFF_INDICATOR_OUTPUT_PIN   A0   // Shows if hot tub is on or off
#define PUMP_INDICATOR_OUTPUT_PIN     13 
#define BUBBLER_INDICATOR_OUTPUT_PIN  A1 

//=== Pushbutton Inputs ===
//#define HOT_TUB_BUTTON_INPUT_PIN     A6  // Note: A6 & A7 on pro-mini can't be used as digital input, only analog
//#define JETS_BUTTON_INPUT_PIN        A7  
//#define BUBBLER_BUTTON_INPUT_PIN     A2  

//=== Encoder Inputs ===
#define ENCODERA    2
#define ENCODERB    3
#define ENCODERPB  12    // Encoder pushbutton - flips LCD so it can be read from the other side


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


// Set Max and Min water temp settings with encoder
#define WATER_TEMP_MAX     105
#define WATER_TEMP_MIN      70


// Encoder setup
enum PinAssignments
{
  encoderPinA = ENCODERA,   // right
  encoderPinB = ENCODERB,   // left
  interruptA  = 0,   // Interrupt number (not pin #) for encoderPinA
  interruptB  = 1    // Interrupt number for encoderPinB
};

volatile unsigned int encoderTemp;  // counter for the dial  http://arduino.cc/en/Reference/Volatile
unsigned int oldEncoderTemp;        // change management
boolean rotating = false;           // debounce management

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


// byte i2cCmd; // srg can remove if you get I2C working in class

void setup()
{
  
  Serial.begin(9600);
  
  // Define I/O pins.  Some I/O is defined in libraries
  pinMode(ON_OFF_INDICATOR_OUTPUT_PIN,  OUTPUT);
  pinMode(PUMP_INDICATOR_OUTPUT_PIN,    OUTPUT);
  pinMode(BUBBLER_INDICATOR_OUTPUT_PIN, OUTPUT);
  pinMode(BACKLIGHT_RED,   OUTPUT);
  pinMode(BACKLIGHT_GRN,   OUTPUT);
  pinMode(BACKLIGHT_BLU,   OUTPUT);
  pinMode(ENCODERPB, INPUT_PULLUP);

  // Initialize Encoder pins
  EncoderSetup();
  
  // initialize LCD and set the contrast to 0x18
  glcd.begin(0x18);
  glcd.st7565_set_brightness(32);
  glcd.clear();

  hottub.begin(); // configures pushbutton pins and I2C communication
  
  
  // Initialize encoder default temperature values
  encoderTemp = hottub.getWaterTempDefault();                
  oldEncoderTemp = hottub.getWaterTempDefault();              
  
} // setup()


void loop()
{
  
  hottub.processButtons();   // Read button state and update on/off status of Hot tub, pump, bubbler lights inside the pushbuttons
  
  // Update pushbuttonbutton LEDs
  digitalWrite( ON_OFF_INDICATOR_OUTPUT_PIN,  hottub.isHotTubOn() );
  digitalWrite( PUMP_INDICATOR_OUTPUT_PIN,    hottub.isPumpOn() );
  digitalWrite( BUBBLER_INDICATOR_OUTPUT_PIN, hottub.isBubblerOn() );
  
  // See if there is a new setpoint temperature from encoder.  Returns true if there is a new temperature setpoint
  bool isNewTempSetpoint = refreshTempSetpoint();

  // update LCD display
  // UpdateDisplay() will check to see if anything has changed before it sends new data to the display
  char LCDMsg[25];
  if ( isNewTempSetpoint == true )
  {
    // Display new temperature setpoint
    strcpy(LCDMsg, "New Setpoint");
    UpdateDisplay(hottub.getWaterTempSetpoint(), LCDMsg );
  }
  else if ( hottub.isHotTubOn() == false )
  {
     // hot tub is off, send actual water tempurature to display
    sprintf(LCDMsg, "%d F",  hottub.getWaterTemp()); 
    UpdateDisplay(0, LCDMsg );
  }
  else
  {
    // send actual water temperature to display
    sprintf(LCDMsg, "Setpoint = %d",  hottub.getWaterTempSetpoint()); 
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
    
    if( hottub.isHotTubOn() )
    { 
      // Position for big digits
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
      if( new_Bubbler == true )
      { glcd.drawstring(3, 0, "Bubble"); }
      if( new_Heater == true )
      { glcd.drawstring(50, 0, "Heat"); }
      if( new_Pump == true )
      { glcd.drawstring(100, 0, "Jets"); }
      
      int leftPos = (128 - strlen(statusMsg) * 6) / 2;
      glcd.drawstring(leftPos, 7, statusMsg);
      
      // Set LCD backlight color
      int red = -7.2857 * (float) temperature + 765;
      if (temperature >= 105)
      { red = 0; }
      if (temperature <= 70 )
      { red = 255; }
      analogWrite(BACKLIGHT_RED, red);  // 0=on, 255=off
      analogWrite(BACKLIGHT_GRN, 255);
      analogWrite(BACKLIGHT_BLU, 255 - red);
    }
    else
    { // Hot tub is off
      analogWrite(BACKLIGHT_RED, 10);  // 0=on, 255=off
      analogWrite(BACKLIGHT_GRN, 255);
      analogWrite(BACKLIGHT_BLU, 255);
      
      glcd.drawstring(22, 3, "HOT TUB IS OFF");
      sprintf(statusMsg, "%d F", temperature);
      glcd.drawstring(50, 5, statusMsg);
    }
    
    glcd.display();
  }  // end something changed
  
  // save display setting and compare on next loop
  prev_dispInverted = new_Inverted;
  prev_temperature = temperature;
  prev_Bubbler = new_Bubbler;
  prev_Heater =  new_Heater;
  prev_Pump =    new_Pump;
  strcpy(prev_statusMsg, statusMsg);
  
} // end UpdateDisplay()


//*********************************************************************************
// Setup Encoder pins
//*********************************************************************************
void EncoderSetup()
{
  
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);
  
  // turn on pullup resistors
  digitalWrite(encoderPinA, HIGH);
  digitalWrite(encoderPinB, HIGH);
  
    // encoder pin on interrupt 0 (pin 2)
  attachInterrupt(interruptA, doEncoderA, CHANGE);
  
    // encoder pin on interrupt 1 (pin 3)
  attachInterrupt(interruptB, doEncoderB, CHANGE);
  
  Serial.println(F("EncoderSetup() complete"));
  
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
  if( hottub.isHotTubOn() == false )
  { encoderTemp = oldEncoderTemp;  }
  
  // check to see if encoder was moved
  // If so, reset heater on delay interval so heater can come on right away
  if (oldEncoderTemp != encoderTemp)
  {

/* SRG - move back to main sketch    
    if (millis() < HEATER_ON_DELAY)
    {last_heater_on_check = 0;}  // millis is too small to subtract HEATER_ON_DELAY
    else
    {last_heater_on_check = millis() - HEATER_ON_DELAY + 1000;} // Make heater delay 1 second after encoder is moved
*/    
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
  if ( rotating ) delay(1);  // wait a little until the bouncing is done
  
    // Test transition, did things really change?
  if( digitalRead(encoderPinA) != A_set )
  {  // debounce once more
    A_set = !A_set;
    
      // adjust counter + if A leads B
    if ( A_set && !B_set )
      encoderTemp += 1;
    
    rotating = false;  // no more debouncing until loop() hits again
  }
} //doEncoderA()

//*********************************************************************************
// Interrupt on B changing state, same as A above
//*********************************************************************************
void doEncoderB()
{
  if ( rotating ) delay(1);
  if( digitalRead(encoderPinB) != B_set )
  {
    B_set = !B_set;
      //  adjust counter - 1 if B leads A
    if( B_set && !A_set )
      encoderTemp -= 1;
    
    rotating = false;
  }
} //doEncoderB()


