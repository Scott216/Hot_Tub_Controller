// Hot_Tub_LCDLibrary.cpp
// Created by 	Scott Goldthwaite
// License		GNU General Public License

#include <Wire.h>
#include "Hot_Tub_LCD_Library.h"

// Static variables are like non-inline member functions in that they are declared in a class
// declaration and defined in the corresponding source file. http://bit.ly/1djyt1S & http://ibm.co/1bxfbVy
bool HotTub::_isHotTubOn;
bool HotTub::_isHotTubBtnOn;
bool HotTub::_isPumpOn;
bool HotTub::_isPumpBtnOn;
bool HotTub::_isBubblerOn;
bool HotTub::_isBubblerBtnOn;
bool HotTub::_isHeaterOn;
int  HotTub::_tempSetpoint;
int  HotTub::_tempActual;
byte HotTub::_i2cCmd;
uint32_t HotTub::_buttonChangeTime;

uint32_t _lastI2cTime; // millis() timestamp of most recent I2C communication

HotTub::HotTub()
{}

HotTub::~HotTub()
{}

void HotTub::begin()
{
  // Note: HOT_TUB_BUTTON_INPUT_PIN and BUBBLER_BUTTON_INPUT_PIN are on pins A6 & A7 which can't be configured as digital
  pinMode(JETS_BUTTON_INPUT_PIN, INPUT);
  pinMode(ENCODERPB,             INPUT);
  
  // Setup I2C handlers
  Wire.begin (SLAVE_ID);
  Wire.onReceive (i2cReceiveCmd);  // interrupt handler for incoming commands
  Wire.onRequest (i2cSendData);    // interrupt handler to send data to the master when the master requests it
  
  
  // Set water temperature to default
  setWaterTemp( getWaterTempDefault() );
}


// See if any pushbuttons are engaged
// Returns true if any pushbutton press was detected
byte HotTub::processButtons()
{
  
  if ( (long) (millis() - _debounceTimout) < 0 )
  { return false; } // Haven't waited past debounce delay. Just exit
  
  // Check On/Off button.  If hot tub is on, turn it off and visa-versa
  if( analogRead(HOT_TUB_BUTTON_INPUT_PIN) < 100 )
  {
    _debounceTimout = millis() + debounceDelay;
    _buttonChangeTime = millis();  // time button was pressed
    if ( isHotTubBtnOn() )
    { 
      setHotTubBtnOff(); 
      return HOT_TUB_TURNED_OFF;  
    }
    else
    { 
      setHotTubBtnOn(); 
      return 2; 
    }
  }
  
  // Check Pump button
  if( digitalRead(JETS_BUTTON_INPUT_PIN) == BTN_ON )
  {
    _debounceTimout = millis() + debounceDelay;
    _buttonChangeTime = millis();  // time button was pressed
    if ( isPumpBtnOn() )
    { 
       setPumpBtnOff();
       return 3;
    }
    else
    {
      setPumpBtnOn();
      return 4;
    }
  }
  
  // Check Bubbler button
  if( analogRead(BUBBLER_BUTTON_INPUT_PIN) < 100 )
  {
    _debounceTimout = millis() + debounceDelay;
    _buttonChangeTime = millis();  // time button was pressed
    if (isBubblerBtnOn())
    { 
      setBubblerBtnOff();
      return 5;
    }
    else
    { 
      setBubblerBtnOn();
      return 6; 
    }
  }
  
  // Check encoder pushbutton - invert display
  if( digitalRead(ENCODERPB) == BTN_ON )
  {
    _debounceTimout = millis() + debounceDelay;
    _isDisplayInverted = !_isDisplayInverted;  // toggle display
    return 7;
  }
  
  return false; // no buttons pressed
  
}  // processButtons()



bool HotTub::isHotTubOn()
{
  return _isHotTubOn;
}

bool HotTub::isHotTubBtnOn()
{
  return _isHotTubBtnOn;
}

void HotTub::setHotTubBtnOn()
{
  _isHotTubBtnOn = true;
  _isHotTubOn = true; 
}

void HotTub::setHotTubBtnOff()
{
  // Turn everything off
  _isHotTubBtnOn =  false;
  _isHotTubOn =     false;
  _isPumpBtnOn =    false;
  _isBubblerBtnOn = false;
}

// Returns status of pump on main control panel
bool HotTub::isPumpOn ()
{
  return _isPumpOn;
}

bool HotTub::isPumpBtnOn()
{
  return _isPumpBtnOn;
}

void HotTub::setPumpBtnOn()
{
  // Only turn pump on if hot tub is on.
  if ( isHotTubBtnOn() )
  {
    _isPumpBtnOn = true;
    _pumpStartTime = millis();
  }
  else
  { _isPumpBtnOn = false; }
}

void HotTub::setPumpBtnOff()
{
  _isPumpBtnOn = false;
}

// returns millis time when pump was turned on
uint32_t HotTub::getPumpOnTime()
{
  return _pumpStartTime;
}

// Returns status of air bubbler on main control panel
bool HotTub::isBubblerOn()
{
  return _isBubblerOn;
}

bool HotTub::isBubblerBtnOn()
{
  return _isBubblerBtnOn;
}

void HotTub::setBubblerBtnOn()
{
  // Only turn bubbler on if hot tub is on
  if ( isHotTubBtnOn() )
  {
    _isBubblerBtnOn = true;
    _bubblerStartTime = millis();
  }
  else
  { _isBubblerBtnOn = false; }
}

void HotTub::setBubblerBtnOff()
{
  _isBubblerBtnOn = false;
}

// returns millis time when bubbler was turned on
uint32_t HotTub::getBubblerOnTime()
{
  return _bubblerStartTime;
}


bool HotTub::isHeaterOn()
{
  return _isHeaterOn;
}

// Real-time water temperature
int HotTub::getWaterTemp()
{
  return _tempActual;
}


int HotTub::getWaterTempDefault()
{
  return 100; // Initial water temp setting on statup
}


void HotTub::setWaterTemp(int tempSetpoint)
{
  _tempSetpoint = tempSetpoint;
}

int HotTub::getWaterTempSetpoint()
{
  return _tempSetpoint;
}


bool HotTub::isDisplayInverted()
{
  return _isDisplayInverted;
}

// Compares the last time there was I2C communication to the threshold and returns true if it timed out
bool isI2cTimeout(int threshold)
{
  if ( (millis() - _lastI2cTime) > (threshold * 1000) )
  { return true; }  // I2C timed out
  else
  { return false; } // I2C ok
}

// I2C Master sends one byte stating which data it wants to get back
// If it sends MAX_I2C_BYTES it means the master is sending data for this function to save
// This function is defined as a static function in header file
void HotTub::i2cReceiveCmd(int bytesReceived)
{
  
  _lastI2cTime = millis(); // Track last time I2C communication was triggered
  
  byte i2cBuf[MAX_I2C_BYTES];  // temporary array to hold incomming data
  for (int a = 0; a < bytesReceived; a++)
  {
    if ( a < MAX_I2C_BYTES )
    { i2cBuf[a] = Wire.read(); } 
    else
    { Wire.read(); } // discard any extra data
  }
  
  // Copy data from buffer in vars
  _i2cCmd = i2cBuf[0];
  
  if ( bytesReceived > 1 ) // if more then 1 byte was sent it means there is data to save locally
  {
    switch ( _i2cCmd )
    {
      case ADR_ONOFF_STAT:
        if( millis() > _buttonChangeTime + 1000)  // Only accept command from main board if button was pressed more then a second ago
        { _isHotTubOn =  i2cBuf[1]; }
        break;
      case ADR_PUMP_STAT:
        if( millis() > _buttonChangeTime + 1000)  // Only accept command from main board if button was pressed more then a second ago
        { _isPumpOn =  i2cBuf[1]; }
        break;
      case ADR_BUBBLE_STAT:
        if( millis() > _buttonChangeTime + 1000)  // Only accept command from main board if button was pressed more then a second ago
        { _isBubblerOn =  i2cBuf[1]; }
        break;
      case ADR_HEATER_STAT:
        _isHeaterOn =  i2cBuf[1];
        break;
      case ADR_TEMP_SETPT:
        _tempActual =  i2cBuf[1];
        break;
      default:
        break;
    } // switch
  } // end if
  
}  // i2cReceiveCmd()


// Master requests data, this function sends it back
// Defined as a static function in header file
// Note: On/Off variables are more of a on/off request to the main controller
void HotTub::i2cSendData()
{
  
  _lastI2cTime = millis(); // Track last time I2C communication was triggered
  
  switch(_i2cCmd)
  {
    case CMD_SLAVE_ID:
      Wire.write(SLAVE_ID);
      break;
    case CMD_ONOFF_BTN:
      Wire.write(_isHotTubBtnOn);
      break;
    case CMD_PUMP_BTN:
      Wire.write(_isPumpBtnOn);
      break;
    case CMD_BUBBLE_BTN:
      Wire.write(_isBubblerBtnOn);
      break;
    case CMD_TEMP_SETPT:
      Wire.write(_tempSetpoint);
      break;
    default:
      Wire.write(250);  // invalid i2C command
      break;
  } // switch
  
  //  _i2cCmd = 0; // reset I2C command
  
} // i2cSendData()




