
#include "LocalLibrary.h"
#include <Wire.h>


// Static variables are like non-inline member functions in that they are declared in a class
// declaration and defined in the corresponding source file. http://bit.ly/1djyt1S & http://ibm.co/1bxfbVy
bool HotTub::_isBubblerOn;
bool HotTub::_isHotTubOn;
bool HotTub::_isPumpOn;
bool HotTub::_isHeaterOn;
int  HotTub::_tempSetpoint;
int  HotTub::_tempActual;
byte HotTub::_i2cCmd;


HotTub::HotTub()
{
  // Constructors can accept arguments
}

HotTub::~HotTub()
{
  // Destructors do not accept arguments
}

void HotTub::begin()
{
  pinMode(HOT_TUB_BUTTON_INPUT_PIN, INPUT);   // PCB has pullup resistors, don't need internal ones
  pinMode(JETS_BUTTON_INPUT_PIN,    INPUT);
  pinMode(BUBBLER_BUTTON_INPUT_PIN, INPUT);
  pinMode(ENCODERPB,                INPUT);

  // Setup I2C handlers
  Wire.begin (SLAVE_ID);
  Wire.onReceive (i2cReceiveCmd);  // interrupt handler for incoming commands
  Wire.onRequest (i2cSendData);    // interrupt handler to send data to the master when the master requests it

  // Set water temperature to default
  setWaterTemp( getWaterTempDefault() );
}


// See if any pushbuttons are engaged
// Returns true if any pushbutton press was detected
bool HotTub::processButtons()
{
  if ((long) (millis() - _debounceTimout) < 0 )
  { return false; } // Haven't waited past debounce delay. Just exit

  // Check On/Off button.  If hot tub is on, turn it off and visa-versa
  if( digitalRead(HOT_TUB_BUTTON_INPUT_PIN) == BTN_ON )
  {
    _debounceTimout = millis() + debounceDelay;
    if (isHotTubOn())
    {
      setHotTubOff();
      _requestHotTubOn = false;
    }
    else
    {
      setHotTubOn();
      _requestHotTubOn = true;
    }
    return true;
  }

  // Check Pump button
  if( digitalRead(JETS_BUTTON_INPUT_PIN) == BTN_ON )
  {
    _debounceTimout = millis() + debounceDelay;
    if (isPumpOn())
    {
      setPumpOff();
      _requestPumpOn = false;
    }
    else
    {
      setPumpOn();
      _requestPumpOn = true;
    }
    return true;
  }


  // Check Bubbler button
  if( digitalRead(BUBBLER_BUTTON_INPUT_PIN) == BTN_ON )
  {
    _debounceTimout = millis() + debounceDelay;
    if (isBubblerOn())
    {
      setBubblerOff();
      _requestBubblerOn = false;
    }
    else
    {
      setBubblerOn();
      _requestBubblerOn = true;
    }
    return true;
  }

  // Check encoder pushbutton - invert display
  if( digitalRead(ENCODERPB) == BTN_ON )
  {
    _debounceTimout = millis() + debounceDelay;
    _isDisplayInverted = !_isDisplayInverted;  // toggle display
    return true;
  }

  return false; // no buttons pressed

}  // processButtons()



bool HotTub::isHotTubOn ()
{
  return _isHotTubOn;
}

void HotTub::setHotTubOn()
{
  _isHotTubOn = true;
}

void HotTub::setHotTubOff()
{
  // Turn everything off
  _isHotTubOn =  false;
  _isPumpOn =    false;
  _isBubblerOn = false;
}


bool HotTub::isPumpOn ()
{
  return _isPumpOn;
}

void HotTub::setPumpOn()
{
  // Only turn pump on if hot tub is on.
  if ( _isHotTubOn )
  { _isPumpOn = true; }
  else
  { _isPumpOn = false; }
}

void HotTub::setPumpOff()
{
  _isPumpOn = false;
}


bool HotTub::isBubblerOn()
{
  return _isBubblerOn;
}

void HotTub::setBubblerOn()
{
  // Only turn bubbler on if hot tub is on
  if ( _isHotTubOn )
  { _isBubblerOn = true; }
  else
  { _isBubblerOn = false; }
}

void HotTub::setBubblerOff()
{
  _isBubblerOn = false;
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

// Master sends one byte stating which data it wants to get back
// Defined as a static function in header file
// the _i2cCmd value tells us what data to send back to the master in 
void HotTub::i2cReceiveCmd(int bytesReceived)
{
  // If received just one byte, this it's a command for data to send back
  if (bytesReceived == 1)
  {
    _i2cCmd = Wire.read();  // read and save the command from Master in i2cSendData()
  }

  // If we received MAX_I2C_BYTES, then Master is sending data for slave to store
  if (bytesReceived == MAX_I2C_BYTES)
  {
    // Verify 1st byte = 0
    _i2cCmd = Wire.read(); 
    if( _i2cCmd == 0 )
    {
      // First byte = 0 which means master is sending data to slave, read the remaining bytes
      _isHotTubOn =  Wire.read(); // 2nd byte
      _isBubblerOn = Wire.read(); // 3rd byte
      _isPumpOn =    Wire.read(); // 4th byte
      _isHeaterOn =  Wire.read(); // 5th byte
      _tempActual =  Wire.read(); // 6th byte

    }
  }
}



// Master requests data, this function sends it back
// Defined as a static function in header file
// Note: On/Off variables are more of a on/off request to the main controller
void HotTub::i2cSendData()
{
  switch(_i2cCmd)
  {
    case CMD_SLAVE_ID:
      Wire.write(SLAVE_ID); // return slave address
      break;
    case CMD_ONOFF_BTN:
      Wire.write(_isHotTubOn);  // return isHotTubOn
      break;
    case CMD_PUMP_BTN:
      Wire.write(_isPumpOn);  // return isPumpOn
      break;
    case CMD_BUBBLE_BTN:
      Wire.write(_isBubblerOn);  // return isBubblerOn
      break;
    case CMD_TEMP_SETPT:
      Wire.write((byte)_tempSetpoint);  // return temperature setpoint
      break;
    default:
      break;
  }
  _i2cCmd = 0; // reset I2C command

}
