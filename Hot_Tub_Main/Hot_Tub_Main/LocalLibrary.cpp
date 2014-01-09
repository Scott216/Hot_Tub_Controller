
#include "LocalLibrary.h"


HotTubControl::HotTubControl()
{
}

HotTubControl::~HotTubControl()
{

}


void HotTubControl::begin()
{
  // Setup I2C
  I2c.begin();  // Initialize I2C
  I2c.pullup(0); // turn off internal pullups (doesn't do anything on mega)
  I2c.timeOut(30000); // 30 second timeout


}

void HotTubControl::readPanelStatus()
{
  // Read pushbuttons status and temperature setpoint from user panel
  I2c.write(SLAVE_ID, CMD_ONOFF_BTN);
  delay(1);
  I2c.read(SLAVE_ID, 1);                                           // request on/off button status
  _hotTubBtn = I2c.receive();    // Read the on/off button status from slave

  I2c.write(SLAVE_ID, CMD_PUMP_BTN);
  delay(1);
  I2c.read(SLAVE_ID, 1);
  _pumpBtn = I2c.receive();

  I2c.write(SLAVE_ID, CMD_BUBBLE_BTN);
  delay(1);
  I2c.read(SLAVE_ID, 1);
  _bubbleBtn = I2c.receive();

  I2c.write(SLAVE_ID, CMD_TEMP_SETPT);
  delay(1);
  I2c.read(SLAVE_ID, 1);
  _targetTemp = I2c.receive();

}

// Send status to panel: current water temp, pump on, heat on, bubbles on
void HotTubControl::writePanelStatus(float currentTemp, bool pumpState, bool bubbleState, bool heatState)
{

  int i2cWriteDelay = 15;  // srg temp - may not need delay
  
  // Send data to User Panel via I2C bus
  I2c.write( SLAVE_ID, ADR_ONOFF_STAT,  _hotTubBtn );
  delay(i2cWriteDelay);  
  I2c.write( SLAVE_ID, ADR_PUMP_STAT,   pumpState );
  delay(i2cWriteDelay);
  I2c.write( SLAVE_ID, ADR_BUBBLE_STAT, bubbleState );
  delay(i2cWriteDelay);
  I2c.write( SLAVE_ID, ADR_HEATER_STAT, heatState );
  delay(i2cWriteDelay);
  I2c.write( SLAVE_ID, ADR_TEMP_SETPT,  currentTemp );
  delay(i2cWriteDelay);

}

// Read sensors
void HotTubControl::refreshSensors()
{

  
}


// Determine if hot tub heaters should come on
// If heater is already on, heat it up to setpoint + 0.8 degrees
// If heater is off, turn it on when temp drops to setpoint - 0.8 degree
// Once heat is called for, keep it on for at least 3 minutes
bool HotTubControl::needHeatt()
{
/*
  if(digitalRead(HEATER_ON_OFF_OUTPUT_PIN) == HIGH)
  { // heater is on
    if (tempPreHeat > (float) Temperature_Setpoint + 0.4 )  // temperature has reached setpoint, don't need heat anymore
    {
      // If heater was just turned on, then don't turn off until 3 minutes has passed
      if ( (long)(millis() - heatOntime) > 3UL * 60000UL )
      {
        // 3 mintues has passed, okay to turn off
        heatOntime = 0;
        return false;
      }
      else
      { return true; }  // 3 minutes has not passed, keep heat on
    }
    else // temperature is below setpoint, call for heat
    { return true; }
  }
  else // heater is off
  {
    if (tempPreHeat < (float) Temperature_Setpoint - 0.4 )  // Turn on heater if it's 0.8 degree below setpoint
    {
      heatOntime = millis(); // Heater was off and it needs to be turned on.  Record heater on time
      return true;
    }
    else
    {
      return false;
    }
  }
*/
} // End needHeat()



// Returns state of On/Off pushbutton on control panel
bool HotTubControl::isHotTubBtnOn()
{
  return _hotTubBtn;
}


// Returns state of pump pushbutton on control panel
bool HotTubControl::isPumpBtnOn()
{
  return _pumpBtn;
}

// Returns state of bubbles pushbutton on control panel
bool HotTubControl::isBubbleBtnOn()
{
  return _bubbleBtn;
}


// returns the setpoint temp
byte HotTubControl::getTempSetpoint()
{
  return _targetTemp;
}



