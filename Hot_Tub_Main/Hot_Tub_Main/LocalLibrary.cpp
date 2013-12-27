
#include "LocalLibrary.h"


HotTubControl::HotTubControl()
{
}

HotTubControl::~HotTubControl()
{

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


void refreshSensors();
bool isHotTubBtnOn();
bool isPumpBtnOn();
bool isBubbleBtnOn();
void setTempSetpoint(byte targetTemp);  // saves the setpoint temp, which is set from control panel
byte getTempSetpoint();                 // returns the setpoint temp


