

// === Libraries ===
#include "Arduino.h"
#include <OneWire.h>           // http://www.pjrc.com/teensy/td_libs_OneWire.html  http://playground.arduino.cc/Learning/OneWire
#include <DallasTemperature.h> // http://milesburton.com/index.php?title=Dallas_Temperature_Control_Library
#include <I2C.h>               // use for I2C communication  http://dsscircuits.com/articles/arduino-i2c-master-library.html
#include "LocalLibrary.h"


// Create HotTubController instance
HotTubControl hotTubControl;

// Timer Intervals
#define ALARM_CHECK_INTERVAL       5000  // Check for alarm every 5 seconds
uint32_t last_alarm_check;
#define SENSOR_CHECK_INTERVAL      1000  // Check sensor every second
uint32_t last_sensor_check;
#define HEATER_ON_DELAY           30000  // Delay before heaters can be turned on again after being off
uint32_t last_heater_on_check =       0;
#define PUMP_ON_DELAY              5000  // Delay before pump can be turned on again after being off
#define BUBBLER_ON_DELAY            700  // Delay before bubbler can be turned on again after being off
uint32_t last_bubbler_on_check =      0;
#define HEATER_COOLDOWN_DELAY     10000  // When heater is turned off, keep pump running a few seconds to help heating coils cool down
uint32_t heater_cooldown_timer =      0; // Timer used to keep pump on for a few seconds after heater has turned off
uint32_t heatOntime;                     // Records time heat was called for in NeedHeat().  Used to keep heat on for a min of 3 minutes

// Alarm setpoints
#define ALARM_HEATER_AMPS_HIGH   30  // Max heater amps allowed
#define PUMP_AMPS_THRESHOLD       2  // Min amps needed verify pump is on
#define PUMP_PRESSURE_THRESHOLD   5  // Min PSI needed to verify pump is running


// Initialize OneWire temp sensors
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature oneWireBus(&oneWire);


// Define Function Prototypes
void CheckAlarms();
void OutputAlarm(char AlarmText[]);
void PrintStatus();
boolean NeedHeat();



//============================================================================
void setup()
{
  Serial.begin(9600);

  // Define output pins
  pinMode(PUMP_ON_OFF_OUTPUT_PIN,    OUTPUT);
  pinMode(HEATER_ON_OFF_OUTPUT_PIN,  OUTPUT);
  pinMode(BUBBLER_ON_OFF_OUTPUT_PIN, OUTPUT);

  // initialize hotTubControl 
  hotTubControl.begin();
  
  // Initialize timers
  heater_cooldown_timer = 0;
  last_heater_on_check =  0;
  
  Serial.println(F("Finished Hot Tub setup()"));
  
} // setup()



//============================================================================
void loop()
{
  static uint32_t lastPumpOnTime; // Millis() Timestamp of when pump was last turned on.
                                  // Updates every cycle that pump should be on.  Used to keep pump from cycling on/off too fast

  // Read pushbuttons status and temperature setpoint from user panel
  hotTubControl.readPanelStatus();

  
  // Check sensor inputs
  if ((long)(millis() - last_sensor_check) > SENSOR_CHECK_INTERVAL)
  {
    last_sensor_check = millis();
    hotTubControl.refreshSensors();
  }
  
  // Check alarms
  if ((long)(millis() - last_alarm_check) > ALARM_CHECK_INTERVAL)
  {
    last_alarm_check = millis();
    CheckAlarms();
  }

  PrintStatus();  // Print for debugging
  

  // Turn off pump
  // If Hot tub is turned off and cooldown delay has passed, or
  // Jets button is off and we don't need heat, and cooldown delay has passed
  if((hotTubControl.isHotTubBtnOn() == LOW                     && (long)(millis() - heater_cooldown_timer) > 0) ||
     (hotTubControl.isPumpBtnOn() == LOW && NeedHeat() == false && (long)(millis() - heater_cooldown_timer) > 0) )
  {
    // srg debug
    // Print values so you can see why pump is turned off
/*    if(digitalRead(PUMP_ON_OFF_OUTPUT_PIN) == HIGH)
    {
     Serial.println("Turning Pump Off");
     Serial.println("OnOff BTN\tJetsBtn\tNeedHeat()\ttemp\theater\ttimer\tCooldown");
     Serial.print("   ");
     Serial.print(hotTubControl.isHotTubBtnOn());
     Serial.print("\t\t");
     Serial.print(hotTubControl.isPumpBtnOn());
     Serial.print("\t");
     Serial.print(NeedHeat());
     Serial.print("\t\t");
     Serial.print(hotTubControl.getTempPreHeat());
     Serial.print("\t");
     Serial.print(digitalRead(HEATER_ON_OFF_OUTPUT_PIN));
     Serial.print("\t");
     Serial.print((long)(millis() - lastPumpOnTime));
     Serial.print("\t\t");
     Serial.print(millis() - heater_cooldown_timer);
     Serial.println();
    }
*/
    digitalWrite(PUMP_ON_OFF_OUTPUT_PIN, LOW);
  }  // end turn off pump
  
  // Turn on pump
  // Pump is turned on either by manually by pushbutton or by the heater
  if((hotTubControl.isHotTubBtnOn() == HIGH) &&                          // Hot tub must be on
     ((hotTubControl.isPumpBtnOn() == HIGH) || (NeedHeat() == true)) &&   // Pushbutton OR Need heat
     ((long)(millis() - lastPumpOnTime) > PUMP_ON_DELAY))                           // Delay before pump can be turned on again after being off, prevents fast cycling if there is a problem
  {
    // srg debug
    // Print values so you can see why pump is turned off
/*    if(digitalRead(PUMP_ON_OFF_OUTPUT_PIN) == LOW)
    {
     Serial.println("Turning Pump On");
     Serial.println("OnOff BTN\tJetsBtn\tNeedHeat()\ttemp\theater\tLastPumpCheck");
     Serial.print("   ");
     Serial.print(hotTubControl.isHotTubBtnOn());
     Serial.print("\t\t");
     Serial.print(hotTubControl.isPumpBtnOn());
     Serial.print("\t");
     Serial.print(NeedHeat());
     Serial.print("\t\t");
     Serial.print(hotTubControl.getTempPreHeat());     
     Serial.print("\t");
     Serial.print(digitalRead(HEATER_ON_OFF_OUTPUT_PIN));
     Serial.print("\t\t");
     Serial.print(millis() - lastPumpOnTime);
     Serial.println();
    }
*/
    digitalWrite(PUMP_ON_OFF_OUTPUT_PIN, HIGH);
    lastPumpOnTime = millis();
  }  // end turn on pump
    
  
  // Turn on heater
  // Prerequisites: Hot Tub On + Pump amps and pressure are above threshold + Low water temp + Delay from last time on
  if((hotTubControl.isHotTubBtnOn() == HIGH) &&
     (digitalRead(PUMP_ON_OFF_OUTPUT_PIN) == HIGH) &&
     (hotTubControl.getAmpsPump() >= PUMP_AMPS_THRESHOLD) &&
     (hotTubControl.getPressure() >= PUMP_PRESSURE_THRESHOLD) &&
     (NeedHeat() == true) &&
     ((long)(millis() - last_heater_on_check) > HEATER_ON_DELAY))
  {
    digitalWrite(HEATER_ON_OFF_OUTPUT_PIN, HIGH);
    Serial.print("Millis 2: "); Serial.println(millis()); // srg debug
    delay(300); // srg testing pump cycle
  }
  
  // If heater is on, update last heater on check, and heater_cooldown_timer
  if(digitalRead(HEATER_ON_OFF_OUTPUT_PIN) == HIGH)
  {
    last_heater_on_check  = millis();
    heater_cooldown_timer = millis() + HEATER_COOLDOWN_DELAY;
  }
  
  
  // Turn off heater
  // Turn off when water temp reaches setpoint or if heater amps is too high or Hot Tub is turned off
  // Or Pump pressure is too low, or pump amps is too low.
  if((NeedHeat() == false) ||
     (hotTubControl.getAmpsHeater() >= ALARM_HEATER_AMPS_HIGH) ||
     (hotTubControl.getAmpsPump() < PUMP_AMPS_THRESHOLD) ||
     (hotTubControl.getPressure() < PUMP_PRESSURE_THRESHOLD) ||
     (hotTubControl.isHotTubBtnOn() == LOW))
  {
    digitalWrite(HEATER_ON_OFF_OUTPUT_PIN, LOW);
  }
  
  
  // Turn on bubbler
  if((hotTubControl.isHotTubBtnOn() == HIGH) &&
     (hotTubControl.isBubbleBtnOn() == HIGH) &&
     ((long)(millis() - last_bubbler_on_check) > BUBBLER_ON_DELAY))
  {
    digitalWrite(BUBBLER_ON_OFF_OUTPUT_PIN, HIGH);
    last_bubbler_on_check = millis();
  }

  // Turn off bubbler
  if(hotTubControl.isBubbleBtnOn() == LOW || hotTubControl.isHotTubBtnOn() == LOW)
  {
    digitalWrite(BUBBLER_ON_OFF_OUTPUT_PIN, LOW);
  }

  // Update user panel with hot tub info: Water temp, Motor on/off state
  hotTubControl.writePanelStatus( hotTubControl.getTempPreHeat(),
                                  digitalRead(PUMP_ON_OFF_OUTPUT_PIN),
                                  digitalRead(BUBBLER_ON_OFF_OUTPUT_PIN),
                                  digitalRead(HEATER_ON_OFF_OUTPUT_PIN) );

}  // loop()



// Determine if hot tub heaters should come on
// If heater is already on, heat it up to setpoint + 0.8 degrees
// If heater is off, turn it on when temp drops to setpoint - 0.8 degree
// Once heat is called for, keep it on for at least 3 minutes
boolean NeedHeat()
{ 
  if(digitalRead(HEATER_ON_OFF_OUTPUT_PIN) == HIGH)
  { // heater is on
    if (hotTubControl.getTempPreHeat() > (float) hotTubControl.getTempSetpoint() + 0.4 )  // temperature has reached setpoint, don't need heat anymore
    {
      // If heater was just turned on, then don't turn off until 3 minutes has passed
      if ( (long)(millis() - heatOntime) > 3UL * 60000UL )
       { 
         // 3 mintues has passed, okay to turn off
         heatOntime = 0;
Serial.print("Heater Off.  Millis - heatOntime = "); Serial.println( (long)(millis() - heatOntime)); delay(20);  // srg debug
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
    if (hotTubControl.getTempPreHeat() < (float) hotTubControl.getTempSetpoint() - 0.4 )  // Turn on heater if it's 0.8 degree below setpoint
    {
      heatOntime = millis(); // Heater was off and it needs to be turned on.  Record heater on time
      return true;
    }
    else
    {
      if ((long)(millis() - heatOntime) < 1000)
      { Serial.print("\nTurned off heater ref 1.  Millis = "); Serial.println(millis());} // srg debug
      return false;
    } 
  }

} // End NeedHeat()




//*********************************************************************************
/*
 Check for alarms
 High Water Temp
 Low Water Temp
 High Motor Temp
 Heater on, but no tempature change
 High Pressure
 Low pressure
 High Heater Amps
 Low Heater Amps
 Differnet heater amps from Ph 1 to Ph 2
*/
//*********************************************************************************
void CheckAlarms()
{
  char txtAlarm[75];
  
  // High water temp, pre-heater
  if ( hotTubControl.getTempPreHeat() > 120 )
  {
    sprintf_P(txtAlarm, PSTR("High pre-heater temperature = %d degrees"), (int) hotTubControl.getTempPreHeat() );
    OutputAlarm(txtAlarm);
  }
  
  // high water temp, post heater
  if (  hotTubControl.getTempPostHeat() > 150 )
  {
    sprintf_P(txtAlarm, PSTR("High post-heater temperature = %d degrees"), (int) hotTubControl.getTempPostHeat() );
    OutputAlarm(txtAlarm);
  }
  
  // Pump housing high temperature
  if ( hotTubControl.getTempPump() > 150 )
  {
    sprintf_P(txtAlarm, PSTR("High pump housing temperature = %d degrees"), (int) hotTubControl.getTempPump() );
    OutputAlarm(txtAlarm);
  }
  
  // Low temp alarm
  if ( hotTubControl.getTempPreHeat() < 50 )
  {
    sprintf_P(txtAlarm, PSTR("Low pre-heater temperature = %d degrees"), (int) hotTubControl.getTempPreHeat() );
    OutputAlarm(txtAlarm);
  }
  
  // High pressure alarm
  if ( hotTubControl.getPressure() > 20.0 )
  {
    sprintf_P(txtAlarm, PSTR("High pressure alarm, pressure = %d PSI"), (int) hotTubControl.getPressure() );
    OutputAlarm(txtAlarm);
  }
  
  // High Pump Amps alarm
  if ( hotTubControl.getAmpsPump() > 18.0 )
  {
    sprintf_P(txtAlarm, PSTR("High pump amps alarm = %d amps"), (int) hotTubControl.getAmpsPump() );
    OutputAlarm(txtAlarm);
  }
  
  // Pump on but pressure is low.  Will need delay after pump is first turned on before you check for alarm
  if ( (hotTubControl.getPressure() < 5.0) && (digitalRead(PUMP_ON_OFF_OUTPUT_PIN) == HIGH) )
  {
    sprintf_P(txtAlarm, PSTR("Low pressure alarm while pump is on, pressure = %d PSI"), (int) hotTubControl.getPressure() );
    OutputAlarm(txtAlarm);
  }
  
  // High heater amps
  if ( hotTubControl.getAmpsHeater() > ALARM_HEATER_AMPS_HIGH )
  {
    sprintf_P(txtAlarm, PSTR("High heater amps = %d"), (int) hotTubControl.getAmpsHeater() );
    OutputAlarm(txtAlarm);
  }
  
  // Low heater amps
  if ( (hotTubControl.getAmpsHeater() < 15) && (digitalRead(HEATER_ON_OFF_OUTPUT_PIN) == HIGH) )
  {
    sprintf_P(txtAlarm, PSTR("Low heater amps while heater is on = %d amps"), (int) hotTubControl.getAmpsHeater() );
    OutputAlarm(txtAlarm);
  }
  
} // end CheckAlarms()



//*********************************************************************************
// Print Alarm Text  
//*********************************************************************************
void OutputAlarm(char AlarmText[])
{
  Serial.print(F("Alarm: "));
  Serial.println(AlarmText);
} // OutputAlarm()




//*********************************************************************************
//*********************************************************************************
void PrintStatus()
{
  
  static int8_t   cntHeading;  // prints new heading every 20 rows
  static uint32_t PrintDelay;
  
  if ((long)(millis() - PrintDelay) > 0)
  {
    PrintDelay = millis() + 500UL;

    
    //Print Pushbutton state and Led indicator state
    if(cntHeading > 40 || millis() < 1000)
    {
      Serial.println(F("OnOff\tPump\tHeat\tA-temp\tsetpt\tbubble\tT1\tT2\tT3\tpres\tampH\tampP\tampB\tHtOnTm"));
      cntHeading = 0;
    }
    cntHeading++;
    
    Serial.print(hotTubControl.isHotTubBtnOn());
    Serial.print("\t");
    
    Serial.print(hotTubControl.isPumpBtnOn());
    Serial.print(digitalRead(PUMP_ON_OFF_OUTPUT_PIN));
    Serial.print(F("\t"));
    
    Serial.print(digitalRead(HEATER_ON_OFF_OUTPUT_PIN));
    Serial.print(NeedHeat());
    Serial.print(F("\t"));

    Serial.print(hotTubControl.getTempPreHeat());
    Serial.print(F("\t"));

    Serial.print(hotTubControl.getTempSetpoint());
    Serial.print(F("\t"));

    Serial.print(hotTubControl.isBubbleBtnOn());
    Serial.print(digitalRead(BUBBLER_ON_OFF_OUTPUT_PIN));
    Serial.print(F("\t"));

    Serial.print(hotTubControl.getTempPreHeat());
    Serial.print(F("\t"));
    Serial.print(hotTubControl.getTempPostHeat());
    Serial.print(F("\t"));
    Serial.print(hotTubControl.getTempPump());
    Serial.print(F("\t"));
    Serial.print(hotTubControl.getPressure());
    Serial.print(F("\t"));
    Serial.print(hotTubControl.getAmpsHeater());
    Serial.print(F("\t"));
    Serial.print(hotTubControl.getAmpsPump());
    Serial.print(F("\t"));
    Serial.print(hotTubControl.getAmpsBubbler());
    Serial.print(F("\t"));
    Serial.print((millis() - heatOntime)/1000UL);
    
    Serial.println("");



/*    
    if (heater_cooldown_timer > millis())
//    { Serial.print((heater_cooldown_timer - millis())/1000);}  // print seconds heaters need to cooldown before pump can turn off
    { Serial.print(heater_cooldown_timer);}  // print seconds heaters need to cooldown before pump can turn off
    else
    { Serial.print(heater_cooldown_timer);}
    Serial.print("\t");
    
    
    Serial.print((int)((millis() - last_heater_on_check)/1000));
    Serial.println();
*/

/*
    
     if(cntHeading > 40)
     {
     Serial.println("T1\tT2\tT3\tpres\tampH\tampP\tampB");
     cntHeading = 0;
     }
     cntHeading++;
     
     Serial.print(hotTubControl.getTempPreHeat());
     Serial.print("\t");
     Serial.print(hotTubControl.getTempPostHeat());
     Serial.print("\t");
     Serial.print(hotTubControl.getTempPump());
     Serial.print("\t");
     Serial.print(hotTubControl.getPressure());
     Serial.print("\t");
     Serial.print(hotTubControl.getAmpsHeater());
     Serial.print("\t");
     Serial.print(hotTubControl.getAmpsPump());
     Serial.print("\t");
     Serial.print(hotTubControl.getAmpsBubbler());
     Serial.println();
*/


    /*
     if(cntHeading > 40)
     {
     Serial.println("T1\tT2\tT3\tT4\tI1\tI2\tI3\tI4");
     cntHeading = 0;
     }
     cntHeading++;
     
     Serial.print(TempPreHeat1.readCelsius());
     Serial.print("\t");
     Serial.print(TempPreHeat2.readCelsius());
     Serial.print("\t");
     Serial.print(TempPostHeat.readCelsius());
     Serial.print("\t");
     Serial.print(TempPumpHousing.readCelsius());
     Serial.print("\t");
     Serial.print(TempPreHeat1.readInternal());
     Serial.print("\t");
     Serial.print(TempPreHeat2.readInternal());
     Serial.print("\t");
     Serial.print(TempPostHeat.readInternal());
     Serial.print("\t");
     Serial.print(TempPumpHousing.readInternal());
     Serial.print("\t");
     Serial.println();
     */
    
  }
} // end PrintStatus()









