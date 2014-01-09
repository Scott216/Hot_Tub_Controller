/*
Do you really need heater_cooldown_timer
When hot tub is about to come on because it needs to heat up, pump sometimes comes on then goes right away.
Get encoder code from tardis alarm, this one doesn't work well



Directory:
	

*/


// === Libraries ===
#include "Arduino.h"
#include <OneWire.h>    // http://www.pjrc.com/teensy/td_libs_OneWire.html  http://playground.arduino.cc/Learning/OneWire
#include <DallasTemperature.h> // http://milesburton.com/index.php?title=Dallas_Temperature_Control_Library
#include "I2C.h"  // use for I2C communication  http://dsscircuits.com/articles/arduino-i2c-master-library.html
#include "LocalLibrary.h"

#define MEGA  // set this if using an Arduino Mega
// #define PANSTAMP // set this if using Panstamp



// Define I/0 Pins
#ifdef MEGA
  //=== Analog Inputs for CTs measuring current ===
  #define CT_PUMP       8  // Pump amps input 20 Amp CT
  #define CT_HEATER1    9  // Heater leg 1 amps, 50 Amp CT
  #define CT_HEATER2   10  // Heater leg 2 amps, 50 Amp CT
  #define CT_BUBBLER   11  // Bubbler amps, 20 Amp CT
  
  //===Analog Inputs from Pressure===
  #define PRESSURE_GAUGE 12  // Pressure, comes from 4-20mA gauge
  
  //=== Output for Solid State Relays ===
  #define PUMP_ON_OFF_OUTPUT_PIN     11
  #define HEATER_ON_OFF_OUTPUT_PIN    9
  #define BUBBLER_ON_OFF_OUTPUT_PIN   8
  
  #define ONE_WIRE_BUS 37 // OneWire data pin
#endif

#ifdef PANSTAMP
  //=== Analog Inputs for CTs measuring current ===
  #define CT_PUMP       0  // Pump amps input 20 Amp CT
  #define CT_HEATER1    1  // Heater leg 1 amps, 50 Amp CT
  #define CT_HEATER2    2  // Heater leg 2 amps, 50 Amp CT
  #define CT_BUBBLER    3  // Bubbler amps, 20 Amp CT
  
  //===Analog Inputs from Pressure===
  #define PRESSURE_GAUGE 6  // Pressure, comes from 4-20mA gauge
  
  //=== Output for Solid State Relays ===
  #define PUMP_ON_OFF_OUTPUT_PIN      3
  #define HEATER_ON_OFF_OUTPUT_PIN    4
  #define BUBBLER_ON_OFF_OUTPUT_PIN   5
  
  #define ONE_WIRE_BUS 6 // OneWire data pin
#endif


// Initialize OneWire temp sensors
OneWire oneWire(ONE_WIRE_BUS); 
DallasTemperature oneWireBus(&oneWire);
static uint8_t tempSensor[4][8] =
{
  { 0x28, 0x0A, 0x48, 0x00, 0x05, 0x00, 0x00, 0x33 },  // pre heater temp sensor 1, lower sensor
  { 0x10, 0x33, 0x71, 0x6A, 0x02, 0x08, 0x00, 0x5D },  // pre heater temp sensor 2, upper sensor
  { 0x28, 0x20, 0xCE, 0xB3, 0x04, 0x00, 0x00, 0xAF },  // post heater temp sensor
  { 0x28, 0x1F, 0xA2, 0xB2, 0x04, 0x00, 0x00, 0xD9 }   // Pump temp
};

// Create HotTubController instance
HotTubControl hotTubControl;

// Setup sensor variables
float tempPreHeat;
float tempPostHeat;
float tempPump;
float pressure;
float pump_amps;
float heater_amps;
float bubbler_amps;


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

// Define Function Prototypes
void ReadSensorInputs();
void CheckAlarms();
void OutputAlarm(char AlarmText[]);
void PrintStatus();
boolean NeedHeat();



//============================================================================
void setup()
{
  Serial.begin(9600);
  
  oneWireBus.begin(); // Initialize OneWire


  pinMode(PUMP_ON_OFF_OUTPUT_PIN,    OUTPUT);
  pinMode(HEATER_ON_OFF_OUTPUT_PIN,  OUTPUT);
  pinMode(BUBBLER_ON_OFF_OUTPUT_PIN, OUTPUT);

  
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
    ReadSensorInputs();
  }
  
  // Check alarms
  if ((long)(millis() - last_alarm_check) > ALARM_CHECK_INTERVAL)
  {
    last_alarm_check = millis();
    CheckAlarms();
  }

  PrintStatus();  // Print for debugging
  

  // Turn off pump         srg - pump sometimes turns off right after turning on
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
     Serial.print(tempPreHeat);
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
     Serial.print(tempPreHeat);     
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
     (pump_amps >= PUMP_AMPS_THRESHOLD) &&
     (pressure >= PUMP_PRESSURE_THRESHOLD) &&
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
     (heater_amps >= ALARM_HEATER_AMPS_HIGH) ||
     (pump_amps < PUMP_AMPS_THRESHOLD) ||
     (pressure < PUMP_PRESSURE_THRESHOLD) ||
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

  // Update user panel with hot tub info: Water temp, Hot 
  hotTubControl.writePanelStatus( tempPreHeat,  digitalRead(PUMP_ON_OFF_OUTPUT_PIN),  digitalRead(BUBBLER_ON_OFF_OUTPUT_PIN),  digitalRead(HEATER_ON_OFF_OUTPUT_PIN) );

}  // loop()



//*********************************************************************************
// Read Amps, Temperatures and pressure
// Take 25 samples then take average
//*********************************************************************************
void ReadSensorInputs()
{
  // Get temperatures from Thermocouples
  // I didn't put in loop because reading temps is kind of slow
  // Determine which sensor to take measurement from
  float validTemp;
  float PreHeaterTemp1;
  float PreHeaterTemp2;
  oneWireBus.requestTemperatures();   // Send the command to get temperatures
  validTemp = oneWireBus.getTempF(&tempSensor[0][0]);
  if (validTemp > 40)
  { PreHeaterTemp1 = validTemp; }
  
  validTemp = oneWireBus.getTempF(&tempSensor[1][0]);
  if (validTemp > 40)
  { PreHeaterTemp2 = validTemp; }
  
  validTemp = oneWireBus.getTempF(&tempSensor[2][0]);
  if (validTemp > 40)
  { tempPostHeat = validTemp; }
  
  validTemp = oneWireBus.getTempF(&tempSensor[3][0]);
  if (validTemp > 40)
  { tempPump = validTemp; }

  if(abs(PreHeaterTemp1 - PreHeaterTemp2) < 4)
  {
    // Two probes are relatively close, just take the average for the temp
    tempPreHeat =  (PreHeaterTemp1 + PreHeaterTemp2) / 2;
  }
  else if(PreHeaterTemp1 > PreHeaterTemp2)    // Temp probes are not very close, choose higher one so heater doesn't stay on all the time
  { tempPreHeat =  PreHeaterTemp1; }
  else
  { tempPreHeat = PreHeaterTemp2; }
  
  
  // Take multiple samples and get average
  // Initialize variables
  pump_amps =    0.0;
  heater_amps =  0.0;
  bubbler_amps = 0.0;
  pressure =     0.0;
  int samples;
  
  for (samples = 0; samples < 25; samples++)
  {
    // Get Amps from CTs
    pump_amps    += analogRead(CT_PUMP);  // ADC value for pump is about 630
    heater_amps  += (analogRead(CT_HEATER1) + analogRead(CT_HEATER2))/2.0;  // There are 2 CTs for the heater, take average.  Each one should read about 22.8 amps
    bubbler_amps += analogRead(CT_BUBBLER);
    pressure += analogRead(PRESSURE_GAUGE);   // Get Pressure.  30 PSI Max, 4-20mA output
    delay(1);
  }
  
  // Calculate averages from samples
  pump_amps =    (pump_amps    / (float) samples) * (20.0 / 1024.0) - 1.3;    // 20 Amp CT, -1.3 calibration offset
  heater_amps =  (heater_amps  / (float) samples) * (50.0 / 1024.0) - 4.75;   // 50 Amp CTs, -4.75 amps calibration offset
  bubbler_amps = (bubbler_amps / (float) samples) * (20.0 / 1024.0) - 1.0;    // 20 Amp CTs, -1.0 amp calibration offset
  pressure =     (pressure     / (float) samples) * 0.0377 - 7.5094 + 0.25;   // 0.25 PSI calibration offset
  // If values are close to zero, then set to zero
  if (pump_amps    < 1.0) pump_amps =    0.0;
  if (heater_amps  < 1.0) heater_amps =  0.0;
  if (bubbler_amps < 0.5) bubbler_amps = 0.0;
  if (pressure     < 2.0) pressure =     0.0;

/*
  // SRG dummy data when not connected to main controller board
  tempPostHeat = 110;
  tempPump = 150;
  tempPreHeat = 98;
  pump_amps = 9.0;
  heater_amps = 22.0;
  bubbler_amps = 5.0;
  pressure = 15.0;
*/

}  // ReadSensorInputs()



// Determine if hot tub heaters should come on
// If heater is already on, heat it up to setpoint + 0.8 degrees
// If heater is off, turn it on when temp drops to setpoint - 0.8 degree
// Once heat is called for, keep it on for at least 3 minutes
boolean NeedHeat()
{ 
  if(digitalRead(HEATER_ON_OFF_OUTPUT_PIN) == HIGH)
  { // heater is on
    if (tempPreHeat > (float) hotTubControl.getTempSetpoint() + 0.4 )  // temperature has reached setpoint, don't need heat anymore
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
    if (tempPreHeat < (float) hotTubControl.getTempSetpoint() - 0.4 )  // Turn on heater if it's 0.8 degree below setpoint
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
  if ( tempPreHeat > 120 )
  {
    sprintf_P(txtAlarm, PSTR("High pre-heater temperature = %d degrees"), (int) tempPreHeat );
    OutputAlarm(txtAlarm);
  }
  
  // high water temp, post heater
  if (  tempPostHeat > 150 )
  {
    sprintf_P(txtAlarm, PSTR("High post-heater temperature = %d degrees"), (int) tempPostHeat );
    OutputAlarm(txtAlarm);
  }
  
  // Pump housing high temperature
  if ( tempPump > 150 )
  {
    sprintf_P(txtAlarm, PSTR("High pump housing temperature = %d degrees"), (int) tempPump );
    OutputAlarm(txtAlarm);
  }
  
  // Low temp alarm
  if ( tempPreHeat < 50 )
  {
    sprintf_P(txtAlarm, PSTR("Low pre-heater temperature = %d degrees"), (int) tempPreHeat );
    OutputAlarm(txtAlarm);
  }
  
  // High pressure alarm
  if ( pressure > 20.0 )
  {
    sprintf_P(txtAlarm, PSTR("High pressure alarm, pressure = %d PSI"), (int) pressure );
    OutputAlarm(txtAlarm);
  }
  
  // High Pump Amps alarm
  if ( pump_amps > 18.0 )
  {
    sprintf_P(txtAlarm, PSTR("High pump amps alarm = %d amps"), (int) pump_amps );
    OutputAlarm(txtAlarm);
  }
  
  // Pump on but pressure is low.  Will need delay after pump is first turned on before you check for alarm
  if ( (pressure < 5.0) && (digitalRead(PUMP_ON_OFF_OUTPUT_PIN) == HIGH) )
  {
    sprintf_P(txtAlarm, PSTR("Low pressure alarm while pump is on, pressure = %d PSI"), (int) pressure );
    OutputAlarm(txtAlarm);
  }
  
  // High heater amps
  if ( heater_amps > ALARM_HEATER_AMPS_HIGH )
  {
    sprintf_P(txtAlarm, PSTR("High heater amps = %d"), (int) heater_amps );
    OutputAlarm(txtAlarm);
  }
  
  // Low heater amps
  if ( (heater_amps < 15) && (digitalRead(HEATER_ON_OFF_OUTPUT_PIN) == HIGH) )
  {
    sprintf_P(txtAlarm, PSTR("Low heater amps while heater is on = %d amps"), (int) heater_amps );
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

    Serial.print(tempPreHeat);
    Serial.print(F("\t"));

    Serial.print(hotTubControl.getTempSetpoint());
    Serial.print(F("\t"));

    Serial.print(hotTubControl.isBubbleBtnOn());
    Serial.print(digitalRead(BUBBLER_ON_OFF_OUTPUT_PIN));
    Serial.print(F("\t"));

    Serial.print(tempPreHeat);
    Serial.print(F("\t"));
    Serial.print(tempPostHeat);
    Serial.print(F("\t"));
    Serial.print(tempPump);
    Serial.print(F("\t"));
    Serial.print(pressure);
    Serial.print(F("\t"));
    Serial.print(heater_amps);
    Serial.print(F("\t"));
    Serial.print(pump_amps);
    Serial.print(F("\t"));
    Serial.print(bubbler_amps);
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
     
     Serial.print(tempPreHeat);
     Serial.print("\t");
     Serial.print(tempPostHeat);
     Serial.print("\t");
     Serial.print(tempPump);
     Serial.print("\t");
     Serial.print(pressure);
     Serial.print("\t");
     Serial.print(heater_amps);
     Serial.print("\t");
     Serial.print(pump_amps);
     Serial.print("\t");
     Serial.print(bubbler_amps);
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









