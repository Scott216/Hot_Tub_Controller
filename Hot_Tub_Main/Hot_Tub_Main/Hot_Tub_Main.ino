/*
Do you really need heater_cooldown_timer
Spikes form heater contacter are messing up LCD display
When hot tub is about to come on because it needs to heat up, pumpp comes on then goes off after a second, it will do this several times
Buttons don't latch very well, then need to be tuned
Get encoder code from tardis alarm, this one doesn't work well
Need a timer on needHeat to it stays true for a few seconds when heat first turns on

Bubbler draws about 5 amps


Directory:
cd Dropbox/Arduino/Hot_Tub_Controller

*/


// === Libraries ===
#include "Arduino.h"
#include <OneWire.h>    // http://www.pjrc.com/teensy/td_libs_OneWire.html  http://playground.arduino.cc/Learning/OneWire
#include <DallasTemperature.h> // http://milesburton.com/index.php?title=Dallas_Temperature_Control_Library


// Include application, user and local libraries
#include "LocalLibrary.h"

#define ONE_WIRE_BUS 29 // OneWire data pin

//=== Analog Inputs for CTs measuring current ===
#define CT_PUMP        8  // Pump amps input 20 Amp CT
#define CT_HEATER1    11  // Heater leg 1 amps, 50 Amp CT
#define CT_HEATER2    10  // Heater leg 2 amps, 50 Amp CT
#define CT_BUBBLER     9  // Bubbler amps, 20 Amp CT

//===Analog Inputs from Pressure===
#define PRESSURE_GAUGE 12  // Pressure, comes from 4-20mA gauge

//=== Output for Solid State Relays ===
#define PUMP_ON_OFF_OUTPUT_PIN         11
#define HEATER_ON_OFF_OUTPUT_PIN        9
#define BUBBLER_ON_OFF_OUTPUT_PIN       8

#define WATER_TEMP_DEFAULT  82
int Temperature_Setpoint = WATER_TEMP_DEFAULT;  //srg this is in both sketches



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



// Setup input sensor variables
float tempTC[3];  // water temperatures
#define PRE_HEATER     0
#define POST_HEATER    1
#define PUMP_HOUSING   2
float  pressure;
float  pump_amps;
float  heater_amps;
float  bubbler_amps;

bool InputButtonsCurrentState[3];    // Array for Input buttons, current state
#define BTN_HOT_TUB_ON_OFF   0  // Hot tub On/Off Button
#define BTN_JETS_ON_OFF      1  // Jests On/Off Button
#define BTN_BUBBLER_ON_OFF   2  // Bubbler On/Off button



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
 
// Alarm setpoints
const float ALARM_HEATER_AMPS_HIGH = 30.0;
#define PUMP_AMPS_THRESHOLD       2  // Amps needed verify pump is on
#define PUMP_PRESSURE_THRESHOLD   5  // Pump pressure (PSI) to verify pump is on

// Define Function Prototypes
void ReadSensorInputs();
void CheckAlarms();
void OutputAlarm(char AlarmText[]);
void PrintStatus();
boolean NeedHeat();
void WDTSetup(void);


//============================================================================
void setup()
{
  Serial.begin(9600);
  
  //  WDTSetup();  // Initialize WDT timer

  // Initialize Digital I/O
  pinMode(PUMP_ON_OFF_OUTPUT_PIN,       OUTPUT);
  pinMode(HEATER_ON_OFF_OUTPUT_PIN,     OUTPUT);
  pinMode(BUBBLER_ON_OFF_OUTPUT_PIN,    OUTPUT);
  
    oneWireBus.begin(); // Initialize OneWire  
  
    // Initialize input buttons arrays
  for(int btn=0; btn < 3; btn++)
  {
    InputButtonsCurrentState[btn] = LOW;
  }
  
  
  // Initialize timers
  heater_cooldown_timer = 0;
  last_heater_on_check =  0;
  
  Serial.println("Finished setup()");
  
} // setup()



//============================================================================
void loop()
{
  
  static uint32_t lastPumpOnTime; // Millis() Timestamp of when pump was last turned on.  Updates every cycle that pump should be on.  Used to keep pump from cycling on/off too fast
 
  
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
  
  
  PrintStatus();  //Print for debugging
  

  // Turn off pump         srg - pump sometimes turns off right after turning on
  // If Hot tub is turned off and cooldown delay has passed, or
  // Jets button is off and we don't need heat, and cooldown delay has passed
  if((InputButtonsCurrentState[BTN_HOT_TUB_ON_OFF] == LOW                     && (long)(millis() - heater_cooldown_timer) > 0) ||
     (InputButtonsCurrentState[BTN_JETS_ON_OFF] == LOW && NeedHeat() == false && (long)(millis() - heater_cooldown_timer) > 0) )
  {
    // srg debug
     // Print values so you can see why pump is turned off
    if(digitalRead(PUMP_ON_OFF_OUTPUT_PIN) == HIGH)
    {
     Serial.println("Turning Pump Off");
     Serial.println("OnOff BTN\tJetsBtn\tNeedHeat()\ttemp\theater\ttimer\tCooldown");
     Serial.print("   ");
     Serial.print(InputButtonsCurrentState[BTN_HOT_TUB_ON_OFF]);
     Serial.print("\t\t");
     Serial.print(InputButtonsCurrentState[BTN_JETS_ON_OFF]);
     Serial.print("\t");
     Serial.print(NeedHeat());
     Serial.print("\t\t");
     Serial.print(tempTC[PRE_HEATER]);
     Serial.print("\t");
     Serial.print(digitalRead(HEATER_ON_OFF_OUTPUT_PIN));
     Serial.print("\t");
     Serial.print((long)(millis() - lastPumpOnTime));
     Serial.print("\t\t");
     Serial.print(millis() - heater_cooldown_timer);
     Serial.println();
    }

    digitalWrite(PUMP_ON_OFF_OUTPUT_PIN, LOW);
  }
  
  // Turn on pump
  // Pump is turned on either by manually by pushbutton or by the heater
  if((InputButtonsCurrentState[BTN_HOT_TUB_ON_OFF] == HIGH) &&                          // Hot tub must be on
     ((InputButtonsCurrentState[BTN_JETS_ON_OFF] == HIGH) || (NeedHeat() == true)) &&   // Pushbutton OR Need heat
     ((long)(millis() - lastPumpOnTime) > PUMP_ON_DELAY))                           // Delay before pump can be turned on again after being off, prevents fast cycling if there is a problem
  {
    // srg debug
     // Print values so you can see why pump is turned off
    if(digitalRead(PUMP_ON_OFF_OUTPUT_PIN) == LOW)
    {
     Serial.println("Turning Pump On");
     Serial.println("OnOff BTN\tJetsBtn\tNeedHeat()\ttemp\theater\tLastPumpCheck");
     Serial.print("   ");
     Serial.print(InputButtonsCurrentState[BTN_HOT_TUB_ON_OFF]);
     Serial.print("\t\t");
     Serial.print(InputButtonsCurrentState[BTN_JETS_ON_OFF]);
     Serial.print("\t");
     Serial.print(NeedHeat());
     Serial.print("\t\t");
     Serial.print(tempTC[PRE_HEATER]);     
     Serial.print("\t");
     Serial.print(digitalRead(HEATER_ON_OFF_OUTPUT_PIN));
     Serial.print("\t\t");
     Serial.print(millis() - lastPumpOnTime);
     Serial.println();
    }

    digitalWrite(PUMP_ON_OFF_OUTPUT_PIN, HIGH);
    lastPumpOnTime = millis();
  }
    
  
  // Turn on heater
  // Prerequisites: Hot Tub On + Pump amps and pressure are above threshold + Low water temp + Delay from last time on
  if((InputButtonsCurrentState[BTN_HOT_TUB_ON_OFF] == HIGH) &&
     (digitalRead(PUMP_ON_OFF_OUTPUT_PIN) == HIGH) &&
     (pump_amps >= PUMP_AMPS_THRESHOLD) &&
     (pressure >= PUMP_PRESSURE_THRESHOLD) &&
     (NeedHeat() == true) &&
     ((long)(millis() - last_heater_on_check) > HEATER_ON_DELAY))
  {
    digitalWrite(HEATER_ON_OFF_OUTPUT_PIN, HIGH);
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
     (InputButtonsCurrentState[BTN_HOT_TUB_ON_OFF] == LOW))
  {
    digitalWrite(HEATER_ON_OFF_OUTPUT_PIN, LOW);
  }
  
  
  // Turn on bubbler
  if((InputButtonsCurrentState[BTN_HOT_TUB_ON_OFF] == HIGH) &&
     (InputButtonsCurrentState[BTN_BUBBLER_ON_OFF] == HIGH) &&
     ((long)(millis() - last_bubbler_on_check) > BUBBLER_ON_DELAY))
  {
    digitalWrite(BUBBLER_ON_OFF_OUTPUT_PIN, HIGH);
    last_bubbler_on_check = millis();
  }
  
  // Turn off bubbler
  if(InputButtonsCurrentState[BTN_BUBBLER_ON_OFF] == LOW || InputButtonsCurrentState[BTN_HOT_TUB_ON_OFF] == LOW)
  {
    digitalWrite(BUBBLER_ON_OFF_OUTPUT_PIN, LOW);
    InputButtonsCurrentState[BTN_BUBBLER_ON_OFF] = LOW;
  }
  
  
  
  

}  // loop()




//*********************************************************************************
/*
 8888888b.                        888  .d8888b.                                            8888888                            888
 888   Y88b                       888 d88P  Y88b                                             888                              888
 888    888                       888 Y88b.                                                  888                              888
 888   d88P .d88b.   8888b.   .d88888  "Y888b.    .d88b.  88888b.  .d8888b   .d88b.  888d888 888   88888b.  88888b.  888  888 888888 .d8888b
 8888888P" d8P  Y8b     "88b d88" 888     "Y88b. d8P  Y8b 888 "88b 88K      d88""88b 888P"   888   888 "88b 888 "88b 888  888 888    88K
 888 T88b  88888888 .d888888 888  888       "888 88888888 888  888 "Y8888b. 888  888 888     888   888  888 888  888 888  888 888    "Y8888b.
 888  T88b Y8b.     888  888 Y88b 888 Y88b  d88P Y8b.     888  888      X88 Y88..88P 888     888   888  888 888 d88P Y88b 888 Y88b.       X88
 888   T88b "Y8888  "Y888888  "Y88888  "Y8888P"   "Y8888  888  888  88888P'  "Y88P"  888   8888888 888  888 88888P"   "Y88888  "Y888  88888P'
.                                                                                                           888
.                                                                                                           888
.                                                                                                           888
 */
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
  { tempTC[POST_HEATER] = validTemp; }
  
  validTemp = oneWireBus.getTempF(&tempSensor[3][0]);
  if (validTemp > 40)
  { tempTC[PUMP_HOUSING] = validTemp; }

  if(abs(PreHeaterTemp1 - PreHeaterTemp2) < 4)
  {
      // Two probes are relatively close, just take the average for the temp
    tempTC[PRE_HEATER] =  (PreHeaterTemp1 + PreHeaterTemp2) / 2;
  }
  else if(PreHeaterTemp1 > PreHeaterTemp2)    // Temp probes are not very close, choose higher one so heater doesn't stay on all the time
  { tempTC[PRE_HEATER] =  PreHeaterTemp1; }
  else
  { tempTC[PRE_HEATER] = PreHeaterTemp2; }
  
  
    // Take multiple samples and get average
    // Initialize variables
  pump_amps =            0.0;
  heater_amps =          0.0;
  bubbler_amps =         0.0;
  pressure =             0.0;
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
  heater_amps =  (heater_amps  / (float) samples) * (50.0 / 1024.0) - 4.75;    // 50 Amp CTs, -4.75 amps calibration offset
  bubbler_amps = (bubbler_amps / (float) samples) * (20.0 / 1024.0) - 1.0;    // 20 Amp CTs, -1.0 amp calibration offset
  pressure =     (pressure     / (float) samples) * 0.0377 - 7.5094 + 0.25;   //  0.25 PSI calibration offset
  if (pump_amps    < 1.0) pump_amps = 0.0;
  if (heater_amps  < 1.0) heater_amps = 0.0;
  if (bubbler_amps < 0.5) bubbler_amps = 0.0;
  if (pressure     < 2.0) pressure = 0.0;
  
}  // ReadSensorInputs()



// Determine if hot tub heaters should come on
// If heater is already on, heat it up to setpoint + 0.8 degrees
// If heater is off, turn it on when temp drops to setpoint - 0.8 degree
boolean NeedHeat()
{ 
  if(digitalRead(HEATER_ON_OFF_OUTPUT_PIN) == HIGH)
  { // heater is on
    if (tempTC[PRE_HEATER] > (float) Temperature_Setpoint + 0.4 )  // temperature has reached setpoint, don't need heat anymore
    {return false;}
    else
    {return true;}  // temperature is below setpoint, call for heat
  }
  else
  { // heater is off
    if (tempTC[PRE_HEATER] < (float) Temperature_Setpoint - 0.4 )  // Turn on heater if it's 0.8 degree below setpoint
    {return true;}
    else
    {return false;} 
  }

} // End NeedHeat()




  //*********************************************************************************
/*
 .d8888b.  888                        888             d8888 888
 d88P  Y88b 888                        888            d88888 888
 888    888 888                        888           d88P888 888
 888        88888b.   .d88b.   .d8888b 888  888     d88P 888 888  8888b.  888d888 88888b.d88b.  .d8888b
 888        888 "88b d8P  Y8b d88P"    888 .88P    d88P  888 888     "88b 888P"   888 "888 "88b 88K
 888    888 888  888 88888888 888      888888K    d88P   888 888 .d888888 888     888  888  888 "Y8888b.
 Y88b  d88P 888  888 Y8b.     Y88b.    888 "88b  d8888888888 888 888  888 888     888  888  888      X88
 "Y8888P"  888  888  "Y8888   "Y8888P 888  888 d88P     888 888 "Y888888 888     888  888  888  88888P'
 
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
  if ( tempTC[PRE_HEATER] > 120 )
  {
    sprintf(txtAlarm, "High Pre-heater temperature = %d degrees", (int) tempTC[PRE_HEATER]);
    OutputAlarm(txtAlarm);
  }
  
  // high water temp, post heater
  if (  tempTC[POST_HEATER] > 150 )
  {
    sprintf(txtAlarm, "High Post-heater temperatur = %d degrees", (int) tempTC[POST_HEATER]);
    OutputAlarm(txtAlarm);
  }
  
  // Pump housing high temperature
  if ( tempTC[PUMP_HOUSING] > 150 )
  {
    sprintf(txtAlarm, "High pump housing temperature = %d degrees", (int) tempTC[PUMP_HOUSING]);
    OutputAlarm(txtAlarm);
  }
  
  // Low temp alarm
  if ( tempTC[PRE_HEATER] < 50 )
  {
    sprintf(txtAlarm, "Low Pre-heater temperature = %d degrees", (int) tempTC[PRE_HEATER]);
    OutputAlarm(txtAlarm);
  }
  
  // High pressure alarm
  if ( pressure > 20.0 )
  {
    sprintf(txtAlarm, "High Pressure Alarm, pressure = %d PSI", (int) pressure);
    OutputAlarm(txtAlarm);
  }
  
  // High Pump Amps alarm
  if ( pump_amps > 18.0 )
  {
    sprintf(txtAlarm, "High Pump Amps Alarm = %d amps", (int) pump_amps);
    OutputAlarm(txtAlarm);
  }
  
  // Pump on but pressure is low.  Will need delay after pump is first turned on before you check for alarm
  if ( (pressure < 5.0) && (digitalRead(PUMP_ON_OFF_OUTPUT_PIN) == HIGH) )
  {
    sprintf(txtAlarm, "Low Pressure Alarm while pump is on, pressure = %d PSI", (int) pressure);
    OutputAlarm(txtAlarm);
  }
  
  // High heater amps
  if ( heater_amps > ALARM_HEATER_AMPS_HIGH )
  {
    sprintf(txtAlarm, "High heater amps = %d", (int) heater_amps);
    OutputAlarm(txtAlarm);
  }
  
  // Low heater amps
  if ( (heater_amps < 15) && (digitalRead(HEATER_ON_OFF_OUTPUT_PIN) == HIGH) )
  {
    sprintf(txtAlarm, "Low heater amps while heater is on = %d amps.", (int) heater_amps);
    OutputAlarm(txtAlarm);
  }
  
} // end CheckAlarms()



//*********************************************************************************
/*
 .d88888b.           888                      888           d8888 888
 d88P" "Y88b          888                      888          d88888 888
 888     888          888                      888         d88P888 888
 888     888 888  888 888888 88888b.  888  888 888888     d88P 888 888  8888b.  888d888 88888b.d88b.
 888     888 888  888 888    888 "88b 888  888 888       d88P  888 888     "88b 888P"   888 "888 "88b
 888     888 888  888 888    888  888 888  888 888      d88P   888 888 .d888888 888     888  888  888
 Y88b. .d88P Y88b 888 Y88b.  888 d88P Y88b 888 Y88b.   d8888888888 888 888  888 888     888  888  888
 "Y88888P"   "Y88888  "Y888 88888P"   "Y88888  "Y888 d88P     888 888 "Y888888 888     888  888  888
.                            888
.                            888
.                            888
 
 Print Alarm Text   */
//*********************************************************************************
void OutputAlarm(char AlarmText[])
{
  Serial.print("Alarm: ");
  Serial.println(AlarmText);
} // OutputAlarm()




//*********************************************************************************
/*
 8888888b.          d8b          888    .d8888b.  888             888
 888   Y88b         Y8P          888   d88P  Y88b 888             888
 888    888                      888   Y88b.      888             888
 888   d88P 888d888 888 88888b.  888888 "Y888b.   888888  8888b.  888888 888  888 .d8888b
 8888888P"  888P"   888 888 "88b 888       "Y88b. 888        "88b 888    888  888 88K
 888        888     888 888  888 888         "888 888    .d888888 888    888  888 "Y8888b.
 888        888     888 888  888 Y88b. Y88b  d88P Y88b.  888  888 Y88b.  Y88b 888      X88
 888        888     888 888  888  "Y888 "Y8888P"   "Y888 "Y888888  "Y888  "Y88888  88888P'
 */
//*********************************************************************************
void PrintStatus()
{
  
  static int8_t   cntHeading;  // prints new heading every 20 rows
  static uint32_t PrintDelay;
  
  if ((long)(millis() - PrintDelay) > 0)
  {
    PrintDelay = millis() + 500UL;
 /*   
    //Print Pushbutton state and Led indicator state
    if(cntHeading > 40 || millis() < 5000)
    {
      Serial.println("OnOff\tPump\tHeat\tcooldn\tpumpdly\theatdly");
      cntHeading = 0;
    }
    cntHeading++;
    
    Serial.print(InputButtonsCurrentState[BTN_HOT_TUB_ON_OFF]);
    Serial.print("\t");
    
    Serial.print(InputButtonsCurrentState[BTN_JETS_ON_OFF]);
    Serial.print(digitalRead(PUMP_ON_OFF_OUTPUT_PIN));
    Serial.print("\t");
    
    Serial.print(digitalRead(HEATER_ON_OFF_OUTPUT_PIN));
    Serial.print(NeedHeat());
    Serial.print("\t");
    
    if (heater_cooldown_timer > millis())
//    { Serial.print((heater_cooldown_timer - millis())/1000);}  // print seconds heaters need to cooldown before pump can turn off
    { Serial.print(heater_cooldown_timer);}  // print seconds heaters need to cooldown before pump can turn off
    else
    { Serial.print(heater_cooldown_timer);}
    Serial.print("\t");
    
    Serial.print((int)((millis() - last_pump_on_check)/1000));
    Serial.print("\t");
    
    Serial.print((int)((millis() - last_heater_on_check)/1000));
    Serial.println();
 
 */   
    
     if(cntHeading > 40)
     {
     Serial.println("T1\tT2\tT3\tpres\tampH\tampP\tampB");
     cntHeading = 0;
     }
     cntHeading++;
     
     Serial.print(tempTC[PRE_HEATER]);
     Serial.print("\t");
     Serial.print(tempTC[POST_HEATER]);
     Serial.print("\t");
     Serial.print(tempTC[PUMP_HOUSING]);
     Serial.print("\t");
     Serial.print(pressure);
     Serial.print("\t");
     Serial.print(heater_amps);
     Serial.print("\t");
     Serial.print(pump_amps);
     Serial.print("\t");
     Serial.print(bubbler_amps);
     Serial.println();
    
    
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









