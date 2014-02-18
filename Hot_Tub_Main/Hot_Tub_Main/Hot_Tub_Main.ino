/*
To do: 
try different encoder code
Can pushbuttons be in interrupts - yes
Have an alarm condition shut down Hot Tub
Have pump come on for 30 seconds before heat comes on just to make sure hot tub really needs heat
Put turning heater, pump and bubbler functions in class, currently it's done with digitalWrite in the ino file

*/

// === Libraries ===
#include "Arduino.h"
#include <OneWire.h>           // http://www.pjrc.com/teensy/td_libs_OneWire.html  http://playground.arduino.cc/Learning/OneWire
#include <DallasTemperature.h> // http://milesburton.com/index.php?title=Dallas_Temperature_Control_Library
#include <I2C.h>               // use for I2C communication  http://dsscircuits.com/articles/arduino-i2c-master-library.html
#include <SSD1306_I2C_DSS.h>   // Library for OLED display http://github.com/Scott216/SSD1306_I2C_DSS
#include <Adafruit_GFX.h>      // Library for OLED display http://github.com/adafruit/Adafruit-GFX-Library
#include "Hot_Tub_MainLibrary.h"


// Create HotTubController instance
HotTubControl hotTubControl;

// Values for NeedHestStatus
enum needHeatStatus_t { DONT_NEED_HEAT, PRE_HEAT_CHECK, NEED_HEAT };
needHeatStatus_t needHeatStatus = DONT_NEED_HEAT; 

// Alarm setpoints
#define ALARM_HEATER_AMPS_HIGH   30  // Max heater amps allowed
#define PUMP_AMPS_THRESHOLD       2  // Min amps needed verify pump is on
#define PUMP_PRESSURE_THRESHOLD   5  // Min PSI needed to verify pump is running

#define I2C_TIMEOUT 20000
Adafruit_SSD1306 display(OLED_RESET, I2C_TIMEOUT);


// Initialize OneWire temp sensors, they are used in LocalLibrary.cpp
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature oneWireBus(&oneWire);


// Define Function Prototypes
bool CheckAlarms();
void OutputAlarm(char AlarmText[]);
void updateDisplay();
boolean NeedHeat();
void PrintStatus();
void PrintStatusAll();
void printHeaterInfo();
void PrintPumpInfo();


//============================================================================
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
  

  // Initialize OLED display
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x32)
  display.setRotation(2);  // Orientation 0 = right side up, 2 = upside down
  display.clearDisplay();  // clears the screen and buffer
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,8);
  display.println(F("Finished setup()"));
  display.display();

  Serial.println(F("Finished Hot Tub Main Setup()"));
  
} // setup()


//============================================================================
//============================================================================
void loop()
{
  // Setup timers
  const uint32_t PUMP_ON_DELAY =            5000;  // Delay before pump can be turned on again after being off
  const uint32_t ALARM_CHECK_INTERVAL =     5000;  // Check for alarm every 5 seconds
  const uint32_t SENSOR_CHECK_INTERVAL =    1000;  // Check sensor every second
  const uint32_t BUBBLER_ON_DELAY =          700;  // Delay before bubbler can be turned on again after being off
  const uint32_t HEATER_COOLDOWN_DELAY =   10000;  // When heater is turned off, keep pump running a few seconds to help heating coils cool down
  const uint32_t MIN_HEATER_ON_TIME    =   60000;  // Minimum time heater will stay on
  
  static uint32_t lastSensorCheck =        0;  // Timer used to read sensor values every second
  static uint32_t lastAlarmCheck =         0;  // Timer used to check alarms every few seconds
  static uint32_t lastPumpOnTime;              // Millis() Timestamp of when pump was last turned on.
                                               // Updates every cycle that pump should be on.  Used to keep pump from cycling on/off too fast
  static uint32_t last_bubbler_on_check =  0;  // Timer used to prevent bubbler from being switched on too quickly
  static uint32_t heater_cooldown_timer =  0;  // Timer used to keep pump on for a few seconds after heater has turned off
  static uint32_t pre_heat_pump_on_timer = 0;  // Timer used to turn pump on a few seconds to verify we actually need heat

  // Read pushbuttons status and temperature setpoint from user panel
  hotTubControl.readPanelStatus();

  
  // Check sensor inputs
  if ((long)(millis() - lastSensorCheck) > SENSOR_CHECK_INTERVAL)
  {
    lastSensorCheck = millis();
    hotTubControl.refreshSensors();

    if (digitalRead(PUMP_ON_OFF_OUTPUT_PIN) == LOW && NeedHeat() && needHeatStatus == DONT_NEED_HEAT )
    {
      // Pump is off, and we need heat, start timer that makes pump run to a little bit before we turn on heat
      pre_heat_pump_on_timer = millis() + 15000; 
      needHeatStatus = PRE_HEAT_CHECK; 
    }
    
    bool pre_heat_pump_on_timer_expired = (long) (millis() - pre_heat_pump_on_timer) > 0;
    if ( pre_heat_pump_on_timer_expired && needHeatStatus == PRE_HEAT_CHECK )
    {
      // Now that pump has been on for a few seconds, see if hot tub still needs heat
      if ( NeedHeat() )
      { needHeatStatus = NEED_HEAT; }
      else
      { needHeatStatus = DONT_NEED_HEAT; } 
    }
    
    // If water has reached setpoint, change needHeatStatus so pump and heater turn off
    if ( needHeatStatus == NEED_HEAT && !NeedHeat() )
    { needHeatStatus = DONT_NEED_HEAT; }
    
    updateDisplay();
  }
  
  // Check alarms
  if ((long)(millis() - lastAlarmCheck) > ALARM_CHECK_INTERVAL)
  {
    CheckAlarms();
    lastAlarmCheck = millis();
  }

  PrintStatus();  // srg Print for debugging

  // Turn off pump
  bool coolDownTimerExpired = (long)(millis() - heater_cooldown_timer) > 0;  // runs water past heater after heater is off so it will cool down
  if ( coolDownTimerExpired )
  {
    if ( !hotTubControl.isHotTubBtnOn() || ( !hotTubControl.isPumpBtnOn() && needHeatStatus == DONT_NEED_HEAT ) )
    { digitalWrite(PUMP_ON_OFF_OUTPUT_PIN, LOW); }
  }
  
  // Turn on pump
  bool pumpOnDelayTimerExpired = (long)(millis() - lastPumpOnTime) > PUMP_ON_DELAY;  // Timer used to prevent pump from cycling too quickly
  bool needPump = hotTubControl.isPumpBtnOn() || needHeatStatus != DONT_NEED_HEAT; // Either button or heater needs pump turned on
  if( hotTubControl.isHotTubBtnOn() && needPump && pumpOnDelayTimerExpired )
  {
    digitalWrite(PUMP_ON_OFF_OUTPUT_PIN, HIGH);
    lastPumpOnTime = millis();
  }


  // Heater control
  bool pumpIsRunning = digitalRead(PUMP_ON_OFF_OUTPUT_PIN) == HIGH && hotTubControl.getAmpsPump() >= PUMP_AMPS_THRESHOLD && hotTubControl.getPressure() >= PUMP_PRESSURE_THRESHOLD;
  bool heaterAmpsOK = hotTubControl.getAmpsHeater() <= ALARM_HEATER_AMPS_HIGH; 
  if ( hotTubControl.isHotTubBtnOn() && pumpIsRunning && needHeatStatus == NEED_HEAT && heaterAmpsOK ) 
  { // Turn on heater
    digitalWrite(HEATER_ON_OFF_OUTPUT_PIN, HIGH);
    heater_cooldown_timer = millis() + HEATER_COOLDOWN_DELAY; 
  }
  else // Turn off heater
  {
    // If heater is changing from on to off, reset needHeatStatus to false since 
    // needHeatStatus may not be updated right away
    if ( digitalRead(HEATER_ON_OFF_OUTPUT_PIN) == HIGH )
    {  needHeatStatus = DONT_NEED_HEAT; } // Set needHeatStatus since it won't be checked right away
 
    digitalWrite(HEATER_ON_OFF_OUTPUT_PIN, LOW); 
  }
  
  
  // Bubbler Control
  bool needBubbler = hotTubControl.isHotTubBtnOn() &&  hotTubControl.isBubbleBtnOn(); 
  bool bubbleDelayExpired = (long)(millis() - last_bubbler_on_check) > BUBBLER_ON_DELAY; // Delay to prevent bubbler from cycling too fast
  if ( needBubbler & bubbleDelayExpired )
  {
    digitalWrite(BUBBLER_ON_OFF_OUTPUT_PIN, HIGH);  
    last_bubbler_on_check = millis();
  }
  else if (!needBubbler)
  { digitalWrite(BUBBLER_ON_OFF_OUTPUT_PIN, LOW); }  


  // Send data to user panel
  hotTubControl.writePanelStatus( hotTubControl.getTempPreHeat(),
                                  digitalRead(PUMP_ON_OFF_OUTPUT_PIN),
                                  digitalRead(BUBBLER_ON_OFF_OUTPUT_PIN),
                                  digitalRead(HEATER_ON_OFF_OUTPUT_PIN) );

}  // loop()


//============================================================================
// Determine if hot tub heater should be turned on
// If heater is on, keep it on until temp goes above upper limit
// if heater is off, keep it off until temp drops below lower limit
//============================================================================
boolean NeedHeat()
{
  float upperLimit = (float) hotTubControl.getTempSetpoint() + 0.4;
  float lowerLimit = (float) hotTubControl.getTempSetpoint() - 0.4;

  if( digitalRead(HEATER_ON_OFF_OUTPUT_PIN) == HIGH && hotTubControl.getTempPreHeat() < upperLimit )
  { return true; } // Heater is on and hasn't reached upper limit yet
  else if (hotTubControl.getTempPreHeat() < lowerLimit )
  { return true; } // Temp is below lower limit, turn heater on
  else
  { return false; } // Temp is between LL and UL, keep off

} // end NeedHeat()


//============================================================================
// Update OLED display with stats on the sensors
//============================================================================
void updateDisplay()
{
  char dispTxt[22];
  
  display.clearDisplay();
  display.setCursor(0,0);
  sprintf(dispTxt, "Amp H:%d P:%d B:%d", (int)hotTubControl.getAmpsHeater(), (int)hotTubControl.getAmpsPump(), (int)hotTubControl.getAmpsBubbler());
  display.println(dispTxt);
  sprintf(dispTxt, "Temp %d %d %d %d",  (int)hotTubControl.getTempSetpoint(), (int)hotTubControl.getTempPreHeat(), (int)hotTubControl.getTempPostHeat(), (int)hotTubControl.getTempPump()); 
  display.setCursor(0,9);
  display.println(dispTxt);
  display.setCursor(0,19);
  sprintf(dispTxt, "P %d, Htr %d",  (int)hotTubControl.getPressure(), digitalRead(HEATER_ON_OFF_OUTPUT_PIN)); 
  display.println(dispTxt);  
  display.display();

}  // end updateDisplay()

//============================================================================
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
//============================================================================
bool CheckAlarms()
{
  char txtAlarm[75];
  bool isAlarm = false;
  
  // High water temp, pre-heater
  if ( hotTubControl.getTempPreHeat() > 120 )
  {
    sprintf_P(txtAlarm, PSTR("High pre-heater temperature = %d degrees"), (int) hotTubControl.getTempPreHeat() );
    isAlarm = true;
    OutputAlarm(txtAlarm);
  }
  
  // high water temp, post heater
  if (  hotTubControl.getTempPostHeat() > 150 )
  {
    sprintf_P(txtAlarm, PSTR("High post-heater temperature = %d degrees"), (int) hotTubControl.getTempPostHeat() );
    isAlarm = true;
    OutputAlarm(txtAlarm);
  }
  
  // Pump housing high temperature
  if ( hotTubControl.getTempPump() > 150 )
  {
    sprintf_P(txtAlarm, PSTR("High pump housing temperature = %d degrees"), (int) hotTubControl.getTempPump() );
    isAlarm = true;
    OutputAlarm(txtAlarm);
  }
  
  // Low temp alarm
  if ( hotTubControl.getTempPreHeat() < 50 )
  {
    sprintf_P(txtAlarm, PSTR("Low pre-heater temperature = %d degrees"), (int) hotTubControl.getTempPreHeat() );
    isAlarm = true;
    OutputAlarm(txtAlarm);
  }
  
  // High pressure alarm
  if ( hotTubControl.getPressure() > 20.0 )
  {
    sprintf_P(txtAlarm, PSTR("High pressure alarm, pressure = %d PSI"), (int) hotTubControl.getPressure() );
    isAlarm = true;
    OutputAlarm(txtAlarm);
  }
  
  // High Pump Amps alarm
  if ( hotTubControl.getAmpsPump() > 18.0 )
  {
    sprintf_P(txtAlarm, PSTR("High pump amps alarm = %d amps"), (int) hotTubControl.getAmpsPump() );
    isAlarm = true;
    OutputAlarm(txtAlarm);
  }
  
  // Pump on but pressure is low.  Will need delay after pump is first turned on before you check for alarm
  if ( (hotTubControl.getPressure() < 5.0) && (digitalRead(PUMP_ON_OFF_OUTPUT_PIN) == HIGH) )
  {
    sprintf_P(txtAlarm, PSTR("Low pressure alarm while pump is on, pressure = %d PSI"), (int) hotTubControl.getPressure() );
    isAlarm = true;
    OutputAlarm(txtAlarm);
  }
  
  // High heater amps
  if ( hotTubControl.getAmpsHeater() > ALARM_HEATER_AMPS_HIGH )
  {
    sprintf_P(txtAlarm, PSTR("High heater amps = %d"), (int) hotTubControl.getAmpsHeater() );
    isAlarm = true;
    OutputAlarm(txtAlarm);
  }
  
  // Low heater amps
  if ( (hotTubControl.getAmpsHeater() < 15) && (digitalRead(HEATER_ON_OFF_OUTPUT_PIN) == HIGH) )
  {
    sprintf_P(txtAlarm, PSTR("Low heater amps while heater is on = %d amps"), (int) hotTubControl.getAmpsHeater() );
    isAlarm = true;
    OutputAlarm(txtAlarm);
  }
/*
  if (isAlarm == false )
  {
    display.clearDisplay();  // clears the screen and buffer
    display.setCursor(0,0);
    display.println(F("Everything okay"));
    display.display();
  }
*/
  return isAlarm;
} // end CheckAlarms()


//============================================================================
// Print Alarm Text  
//============================================================================
void OutputAlarm(char AlarmText[])
{
  Serial.print(F("Alarm: "));
  Serial.println(AlarmText);
/*
  display.clearDisplay();  // clears the screen and buffer
  display.setCursor(0,0);
  display.println(AlarmText);
  display.display();
*/
} // OutputAlarm()



void PrintStatus()
{ 
//  PrintPumpInfo();
  PrintStatusAll();
} // PrintStatus()


void printHeaterInfo()
{
  static int8_t   cntHeading;  // prints new heading every 20 rows
  static uint32_t PrintDelay;
  
  if ((long)(millis() - PrintDelay) > 0)
  {
    PrintDelay = millis() + 500UL;

    //Print Pushbutton state and Led indicator state
    if(cntHeading > 30 || millis() < 1000)
    {
      Serial.println(F("Need\tstat\ttemp"));
      cntHeading = 0;
    }
    cntHeading++;
   
    Serial.print(NeedHeat());
    Serial.print("\t");
    Serial.print(needHeatStatus);
    Serial.print("\t");
    Serial.print(hotTubControl.getTempPreHeat());
    
    Serial.println();  
  }
  
}  // printHeaterInfo()


void PrintPumpInfo()
{

  static int8_t   cntHeading;  // prints new heading every 20 rows
  static uint32_t PrintDelay;
  
  if ((long)(millis() - PrintDelay) > 0)
  {
    PrintDelay = millis() + 500UL;

    //Print Pushbutton state and Led indicator state
    if(cntHeading > 30 || millis() < 1000)
    {
      Serial.println(F("PumpPn\tisPumpBtnOn()\tneedheat"));
      cntHeading = 0;
    }
    cntHeading++;
   
    Serial.print(digitalRead(PUMP_ON_OFF_OUTPUT_PIN));
    Serial.print("\t\t");
    Serial.print(hotTubControl.isPumpBtnOn());
    Serial.print("\t");
    Serial.print(needHeatStatus);
   
    
    Serial.println();  
  }
} // PrintPumpInfo()


//============================================================================
//============================================================================
void PrintStatusAll()
{
  static int8_t   cntHeading;  // prints new heading every 20 rows
  static uint32_t PrintDelay;
  
  if ((long)(millis() - PrintDelay) > 0)
  {
    PrintDelay = millis() + 500UL;

    //Print Pushbutton state and Led indicator state
    if(cntHeading > 40 || millis() < 1000)
    {
      Serial.println(F("OnOff\tPump\tHeat\tair\tsetPt\tT1\tT2\tT3\tpres\tampH\tampP\tampB\tRunTimeMin"));
      cntHeading = 0;
    }
    cntHeading++;
    
    Serial.print(hotTubControl.isHotTubBtnOn());
    Serial.print("\t");
    
    Serial.print(hotTubControl.isPumpBtnOn());
    Serial.print(digitalRead(PUMP_ON_OFF_OUTPUT_PIN));
    Serial.print(F("\t"));
    
    Serial.print(NeedHeat());
    Serial.print(needHeatStatus);
    Serial.print(digitalRead(HEATER_ON_OFF_OUTPUT_PIN));
    Serial.print(F("\t"));

    Serial.print(hotTubControl.isBubbleBtnOn());
    Serial.print(digitalRead(BUBBLER_ON_OFF_OUTPUT_PIN));
    Serial.print(F("\t"));

    Serial.print(hotTubControl.getTempSetpoint());
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
    Serial.print(millis()/60000UL);

    Serial.println("");
  }
}  // PrintStatusAll()









