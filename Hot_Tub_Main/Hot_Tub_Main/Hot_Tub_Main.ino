
// To do: output for OLED display


// === Libraries ===
#include "Arduino.h"
#include <OneWire.h>           // http://www.pjrc.com/teensy/td_libs_OneWire.html  http://playground.arduino.cc/Learning/OneWire
#include <DallasTemperature.h> // http://milesburton.com/index.php?title=Dallas_Temperature_Control_Library
#include <I2C.h>               // use for I2C communication  http://dsscircuits.com/articles/arduino-i2c-master-library.html
#include <SSD1306_I2C_DSS.h>   // Library for OLED display http://github.com/Scott216/SSD1306_I2C_DSS
#include <Adafruit_GFX.h>      // Library for OLED display http://github.com/adafruit/Adafruit-GFX-Library
#include "LocalLibrary.h"


// Create HotTubController instance
HotTubControl hotTubControl;

// Setup timer variables 
const uint32_t ALARM_CHECK_INTERVAL =     5000;  // Check for alarm every 5 seconds
      uint32_t last_alarm_check =            0;  // Timer used to check alarms every few seconds
const uint32_t SENSOR_CHECK_INTERVAL =    1000;  // Check sensor every second
      uint32_t last_sensor_check =           0;  // Timer used to read sensor values every second
const uint32_t HEATER_ON_DELAY =         30000;  // Delay before heaters can be turned on again after being off
const uint32_t PUMP_ON_DELAY =            5000;  // Delay before pump can be turned on again after being off
const uint32_t BUBBLER_ON_DELAY =          700;  // Delay before bubbler can be turned on again after being off
      uint32_t last_bubbler_on_check =       0;  // Timer used to prevent bubbler from being switched on too quickly
const uint32_t HEATER_COOLDOWN_DELAY =   10000;  // When heater is turned off, keep pump running a few seconds to help heating coils cool down
      uint32_t heater_cooldown_timer =       0;  // Timer used to keep pump on for a few seconds after heater has turned off
      uint32_t heaterChangeTime =            0;  // Records time heater state changed.  Used to keep heat on or off for at least 3 minutes


bool needHeatStatus = false;  // Saves NeedHeat() status, used so the NeedHeat() function is not called every time sketch wants to know if it needs heat

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

  // Initialize OLED display
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x32)
  display.setRotation(0);  // Orientation 0 = right side up, 2 = upside down
  display.clearDisplay();  // clears the screen and buffer
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.println(F("Finished setup()"));
  display.display();

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

    // If heater has changed state in the last 3 minutes or less, don't check yet. This is used to keep heater from going on/off too quickly
    if ( (long)(millis() - heaterChangeTime ) > 3UL * 60000UL )
    { needHeatStatus = NeedHeat(); } // 3 mintues has passed, okay to check NeedHeat()
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
  if((hotTubControl.isHotTubBtnOn() == LOW && (long)(millis() - heater_cooldown_timer) > 0) ||
     (hotTubControl.isPumpBtnOn() == LOW && needHeatStatus == false && (long)(millis() - heater_cooldown_timer) > 0) )
  {
    digitalWrite(PUMP_ON_OFF_OUTPUT_PIN, LOW);
  }
  
  // Turn on pump
  // Pump is turned on either by manually by pushbutton or by the heater
  if((hotTubControl.isHotTubBtnOn() == HIGH) &&                          // Hot tub must be on
     ((hotTubControl.isPumpBtnOn() == HIGH) || (needHeatStatus == true)) &&   // Pushbutton OR Need heat
     ((long)(millis() - lastPumpOnTime) > PUMP_ON_DELAY))                           // Delay before pump can be turned on again after being off, prevents fast cycling if there is a problem
  {
    digitalWrite(PUMP_ON_OFF_OUTPUT_PIN, HIGH);
    lastPumpOnTime = millis();
  }
    
  
  // Turn on heater
  // Prerequisites: Hot Tub On + Pump amps and pressure are above threshold + Low water temp + Delay from last time on
  if((hotTubControl.isHotTubBtnOn() == HIGH) &&
     (digitalRead(PUMP_ON_OFF_OUTPUT_PIN) == HIGH) &&
     (hotTubControl.getAmpsPump() >= PUMP_AMPS_THRESHOLD) &&
     (hotTubControl.getPressure() >= PUMP_PRESSURE_THRESHOLD) &&
     (needHeatStatus == true)) 
  {
    // If heater is changing state from off to on, reset heaterChangeTime 
    if ( digitalRead(HEATER_ON_OFF_OUTPUT_PIN) == LOW )
    { heaterChangeTime = millis(); }       
    digitalWrite(HEATER_ON_OFF_OUTPUT_PIN, HIGH);
    Serial.print("Millis 2: "); Serial.println(millis()); // srg debug
    delay(300); // srg testing pump cycle
  }
  
  // If heater is on, update last heater on check, and heater_cooldown_timer
  if(digitalRead(HEATER_ON_OFF_OUTPUT_PIN) == HIGH)
  { heater_cooldown_timer = millis() + HEATER_COOLDOWN_DELAY; }
  
  
  // Turn off heater
  // Turn off when water temp reaches setpoint or if heater amps is too high or Hot Tub is turned off
  // Or Pump pressure is too low, or pump amps is too low.
  if((needHeatStatus == false) ||
     (hotTubControl.getAmpsHeater() >= ALARM_HEATER_AMPS_HIGH) ||
     (hotTubControl.getAmpsPump() < PUMP_AMPS_THRESHOLD) ||
     (hotTubControl.getPressure() < PUMP_PRESSURE_THRESHOLD) ||
     (hotTubControl.isHotTubBtnOn() == LOW))
  {
    // If heater is changing state from on to off, reset heaterChangeTime
    if ( digitalRead(HEATER_ON_OFF_OUTPUT_PIN) == HIGH )
    { 
      heaterChangeTime = millis(); 
      needHeatStatus = false; // Set needHeatStatus to false since it won't be checked for 3 minutes  srg - not sure if I need this
    }
    digitalWrite(HEATER_ON_OFF_OUTPUT_PIN, LOW);  // turn off heater
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
  { digitalWrite(BUBBLER_ON_OFF_OUTPUT_PIN, LOW); }

  // Update user panel with hot tub info: Water temp, Motor on/off state
  hotTubControl.writePanelStatus( hotTubControl.getTempPreHeat(),
                                  digitalRead(PUMP_ON_OFF_OUTPUT_PIN),
                                  digitalRead(BUBBLER_ON_OFF_OUTPUT_PIN),
                                  digitalRead(HEATER_ON_OFF_OUTPUT_PIN) );

}  // loop()



// Determine if hot tub heater should be turned on
// If heater is on, keep it on until temp goes above upper limit
// if heater is off, keep it off until temp drops below lower limit
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

  if (isAlarm == false )
  {
    display.clearDisplay();  // clears the screen and buffer
    display.setCursor(0,0);
    display.println(F("Everything okay"));
    display.display();
  }

  return isAlarm;

} // end CheckAlarms()



//*********************************************************************************
// Print Alarm Text  
//*********************************************************************************
void OutputAlarm(char AlarmText[])
{
  Serial.print(F("Alarm: "));
  Serial.println(AlarmText);

  display.clearDisplay();  // clears the screen and buffer
  display.setCursor(0,0);
  display.println(AlarmText);
  display.display();

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
    Serial.print(needHeatStatus);
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
    Serial.print((millis() - heaterChangeTime)/1000UL);
    
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









