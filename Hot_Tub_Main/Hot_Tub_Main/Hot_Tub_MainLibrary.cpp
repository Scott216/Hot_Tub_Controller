//
// Hot_Tub_MainLibrary.cpp 
// Library C++ code
// ----------------------------------
// Developed with embedXcode+ 
// http://embedXcode.weebly.com
//
// Project 		Hot_Tub_Main Library
//
// Created by 	Scott Goldthwaite, 1/22/14 4:28 PM
// 				Scott Goldthwaite
//
// Copyright 	© Scott Goldthwaite, 2014
// License		GNU General Public License
//
// See 			Hot_Tub_MainLibrary.cpp.h and ReadMe.txt for references
//


#include "Hot_Tub_MainLibrary.h"


HotTubControl::HotTubControl()
{ }


HotTubControl::~HotTubControl()
{ }


void HotTubControl::begin()
{
    // Setup I2C
    I2c.begin();  // Initialize I2C
    I2c.pullup(0); // turn off internal pullups (doesn't do anything on mega)
    I2c.timeOut(30000); // 30 second timeout
    
    oneWireBus.begin(); // Initialize OneWire
} // begin()


void HotTubControl::readPanelStatus()
{
    // Read pushbuttons status and temperature setpoint from user panel
    I2c.write(SLAVE_ID, CMD_ONOFF_BTN);
    delay(1);
    I2c.read(SLAVE_ID, 1);        // Request on/off button status
    _hotTubBtn = I2c.receive();   // Read the on/off button status from slave
    
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
    
} // readPanelStatus()


// Send status to panel: current water temp, hot tub on/off status, output state for: pump, bubbler, heater
void HotTubControl::writePanelStatus(float currentTemp, bool pumpState, bool bubbleState, bool heatState)
{
    
    int i2cWriteDelay = 1;  // srg temp - may not need delay.  was 15
    
    // Send data to User Panel via I2C bus
//    I2c.write( SLAVE_ID, ADR_ONOFF_STAT,  _hotTubBtn );   // srg, I don't think I need to send this to user panel unless an alarm shuts down Hot Tub
//    delay(i2cWriteDelay);
    I2c.write( SLAVE_ID, ADR_PUMP_STAT,   pumpState );
    delay(i2cWriteDelay);
    I2c.write( SLAVE_ID, ADR_BUBBLE_STAT, bubbleState );
    delay(i2cWriteDelay);
    I2c.write( SLAVE_ID, ADR_HEATER_STAT, heatState );
    delay(i2cWriteDelay);
    I2c.write( SLAVE_ID, ADR_TEMP_SETPT,  currentTemp );
    delay(i2cWriteDelay);
    
} // writePanelStatus()


// Read sensors
void HotTubControl::refreshSensors()
{
    readTemperature();
    readPressure();
    readAmps();

#ifdef TESTMODE
   // create dummy data when if connected to main controller board
   _tempPreheat =   98.0;
   _tempPostHeat = 110.0;
   _tempPump =     150.0;
   _ampsPump =       9.0;
   _ampsHeater =    22.0;
   _ampsBubbler =    5.0;
   _Pressure =      15.0;
#endif

} // refreshSensors()


// Get temperatures from 1-wire sensors
void HotTubControl::readTemperature()
{
    
    float validTemp;
    float PreHeaterTemp1;
    float PreHeaterTemp2;
    
    // Refresh temperatures
    oneWireBus.requestTemperatures();
    
    validTemp = oneWireBus.getTempF(&tempSensor[0][0]);
    if (validTemp > 40.0)
    { PreHeaterTemp1 = validTemp; }
    
    validTemp = oneWireBus.getTempF(&tempSensor[1][0]);
    if (validTemp > 40.0)
    { PreHeaterTemp2 = validTemp; }
    
    validTemp = oneWireBus.getTempF(&tempSensor[2][0]);
    if (validTemp > 40.0)
    { _tempPostHeat = validTemp; }
    
    validTemp = oneWireBus.getTempF(&tempSensor[3][0]);
    if (validTemp > 40.0)
    { _tempPump = validTemp; }
    
    if(abs(PreHeaterTemp1 - PreHeaterTemp2) < 4.0)
    {
        // Two probes are relatively close, just take the average for the temp
        _tempPreheat =  (PreHeaterTemp1 + PreHeaterTemp2) / 2.0;
    }
    else if(PreHeaterTemp1 > PreHeaterTemp2)    // Temp probes are not very close, choose higher one so heater doesn't stay on all the time
    { _tempPreheat =  PreHeaterTemp1; }
    else
    { _tempPreheat = PreHeaterTemp2; }
    
} // readTemperature()


// Read pressure gauge
// Average 25 samples
void HotTubControl::readPressure()
{
    float pressure = 0.0;
    int samples;
    
    for (samples = 0; samples < 25; samples++)
    {
        pressure += analogRead(PRESSURE_GAUGE);   // Get Pressure.  30 PSI Max, 4-20mA output
        delay(1);
    }
    
    // Calculate averages from samples
    _Pressure = ( pressure / (float)samples ) * 0.0377 - 7.5094 + 0.25;   // 0.25 PSI calibration offset
    
    // If value is close to zero, then set to zero
    if (_Pressure < 2.0) _Pressure = 0.0;
    
} // readPressure()


// Read amps sensors
// Average 25 samples
void HotTubControl::readAmps()
{
    // Take multiple samples and get average
    // Initialize variables
    float pump_amps =    0.0;
    float heater_amps =  0.0;
    float bubbler_amps = 0.0;
    int samples;

    for (samples = 0; samples < 25; samples++)
    {
        // Get Amps from CTs
        pump_amps    += analogRead(CT_PUMP);    // ADC value for pump is about 630
        heater_amps  += analogRead(CT_HEATER);  // Should read about 22.8 amps
        bubbler_amps += analogRead(CT_BUBBLER);
        delay(1);
    }
    
    // Calculate averages from samples
    _ampsPump =    (pump_amps    / (float) samples) * (20.0 / 1024.0) - 1.3;    // 20 Amp CT,  -1.30 amps calibration offset
    _ampsHeater =  (heater_amps  / (float) samples) * (50.0 / 1024.0) - 4.75;   // 50 Amp CTs, -4.75 amps calibration offset
    _ampsBubbler = (bubbler_amps / (float) samples) * (20.0 / 1024.0) - 1.0;    // 20 Amp CTs, -1.00 amps calibration offset
    // If values are close to zero, then set to zero
    if (_ampsPump    < 1.0) _ampsPump =    0.0;
    if (_ampsHeater  < 1.0) _ampsHeater =  0.0;
    if (_ampsBubbler < 0.5) _ampsBubbler = 0.0;
    
} // readAmps()


float HotTubControl::getTempPreHeat()
{ return _tempPreheat; }


float HotTubControl::getTempPostHeat()
{ return _tempPostHeat; }


float HotTubControl::getTempPump()
{ return _tempPump; }


float HotTubControl::getPressure()
{ return _Pressure; }


float HotTubControl::getAmpsPump()
{ return _ampsPump; }


float HotTubControl::getAmpsHeater()
{ return _ampsHeater; }


float HotTubControl::getAmpsBubbler()
{ return _ampsBubbler; }


// Returns state of On/Off pushbutton on control panel
bool HotTubControl::isHotTubBtnOn()
{ return _hotTubBtn; }


// Returns state of pump pushbutton on control panel
bool HotTubControl::isPumpBtnOn()
{ return _pumpBtn; }


// Returns state of bubbles pushbutton on control panel
bool HotTubControl::isBubbleBtnOn()
{ return _bubbleBtn; }


// returns the setpoint temp
byte HotTubControl::getTempSetpoint()
{ return _targetTemp; }



