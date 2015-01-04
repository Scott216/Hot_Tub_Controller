#ifndef Hot_Tub_MainLibrary_h
#define Hot_Tub_MainLibrary_h

#include <Arduino.h>
#include <OneWire.h>           // http://www.pjrc.com/teensy/td_libs_OneWire.html  http://playground.arduino.cc/Learning/OneWire
#include <DallasTemperature.h> // http://milesburton.com/index.php?title=Dallas_Temperature_Control_Library
#include <I2C.h>               // use for I2C communication  http://dsscircuits.com/articles/arduino-i2c-master-library.html


const int SLAVE_ID =     46; // I2C address of LED backpack on user panel
const int I2C_SUCCESS =   0; // When I2C read/writes are sucessful, function returns 0
const int MAX_I2C_BYTES = 6; // Max I2C bytes to send data to slave

// Define I/0 Pins
// I/O used: Analog: 0,1,2,3,6   Digital: 3,4,5,6,7 
//=== Analog Inputs for CTs measuring current ===
#define CT_PUMP       0  // Pump amps input 20 Amp CT
#define CT_HEATER     1  // Heater amps, 50 Amp CT
#define CT_BUBBLER    3  // Bubbler amps, 20 Amp CT

//===Analog Inputs from Pressure===
#define PRESSURE_GAUGE 6  // Pressure, comes from 4-20mA gauge

//=== Output pins for motor relays ===
#define AUX_OUTPUT_PIN              8
#define PUMP_ON_OFF_OUTPUT_PIN      7
#define HEATER_ON_OFF_OUTPUT_PIN    6
#define BUBBLER_ON_OFF_OUTPUT_PIN   5

#define ONE_WIRE_BUS 3 // OneWire data pin
#define OLED_RESET 4


// I2C Commands to read data from LED backpack user panel
enum 
{
  CMD_SAVE_ALL    =  1,  // writes all data from Mater to slave - not used
  CMD_SLAVE_ID    =  2,  // commands slave to return slave ID
  CMD_ONOFF_BTN   =  3,  // commands slave to return on/off button status
  CMD_PUMP_BTN    =  4,  // commands slave to return pump button status
  CMD_BUBBLE_BTN  =  5,  // commands slave to return bubble button status
  CMD_TEMP_SETPT  =  6,  // commands slave to return  setpoint temperature
  ADR_ONOFF_STAT  =  7, // Register addresses for on/off status
  ADR_PUMP_STAT   =  8, // Register addresses for pump status
  ADR_BUBBLE_STAT =  9, // Register addresses for bubbler status
  ADR_HEATER_STAT = 10, // Register addresses for heater status
  ADR_TEMP_SETPT  = 11  // Register addresses for temperature setpoint
};

// link to OneWire temp sensors defined in Hot_Tub_Main.ino
extern OneWire oneWire;
extern DallasTemperature oneWireBus;

// Address IDs for OneWire temperature sensors
static uint8_t tempSensor[4][8] =
{
  { 0x28, 0x0A, 0x48, 0x00, 0x05, 0x00, 0x00, 0x33 },  // pre heater temp sensor 1, lower sensor
  { 0x10, 0x33, 0x71, 0x6A, 0x02, 0x08, 0x00, 0x5D },  // pre heater temp sensor 2, upper sensor
  { 0x28, 0x20, 0xCE, 0xB3, 0x04, 0x00, 0x00, 0xAF },  // post heater temp sensor
  { 0x28, 0x1F, 0xA2, 0xB2, 0x04, 0x00, 0x00, 0xD9 }   // Pump temp
};


class HotTubControl
{
public:
  HotTubControl();
  ~HotTubControl();
  void begin();
  void readPanelStatus();  // Read status of control panel via I2C
  void writePanelStatus(float currentTemp, bool pumpState, bool bubbleState, bool heatState); // Send update to panel
  void refreshSensors();   // Read temp, pressure, amp sensors
  bool isHotTubBtnOn();    // Returns state of On/Off button on control panel
  bool isJetsBtnOn();      // Returns state of Pump button on control panel
  bool isBubbleBtnOn();    // Returns state of Bubble button on control panel
  byte getTempSetpoint();  // Returns the setpoint temp
  float getTempPreHeat();  // Returns Pre-heater temperature
  float getTempPostHeat(); // Returns Post-heater temperature
  float getTempPump();     // Returns pump housing temperature
  float getPressure();     // Returns water pressure
  float getAmpsPump();     // Returns pump amps
  float getAmpsHeater();   // Returns heater amps
  float getAmpsBubbler();  // Returns bubbler amps


private:
  bool  _needHeat;
  bool  _hotTubBtn;
  bool  _pumpBtn;
  bool  _bubbleBtn;
  byte  _targetTemp;
  float _tempPreheat;
  float _tempPostHeat;
  float _tempPump;
  float _Pressure;
  float _ampsHeater;
  float _ampsPump;
  float _ampsBubbler;
  void  readTemperature();  // Reads the OneWire temp sensors
  void  readPressure();     // Reads the pressure sensor
  void  readAmps();         // Reads the amps sensors
  
};

#endif  //Hot_Tub_MainLibrary_h



#ifndef Hot_Tub_Main_RELEASE
#define Hot_Tub_Main_RELEASE 100
#endif
