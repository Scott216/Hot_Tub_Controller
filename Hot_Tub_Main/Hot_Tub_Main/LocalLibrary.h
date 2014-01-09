

#ifndef Hot_Tub_Controller_LocalLibrary_h
#define Hot_Tub_Controller_LocalLibrary_h
#include <Arduino.h>
#include "I2C.h"  // use for I2C communication  http://dsscircuits.com/articles/arduino-i2c-master-library.html

#define SLAVE_ID     46 // I2C address of LED backpack on user panel
#define I2C_SUCCESS   0 // When I2C read/writes are sucessful, function returns 0
#define MAX_I2C_BYTES 6 // Max I2C bytes to send data to slave

// I2C Commands to read data from LED backpack user panel
enum {
  CMD_SAVE_ALL    =  1,  // writes all data from Mater to slave
  CMD_SLAVE_ID    =  2,
  CMD_ONOFF_BTN   =  3,
  CMD_PUMP_BTN    =  4,
  CMD_BUBBLE_BTN  =  5,
  CMD_TEMP_SETPT  =  6,
  ADR_ONOFF_STAT  =  7, // Addresses to save data to slave
  ADR_PUMP_STAT   =  8,
  ADR_BUBBLE_STAT =  9,
  ADR_HEATER_STAT = 10,
  ADR_TEMP_SETPT  = 11
};


class HotTubControl
{
public:
  HotTubControl();
  ~HotTubControl();
  void begin();
  void readPanelStatus();  // Read status of control panel via I2C
  void writePanelStatus(float currentTemp, bool pumpState, bool bubbleState, bool heatState); // Send update to panel
  bool needHeatt();        // True if need heat
  void refreshSensors();   // Read temp, pressure, amp sensors
  bool isHotTubBtnOn();    // Returns state of On/Off button on control panel
  bool isPumpBtnOn();      // Returns state of Pump button on control panel
  bool isBubbleBtnOn();    // Returns state of Bubble button on control panel
  byte getTempSetpoint();  // returns the setpoint temp
  

private:
  bool _needHeat;
  bool _hotTubBtn;
  bool _pumpBtn;
  bool _bubbleBtn;
  byte _targetTemp;
  

};

#endif




