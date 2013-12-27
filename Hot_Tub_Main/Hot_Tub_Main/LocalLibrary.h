

#ifndef Hot_Tub_Controller_LocalLibrary_h
#define Hot_Tub_Controller_LocalLibrary_h
#include <Arduino.h>


class HotTubControl
{
public:
  HotTubControl();
  ~HotTubControl();
  bool needHeatt();
  void refreshSensors();
  bool isHotTubBtnOn();
  bool isPumpBtnOn();
  bool isBubbleBtnOn();
  void setTempSetpoint(byte targetTemp);  // saves the setpoint temp, which is set from control panel
  byte getTempSetpoint();                 // returns the setpoint temp
  

private:
  bool _needHeat;
  bool _hotTubBtn;
  bool _pumpBtn;
  bool _bubbleBtn;
  byte _targetTemp;
  

};

#endif




