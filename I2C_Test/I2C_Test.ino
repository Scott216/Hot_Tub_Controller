
#include "I2C.h"  // use for I2C communication  http://dsscircuits.com/articles/arduino-i2c-master-library.html

#define SLAVE_ID     46 // I2C address of LED backpack on user panel
#define I2C_SUCCESS   0 // When I2C read/writes are sucessful, function returns 0
#define MAX_I2C_BYTES 6 // Max I2C bytes to send data to slave

// I2C Commands to read data from LED backpack user panel
enum {
  CMD_SLAVE_ID   = 1,
  CMD_ONOFF_BTN  = 2,
  CMD_PUMP_BTN   = 3,
  CMD_BUBBLE_BTN = 4,
  CMD_TEMP_SETPT = 5
};



void setup()
{

  Serial.begin(9600);
  
  I2c.begin();  // Initialize I2C
  I2c.pullup(1); // turn off internal pullups (doesn't do anything on mega)
  I2c.timeOut(30000); // 30 second timeout


  
}

void loop()
{

  
  delay(200); // srg debug

  static uint32_t lastPumpOnTime; // Millis() Timestamp of when pump was last turned on.  Updates every cycle that pump should be on.  Used to keep pump from cycling on/off too fast

  // Read pushbuttons status and temperature setpoint from user panel
  int status = I2c.write(SLAVE_ID, CMD_ONOFF_BTN);
  if( status == I2C_SUCCESS )            // Set pointer to On/Off button status
  {
    I2c.read(SLAVE_ID, 1);                                           // request on/off button status
    int btnStatus = I2c.receive();    // Read the on/off button status
  }
  else
  {
    Serial.print(F("I2C Write Failed - on/off btn.  Status = "));
    Serial.print(status, HEX);
    Serial.print("  ");
    status = I2c.read(SLAVE_ID, 1);
    Serial.print(status, HEX );
    Serial.print("  ");
    Serial.println( I2c.receive());

  }

  

 
}


