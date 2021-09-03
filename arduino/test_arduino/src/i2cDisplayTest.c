/*  simple test for the stm8 i2c 7segment display
 *  notes for STM8 :
 *  -> STM8 doesn't have pullups on PB4/PB5 (SCL/SDA), so I2C needs external pullups
 *  -> STM8S105 : I2C needs to be enabled via OPT2.AFR6 byte
*/

// for test on a STM8S105

#include "Arduino.h"
#include "Wire.h"
void setup() {
  Wire_begin();        // join i2c bus (address optional for master)
  Serial_begin(115200);  // start serial for output
  Wire_beginTransmission(0x70); 
  Wire_write_s(""); // send no data to clear the display -> works!
  Wire_write_s("gogo");
  Wire_endTransmission();

  Serial_println_s("check the display!");
  pinMode (LED_BUILTIN,OUTPUT);
  digitalWrite(LED_BUILTIN,HIGH);
}
char dispText[4];
uint8_t dispDigit = 0;
uint32_t blinkMillis;
void loop() {
  // simple blinkie
  if ((millis() - blinkMillis) > 500) {
    digitalWrite(LED_BUILTIN,!digitalRead(LED_BUILTIN));
    blinkMillis = millis();
    Wire_beginTransmission(0x70); 
    Wire_write (dispDigit+0x30);
    Wire_endTransmission();
    dispDigit += 1;
    if (dispDigit >= 10) dispDigit = 0;
  }


/*  
  for (int i=0;i<10000;i++) {
    Wire_beginTransmission(0x70); 
    Wire_write(itoa(i,dispText,10));
    Wire_endTransmission();
    delay(20);    
  }
*/  
}

