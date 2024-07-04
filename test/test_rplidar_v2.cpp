#include "rplidar.h"
#define LED_BUILTIN 2

rplidarHandler rplidar; 

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  rplidar.init();
}

void loop() {
  rplidar.loop();
  if(rplidar.is_measurement_ready())
  {
    rplidar.get_measurement();
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  
  }
}
