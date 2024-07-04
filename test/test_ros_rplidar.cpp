
#include "ros_node.h"

airlab::Node car_node;

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  car_node.init();
}

void loop() {
 car_node.loop();
}
