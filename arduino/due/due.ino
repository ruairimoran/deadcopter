//  Deadcopter is learning. Stay tuned.
// 2020-12-04 18:58:27.259198

#include <Arduino.h>
#include "actuators.h"

Esc esc_control;


void setup(){
    Serial.begin(115000);
    Serial.println("reading");
    Receiver.read();
    Serial.println("read done");
}