//  Deadcopter is learning. Stay tuned.
// {{timestamp}}

#include <Arduino.h>
#include "actuators.h"

Esc esc_control;


void setup(){
    Serial.begin(115000);
    Serial.println("reading");
    Receiver.read();
    Serial.println("read done");
}
