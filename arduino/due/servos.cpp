/*
  Servo Motor Control using the Arduino Servo Library
*/

#include <servos.h>


void setup() {
  9g_servo_Left.attach(8,600,2450);  // (digital pin, min pulse length, max pulse length) // HXT900(x, 600, 2450) // DSservo(x, 600, 2500)
  9g_servo_Right.attach(9,600,2450);
  25g_servo_Left.attach(10,600,2500);
  25g_servo_Right.attach(11,600,2500);
}

void loop() {
  9g_servo_Left.write(0);  // tell servo to go to a particular angle
  delay(1000);
  
  9g_servo_Left.write(90);              
  delay(1000); 
  
  9g_servo_Left.write(180);              
  delay(1000);
  
  9g_servo_Left.write(0);              
  delay(1000);                     
}
