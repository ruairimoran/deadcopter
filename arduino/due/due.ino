/*
     Servo Motor Control using the Arduino Servo Library
           by Dejan, https://howtomechatronics.com
*/
#include <Servo.h>  // 0 - 180 degrees mapped onto any servo range
Servo blue_servo_Left;  // create servo object to control a servo
Servo ESC_BackRight;  // create servo object to control an ESC

int potValue;  // value from the analog pin

void setup() {
  blue_servo_Left.attach(9,600,2450);  // (digital pin, min pulse length, max pulse length) // HXT900(x, 600, 2450) // DSservo(x, 600, 2500)
  ESC_BackRight.attach(10, 1000, 2000);  // (x, 1000, 2000)
}

void loop() {
  potValue = analogRead(A0);  // reads the value of the potentiometer (value between 0 and 1023)
  potValue = map(potValue, 0, 1023, 0, 180);  // scale it to use it with the servo library (value between 0 and 180)
  ESC_BackRight.write(potValue);  // Send the signal to the ESC
}
