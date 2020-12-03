// {{timestamp}}


#ifndef actuators.h
#define actuators.h


#include <Arduino.h>
#include <Servo.h>  // must map range onto 0 - 180 degrees


class ESCs {
    private:
    Servo ESC_FrontLeft;  // create servo object to control an ESC
    Servo ESC_FrontRight;
    Servo ESC_BackLeft;
    Servo ESC_BackRight;
    double mapped_rotor_speed_FrontLeft;
    double mapped_rotor_speed_FrontRight;
    double mapped_rotor_speed_BackLeft;
    double mapped_rotor_speed_BackRight;

    public:
    ESCs();
    int rotor_speed_FrontLeft;
    int rotor_speed_FrontRight;
    int rotor_speed_BackLeft;
    int rotor_speed_BackRight;


}

/*--------------------------------------------------------------------------------------------------------------------*/

ESCs::ESCs(){

}

void attach_ESC_to_PWM_pin(void){
    ESC_FrontLeft.attach({{FrontLeft_pin}}, 1150, 1850);  // (PWN pin, min (1000), (2000))
    ESC_FrontRight.attach({{FrontRight_pin}}, 1150, 1850);  // 1150 so props idle
    ESC_BackRight.attach({{BackLeft_pin}}, 1150, 1850);  // 1850 so full throttle still allows room for LQR control
    ESC_BackLeft.attach({{BackRight_pin}}, 1150, 1850);
}

void map_to_servo_range(rotor_speed_FrontLeft, rotor_speed_FrontRight, rotor_speed_BackLeft, rotor_speed_BackRight){
    mapped_rotor_speed_FrontLeft = map(rotor_speed_FrontLeft, 1000, 2000, 0, 180);  // scale it to use it with the servo library (value between 0 and 180)
    mapped_rotor_speed_FrontRight = map(rotor_speed_FrontRight, 1000, 2000, 0, 180);
    mapped_rotor_speed_BackLeft = map(rotor_speed_BackLeft, 1000, 2000, 0, 180);
    mapped_rotor_speed_BackRight = map(rotor_speed_BackRight, 1000, 2000, 0, 180);
}

void write_speed_to_ESC(rotor_speed_FrontLeft, rotor_speed_FrontRight, rotor_speed_BackLeft, rotor_speed_BackRight){
    ESC_FrontLeft.write(rotor_speed_FrontLeft);  // send the signal to the ESC
    ESC_FrontRight.write(rotor_speed_FrontRight);
    ESC_BackLeft.write(rotor_speed_BackLeft);
    ESC_BackRight.write(rotor_speed_BackRight);
}