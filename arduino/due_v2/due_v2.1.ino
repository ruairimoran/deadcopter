#include "debug.h"
#include "imu.h"
#include "vars.h"
#include "radio.h"
#include "motors.h"
#include "controller.h"
#include "fail_safes.h"
//#include <DueTimer.h>

void setup() {
  //Serial.begin(9600);
  debug_begin(115200);
  //RADIO_BEGIN(2000000);
  begin_imu();
  //start_buzzer();
  //get_radio_data();
  //delay(100);
  // while (!is_armed())
  // {
  //   get_radio_data();
  // }
  //attach_motors();
  buzz_100ms();
  buzz_100ms();
  buzz_100ms();
  arm();
  debug_println("Armed!");
  dt = micros();
}


///the loop should run at a fixed frequency
void loop() {
  dt = micros() - dt;
  if (kill()) {
    //don't read radio data once kill is activated, you should not me able to arm agai
    disarm();
    debug_println("KILL!!!");
    buzz_100ms();
    buzz_100ms();
    buzz_100ms();
    HALT;
  }
  if (get_orientation()) {
    //
    //    debug_prin/t16("\nyaw: ", orientation.heading, " pitch: ", orientation.pitch, " roll: ", orientation.roll, " q0: ", quaternions.q0, " q1: ", quaternions.q1, " q2: ", quaternions.q2, " q3: ", quaternions.q3, dt, "\n");
    madgwick_lib.computeAngles();
    Serial.print(orientation.pitch);
    Serial.print("\t");
    Serial.println(orientation.roll);
  }
  get_radio_data();
  generate_inputs();
  //debug_print16("\nY: ", radio_data.orientation.heading, " P: ", radio_data.orientation.pitch, " R: ", radio_data.orientation.roll, " T: ", radio_data.throttle, " S0: ", radio_data.switches[0], " S1: ", radio_data.switches[1], " S2: ", radio_data.switches[2], " S3: ", radio_data.switches[3]);
  //debug_print16(" S4: ", radio_data.switches[4], " S5: ", radio_data.switches[5], " S6: ", radio_data.switches[6], " S7: ", radio_data.switches[7], " S8: ", radio_data.switches[8], " S9: ", radio_data.switches[9], " S10: ", radio_data.switches[10], " S11: ", radio_data.switches[11]);
  //debug_print16("\nY: ", radio_data.orientation.heading, " P: ", radio_data.orientation.pitch, " R: ", radio_data.orientation.roll, " T: ", radio_data.throttle, " SA: ", radio_data.switches[5], " SB: ", radio_data.switches[4], " SC: ", radio_data.switches[0], " SD: ", radio_data.switches[6]);
  //  debug_print8("\nu1: ", inputs.u1, " u2: ", inputs.u2, " u3: ", inputs.u3," a", " b"/);
  //debug_print8("\nFL: ", motor_speeds.motor0, " FR: ",  motor_speeds.motor1, " BL: ",  motor_speeds.motor2, " BR: ",  motor_speeds.motor3);
  write_speed_to_esc();
  dt = micros();
}
