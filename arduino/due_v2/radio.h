#include "Arduino.h"
#include "vars.h"
#include "config.h"
#include "gen_functions.h"

#define MAP(value_to_map, from_min, from_max, to_min, to_max) \
  (value_to_map - from_min) * (to_max - to_min) / (from_max - from_min) + to_min
#define CAP(value_to_cap, min_val, max_val) \
  (value_to_cap < min_val) ? min_val : (value_to_cap > max_val ? max_val : value_to_cap)
#define RADIO_BEGIN(baud) RADIO_SERIAL.begin(baud);


float _invNorm;
int data[16];

void __to_quaternions() {
  _to_quaternions(radio_data.orientation, DEG_TO_RAD, &radio_quaternions);
  _invNorm = _invSqrt(radio_quaternions.q0 * radio_quaternions.q0 + radio_quaternions.q1 * radio_quaternions.q1
                      + radio_quaternions.q2 * radio_quaternions.q2 + radio_quaternions.q3 * radio_quaternions.q3);
  radio_quaternions.q0 *= _invNorm;
  radio_quaternions.q1 *= _invNorm;
  radio_quaternions.q2 *= _invNorm;
  radio_quaternions.q3 *= _invNorm;
}

void _process_radio_data() {
  radio_data.orientation.heading = CAP(MAP(radio_data.orientation.heading, RECEIVER_YAW_MIN, RECEIVER_YAW_MAX, -COPTER_YAW_LIMIT, COPTER_YAW_LIMIT),  -COPTER_YAW_LIMIT, COPTER_YAW_LIMIT);
  radio_data.orientation.pitch = CAP(MAP(radio_data.orientation.pitch, RECEIVER_PITCH_MIN, RECEIVER_PITCH_MAX, -COPTER_PITCH_LIMIT, COPTER_PITCH_LIMIT), -COPTER_PITCH_LIMIT, COPTER_PITCH_LIMIT);
  radio_data.orientation.roll = CAP(MAP(radio_data.orientation.roll, RECEIVER_ROLL_MIN, RECEIVER_ROLL_MAX, -COPTER_ROLL_LIMIT, COPTER_ROLL_LIMIT), -COPTER_ROLL_LIMIT, COPTER_ROLL_LIMIT);
  radio_data.throttle = CAP(MAP(radio_data.throttle, RECEIVER_THROTTLE_MIN, RECEIVER_THROTTLE_MAX, 0, COPTER_THROTTLE_MAX), 0, COPTER_THROTTLE_MAX);
}

void _read_pi_data(void) {
  if (Serial.available() > 0) {
    String dataFromClient = Serial.readStringUntil('\n');

    for (int i = 0; i < 16; i++) {
      // take the substring from the start to the first occurence of a comma, convert it to int and save it in the array
      data[i] = dataFromClient.substring(1, dataFromClient.indexOf(",")).toInt();

      //cut the data string after the first occurence of a comma
      dataFromClient = dataFromClient.substring(dataFromClient.indexOf(",") + 1);
    }
  }
}

void get_radio_data() {
  _read_pi_data();

  radio_data.orientation.heading = data[0]; //old index 3
  radio_data.orientation.pitch = data[1];  //old index 1
  radio_data.orientation.roll = data[3];  //old index 0
  radio_data.throttle = data[2];
  radio_data.switches[0] = data[4];
  radio_data.switches[1] = data[5];
  radio_data.switches[2] = data[6];
  radio_data.switches[3] = data[7];
  radio_data.switches[4] = data[8];
  radio_data.switches[5] = data[9];
  radio_data.switches[6] = data[10];
  radio_data.switches[7] = data[11];
  radio_data.switches[8] = data[12];
  radio_data.switches[9] = data[13];
  radio_data.switches[10] = data[14];
  radio_data.switches[11] = data[15];

  _process_radio_data();
  __to_quaternions();
}
