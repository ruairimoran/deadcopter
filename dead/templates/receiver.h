// {{timestamp}}

#ifndef receiver.h
#define receiver.h

#include <Arduino.h>
#include <math.h>

#define RX_PIN {{receiver_pin}}  // input pin for wire from receiver
#define NO_OF_CHANNELS {{number_of_rx_channels}} // number of receiver channels
#define PULSE_GAPS_MEASURED 2*NO_OF_CHANNELS+1  // minimum needed to guarantee all channels read after first frame end
#define FRAME_CHANGE {{frame_change_time}}  // must be less than time between last pulse in one frame and first pulse in next frame,
                           // but more than maximum time between any consecutive pulses in the same frame,
                           // measured using "receiver_pulse_test_time.ino" in microseconds (default: 5000)
#define RECEIVER_MIN {{min_receiver_pwm}}  // minimum pwm input from receiver channel
#define RECEIVER_MAX {{max_receiver_pwm}}  // maximum pwm input from receiver channel
#define THROTTLE_MIN {{min_throttle_pwm}}  // minimum throttle input
#define THROTTLE_MAX {{max_throttle_pwm}}  // maximum throttle input
#define ABSOLUTE_MAX_COPTER_ANGLE {{max_allowed_tilt_degrees}}  // maximum angle the quadcopter can tilt from upright

class Receiver {
    private:
    unsigned long int current_time = 0;  // for calculating pulse separation time
    unsigned long int previous_time = 0;
    unsigned long int time_difference = 0;
    int output_rx[NO_OF_CHANNELS+1] = {0};  // store receiver channel values

    public:
    Receiver();
    int aux_channel_1 = 0;
    int aux_channel_2 = 0;
    int aux_channel_3 = 0;
    int aux_channel_4 = 0;
    void read_ppm(void);
    void read_channels(int &rx_throttle, int &rx_roll, int &rx_pitch, int &rx_yaw);
};

/*--------------------------------------------------------------------------------------------------------------------*/

Receiver::Receiver() {
    pinMode(RX_PIN, INPUT_PULLUP);
}

void Receiver::read_ppm(void) {
    current_time = micros();  // store current time in microseconds
    time_difference = current_time - previous_time;
    previous_time = current_time;  // update previous_time for next interrupt call
    if(time_difference>5000) {  // if time difference between pulses is >5000us, this indicates the start of a PPM frame (which are 20ms)
        channel = 1;  // therefore the next pulse time read will be channel 1
    }
    else {
        if((900<time_difference) && (time_difference<2100) && (channel<NO_OF_CHANNELS+1)) {  // PPM signals only valid between 900 and 2100us
            output_rx[channel] = time_difference;  // Set channel value
        }
        channel += 1;  // Next interrupt will calculate next channel
    }
}

void Receiver::read_channels(int &rx_throttle, int &rx_roll, int &rx_pitch, int &rx_yaw) {
    rx_throttle = map(output_rx[{{throttle_channel}}], RECEIVER_MIN, RECEIVER_MAX, THROTTLE_MIN, THROTTLE_MAX);
    rx_roll = map(output_rx[{{roll_channel}}], RECEIVER_MIN, RECEIVER_MAX, -ABSOLUTE_MAX_COPTER_ANGLE, ABSOLUTE_MAX_COPTER_ANGLE);
    rx_pitch = map(output_rx[{{pitch_channel}}], RECEIVER_MIN, RECEIVER_MAX, -ABSOLUTE_MAX_COPTER_ANGLE, ABSOLUTE_MAX_COPTER_ANGLE);
    rx_yaw = map(output_rx[{{rudder_channel}}], RECEIVER_MIN, RECEIVER_MAX, -ABSOLUTE_MAX_COPTER_ANGLE, ABSOLUTE_MAX_COPTER_ANGLE);
    aux_channel_1 = output_rx[5];
    aux_channel_2 = output_rx[6];
    aux_channel_3 = output_rx[7];
    aux_channel_4 = output_rx[8];
}

#endif
