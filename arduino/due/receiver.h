// 2021-01-07 15:49:37.113125

#ifndef receiver.h
#define receiver.h

#include <Arduino.h>
#include <math.h>

#define RX_PIN 7  // input pin for wire from receiver
#define NO_OF_CHANNELS 8 // number of receiver channels
#define PULSE_GAPS_MEASURED 2*NO_OF_CHANNELS+1  // minimum needed to guarantee all channels read after first frame end
#define FRAME_CHANGE 5000  // must be less than time between last pulse in one frame and first pulse in next frame,
                           // but more than maximum time between any consecutive pulses in the same frame,
                           // measured using "receiver_pulse_test_time.ino" in microseconds (default: 5000)
#define RECEIVER_MIN 1070  // minimum pwm input from receiver channel
#define RECEIVER_MAX 1930  // maximum pwm input from receiver channel
#define THROTTLE_MIN 1150  // minimum throttle input
#define THROTTLE_MAX 1850  // maximum throttle input
#define ABSOLUTE_MAX_COPTER_ANGLE 40  // maximum angle the quadcopter can tilt from upright

class Receiver {
    private:
    unsigned long int current_time = 0;  // for calculating pulse separation time
    unsigned long int previous_time = 0;
    unsigned long int time_difference = 0;
    int read_rx[PULSE_GAPS_MEASURED] = {0};  // store time difference value on receiver pulse interrupt
    int decode_rx[PULSE_GAPS_MEASURED] = {0};  // store full set of time differences
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
    uint16_t now,diff;
    static uint16_t last = 0;
    static uint8_t chan = 1;
    now = micros();
    diff = now - last;
    last = now;
    if(diff>3000) chan = 1;
    else {
        if(900<diff && diff<2200 && chan<NO_OF_CHANNELS+1 ) {  //Only if the signal is between these values it is valid
            output_rx[chan] = diff;
        }
    chan++;
    }
}

void Receiver::read_channels(int &rx_throttle, int &rx_roll, int &rx_pitch, int &rx_yaw) {
    rx_throttle = map(output_rx[3], RECEIVER_MIN, RECEIVER_MAX, THROTTLE_MIN, THROTTLE_MAX);
    rx_roll = map(output_rx[1], RECEIVER_MIN, RECEIVER_MAX, -ABSOLUTE_MAX_COPTER_ANGLE, ABSOLUTE_MAX_COPTER_ANGLE);
    rx_pitch = map(output_rx[2], RECEIVER_MIN, RECEIVER_MAX, -ABSOLUTE_MAX_COPTER_ANGLE, ABSOLUTE_MAX_COPTER_ANGLE);
    rx_yaw = map(output_rx[4], RECEIVER_MIN, RECEIVER_MAX, -ABSOLUTE_MAX_COPTER_ANGLE, ABSOLUTE_MAX_COPTER_ANGLE);
    aux_channel_1 = output_rx[5];
    aux_channel_2 = output_rx[6];
    aux_channel_3 = output_rx[7];
    aux_channel_4 = output_rx[8];
}

#endif
