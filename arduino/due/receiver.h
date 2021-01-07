// 2021-01-06 14:54:00.454812

#ifndef receiver.h
#define receiver.h

#include <Arduino.h>
#include <PulsePosition.h>
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
    PulsePositionInput ppm_lib;
//    float output_rx[NO_OF_CHANNELS+1] = {0};  // store receiver channel values

    public:
    Receiver();
    int aux_channel_1 = 0;
    int aux_channel_2 = 0;
    int aux_channel_3 = 0;
    int aux_channel_4 = 0;
    void configure_receiver(void);
    void read_channels(int &rx_throttle, int &rx_roll, int &rx_pitch, int &rx_yaw);
};

/*--------------------------------------------------------------------------------------------------------------------*/

Receiver::Receiver() {
//    pinMode(RX_PIN, INPUT_PULLUP);
}

void Receiver::configure_receiver(void) {
    ppm_lib.begin(RX_PIN);
}

void Receiver::read_channels(int &rx_throttle, int &rx_roll, int &rx_pitch, int &rx_yaw) {
    rx_throttle = map(ppm_lib.read(3), RECEIVER_MIN, RECEIVER_MAX, THROTTLE_MIN, THROTTLE_MAX);
    rx_roll = map(ppm_lib.read(1), RECEIVER_MIN, RECEIVER_MAX, -ABSOLUTE_MAX_COPTER_ANGLE, ABSOLUTE_MAX_COPTER_ANGLE);
    rx_pitch = map(ppm_lib.read(2), RECEIVER_MIN, RECEIVER_MAX, -ABSOLUTE_MAX_COPTER_ANGLE, ABSOLUTE_MAX_COPTER_ANGLE);
    rx_yaw = map(ppm_lib.read(4), RECEIVER_MIN, RECEIVER_MAX, -ABSOLUTE_MAX_COPTER_ANGLE, ABSOLUTE_MAX_COPTER_ANGLE);
    aux_channel_1 = ceil(ppm_lib.read(5));
    aux_channel_2 = ceil(ppm_lib.read(6));
    aux_channel_3 = ceil(ppm_lib.read(7));
    aux_channel_4 = ceil(ppm_lib.read(8));
}

#endif
