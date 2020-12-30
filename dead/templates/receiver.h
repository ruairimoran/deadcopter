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
    unsigned long int current_time, previous_time, time_difference;  // for calculating pulse separation time
    int read_rx[PULSE_GAPS_MEASURED], decode_rx[PULSE_GAPS_MEASURED], output_rx[NO_OF_CHANNELS+1];  // arrays to store values

    public:
    Receiver();
    int aux_channel_1, aux_channel_2, aux_channel_3, aux_channel_4;
    void read_ppm(void);
    void decode_ppm(int &rx_throttle, int &rx_roll, int &rx_pitch, int &rx_yaw);
};

/*--------------------------------------------------------------------------------------------------------------------*/

Receiver::Receiver() {
    pinMode(RX_PIN, INPUT_PULLUP);
}

void Receiver::read_ppm(void) {
    static int i, j;  // counters
    // reads frames from RC receiver PPM pin
    // gives channel values from 1000 - 2000
    current_time = micros();  // store current time value when pin value rising
    time_difference = current_time - previous_time;  // calculate time between two rising edges
    previous_time = current_time;
    read_rx[i] = time_difference;  // store value in array
    i += 1;
    if(i==PULSE_GAPS_MEASURED) {
        for(j=0; j<PULSE_GAPS_MEASURED; j++) {
            decode_rx[j] = read_rx[j];  // copy all values from temporary array into analysis array after PULSE_GAPS_MEASURED readings
        }
        i=0;
    }
}

void Receiver::decode_ppm(int &rx_throttle, int &rx_roll, int &rx_pitch, int &rx_yaw) {
    static int p, q, r;  // counters
    for (r=PULSE_GAPS_MEASURED-1; r>-1; r--) {
        if (decode_rx[r]>FRAME_CHANGE) {
            q = r;  // detect first separation space of >FRAME_CHANGEus in analysis array
        }
    }
    for (p=1; p<=NO_OF_CHANNELS; p++){
        output_rx[p] = decode_rx[p+q];  // output 8 channel values after first separation space
    }
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
