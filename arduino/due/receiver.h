// 2020-12-25 18:45:00.655180

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
#define RECEIVER_MIN 1000  // minimum pwm input from receiver channel
#define RECEIVER_MAX 2000  // maximum pwm input from receiver channel
#define ABSOLUTE_MAX_COPTER_ANGLE 30  // maximum angle the quadcopter can tilt from upright

class Receiver {
    private:
    static unsigned long int current_time, previous_time, time_difference;  // for calculating pulse separation time
    static int read_rx[PULSE_GAPS_MEASURED], decode_rx[PULSE_GAPS_MEASURED], output_rx[NO_OF_CHANNELS+1];  // arrays to store values

    public:
    Receiver();
    int rx_throttle, rx_rudder, rx_pitch, rx_roll, aux_channel_1, aux_channel_2, aux_channel_3, aux_channel_4;
    void read_ppm(void);
    void decode_ppm(void);
}

/*--------------------------------------------------------------------------------------------------------------------*/

Receiver::Receiver() {
    pinMode(RX_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(RX_PIN), read_ppm, RISING);  // enabling interrupt on pin RX_PIN
    decode_ppm();
}

void Receiver::read_ppm(void) {
    static int i, j;  // counters
    // reads frames from RC receiver PPM pin
    // gives channel values from 1000 - 2000
    current_time = micros();  // store current time value when pin value rising
    time_difference = current_time - previous_time;  // calculate time between two risng edges
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

void Receiver::decode_ppm(void) {
    static int p, q, r;  // counters
    for (r=PULSE_GAPS_MEASURED-1; r>-1; r--) {
        if (decode_rx[r]>FRAME_CHANGE) {
            q = r;  // detect first separation space of >FRAME_CHANGEus in analysis array
        }
    }
    for (p=1; p<=NO_OF_CHANNELS; p++){
        output_rx[p] = decode_rx[p+q];  // output 8 channel values after first separation space
    }
    rx_throttle = output_rx[1];
    rx_rudder = map(output_rx[2], RECEIVER_MIN, RECEIVER_MAX, -180, 180) * DEG_TO_RAD;
    rx_pitch = map(output_rx[3], RECEIVER_MIN, RECEIVER_MAX, -ABSOLUTE_MAX_COPTER_ANGLE, ABSOLUTE_MAX_COPTER_ANGLE) * DEG_TO_RAD;
    rx_roll = map(output_rx[4], RECEIVER_MIN, RECEIVER_MAX, -ABSOLUTE_MAX_COPTER_ANGLE, ABSOLUTE_MAX_COPTER_ANGLE) * DEG_TO_RAD;
    aux_channel_1 = output_rx[5];
    aux_channel_2 = output_rx[6];
    aux_channel_3 = output_rx[7];
    aux_channel_4 = output_rx[8];
}

#endif