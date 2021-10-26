// 2021-10-26 13:36:49.965797

#ifndef receiver.h
#define receiver.h

#include <Arduino.h>
#include <math.h>

#define RX_PIN 7  // input pin for wire from receiver
#define NO_OF_CHANNELS 8 // number of receiver channels
#define FRAME_CHANGE 3500  // must be less than time between last pulse in one frame and first pulse in next frame,
                                            // but more than maximum time between any consecutive pulses in the same frame,
                                            // measured using "receiver_pulse_test_time.ino" in microseconds (default: 5000)

class Receiver {
    private:
    unsigned long int current_time = 0;  // for calculating pulse separation time
    unsigned long int previous_time = 0;
    unsigned long int time_difference = 0;
    int channel = 0;
    int output_rx[NO_OF_CHANNELS+1] = {0};  // store receiver channel values

    public:
    Receiver();
    void ISR_read_ppm(void);
    int get_channel(void);
    void read_channels(int &rx_throttle, int &rx_roll, int &rx_pitch, int &rx_yaw,
                       int &rx_aux_1, int &rx_aux_2, int &rx_aux_3, int &rx_aux_4);
};

/*--------------------------------------------------------------------------------------------------------------------*/

Receiver::Receiver() {
    pinMode(RX_PIN, INPUT_PULLUP);
}

void Receiver::ISR_read_ppm(void) {
    current_time = micros();  // store current time in microseconds
    time_difference = current_time - previous_time;
    previous_time = current_time;  // update previous_time for next interrupt call
    if(time_difference > FRAME_CHANGE) {  // if time difference between pulses is >FRAME_CHANGEus, this indicates the start of a PPM frame (which are 20ms)
        channel = 1;  // therefore the next pulse time read will be channel 1
    }
    else {
        if((900 < time_difference) && (time_difference < 2100) && (channel < NO_OF_CHANNELS+1)) {  // PPM signals only valid between 900 and 2100us
            output_rx[channel] = time_difference;  // set channel value
        }
        channel += 1;  // next interrupt will calculate next channel
    }
}

int Receiver::get_channel(void) {
    return channel;
}

void Receiver::read_channels(int &rx_throttle, int &rx_roll, int &rx_pitch, int &rx_yaw,
                             int &rx_aux_1, int &rx_aux_2, int &rx_aux_3, int &rx_aux_4) {
    // reformat receiver values
    rx_throttle = output_rx[3];
    rx_roll = output_rx[1];
    rx_pitch = output_rx[2];
    rx_yaw = output_rx[4];
    rx_aux_1 = output_rx[5];
    rx_aux_2 = output_rx[6];
    rx_aux_3 = output_rx[7];
    rx_aux_4 = output_rx[8];
}

#endif