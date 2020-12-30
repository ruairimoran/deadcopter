#define RX_PIN 7  // input pin for wire from receiver
#define NO_OF_CHANNELS 8 // number of receiver channels
#define PULSE_GAPS_MEASURED 2*NO_OF_CHANNELS+1  // minimum needed to guarantee all channels read after first frame end
#define FRAME_CHANGE 5000  // must be less than time between last pulse in one frame and first pulse in next frame, 
                           // but more than maximum time between any consecutive pulses in the same frame,
                           // measured using "ppm_read_test.ino" in microseconds

unsigned long int current_time, previous_time, time_difference;  // for calculating pulse separation time
int read_rx[PULSE_GAPS_MEASURED], decode_rx[PULSE_GAPS_MEASURED], output_rx[NO_OF_CHANNELS+1];  // arrays to store values

void read_ppm() {
  static int i, j;  // counters
  // reads value from RC reciever PPM pin
  // gives channel values from 0 - 1000
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

void decode_ppm() {
  static int p, q, r;  // counters
  for (r=PULSE_GAPS_MEASURED-1; r>-1; r--) {
    if (decode_rx[r]>FRAME_CHANGE) {
      q = r;  // detect first separation space of >5000us in analysis array
    }
  }
  for (p=1; p<=NO_OF_CHANNELS; p++){
    output_rx[p] = (decode_rx[p+q] - 1000);  // output 8 channel values after first separation space
  }
}

void setup() {
Serial.begin(115200);
  pinMode(RX_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RX_PIN), read_ppm, RISING);  // enabling interrupt on pin RX_PIN
}

void loop() {
decode_ppm();

Serial.print(output_rx[1]);Serial.print("\t");
Serial.print(output_rx[2]);Serial.print("\t");
Serial.print(output_rx[3]);Serial.print("\t");
Serial.print(output_rx[4]);Serial.print("\t");
Serial.print(output_rx[5]);Serial.print("\t");
Serial.print(output_rx[6]);Serial.print("\t");
Serial.print(output_rx[7]);Serial.print("\t");
Serial.print(output_rx[8]);Serial.print("\n");

delay(100);
}
