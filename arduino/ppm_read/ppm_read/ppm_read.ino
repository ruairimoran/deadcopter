#define RX_PIN 7  // input pin for wire from receiver
#define NO_OF_CHANNELS 8 // number of receiver channels +1 so don't have to use 0
#define PULSE_GAPS_MEASURED 2*NO_OF_CHANNELS+1  // minimum needed to guarantee all channels read after first frame end

unsigned long int current_time, previous_time, time_difference;  // for calculating pulse separation time
int read_rx[PULSE_GAPS_MEASURED], decode_rx[PULSE_GAPS_MEASURED], output_rx[NO_OF_CHANNELS+1];  // arrays to store values
int i, j, k;  // loop counters

void read_ppm() {
  // reads value from RC reciever PPM pin
  // this code gives channel values from 0 - 1000 values 
  current_time = micros();  // store time value 'a' when pin value rising
  time_difference = current_time - previous_time;  // calculate time inbetween two risng edges
  previous_time = current_time;
  read_rx[i] = time_difference;  // store value in array
  i += 1;
  if(i==PULSE_GAPS_MEASURED) {
    for(j=0; j<PULSE_GAPS_MEASURED; j++) {
      decode_rx[j] = read_rx[j];  // copy all values from temporary array into analysis array after 15 readings  
    }
    i=0;
  }
}

void decode_ppm() {
  i = 0;
  j = 0;
  k = 0;
  for (k=PULSE_GAPS_MEASURED-1; k>-1; k--) {
    if (decode_rx[k]>5000) {
      j = k;  // detect first separation space of >5000us in analysis array
    }
  }
  for (i=1; i<=NO_OF_CHANNELS; i++){
    output_rx[i] = (decode_rx[i+j] - 1000);  // output 8 channel values after first separation space
  }
}

void setup() {
Serial.begin(115200);
  pinMode(RX_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RX_PIN), read_ppm, RISING);  // enabling interrupt on pin 7
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
