
int data[16];

void setup() {
  Serial.begin(9600);
}
void loop() {
  if (Serial.available() > 0) {
    String dataFromClient = Serial.readStringUntil('\n');
    Serial.println(dataFromClient);

        for(int i = 0;i < 16 ;i++)
        {
          // take the substring from the start to the first occurence of a comma, convert it to int and save it in the array
          data[ i ] = dataFromClient.substring(1,dataFromClient.indexOf(",")).toInt();

          //cut the data string after the first occurence of a comma
          dataFromClient = dataFromClient.substring(dataFromClient.indexOf(",")+1);
        }
    Serial.println(data[0]);
    Serial.println(data[1]);
    Serial.println(data[2]);
    Serial.println(data[3]);
  }
}
