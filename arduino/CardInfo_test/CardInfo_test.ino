#include <SD.h>
// #include <EEPROM.h>

File file; //creating object 'file'
byte begin_ok; //boolean control variable

void setup()
{
  begin_ok = 1;
  Serial.begin(9600); //beginning serial comms
  delay(1000);
  if(!SD.begin()) // SD card test
  {
    Serial.println("Error in SD Card Inicialization!");
    begin_ok = 0;
    return;
  }
  Serial.println("Successful SD Card Inicialization!");
}

void loop()
{
  if (begin_ok)
  {
    int option = Serial.read();
    /* the var 'option' serves as a "control key". When the user wants to change the time between readings, he should send 'c' over the terminal.
When he wants to write the signal read on the ADC, he should send 'w'. If he wants to read what is written in the .txt file, he have to press 'r'. In the same way, if he want to
erase what's written on the file, he should send 'e' over the terminal. 'x' stops the current action.*/
    if (option == 'c') //to conf mode
    {
      Serial.println("You are in the Configuration Mode");
      option = Serial.read();
      while(option == -1)
      {
        option = Serial.read();
      }
      int valorConfig = option - 48; //the -48 is to format the ASCII values
//      EEPROM.write(0, valorConfig); // writing the delay between readings on EEPROM.
      Serial.print("Written value: ");
      Serial.println(valorConfig);
    }
    else if (option == 'w') 
    {
      while(option != 'x') //STOP the current action
      {
        int dataSensor = analogRead(0); // The pot is connected to A0 ADC and the data from it will be stored in dataSensor var.
        file = SD.open("sensor.txt", FILE_WRITE); //opening the .txt that will contain the data from ADC
        if(file)
        {
          file.print("Potentiometer values in: ");
          file.print(millis()); //for easily comprehension of the data, we will write the time (millis) when each sample is taken.
          file.print(" ms: ");
          file.println(dataSensor); //printing the values on dataSensor
          file.close(); //closing the .txt
//          delay(1000 * EEPROM.read(0));
          Serial.print("Data acquired in: ");
//          Serial.print(EEPROM.read(0), DEC);
          Serial.println(" segs.");
        }
        else
        {
          Serial.println("Error while opening the file for writing! Please check the restrictions!");
        }
        option = Serial.read();
      }
      Serial.println("Stopping to acquire data from Potentiometer");
    }
    else if (option == 'r') //read option
    {
      file = SD.open("sensor.txt");
      if (file)
      {
        while(file.available()) //loop to read all the data on sensor.txt
        {
          Serial.write(file.read());
        }
        file.close();
        Serial.println("File ending.");
      }
      else
      {
        Serial.println("Error while opening the file for reading!");
      }
    }
    else if (option == 'e') //ERASE command. Please be careful.
    {
      SD.remove("sensor.txt"); //removing ALL the data from sensor.txt. Please be Careful.
      if (!SD.exists("sensor.txt"))
      {
        Serial.println("File successfully erased!");
      }
      else
      {
        Serial.println("Error while erasing file!");
      }
    }
  }
}
