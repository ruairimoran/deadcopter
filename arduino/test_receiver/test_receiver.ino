#define channumber 8 //Cuantos canales tiene tu radio???????/How many channels have your radio???
#define filter 10 // Filtro anti salto/ Glitch Filter
int channel[channumber]; //Valores de canales leidos/ readed Channel values
int lastReadChannel[channumber]; //Ultima lectura obtenida/ Last  values readed
int conta=0; //Contador/couter


void setup()
{
 Serial.begin(9600); //Iniciamos com serial/ Serial Begin
 pinMode(4, INPUT); //Patita 4 como entrada / Pin 4 as input
 pinMode(13, OUTPUT); // Led pin 13
}

void loop()
{

 if(pulseIn(4, HIGH) > 3000) //Si el pulso del pin 4 es > 3000 usegundos continua /If pulse > 3000 useconds, continues
 {
   for(int i = 0; i <= channumber-1; i++) //lee los pulsos de los canales / Read the pulses of the channels
   {
     channel[i]=pulseIn(4, HIGH);
   }
   for(int i = 0; i <= channumber-1; i++) //Promedia los pulsos/Average the pulses
   {
     if((channel[i] > 2000) || (channel[i] <100))//Si se pasa del rango envia ultimo pulso/ If channel > max range, chage the value to the last pulse
     {
      channel[i]= lastReadChannel[i];
     }
     else
     {
     channel[i]=(lastReadChannel[i]+channel[i])/2; //Promedio el pulso pasado con el nuevo pulso/Average the last pulse eith the current pulse
     conta++; //Incrementa el contador/ increment counter
     }
   }

   }
   if(conta > filter)//Si el contador es mayor al filtro imprime valores/ If counter is > than filter, then prints values
   {
     for(int i = 0; i <= channumber-1; i++) //Ciclo para imprimir valores/Cycle to print values
     {
       Serial.print("CH"); //Canal/Channel
       Serial.print(i+1); // Numero del canal / Channel number
       Serial.print(": "); // que te importa
       Serial.println(channel[i]);
       lastReadChannel[i]=channel[i];
     }
     if(channel[4] > 1000) //si el canal 5 tiene un rango mayor a 500 enciende el LED/ If channel 5 is > than 500 turn on the led
     {
       digitalWrite(13, HIGH);
     }
     else
     {
       digitalWrite(13, LOW);//Si no lo apaga/If not turn it off
     }
     delay(400); //Delay
     conta=0;//Reinicia el contador/ Restart couter.
   }
 }
