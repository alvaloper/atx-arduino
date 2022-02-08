#include <Wire.h>
#include <LCD.h>
#include <LiquidCrystal_I2C.h>
#include <OneWire.h> 
#include <DallasTemperature.h>
#include <PID_v1.h>


#define I2C_ADDR    0x27
#define ONE_WIRE_BUS 13 // Data wire is plugged into pin 7 on the Arduino // Pin receptor de la DATA

OneWire oneWire(ONE_WIRE_BUS);  // Setup a oneWire instance to communicate with any OneWire devices  
LiquidCrystal_I2C             lcd(I2C_ADDR,2, 1, 0, 4, 5, 6, 7);

DallasTemperature sensors(&oneWire); // Pass our oneWire reference to Dallas Temperature. // Envio data del cable one wire al sensor dallas

int fanControl = 9; //pwn fan pin
//Define Variables we'll be connecting to
double Setpoint, Input, Output, command;

//PID parameters. 
double kp=200;   //proportional parameter
double ki=100;   //integral parameter
double kd=50;   //derivative parameter

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint,kp,ki,kd, DIRECT);

//Minimum and Maximum PWM command, according fan specs and noise level required
double commandMin = 30;
double commandMax = 320;

unsigned long pulseDuration;

void analogWrite25k(int pin, int value)

{

   switch (pin) {

       case 9:

           OCR1A = value;

           break;

       case 10:

           OCR1B = value;

           break;

       default:

           // no other pin will work

           break;

   }

}

#include <SoftwareSerial.h>         //Incluimos la libreria SoftwareSerial
SoftwareSerial SIM800L(11, 12);       //Declaramos los pines RX(8) y TX(9) que vamos a usar
String Comando;                     //Declaramos una variable de tipo String para almacenar el texto del SMS recibido


void monitoreo1()
//Funcion para mandar mensaje de texto a un celular para el check
{
       sensors.begin(); 
       delay(3000);
       float Input = sensors.getTempCByIndex(0);
       static char outstr[15];
       dtostrf(Input,7, 3, outstr);

Serial.println("Enviando Mensaje SMS...");
SIM800L.write("AT+CMGF=1\r\n");
delay(1000);
SIM800L.write("AT+CMGS=\"+584124735400\"\r\n");
delay(1000);
SIM800L.write("Temperatura: "); // Mensaje enviado
SIM800L.write(outstr); // Mensaje enviado
//SIM800L.write("Temperatura: ", outstr); // Mensaje enviado
delay(1000);
SIM800L.write((char)26); // Comando de finalizacion
delay(100);
SIM800L.println();
delay(5000); // Tiempo para que se envie el mensaje
Serial.println("El Mensaje ha sido enviado");
Serial.println("");
}

void smsmas()
//Funcion para mandar mensaje de texto a un celular para el check
{
Serial.println("Enviando Mensaje SMS...");
SIM800L.write("AT+CMGF=1\r\n");
delay(1000);
SIM800L.write("AT+CMGS=\"+584124735400\"\r\n");
delay(1000);
SIM800L.write("Nuevo Setpoint: 28.50 "); // Mensaje enviado
delay(1000);
SIM800L.write((char)26); // Comando de finalizacion
delay(100);
SIM800L.println();
delay(5000); // Tiempo para que se envie el mensaje
Serial.println("El Mensaje ha sido enviado");
Serial.println("");
}

void smsmenos()
//Funcion para mandar mensaje de texto a un celular para el check
{
Serial.println("Enviando Mensaje SMS...");
SIM800L.write("AT+CMGF=1\r\n");
delay(1000);
SIM800L.write("AT+CMGS=\"+584124735400\"\r\n");
delay(1000);
SIM800L.write("Nuevo Setpoint: 25.50 "); // Mensaje enviado
delay(1000);
SIM800L.write((char)26); // Comando de finalizacion
delay(100);
SIM800L.println();
delay(5000); // Tiempo para que se envie el mensaje
Serial.println("El Mensaje ha sido enviado");
Serial.println("");
}

void smsreset()
//Funcion para mandar mensaje de texto a un celular para el check
{
Serial.println("Enviando Mensaje SMS...");
SIM800L.write("AT+CMGF=1\r\n");
delay(1000);
SIM800L.write("AT+CMGS=\"+584124735400\"\r\n");
delay(1000);
SIM800L.write("Nuevo Setpoint: 27.50 "); // Mensaje enviado
delay(1000);
SIM800L.write((char)26); // Comando de finalizacion
delay(100);
SIM800L.println();
delay(5000); // Tiempo para que se envie el mensaje
Serial.println("El Mensaje ha sido enviado");
Serial.println("");
}


String floatToString( float,int=8,int=2,boolean=true);

int fanPulse = 0;
//unsigned long pulseDuration;


void setup()
   {
       Serial.begin(9600);
       SIM800L.begin(9600);                     //Iniciamos una instancia de la librería SoftwareSerial
       //SIM800L.begin(115200);                     //Iniciamos una instancia de la librería SoftwareSerial
       SIM800L.println("AT+CMGF=1");              //Configuramos el módulo para trabajar con los SMS en modo texto
       delay(1000);                               //Pausa de 1 segundo
       SIM800L.println("AT+CNMI=1,2,0,0,0");      //Configuramos el módulo para que nos muestre los SMS recibidos por comunicacion serie
       
       // Inicializar el display con 16 caraceres 2 lineas
       lcd.begin (16,2);    
       lcd.setBacklightPin(3,POSITIVE);
       lcd.setBacklight(HIGH);

       lcd.setCursor(0, 0); // Fijar el numero de caracteres y de filas
       lcd.print("Aguarde ..."); // Enviar el mensaje
       lcd.setCursor(0, 2); // Fijar el numero de caracteres y de filas
       lcd.print("Recibiendo datos"); // Enviar el mensaje
 
       //Se inicia los sensores y calibra
       sensors.begin(); 
       delay(3000);
       Input = sensors.getTempCByIndex(0);
       Setpoint = 27.50;

        //turn the PID on
        myPID.SetMode(AUTOMATIC);
        myPID.SetOutputLimits(commandMin, commandMax);
        myPID.SetControllerDirection(REVERSE);


Serial.begin(9600);
pinMode(fanPulse, INPUT);
digitalWrite(fanPulse,HIGH);
      
       // Configure Timer 1 for PWM @ 25 kHz.

   TCCR1A = 0;           // undo the configuration done by...

   TCCR1B = 0;           // ...the Arduino core library

   TCNT1 = 0;           // reset timer

   TCCR1A = _BV(COM1A1) // non-inverted PWM on ch. A

           | _BV(COM1B1) // same on ch; B

           | _BV(WGM11); // mode 10: ph. correct PWM, TOP = ICR1

   TCCR1B = _BV(WGM13)   // ditto

           | _BV(CS10);   // prescaler = 1

   ICR1   = 320;         // TOP = 320
       
       //PWM FAN
       pinMode(fanControl, OUTPUT);

   }


void loop() 
   {
       //Se procesa la temperatura y se almacena
       //Serial.print("Aguarde un momento estamos Procesando la Temperatura..."); 
       sensors.requestTemperatures(); // Send the command to get temperature readings // Prepara el sensor para la lectura
       //Serial.println("Procesado!"); 
       Input=sensors.getTempCByIndex(0);
       Serial.println(Input); 
       delay (100);

       //process PID
       myPID.Compute();

       //apply PID processed command
       analogWrite25k(fanControl, Output);


pulseDuration = pulseIn(fanPulse, LOW);
double frequency = 1000/pulseDuration;

//Serial.print("pulse duration:");
//Serial.println(pulseDuration);
//Serial.print("time for full rev. (microsec.):");
//Serial.println(pulseDuration*2);
//Serial.print("freq. (Hz):");
//Serial.println(frequency/2);
//Serial.print("RPM:");
int rpm1 = (frequency/2*60);
//Serial.println(frequency/2*60);
//delay(1000);
       


       if (SIM800L.available()){                                //Si hay datos disponibles
          Comando = SIM800L.readString();                        //Los almacenamos en la variable Comando
          Serial.println("NUEVO SMS ENTRANTE: " + Comando);      //Los sacamos por comunicacion serie
          lcd.setCursor(10,0); // Cursor en la 11° posición de la primera fila
          lcd.print(" SEND");
          lcd.setCursor(10,1); // Cursor en la 11° posición de la 2° fila
          lcd.print(" SMS");
       } 
 
       if(Comando.indexOf("ESTATUS")>=0){                  //Si la variable Comando contiene la palabra ON
          monitoreo1();
          Comando = "";                              //Vaciamos la variable
       }
       else
       if(Comando.indexOf("MAS")>=0){                  //Si la variable Comando contiene el numero 1
          Setpoint = 28.50;
          smsmas();
          Comando = "";                              //Vaciamos la variable
       }
       else
       if(Comando.indexOf("MENOS")>=0){                  //Si la variable Comando contiene el numero 1
          Setpoint = 25.50;
          smsmenos();
          Comando = "";                              //Vaciamos la variable
       }
       else
       if(Comando.indexOf("RESET")>=0){                  //Si la variable Comando contiene el numero 1
          Setpoint = 27.50;
          smsreset();
          Comando = "";                              //Vaciamos la variable
       }
             // else
      // if(Comando.indexOf("SUBE")>=0){                  //Si la variable Comando contiene el numero 1
        //  Setpoint = Setpoint+1;
        //  sube();
        //  Comando = "";                              //Vaciamos la variable
      // }

             lcd.home ();    // go home
       int tiempo=millis()/1000;
       lcd.clear();
       lcd.setCursor(0,0); // Cursor en la primera posición de la primera fila
       lcd.print("T: ");
       lcd.print(Input);
       lcd.print((char)223);
       lcd.print("C "); // "223" -> "°"
       //lcd.setCursor(10,0); // Cursor en la 11° posición de la primera fila
       //lcd.print("");
       //lcd.print(getFanSpeed());
       lcd.setCursor(0,1); // Cursor en la primera posición de la 2° fila
       lcd.print("S: ");
       lcd.print(Setpoint);
       lcd.print((char)223);
       lcd.print("C "); // "223" -> "°"
      // lcd.setCursor(10,1); // Cursor en la 11° posición de la 2° fila
       //lcd.print("t:");
       //lcd.print(getFanSpeed());
      // lcd.print("");
       
       //lcd.print(" s  ");
       delay(200);

       
   }
