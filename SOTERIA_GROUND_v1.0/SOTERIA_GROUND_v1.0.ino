#include <StateMachine.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
//libraries to be used
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SPI.h>
#include <SD.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Adafruit_GPS.h>
#include <Adafruit_BMP280.h>
#include "RF24.h"
#include "printf.h"

//define the files for storing the received telemetry and photos
File Cali;
File Alt;
File YPR;
File GPS1;
File Pos;
File Camera;

//define elements for receiving photos
unsigned long tempo;
unsigned long tempo2;
int flag = 0;
bool radioNumber = 1;
byte buf[32] = {0};
byte addresses[][6] = {"1Node", "2Node"};
bool role = 0;

//define elements for recieving telemetry
#define PloadSize 32
byte pipe;
byte CurrentPloadWidth;
byte newdata;
unsigned char rx_buf[PloadSize] = {0};

//define CE and CSE pins
RF24 radio(9, 10);

//define address for telemetry
const byte address[6] = "00001";

//create an array to store the data that will be sent
struct Data {
  int null1;
  int a;
  int b;
  int c;
  int d;
  int e;
  int f;
  int g;
} Sensor;


void setup() {
  //initialise all components
  Serial.begin(115200);
  radio.begin();
  SPI.begin();
  printf_begin();

  //start radio for the telemetry data (not camera)
  radio.setDataRate(RF24_250KBPS);
  radio.enableDynamicPayloads();
  radio.setChannel(124);
  radio.openWritingPipe(address);
  radio.openReadingPipe(0, address);
  radio.startListening();
  delay(10);
}

//define function to open a file for the recieved pictures to be written into
void openFile()
{
  Camera = SD.open("saved.jpg", FILE_WRITE);
  flag = 1;
}

void loop() {
  //Read the data if available in buffer
  if (radio.available(&pipe))
  {
    //set the size of the payload to correspond to the data recieved
    CurrentPloadWidth = radio.getDynamicPayloadSize();
    if (!CurrentPloadWidth) {
    }
    else
    {
      radio.read(rx_buf, CurrentPloadWidth);
    }
  }
  memcpy(&Sensor, rx_buf, sizeof(Sensor));

  //identify element null1 to know what data is being received (calibration and altimeter)
  if (Sensor.null1 == 0) {
    Serial.print("Calibration: ");
    Serial.print(Sensor.a);
    Serial.print(Sensor.b);
    Serial.print(Sensor.c);
    Serial.print(Sensor.d);

    Serial.print("\tTemp: ");
    Serial.print(Sensor.e);
    Serial.print("\tPressure: ");
    Serial.print(Sensor.f);
    Serial.print("\Altitude: ");
    Serial.println(Sensor.g);

    //initialise the SD card
    if (!SD.begin(BUILTIN_SDCARD)) {
      while (1);
    }

    // open the calibration data file
    Cali = SD.open("Cali.txt", FILE_WRITE);

    // if the file opened okay, write to it:
    if (Cali) {

    //write to file
    Cali.print("Calibration: ");
    Cali.print(Sensor.a);
    Cali.print(Sensor.b);
    Cali.print(Sensor.c);
    Cali.println(Sensor.d);

     delay(10);

   // close the file:
      Cali.close();
    } 

    //initialise the SD card
    if (!SD.begin(BUILTIN_SDCARD)) {
      while (1);
    }

    // open the altimeter data file
    Alt = SD.open("Alt.txt", FILE_WRITE);

    // if the file opened okay, write to it:
    if (Alt) {

    //write to file
    Alt.print("\tTemp: ");
    Alt.print(Sensor.e);
    Alt.print("\tPressure: ");
    Alt.print(Sensor.f);
    Alt.print("\Altitude: ");
    Alt.println(Sensor.g);


     delay(10);

   // close the file:
      Alt.close();
    } 
    
    }
  //identify element null1 to know what data is being received (yaw pitch roll and GPS)
  if (Sensor.null1 == 1) {
    Serial.print("Yaw: ");
    Serial.print(Sensor.a);
    Serial.print("\tPitch: ");
    Serial.print(Sensor.b);
    Serial.print("\tRoll: ");
    Serial.println(Sensor.c);

    Serial.print("\tLatitude: ");
    Serial.print((Sensor.d)/10000);
    Serial.print("\tLongitude: ");
    Serial.print((Sensor.e)/10000);
    Serial.print("\tAltitude: ");
    Serial.print(Sensor.f);
    Serial.print("\tSatelites: ");
    Serial.println(Sensor.g);

    //initialise the SD card
    if (!SD.begin(BUILTIN_SDCARD)) {
      while (1);
    }

    // open the yaw pitch roll file
    YPR = SD.open("YPR.txt", FILE_WRITE);

    // if the file opened okay, write to it:
    if (Cali) {

    //write to file
    YPR.print("Yaw: ");
    YPR.print(Sensor.a);
    YPR.print("\tPitch: ");
    YPR.print(Sensor.b);
    YPR.print("\tRoll: ");
    YPR.println(Sensor.c);

     delay(10);

   // close the file:
      YPR.close();
    } 

    //initialise the SD card
    if (!SD.begin(BUILTIN_SDCARD)) {
      while (1);
    }

    // open the GPS file
    GPS1 = SD.open("GPS1.txt", FILE_WRITE);

    // if the file opened okay, write to it:
    if (GPS1) {

    //write to file
    GPS1.print("\tLatitude: ");
    GPS1.print((Sensor.d)/10000);
    GPS1.print("\tLongitude: ");
    GPS1.print((Sensor.e)/10000);
    GPS1.print("\tAltitude: ");
    GPS1.print(Sensor.f);
    GPS1.print("\tSatelites: ");
    GPS1.println(Sensor.g);

     delay(10);

   // close the file:
      GPS1.close();
    } 
    
  }
  //identify element null1 to know what data is being received (position)
  if (Sensor.null1 == 2) {
    Serial.print("Xpos: ");
    Serial.print(Sensor.a);
    Serial.print("\tYpos: ");
    Serial.print(Sensor.b);
    Serial.print("\Zpos: ");
    Serial.println(Sensor.c);

    //initialise the SD card
    if (!SD.begin(BUILTIN_SDCARD)) {
      while (1);
    }

    // open the position data file
    Pos = SD.open("Pos.txt", FILE_WRITE);

    // if the file opened okay, write to it:
    if (Pos) {

    //write to file
    Pos.print("Xpos: ");
    Pos.print(Sensor.a);
    Pos.print("\tYpos: ");
    Pos.print(Sensor.b);
    Pos.print("\Zpos: ");
    Pos.println(Sensor.c);

     delay(10);

   // close the file:
      Pos.close();
    } 
    
  }
  //identify element null1 to know what data is being received (camera)
  if (Sensor.null1 == 9) {
    //initialise for receiving a photo
    Serial.begin (9600) ;
    delay(10);

   //initialise the SD card
   if (!SD.begin(BUILTIN_SDCARD)) {
     Serial.println("initialization of SD failed!");
     return;
   }

    //set radio settings for camera
    radio.setPayloadSize(32);
    radio.setChannel(0x60);
    radio.setDataRate(RF24_250KBPS);
    radio.setAutoAck(1);                     // Ensure autoACK is enabled
    radio.setRetries(1, 200);
    digitalWrite(13, LOW);
    radio.printDetails();
    radio.startListening();
    if (radioNumber)
    {
      radio.openWritingPipe(addresses[1]);
      radio.openReadingPipe(1, addresses[0]);
    }
    else
    {
      radio.openWritingPipe(addresses[0]);
      radio.openReadingPipe(1, addresses[1]);
    }

    //if radio is available run function openFile defined above
     if (radio.available()) {
      if (flag == 0) {
        openFile();
        }
        //read the image and write it into the file
        radio.read(buf, sizeof(buf));
        //radio.flush_rx();
        Serial.println(F(buf));
        Camera.write(buf, sizeof(buf));
        tempo = millis();
    
      }
      tempo2 = millis();
      if(tempo2-tempo > 5000 && flag == 1){
        Serial.println(F("File closed"));
        Camera.close();
        flag = 0;
      }
        //reset camera settings to recieve telemetry
        radio.setDataRate(RF24_250KBPS);
        radio.enableDynamicPayloads();
        radio.setChannel(124);
        radio.openWritingPipe(address);
        radio.openReadingPipe(0, address);
        radio.startListening();

  }
        delay(100);
}
      


  
