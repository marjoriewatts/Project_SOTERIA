//libraries to be used
#include <StateMachine.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <stdio.h>
#include <math.h>
#include <SPI.h>
#include <SD.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Adafruit_GPS.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_VC0706.h>
#include <Servo.h>
#include <BasicLinearAlgebra.h>
#include "printf.h"


//-----------------------------------------------------------------------------------------------------------

//define files to input live data into on the sd card
File YPR;
File Cali;
File GPS1;
File Alt;
File Pos;

//defining elements for state transitions
int randomState = 0;
#define N 3
int currentTime;
float currentAlt;


//camera setup definitions 
#define cameraconnection Serial1
#define chipSelect BUILTIN_SDCARD
Adafruit_VC0706 cam = Adafruit_VC0706(&cameraconnection);
bool radioNumber = 0;
byte buf[32] = {0};
File myFile;
byte addresses[][6] = {"1Node", "2Node"};
bool role = 0;

//sample rate delay for IMU data
#define BNO055_SAMPLERATE_DELAY_MS (100)


//define the pin numbers of the button and LEDs
const int buttonPin = 2;      // the number of the pushbutton pin 
const int led1Pin =  27;      // the number of the safe state LED pin
const int led2Pin =  29;      // the number of the launch state LED pin
const int led3Pin =  31;      // the number of the acsending LED pin

//initial position of servo (degrees) and button
int pos = 0;
int buttonState = 0;

//start and define components
StateMachine machine = StateMachine();
Adafruit_BNO055 myIMU = Adafruit_BNO055();
Adafruit_BMP280 bmp;
Servo myservo;
RF24 radio(36, 10);  // CE, CSN

//GPS variable definition
#define GPSSerial Serial8
#define GPSECHO false
Adafruit_GPS GPS(&GPSSerial);
uint32_t timer = millis();

//functions for IMU integration calculation to find velocity and position
double xPos = 0, yPos = 0, zPos = 0;  /*velocity = accel*dt (dt in seconds), position = 0.5*accel*dt^2*/
double ACCEL_VEL_TRANSITION =  (double)(BNO055_SAMPLERATE_DELAY_MS) / 1000.0;
double ACCEL_POS_TRANSITION = 0.5 * ACCEL_VEL_TRANSITION * ACCEL_VEL_TRANSITION;
double DEG_2_RAD = 0.01745329251;

//set radio address
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

//-----------------------------------------------------------------------------------------------------------
//function to take, save and send photos
void takePhoto() {
  //initialise camera and test camera connection
  if (cam.begin()) {
    Serial.println("Camera Found:");
  } else {
    Serial.println("No camera found?");
    return;
  }
  // Print out the camera version information (optional)
  char *reply = cam.getVersion();
  if (reply == 0) {
    Serial.print("Failed to get version");
  } else {
    Serial.println("-----------------");
    Serial.print(reply);
    Serial.println("-----------------");
  }

  // Set the picture size 
  //cam.setImageSize(VC0706_640x480);        // biggest
  //cam.setImageSize(VC0706_320x240);        // medium
  cam.setImageSize(VC0706_160x120);          // small

  Serial.println("Capturing");

  delay(10);

  //take picture and report result
  if (! cam.takePicture())
    Serial.println("Failed to snap!");
  else
    Serial.println("Picture taken!");
 
  // Create an image with the name IMAGExx.JPG
  char filename[13];
  strcpy(filename, "IMAGE00.JPG");
  for (int i = 0; i < 100; i++) {
    filename[5] = '0' + i/10;
    filename[6] = '0' + i%10;
    // create if does not exist, do not open existing, write, sync after write
    if (! SD.exists(filename)) {
      break;
    }
  }
 
  // Open the file for writing
  File imgFile = SD.open(filename, FILE_WRITE);

  // Get the size of the image (frame) taken  
  uint16_t jpglen = cam.frameLength();
  Serial.print("Storing ");
  Serial.print(jpglen, DEC);
  Serial.print(" byte image.");

  int32_t time = millis();
  pinMode(8, OUTPUT);
  // Read all the data up to # bytes
  byte wCount = 0; // For counting # of writes
  while (jpglen > 0) {
    // read 32 bytes at a time;
    uint8_t *buffer;
    uint8_t bytesToRead = min((uint16_t)32, jpglen); 
    buffer = cam.readPicture(bytesToRead);
    imgFile.write(buffer, bytesToRead);
    if(++wCount >= 64) { // Every 2K, give feedback so it doesn't appear locked up
      Serial.print('.');
      wCount = 0;
    }
    jpglen -= bytesToRead;
  }
  imgFile.close();
  Serial.println(strcat("Written to ", filename));

  //time taken for picture to be taken
  time = millis() - time;
  Serial.println("done!");
  Serial.print(time); Serial.println(" ms elapsed");

  //radio setup for pictures
  pinMode(53, OUTPUT);
  digitalWrite(53, LOW);
  if (!SD.begin(BUILTIN_SDCARD))
  {
    Serial.println("initialization of SD failed!");
    return;
  }

  Serial.println("initialization done.");
  digitalWrite(53, HIGH);

 
  radio.openWritingPipe(addresses[0]);
  radio.openReadingPipe(1, addresses[1]);

  //fill array to notify the reciever a picture is about to be transmitted
  Sensor.null1 = 9;
  Sensor.a = 9;
  Sensor.b = 9;
  Sensor.c = 9;
  Sensor.d = 9;
  Sensor.e = 9;
  Sensor.f = 9;
  Sensor.g = 9;

  //send notification array
  radio.write(&Sensor, sizeof(Sensor));
  bool nice = radio.write(&Sensor, sizeof(Sensor));
  if (nice) {
    Serial.println("Prepared for camera");
  }
  else {
    Serial.println("Not ready for camera oops");
  }

  //send photo (from sendfile function defined below)
  sendfile(filename);

}
//define function that reads the file the picture is in
void readfile() {
  //fills buffer with 32 bytes of image and sends, repeating until less than 32 bits left
  int i = 0;
  while (myFile.available())
  {
    buf[i] = myFile.read();

    i++;
    if (i == 32)
    {
      Serial.println(F(buf));
      radio.write(buf, sizeof(buf));
      delay(10);
      i = 0;
    }
  }
  //sends final packet (less than 32 bytes)
  if (i > 0) {
    radio.flush_tx();
    radio.write(buf, i);
    Serial.write(buf, i); //Output results at serial port
    delay(1);
    i = 0;
    // Serial.println(F(buf));
    Serial.println("");
    Serial.println("Last stream of less than 32 bytes");
  }
}
//define function that sends the picture over radio
void sendfile(char *filename) {
  Serial.println(strcat("SD -> ", filename));
  myFile = SD.open(filename);
  if (myFile)
  {
    readfile ();
    myFile.close();
    radio.powerDown(); 
  }
  else
  {
    Serial.println("error opening file"); 
  }

  //power the radio back on to continue sending telemetry
  radio.begin();
  //start radio
  //radio.setPALevel(RF24_PA_MIN);
  radio.setPALevel(RF24_PA_MAX);
  radio.setPayloadSize(32);
  //radio.enableDynamicPayloads();
  radio.setChannel(124);
  radio.setDataRate(RF24_250KBPS);
  radio.openWritingPipe(address);
  radio.openReadingPipe(0, address);
  radio.stopListening();
  radio.setRetries(2, 100);
  radio.setAutoAck(1);
}

//-----------------------------------------------------------------------------------------------------------
//create the states for the code to run through throughout the flight

//state 0 is safe state where we can test different functions of the rocket before launch
State* S0 = machine.addState([](){
  Serial.println("Safe State");
  //LED1 on
  digitalWrite(led1Pin, HIGH);
  //SD connection test
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    return;
  //camera connection test
  if (cam.begin()) {
    Serial.println("Camera Found:");
  } else {
    Serial.println("No camera found?");
    return;
  }
  }
    //servo test
  for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
  for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
  });;


//state 1 is launch where an LED is turned on to signify readiness
State* S1 = machine.addState([](){
  Serial.println("Launch");
  digitalWrite(led2Pin, HIGH); //LED2 on
});;


//state 2 is ascension detection where the transition to this state starts the timer that will be used to force the transition between states if the other detection methods fail
State* S2 = machine.addState([](){
  Serial.println("Ascending");
  //LED2 on
  digitalWrite(led3Pin, HIGH);
});;


//state 3 is apogee detection to track the flight and notify the ground that the system is still running through the states
State* S3 = machine.addState([](){
  Serial.println("Apogee");
});;


//state 4 is parachute deploy detection which, like apogee detect, lets ground know the code has moved into the next state and the rocket is now ready to open and start taking photos
State* S4 = machine.addState([](){
  Serial.println("Parachute deploy");
});;


//state 5 is camera where the rocket will take the photos to save and send back to ground for analysis (spotting fires)
State* S5 = machine.addState([](){
  Serial.print("Camera");
  //servo open
  for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
  
  //take, save and send photos
  takePhoto();
});;


//state 6 is landing state where the nosecone closes again to protect the camera 
State* S6 = machine.addState([](){
  Serial.println("Landing mode");
  //close servo
  for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
  //LED1 back on
  digitalWrite(led1Pin, HIGH);
});;


//-----------------------------------------------------------------------------------------------------------
// function to multiply two matrices
void multiply(float mat1[][3], float mat2[][1], float res[][1])
{
    int i, j, k;
    for (i = 0; i < N; i++) {
        for (j = 0; j < N; j++) {
            res[i][j] = 0;
            for (k = 0; k < N; k++)
                res[i][j] += mat1[i][k] * mat2[k][j];
        }
    }
}
//function to display 3x3 matrices (for troubleshooting)
void Dispaly3(float A[3][3]){
  for(int i=0; i< 3; i++) {
  for(int l=0; l<3; l++){
    Serial.println(A[i][l]);
  } 
  }
}
//function to display 3x1 matrices (for troubleshooting)
void Dispaly1(float B[3][1]){
  for(int i=0; i< 3; i++) {
  for(int l=0; l< 1; l++){
    Serial.println(B[i][l], 4);
  } 
  }
  Serial.println("---------------------------");
}

//-----------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------
void setup() {
  //initialise all components
  Serial.begin(115200);
  randomSeed(A0);
  myIMU.begin();
  radio.begin();
  GPS.begin(9600);
  SPI.begin();
  bmp.begin();
  printf_begin();
  while (!Serial) {
    ;
  }
  

  //attaches the servo on pin 25 to the servo object
  myservo.attach(25);
  
  //start GPS
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  GPS.sendCommand(PGCMD_ANTENNA);

  delay(1000); //to give the system time to start up
  
  //start radio for the telemetry data (not camera)
  radio.setPALevel(RF24_PA_MAX);
  radio.setPayloadSize(32);
  //radio.enableDynamicPayloads();
  radio.setChannel(124);
  radio.setDataRate(RF24_250KBPS);
  radio.openWritingPipe(address);
  radio.openReadingPipe(0, address);
  radio.stopListening();
  radio.setRetries(2, 100);
  radio.setAutoAck(1);  // Ensure autoACK is enabled


  //start altimeter
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  //camera size
  //cam.setImageSize(VC0706_640x480);        // biggest
  //cam.setImageSize(VC0706_320x240);        // medium
  cam.setImageSize(VC0706_160x120);          // small
  

  //initialise the LED pins as outputs:
  pinMode(led1Pin, OUTPUT);
  pinMode(led2Pin, OUTPUT);
  pinMode(led3Pin, OUTPUT);

  //initialise pushbutton pin as input:
  pinMode(buttonPin, INPUT);


//-----------------------------------------------------------------------------------------------------------  
//define transitions
  S0->addTransition([](){ //safe to launch
    buttonState = digitalRead(buttonPin);
    if(buttonState == LOW){       //button will be used to transition into Launch state
      Serial.print("Transitioning to state 1 ");
      digitalWrite(led1Pin, LOW);
      S0->setTransition(0, 1);
    } 
    else {
      Serial.println("Condition not met, remaining in state 0");
      S0->setTransition(0,0);
    }

    return true;
  },S0);
  

  S1->addTransition([](){ //launch to ascending
    imu::Vector<3> acc =myIMU.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    double magnitude = sqrt((acc.x() * acc.x()) + (acc.y() * acc.y()) + (acc.z() * acc.z()));
    if(magnitude > (20)){       //if the madnitude of acceleration increases above 20 then the rocket has launched and is now ascending
      Serial.print("Transitioning to state 2 ");
      digitalWrite(led2Pin, LOW); //turn off safe state LED
      const char ascDetect[] = "Ascent Detected!";
      radio.write(&ascDetect, sizeof(ascDetect)); //send message to ground that ascent has been detected
      currentTime = millis();  //start the flight timer (for forced transition between states)
      S1->setTransition(1, 2);
    } 
    /* <for manually running through the states in testing>
    else if(buttonState == LOW){
      Serial.print("Transitioning to 2 ");
      currentTime = millis();  //start the flight timer (for forced transition between states)
      S0->setTransition(1, 2);
    } 
    */
    else {
      Serial.println("Condition not met, remaining in state 1");
      S1->setTransition(1,1);
    }

    return true;
  },S1);


  S2->addTransition([](){ //ascending to apogee
    unsigned long t2 = millis();
    currentAlt = (bmp.readAltitude(1023.7)); /* Local pressure */
    delay(550);
    if(currentAlt < (bmp.readAltitude(1023.7))){        //change state if altitude stops increasing over time (i.e. highest point has been reached)
      Serial.print("Transitioning to 3 ");
      const char apoDetect[] = "Apogee Detected!";
      radio.write(&apoDetect, sizeof(apoDetect)); //send message to ground that apogee has been detected
      S2->setTransition(2, 3);
    } 
    else if (t2 - currentTime > (13500)) {        //forced transition between states after 13.5 seconds has passed (apogee should be reached at 13 seconds)
      digitalWrite(led3Pin, LOW);
      Serial.print("Transitioning to 3 ");
      const char apoDetect[] = "Apogee Detected!";
      radio.write(&apoDetect, sizeof(apoDetect)); //send message to ground that apogee has been detected
      S2->setTransition(2, 3);
    }
    /* <for manually running through the states in testing>
    else if(buttonState == LOW){
      Serial.print("Transitioning to 3 ");
      S0->setTransition(2, 3);
    } 
    */
    else {
      Serial.println("Condition not met, remaining in state 2");
      S2->setTransition(2,2);
    }

    return true;
  },S2);
  

  S3->addTransition([](){ //apogee to parachute deploy
    unsigned long t3 = millis();
    imu::Vector<3> acc =myIMU.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    double magnitude = sqrt((acc.x() * acc.x()) + (acc.y() * acc.y()) + (acc.z() * acc.z()));
    if(1 < magnitude < 4){        //change state if magnitude of acceleration drops almost to zero (i.e. terminal velocity has been reached due to parachute deploy)
      Serial.print("Transitioning to 4 ");
      S3->setTransition(3, 4);
    } 
    else if(t3 - currentTime > (18000)) {       //forced transition between states after 18 seconds has passed (parachute deploy should occur at 14 seconds)
      Serial.print("Transitioning to 4 ");
      S3->setTransition(3, 4);
    }
    /* <for manually running through the states in testing>
    else if(buttonState == LOW){
      Serial.print("Transitioning to 4 ");
      S0->setTransition(3, 4);
    }
    */
    else {
      Serial.println("Condition not met, remaining in state 3");
      S3->setTransition(3,3);
    }

    return true;
  },S0);
  

  S4->addTransition([](){ //parachute deploy to camera
    unsigned long t4 = millis();
    imu::Vector<3> acc =myIMU.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    double magnitude = sqrt((acc.x() * acc.x()) + (acc.y() * acc.y()) + (acc.z() * acc.z()));
    currentAlt = (bmp.readAltitude(1023.7)); /* Local pressure */
    if(currentAlt < 900 and 1 < magnitude < 4){       //change state when rocket is below a certain height and the parachute has definitely deployed so the nosecone doesn't open when falling under gravity
      Serial.print("Transitioning to 5 ");
      S4->setTransition(4, 5);
    } 
    /* <for manually running through the states in testing>
    else if(buttonState == LOW){
      Serial.print("Transitioning to 5 ");
      S0->setTransition(4, 5);
    }
    */ 
    else {
      Serial.println("Condition not met, remaining in state 4");
      S4->setTransition(4,4);
    }

    return true;
  },S4);

  S5->addTransition([](){ //camera to landing
    unsigned long t5 = millis();
    currentAlt = (bmp.readAltitude(1023.7)); /* Local pressure */
    if(currentAlt < 30){        //close the nosecone when 30m above the ground
      Serial.print("Transitioning to 6 ");
      S5->setTransition(5, 6);
    } 
    else if(t5 - currentTime > (120000)) {        //forced transition between states after 18 seconds has passed (parachute deploy should occur at 14 seconds)
      Serial.print("Transitioning to 6 ");
      S5->setTransition(5, 6); 
    }
    /* <for manually running through the states in testing>
    else if(buttonState == LOW){
      Serial.print("Transitioning to 6 ");
      S0->setTransition(5, 6);
    }
    */
    else {
      Serial.println("Condition not met, remaining in state 5");
      S5->setTransition(5,5);
    }

    return true;
  },S5);

}

//-----------------------------------------------------------------------------------------------------------  

void loop() {
  //run through all the states
  machine.run();

  //get calibration data from IMU
  uint8_t system, gyro, accel, mg =0;
  myIMU.getCalibration(&system, &gyro, &accel, &mg);

  //get orientation and acceleration data from IMU
  sensors_event_t orientationData , linearAccelData;
  myIMU.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  myIMU.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  imu::Vector<3> acc =myIMU.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> gyr =myIMU.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> mag =myIMU.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

  sensors_event_t event;
  myIMU.getEvent(&event);

  //integrating IMU acceleration data twice to get position
  xPos = xPos + ACCEL_POS_TRANSITION * linearAccelData.acceleration.x;
  yPos = yPos + ACCEL_POS_TRANSITION * linearAccelData.acceleration.y;
  zPos = zPos + ACCEL_POS_TRANSITION * linearAccelData.acceleration.z;

  //get GPS data
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA()))
      return;
  }
//-----------------------------------------------------------------------------------------------------------  
//define matrices for position
//change in position in the frame of reference of the rocket matrix
float B[3][1] = {
    {ACCEL_POS_TRANSITION * linearAccelData.acceleration.x},
    {ACCEL_POS_TRANSITION * linearAccelData.acceleration.y},
    {ACCEL_POS_TRANSITION * linearAccelData.acceleration.z},
};

//create an empty matrix to be filled with the global FOR data
  float z[3][1] = {
    {0},{0},{0},
};

//convert IMU yaw pitch and roll from degrees to radians
float pitch = (event.orientation.y)*DEG_2_RAD;
float roll = (event.orientation.z)*DEG_2_RAD; 
float yaw =  (event.orientation.x)*DEG_2_RAD;

//defining each element of the rotation matrix in terms of trig functions of angles
float a11 = cos(pitch)*cos(yaw);
float a12 = cos(pitch)*sin(yaw);
float a13 = -sin(pitch);
float a21 = sin(roll)*sin(pitch)*cos(yaw)-cos(roll)*sin(yaw);
float a22 = sin(roll)*sin(pitch)*sin(yaw)+cos(roll)*cos(yaw);
float a23 = cos(pitch)*sin(roll);
float a31 = cos(roll)*sin(pitch)*cos(yaw)+sin(roll)*sin(yaw);
float a32 = cos(roll)*sin(pitch)*sin(yaw)-sin(roll)*cos(yaw);
float a33 = cos(pitch)*cos(roll);

//create the rotation matrix
  float A[3][3] ={
  {a11, a12, a13},
  {a21, a22, a23},
  {a31, a32, a33},
};

//multiply the rotation matrix and the rocket FOR matrix to fill the global FOR matrix
multiply(A,B,z);

//define the new position of the rocket (global FOR) 
xPos = xPos + z[1][1];
yPos = yPos + z[2][1];
zPos = zPos + z[3][1];


//-----------------------------------------------------------------------------------------------------------  
//file writing

  //yaw pitch and roll 
  /* <serial print for troubleshooting>
  Serial.print("Yaw angle: ");
  Serial.print(event.orientation.x, 4);
  Serial.print("\tPitch angle: ");
  Serial.print(event.orientation.y, 4);
  Serial.print("\tRoll angle: ");
  Serial.print(event.orientation.z, 4);
  Serial.println("");
  */

  //initialise the SD card
  Serial.print("Initializing SD card...");

  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("initialization failed!");
    while (1);
  }
  Serial.println("initialization done.");

  // open the yaw pitch and roll file
  YPR = SD.open("YPR.txt", FILE_WRITE);

  // if the file opened okay, write to it:
  if (YPR) {
    Serial.print("Writing to YPR.txt...");
    
    //write to file
    YPR.print("Yaw angle: ");
    YPR.print(event.orientation.x, 4);
    YPR.print("\tPitch angle: ");
    YPR.print(event.orientation.y, 4);
    YPR.print("\tRoll angle: ");
    YPR.print(event.orientation.z, 4);
    YPR.println("");

    delay(10);

    // close the file
    YPR.close();
    Serial.println("done.");
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening YPR.txt");
  }

//-----------------------------------------------------------------------------------------------------------  
  //position
  /* <serial print for troubleshooting>
  Serial.print("Xpos: ");
  Serial.print(xPos);
  Serial.print("\tYpos: ");
  Serial.print(yPos);
  Serial.print("\tZpos: ");
  Serial.println(zPos);
  */

  //initialise the SD card
  Serial.print("Initializing SD card...");

  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("initialization failed!");
    while (1);
  }
  Serial.println("initialization done.");

  // open the position data file
  Pos = SD.open("Pos.txt", FILE_WRITE);

  // if the file opened okay, write to it:
  if (Pos) {
    Serial.print("Writing to Pos.txt...");
    
    //write to file
    Pos.print("Xpos: ");
    Pos.print(xPos);
    Pos.print("\tYpos: ");
    Pos.print(yPos);
    Pos.print("\tZpos: ");
    Pos.println(zPos);

    delay(10);

    // close the file
    Pos.close();
    Serial.println("done.");
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening Pos.txt");
  }
//-----------------------------------------------------------------------------------------------------------
  //calibration 
  /* <serial print for troubleshooting>
  Serial.print("Cali: ");
  Serial.print(system);
  Serial.print(gyro);
  Serial.print(accel);
  Serial.println(mg);
  */

  //initialise the SD card
  Serial.print("Initializing SD card...");

  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("initialization failed!");
    while (1);
  }
  Serial.println("initialization done.");

  // open the calibration data file
  Cali = SD.open("Cali.txt", FILE_WRITE);

  // if the file opened okay, write to it:
  if (Cali) {
    Serial.print("Writing to Cali.txt...");
    
    //write to file
    Cali.print("Cali: ");
    Cali.print(system);
    Cali.print(gyro);
    Cali.print(accel);
    Cali.println(mg);

    delay(10);

    // close the file
    Cali.close();
    Serial.println("done.");
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening Cali.txt");
  }

  
//-----------------------------------------------------------------------------------------------------------
  //GPS
  /* <serial print for troubleshooting>
  Serial.print("Location: ");
  Serial.print(GPS.latitude, 4);
  Serial.print(", ");
  Serial.print(GPS.longitude, 4);
  Serial.print("\tAltitude: ");
  Serial.print(GPS.altitude);
  Serial.print("\tSatNum: ");
  Serial.print((int)GPS.satellites);
  */

  //initialise the SD card
  if (!SD.begin(BUILTIN_SDCARD)) {
  while (1);
  }

  // open the GPS file
  GPS1 = SD.open("GPS1.txt", FILE_WRITE);

  // if the file opened okay, write to it:
  if (GPS1) {
    
    //write to file
    GPS1.print("Location: ");
    GPS1.print(GPS.latitude, 4);
    GPS1.print(", ");
    GPS1.print(GPS.longitude, 4);
    GPS1.print("\tAltitude: ");
    GPS1.print(GPS.altitude);
    GPS1.print("\tSatNum: ");
    GPS1.print((int)GPS.satellites);

    //if (millis() - timer > 2000) {
    //timer = millis(); // reset the timer
    GPS1.print("\tTime: ");
    if (GPS.hour < 10) { GPS1.print('0'); }
    GPS1.print(GPS.hour, DEC); GPS1.print(':');
    if (GPS.minute < 10) { GPS1.print('0'); }
    GPS1.print(GPS.minute, DEC); GPS1.print(':');
    if (GPS.seconds < 10) { GPS1.print('0'); }
    GPS1.print(GPS.seconds, DEC); GPS1.print('.');
    if (GPS.milliseconds < 10) {
      GPS1.print("00");
    } else if (GPS.milliseconds > 9 && GPS.milliseconds < 100) {
      GPS1.print("0");
    }
    GPS1.println(GPS.milliseconds);


    delay(10);
    
    // close the file
    GPS1.close();
  } 


//-----------------------------------------------------------------------------------------------------------
  //altimeter
  /* <serial print for troubleshooting>
  Serial.print("Temp: ");
  Serial.print(bmp.readTemperature());
  Serial.print("\tPressure: ");
  Serial.print(bmp.readPressure());
  Serial.print("\tAltitude: ");
  Serial.println(bmp.readAltitude(1023.7));
  */

  //initialise the SD card
  Serial.print("Initializing SD card...");

  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("initialization failed!");
    while (1);
  }
  Serial.println("initialization done.");

  // open the altimeter data file
  Alt = SD.open("Alt.txt", FILE_WRITE);

  // if the file opened okay, write to it:
  if (Alt) {
    Serial.print("Writing to Alt.txt...");

    //wrtie to file
    Alt.print("Temperature: ");
    Alt.print(bmp.readTemperature());
    Alt.print("\tPressure: ");
    Alt.print(bmp.readPressure());
    Alt.print("\tAltitude: ");
    Alt.println(bmp.readAltitude(1023.7));
    
    delay(10);

    // close the file
    Alt.close();
    Serial.println("done.");
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening Alt.txt");
  } 
  
//-----------------------------------------------------------------------------------------------------------
  //radio sending
  //fill the struct with calibration and altimeter data and send it with first identification element 0
  Sensor.null1 = 0;
  Sensor.a = system;
  Sensor.b = gyro;
  Sensor.c = accel;
  Sensor.d = mg;

  Sensor.e = bmp.readTemperature();
  Sensor.f = bmp.readPressure();
  Sensor.g = bmp.readAltitude(1023.7); //LOCAL PRESSURE

  //write to the receiver
  radio.write(&Sensor, sizeof(Sensor));

  //record if data has been sent
  bool sent0 = radio.write(&Sensor, sizeof(Sensor));
  if (sent0) {
    Serial.println("\tSent0");
  }
  else {
    Serial.println("\tFailed0");

  }

  //fill the struct with YPR and GPS data and send it with first identification element 1
  Sensor.null1 = 1;

  Sensor.a = event.orientation.x;
  Sensor.b = event.orientation.y;
  Sensor.c = event.orientation.z;

  Sensor.d = (GPS.latitude)*10000;
  Sensor.e = (GPS.longitude)*10000;
  Sensor.f = GPS.altitude;
  Sensor.g = (int)GPS.satellites;

  //write to the receiver
  radio.write(&Sensor, sizeof(Sensor));

  //record if data has been sent
  bool sent1 = radio.write(&Sensor, sizeof(Sensor));
  if (sent1) {
    Serial.println("\tSent1");
  }
  else {
    Serial.println("\tFailed1");
    }


  //fill the struct with position data and send it with first identification element 2
  Sensor.null1 = 2;

  Sensor.a = xPos;
  Sensor.b = yPos;
  Sensor.c = zPos;

  Sensor.d = 0;
  Sensor.e = 0;
  Sensor.f = 0;
  Sensor.g = 0;

  //write to receiver
  radio.write(&Sensor, sizeof(Sensor));

  //record if data has been sent
  bool sent2 = radio.write(&Sensor, sizeof(Sensor));
  if (sent2) {
    Serial.println("\tSent2");
  }
  else {
    Serial.println("\tFailed2");
    }
}
