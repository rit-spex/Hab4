///////////////////////
/// RIT SPEX HABIV
/// Main Flight Software
/// April 28th, 2018
///
/// Daniel Mitchell
/// Austin Bodzas
/// Thomas Hall
////////////////////////
#include <stdint.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include "SparkFunBME280.h"
#include "Adafruit_MCP9808.h"
#include <SparkFunLSM9DS1.h>

//Functionality Constants
#define Balloon_BME280_Present false  //Represents a balloon BME280 Sensor

//Communications Constants
#define B_RATE        9600    // Serial baud rate
#define BUFF_SIZE     480     // TODO tailor to packet size
#define CS_0          10      // CS0 pin for SPI

//Pin Constants
#define cubeSatPin    29      // cubesat pin, will be held high to trigger
#define cutDownPin    28      // cutdown pin for balloon
#define statusLED1    2       // Status & Heartbeat LED pins
#define statusLED2    30
#define heartbeatLED  14

// Magnetic field declination, RIT Nov, 21, 2016
// TODO confirm format of declination
#define DECL          -11.44  // 11.44 degrees West

// max height and fall drop forgiveness for CubeSat
#define DEPLOY_HEIGHT 21336.0   //Deployment height for the cubesat payload in meters (70k feet)
#define CUTDOWN_HEIGHT 12192.0  //Cutdown height for the balloon in meters (40k feet)

// LSM9DS1 I2C
#define LSM9DS1_M	    0x1C
#define LSM9DS1_AG    0x6A

//Instantiate Objects
File log_file;
BME280 bme280;
BME280 ball280; //BME280 object for balloon. This is not used for HAB 4
LSM9DS1 imu;
Adafruit_MCP9808 mcp9808 = Adafruit_MCP9808();

//Status Booleans
boolean cubesatDeployed = false;  //Keeps track of when the cubesat is deployed
boolean cutDown = false;          //Keeps track of when the cutdown occurs
boolean error = false;            //Keeps track of an error status       

//Time Control Variables
uint16_t poll_rate = 500; // in milliseconds
unsigned long lastPolled = 0;
unsigned long lastHeartbeatToggle = 0;
unsigned long lastDisplayedAltitude = 0;
unsigned long lastDisplayedTemp = 0;

uint8_t buffer[BUFF_SIZE];
uint8_t *cursor = buffer;
uint32_t consecutiveDescentMeasurements = 0;  //Number of consecutive descending measurements
uint32_t consecutiveDeployMeasurements = 0; //Number of consecutive measurements above the deploy altitude
uint8_t packet_count = 0;
uint8_t max_packet_count = 10;
uint8_t precision = 7;
String stringBuffer = "";
float max_height = 0;

//--------------------------------------SETUP--------------------------------------
void setup() {
  Serial.begin(B_RATE); //Begin Serial comms w/ PC
  delay(1000);  //Delay for a second - Give time for serial to come up
  Serial.println("Serial Initialized"); Serial.println();
  init(); //Call the initialization function
}
//--------------------------------------------------------------------------------
//--------------------------------------LOOP--------------------------------------
//--------------------------------------------------------------------------------
void loop() {
  led_management();

  // Polls sensors according to poll rate
  if (millis() - lastPolled > poll_rate) {
    lastPolled = millis();
    poll_sensors();
  }
}

//Initializes sensors and opens file for IO
void init() {
  pinMode(cubeSatPin, OUTPUT); Serial.println(F("Cubesat Deployment pin configured as an output"));
  digitalWrite(cubeSatPin, LOW); Serial.println(F("Cubesat Deployment pin set to LOW (off)")); Serial.println();

  pinMode(cutDownPin, OUTPUT); Serial.println(F("Balloon Cutdown pin configured as an output"));
  digitalWrite(cutDownPin, LOW); Serial.println(F("Balloon Cutdown pin set to LOW (off)")); Serial.println();

  pinMode(heartbeatLED, OUTPUT); Serial.println(F("Heartbeat LED pin configured as an output"));
  digitalWrite(heartbeatLED, LOW); Serial.println(F("Heartbeat LED pin set to LOW (off)")); Serial.println();

  pinMode(statusLED1, OUTPUT); Serial.println(F("Status LED 1 pin configured as an output"));
  digitalWrite(statusLED1, LOW); Serial.println(F("Status LED 1 pin set to LOW (off)")); Serial.println();

  pinMode(statusLED2, OUTPUT); Serial.println(F("Status LED 2 pin configured as an output"));
  digitalWrite(statusLED2, HIGH); Serial.println(F("Status LED 2 pin set to HIGH (on)")); Serial.println();


  if (SD.begin(CS_0)) {
    Serial.println(F("SD Card Initialized")); Serial.println();
  }
  else {
    Serial.println(F("*****SD Card Initilization Failure*****"));
    Serial.println();
    error = true;
  }

  //log_file = SD.open("tester.bin", FILE_WRITE);
  //Generates a filename with unique extension for multiple runs
  String baseFile = "tester";
  String extension = ".csv";
  String fileName = "";
  int iteration = 0;
  do {
    fileName = baseFile + iteration + extension;
    Serial.print(fileName); Serial.println(F(" has been detected"));
    iteration++;
  } while (SD.exists(fileName.c_str()));
  log_file = SD.open(fileName.c_str(), FILE_WRITE);
  Serial.print(fileName);
  Serial.println(F(" has been detected"));
  Serial.println();

  // Create CSV headers
  stringBuffer += "timeSinceInit(s),";
  stringBuffer += "temperature(C),";  
  stringBuffer += "pressure,";
  stringBuffer += "altitude(m),";
  stringBuffer += "humidity,";
  if (Balloon_BME280_Present) {
    stringBuffer += "balloonTemp,";
    stringBuffer += "balloonPressure,";
  }
  stringBuffer += "gyroscopeX,";
  stringBuffer += "gyroscopeY,";
  stringBuffer += "gyroscopeZ,";
  stringBuffer += "accelerationX,";
  stringBuffer += "accelerationY,";
  stringBuffer += "accelerationZ,";
  stringBuffer += "magnetometerX,";
  stringBuffer += "magnetometerY,";
  stringBuffer += "magnetometerZ,";
  stringBuffer += "temperatureAlt(C),";

  write_string_buffer(); //Writes headers to file
  stringBuffer = "";

  if (log_file) {
    Serial.println(F("Log File has Succesfully Opened"));
    Serial.println();
  }
  else {
    Serial.println(F("*****Log File Failed to Open*****"));
    Serial.println();
    error = true;
  }

  //////////////////////////////////////
  // Setup BME280
  bme280.settings.commInterface   = I2C_MODE;
  bme280.settings.I2CAddress      = 0x76;
  bme280.settings.runMode         = 3;
  bme280.settings.tStandby        = 0;
  bme280.settings.filter          = 4;
  bme280.settings.tempOverSample  = 1;
  bme280.settings.humidOverSample = 1;
  bme280.settings.pressOverSample = 5;
  delay(10);

  if (bme280.begin()) {
    Serial.println(F("BME280 Initialized"));
    Serial.println();
  }
  else {
    Serial.println(F("*****BME280 Initialization Failure*****"));
    Serial.println();
    error = true;
  }

  //////////////////////////////////////
  // Setup ball280 BME280
  if (Balloon_BME280_Present) {
    ball280.settings.commInterface   = I2C_MODE;
    ball280.settings.I2CAddress      = 0x76;
    ball280.settings.runMode         = 3;
    ball280.settings.tStandby        = 0;
    ball280.settings.filter          = 4;
    ball280.settings.tempOverSample  = 1;
    ball280.settings.humidOverSample = 1;
    ball280.settings.pressOverSample = 5;
    delay(10);

    if (ball280.begin()) {
      Serial.println(F("Balloon BME280 Initialized"));
      Serial.println();
    }
    else {
      Serial.println(F("*****Balloon BME280 Failed to Initialize*****"));
      Serial.println();
      error = true;
    }
  }

  //////////////////////////////////////
  // Setup LSM9DS1
  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;

  if (imu.begin()) {
    Serial.println(F("LSM9DS1 Initialized"));
    Serial.println();
  }
  else {
    Serial.println(F("*****LSM9DS1 Failed to Initialize*****"));
    Serial.println();
    error = true;
  }

  //////////////////////////////////////
  // Setup MCP9808
  if (mcp9808.begin(0x1F)) {
    Serial.println(F("MCP9808 Initialized"));
    Serial.println();
  }
  else {
    Serial.println(F("*****MCP9808 Failed to Initialize*****"));
    Serial.println();
    error = true;
  }

  if (error) {
    Serial.println(F("*******************************************"));
    Serial.println(F("***********Initialization Errors***********"));
    Serial.println(F("*******************************************"));
    Serial.println();
  }
  else {
    Serial.println(F("---------Successful Initialization---------"));
    Serial.println();
  }
}

//Handles the Status LEDs
void led_management() {
  if (millis() - lastHeartbeatToggle > 1000) {
    digitalWrite(heartbeatLED, !digitalRead(heartbeatLED));
    lastHeartbeatToggle = millis();
  }

  if(error){
    digitalWrite(statusLED2, HIGH);
  }
  else{
    digitalWrite(statusLED2, LOW);
  }
}

// Polls all the sensors in order
void poll_sensors() {
  stringBuffer += String(millis());
  stringBuffer += ',';

  //buffer_float(millis());
  poll_bme280();
  if(Balloon_BME280_Present){poll_ball280();}
  poll_imu();
  poll_mcp();

  packet_count++;
  write_string_buffer();
}

// Onboard altimeter
void poll_bme280() {
  float temp_c   = bme280.readTempC();
  float pressure = bme280.readFloatPressure();
  float alt_m    = bme280.readFloatAltitudeMeters();
  float humidity = bme280.readFloatHumidity();

  if(millis() - lastDisplayedAltitude > 1000){
    lastDisplayedAltitude = millis();
    Serial.print(F("Altitude: "));
    Serial.print(alt_m);
    Serial.println(F(" meters"));
    Serial.print(F("Pressure: "));
    Serial.print(pressure);
    Serial.println(F(" pascals"));
    Serial.println(F("*********************************"));
  }

  if(alt_m >= DEPLOY_HEIGHT){
    consecutiveDeployMeasurements++;
  }
  else{
    consecutiveDeployMeasurements = 0;
  }
    
  // update the max height
  if(alt_m > (max_height + 2)){
    max_height = alt_m;
    consecutiveDescentMeasurements = 0;
    Serial.println(F("Ascending")); Serial.println();
  }
  else if(alt_m < (max_height + 2) || alt_m > (max_height - 2)){
    //Do Nothing - slight noise immunity case
    //Serial.println(F("No significant change in altitude"));
  }
  else{
    Serial.println(F("Descending")); Serial.println();
    consecutiveDescentMeasurements++;
  }
  
  //Look to see if it's time to deploy the cubesat
  if(!cubesatDeployed){
    //Deploy the cubesat if it's been an hour OR if we have been over 70,000 feet (21336m) for over a minute
    if (millis() >= 3600000 || consecutiveDeployMeasurements >= 120) {
      deployCubesat();
      cubesatDeployed = true;
    }
  }

  if(!cutDown){
    if(millis() >= 10800000){
      if(alt_m >= CUTDOWN_HEIGHT){
        balloonCutdown();
      }
      cutDown = true; //Set this even if cutDown doesn't occur so that the altitute check after 3 hrs only happens once
    }
  }

  //string based buffer for writing csv file
  stringBuffer += String(temp_c, precision);
  stringBuffer += ',';
  stringBuffer += String(pressure, precision);
  stringBuffer += ',';
  stringBuffer += alt_m;
  stringBuffer += ',';
  stringBuffer += String(humidity, precision);
  stringBuffer += ',';
}

// Balloon plug Altimeter
void poll_ball280() {
  float temp_c   = ball280.readTempC();
  float pressure = ball280.readFloatPressure();

  //string based buffer for writing csv file
  stringBuffer += String(temp_c, precision);
  stringBuffer += ',';
  stringBuffer += String(pressure, precision);
  stringBuffer += ',';
}

// 9 DOF Inertial Measurement Unit
void poll_imu() {
  imu.readGyro();

  //string based buffer for writing csv file
  stringBuffer += String(imu.calcGyro(imu.gx), precision);
  stringBuffer += ',';
  stringBuffer += String(imu.calcGyro(imu.gy), precision);
  stringBuffer += ',';
  stringBuffer += String(imu.calcGyro(imu.gz), precision);
  stringBuffer += ',';

  imu.readAccel();

  //string based buffer for writing csv file
  stringBuffer += String(imu.calcAccel(imu.ax), precision);
  stringBuffer += ',';
  stringBuffer += String(imu.calcAccel(imu.ay), precision);
  stringBuffer += ',';
  stringBuffer += String(imu.calcAccel(imu.az), precision);
  stringBuffer += ',';

  imu.readMag();

  //string based buffer for writing csv file
  stringBuffer += String(imu.calcMag(imu.mx), precision);
  stringBuffer += ',';
  stringBuffer += String(imu.calcMag(imu.my), precision);
  stringBuffer += ',';
  stringBuffer += String(imu.calcMag(imu.mz), precision);
  stringBuffer += ',';
}

// Reads external Temperature Sensor
void poll_mcp() {
  mcp9808.shutdown_wake(0); // Wake up sensors

  // string based buffer for writing csv file
  String mcp_temp = String(mcp9808.readTempC(), precision);
  stringBuffer += mcp_temp;
  stringBuffer += ',';
  /*
    buffer_float(mcp9808.readTempC());
  */
  mcp9808.shutdown_wake(1); // Sleep sensor
}

// Writes the current string buffer to file
// If the SD buffer is full, it flushes to the SD card
void write_string_buffer() {
  if (log_file) {
    log_file.println(stringBuffer);
    stringBuffer = "";
  }

  if (packet_count > max_packet_count) {
    log_file.flush();
    packet_count = 0;
    stringBuffer = "";
  }
}

//triggers the external payload deployment switch
void deployCubesat(){
  cubesatDeployed = true;
  digitalWrite(cubeSatPin, HIGH); //Enable current flow through the nichrome wire to deploy the cubesat payload
  Serial.println(F("Cubesat Deployment Enabled"));
  delay(2000);
  digitalWrite(cubeSatPin, LOW); //Disable the current flow through the nichrome wire
  Serial.println(F("Cubesat Deployment Disabled"));
}

void balloonCutdown(){
  digitalWrite(cutDownPin, HIGH);
  Serial.println(F("Balloon Cutdown Enabled"));
  delay(10000);
  digitalWrite(cutDownPin, LOW);
  Serial.println(F("Balloon Cutdown Disabled"));
}

