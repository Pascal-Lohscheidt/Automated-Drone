#include <MPU6050_tockn.h>
#include <Wire.h> //For I2C comm
#include <EasyTransferI2C.h>

#include <math.h>
// #include <printf.h> //just fot AVR code
#include <ESP8266WiFi.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <RF24_config.h>


//==== Settings ====
#define MIN_THROTTLE 250 //250 standard
#define MAX_THROTTLE 700 //350 good testing value  <->  900 flightable

//===== Set Points =====
float xAngleSetPoint = 0;
float yAngleSetPoint = 0;
float zAngleSetPoint = 0;

float xVectorSetPoint = 0;
float yVectorSetPoint = 0;

float heightSetPoint = 2; //height in cm

//======== PID Settings =========
#define PID_INTERVAL 1250//1250; //us -> 1 / 800 -> 800hz / 1000us -> 1khz

float heightIFaktor = 0.01; //0.09//0.045

float zAnglePFaktor = 0.1;
float zAngleIFaktor = 0.000; //0.04
float zAngleDFaktor = 35;

float xyMovePFaktor = 3; //16
float xyMoveIFaktor = 0.00; //0.001
float xyMoveDFaktor = 0; //0.001

float xyAnglePFaktor = 0.1;//0.9//0.85;//0.4 //3
float xyAngleIFaktor = 0.015; //0.02 //0.035
float xyAngleDFaktor = 13;//6.7//6.2//5.5 //90

bool IblockedWithHeight = true; 
float PIDHeightconstantAdjustFaktor = 1;

//====== IMU Variables =======
MPU6050 mpu6050(Wire);

//===== Debug Settings =====
#define showsThrottle false
#define showsAngles false
#define showsData false

//==== Loop settings ====
long pidTimer = 0;
long imuTimer = 0;
long evaluateImuTimestamp = 0;
long heightControlTimer = 0;
long deltaTime = 0;
long radioReceiveTimer = 0;

long currentMillisOfCylce; //TODO find out if int is enough for timers to safe speed;
long currentMicrosOfCylce;

//===== I2C Adressing ======
#define I2C_SLAVE_ADDRESS 9

EasyTransferI2C ET;

struct SEND_DATA_STRUCTURE {
    int16_t motorStartUpPhase;
    int16_t throttleA;
    int16_t throttleB;
    int16_t throttleC;
    int16_t throttleD;
};

SEND_DATA_STRUCTURE nanoPWMData;

//==== NRF24 Settings ====

// static const uint8_t SS    = 15;
// static const uint8_t MOSI  = D7;
// static const uint8_t MISO  = D6;
// static const uint8_t SCK   = D5;

RF24 radio(D4, D3); // CE, CSN
const byte address[][6] = {"0", "1"};
int lastOperation = 0;

struct DataPackage
{
  uint8_t operation = 0;
  uint16_t heightAddition = 0;
};

typedef struct DataPackage Data;
Data data;

struct TelemetryData
{
  float height = 0;
  float heightGoal = 0;
  float rollAngle = 0;
  float pitchAngle = 0;
  float yawAngle = 0;
  float batteryVoltage = 0;
};

typedef struct TelemetryData Telemetry;
Telemetry tData;


//float accJitterFactor = 40;

//====Throttles====
int throttleA; //1 -> pin 8
int throttleB; //2 -> pin 9
int throttleC; //3 -> pin 10
int throttleD; //4 -> pin 11

int heightThrottleAddition = 0;

//===== Drone Info ====
float currentXRotation;
float currentYRotation;
float currentZRotation;

float currentHeight;

int motorStartUpPhase = 0; //0 = not started ; 1 = should start; 2 started; -1 emergency stop


//====== Structs ======
struct PIDSavings {
  int lastYValue = 0;
  int lastDifValue = 0;
  long lastTimer = -1;
  double iSum = 0; // integral sum
};

struct PIDSavings heightSavings;
struct PIDSavings xSavings;
struct PIDSavings ySavings;
struct PIDSavings zSavings;

struct PIDSavings xMoveSavings;
struct PIDSavings yMoveSavings;


struct Vector3 {
  float x;
  float y;
  float z;
};

struct Vector3 moveVector;

//================= Set Up ====================

void setup() {
  WiFi.forceSleepBegin();
  Serial.begin(57600);

  Serial.println("--------------Drone is booting up-------------");
  Serial.println("--------------Establishing I2C Connection-------------");
  //Wire.setClock(400000); //Must be called before wire begin
  Wire.begin();

  delay(1000); // giving rest of the modules time to start up

  ET.begin(details(nanoPWMData), &Wire); //Establish an i2c connection for Arduino

  //==== Gyro setup =====
  Serial.println("\n");
  Serial.println("--MPU Setup--");

  mpu6050.begin();
  mpu6050.calcGyroOffsets();
  //===== NRF24 Setup ====
  Serial.println("\n");
  Serial.println("--NRF24 Radio Setup--");
  radio.begin();
  //radio.openWritingPipe(address[1]);
  radio.openReadingPipe(0, address[0]);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();

  //===== Timer Setup ====
  pidTimer = micros();
  imuTimer = micros();
  deltaTime = micros();

  Serial.println("\n");
  Serial.println("--------------Set up - done-------------");
  delay(2000);
}


//==================== Main Loop ============================

void loop() {

  currentMillisOfCylce = millis();


  handleRadioReceive();

  //===== Measure & Control Loop =====

  if(motorStartUpPhase == 2)
  {
    PIDHeightconstantAdjustFaktor = 1;

    //currentHeight = currentHeight * 0.85 + (getDistance(triggerPin, echoPin) - heightOffset) * 0.15;

    handleHeightControls();

    // Serial.println("After Rounding & Height:");
    // Serial.println(micros());

    handleIMU();

    currentMicrosOfCylce = micros(); 
    if(currentMicrosOfCylce - pidTimer > PID_INTERVAL) 
    { 
      pidTimer = currentMicrosOfCylce;
      // handleHeightThrottle();

      if(currentHeight > 1) IblockedWithHeight = false;

      handleYawStablelisation();

      handleAutoHover();
      // Serial.println("After PID:");
      // Serial.println(micros());

    }

    // evaluateIMUData();
    // Serial.println("After IMU:");
    // Serial.println(micros());


    limitThrottle();//important to limit all throttle values

    // Serial.println("End:");
    // Serial.println(micros());

    showDebug();

  }
  else if(motorStartUpPhase == 1)
  {
      //=== Engine Setup ===
  }
  //===== Send done data to Throttle Handling Nano ======
  TransmitDataToNanoRight();
}

void showDebug()
{
  if(showsAngles)
  {
    Serial.print(currentXRotation);
    Serial.print("\t");
    Serial.print(currentYRotation);
    Serial.print("\t");
    Serial.println(currentZRotation);
  }

  if(showsThrottle)
  {
    Serial.print(throttleA);
    Serial.print("\t");
    Serial.print(throttleB);
    Serial.print("\t");
    Serial.print(throttleC);
    Serial.print("\t");
    Serial.println(throttleD);
  }

  if(showsData)
  {
    Serial.println(data.heightAddition);
  }
}

//====== I2C Methods =====

void TransmitDataToNanoRight()
{
  //=== Setting the throttle data ===
  nanoPWMData.motorStartUpPhase = motorStartUpPhase;

  nanoPWMData.throttleA = throttleA;
  nanoPWMData.throttleB = throttleB;
  nanoPWMData.throttleC = throttleC;
  nanoPWMData.throttleD = throttleD;

  ET.sendData(I2C_SLAVE_ADDRESS);
}

//===== NRF24 Methods ====

void handleTranssmission()
{
  //This method handles telemtery transmission
  tData.height = currentHeight;
  tData.heightGoal = heightSetPoint;
  tData.rollAngle = currentXRotation;
  tData.pitchAngle = currentYRotation;
  tData.yawAngle = currentZRotation;
  radio.stopListening();
  radio.write(&tData, sizeof(tData));
  radio.startListening();
}

void handleRadioReceive()
{
  if(radio.available())
  {
    // if (millis() - radioReceiveTimer > 10)
    // {
    //   radioReceiveTimer = millis();
    noInterrupts();
    radio.read(&data, sizeof(data));
    interrupts();
    if (data.operation > 0 && lastOperation != data.operation)
    {
      lastOperation = data.operation;
      Serial.println("operration:");
      Serial.println(data.operation);
      if(data.operation == 1 && motorStartUpPhase == 0)
        motorStartUpPhase = 2; //set´s the start of the motors
      if (data.operation == 3)
        stopEngines();
      if (data.operation == 4)
        startEngines();
    }
    // }

    Serial.println(data.heightAddition);
  }
}

void handleHeightControls()
{
  setThrottleForAll(MIN_THROTTLE + data.heightAddition);
}

//======= autonomous methods =========


void handleHeightThrottle()
{
  setThrottleForAll(MIN_THROTTLE + ApplyPID(currentHeight, heightSetPoint, &heightSavings, 2, heightIFaktor, 0, false, 600));
}

void handleYawStablelisation()
{
  float angleAdjustThrust = ApplyPID(currentZRotation, zAngleSetPoint, &zSavings, zAnglePFaktor * PIDHeightconstantAdjustFaktor, zAngleIFaktor * PIDHeightconstantAdjustFaktor, zAngleDFaktor * PIDHeightconstantAdjustFaktor, true, 2);
  //A-D B-C

  throttleA += angleAdjustThrust;
  throttleD += angleAdjustThrust;
  throttleB -= angleAdjustThrust;
  throttleC -= angleAdjustThrust;
}

void handleAutoHover()
{
  int xMove = -ApplyPID(moveVector.x, xVectorSetPoint, &xMoveSavings, xyMovePFaktor * PIDHeightconstantAdjustFaktor, xyMoveIFaktor * PIDHeightconstantAdjustFaktor, xyMoveDFaktor * PIDHeightconstantAdjustFaktor, true, 7);
  int yMove = -ApplyPID(moveVector.y, yVectorSetPoint, &yMoveSavings, xyAnglePFaktor * PIDHeightconstantAdjustFaktor, xyAngleIFaktor * PIDHeightconstantAdjustFaktor, xyMoveDFaktor * PIDHeightconstantAdjustFaktor, true, 7);

  int xRot = ApplyPID(currentXRotation, xAngleSetPoint, &xSavings, xyAnglePFaktor, xyAngleIFaktor, xyAngleDFaktor, true, 7);
  int yRot = ApplyPID(currentYRotation, yAngleSetPoint, &ySavings, xyAnglePFaktor, xyAngleIFaktor, xyAngleDFaktor, true, 7);

  int x = 0.0 * xMove + 1 * -yRot;
  int y = 0.0 * yMove + 1 * xRot;

  throttleA += (x + y);
  throttleB += (-x + y);
  throttleC += (x - y);
  throttleD += (-x - y);


  //int x = ApplyPID(currentXRotation, xAngleSetPoint, &xSavings, xyAnglePFaktor, xyAngleIFaktor, 1);
  //int y = ApplyPID(currentYRotation, yAngleSetPoint, &ySavings, xyAnglePFaktor, xyAngleIFaktor, 1);
  //  throttleA += (-x + y);
  //  throttleB += (-x - y);
  //  throttleC += (x + y);
  //  throttleD += (x - y);
}

// ========= Gyro Evaluation methods ===========

void handleIMU()
{
  mpu6050.update();

  currentXRotation = currentXRotation * 0.99 + mpu6050.getAngleX() * 0.01;
  currentYRotation = currentYRotation * 0.99 + mpu6050.getAngleY() * 0.01;
  currentZRotation = currentZRotation * 0.99 + mpu6050.getAngleZ() * 0.01;

  showDebug();
}

// void calculateFlightVector()
// {

//   struct Vector3 filterG;
//   filterG.z = -1;

//   //rolls at x & pitches with y

//   //ApplyEulerMatrix(&filterG, angleRollOutput, anglePitchOutput, angleYawOutput);

//   moveVector.x = accXOutput;
//   moveVector.y = accYOutput;
//   moveVector.z = accZOutput;

//   //  Serial.print(moveVector.x);
//   //  Serial.print("\t");
//   //  Serial.println(moveVector.y );

//   //==filtering vector

//   //  moveVector.x -= filterG.x;
//   //  moveVector.y -= filterG.y;
//   //  moveVector.z -= filterG.z;

//   //ApplyEulerMatrix(&filterG, -angleRollOutput, -anglePitchOutput, -angleYawOutput);
// }


//===== Secure Functions ====

void stopEngines()
{
  motorStartUpPhase = -1;
  Serial.println("-------------------");
  Serial.println("Throttle stopped!!");
  Serial.println("-------------------");
}

void startEngines()
{
  motorStartUpPhase = 2;
  Serial.println("-------------------");
  Serial.println("Engines Started");
  Serial.println("-------------------");
}

void setThrottleForAll(int throttle)
{
  throttleA = throttle;
  throttleB = throttle;
  throttleC = throttle;
  throttleD = throttle;
}

void limitThrottle()
{
  throttleA = throttleA < MIN_THROTTLE ? MIN_THROTTLE : throttleA;
  throttleB = throttleB < MIN_THROTTLE ? MIN_THROTTLE : throttleB;
  throttleC = throttleC < MIN_THROTTLE ? MIN_THROTTLE : throttleC;
  throttleD = throttleD < MIN_THROTTLE ? MIN_THROTTLE : throttleD;

  throttleA = throttleA > MAX_THROTTLE ? MAX_THROTTLE : throttleA;
  throttleB = throttleB > MAX_THROTTLE ? MAX_THROTTLE : throttleB;
  throttleC = throttleC > MAX_THROTTLE ? MAX_THROTTLE : throttleC;
  throttleD = throttleD > MAX_THROTTLE ? MAX_THROTTLE : throttleD;

  throttleA = abs(throttleA);
  throttleB = abs(throttleB);
  throttleC = abs(throttleC);
  throttleD = abs(throttleD);

  if (motorStartUpPhase == -1) //must be called after limitation for saftey reasons
  {
    throttleA = 0;
    throttleB = 0;
    throttleC = 0;
    throttleD = 0;
  }
}

//===== Apply Eulermatrix ====
void ApplyEulerMatrix(struct Vector3 *vector, float x, float y, float z)
{
  x /=  57.2957795; //convert in radians
  y /=  57.2957795;
  z /=  57.2957795;

  vector->x = cos(y) * cos(z) * vector->x + (sin(x) * sin(y) * cos(z) - cos(x) * sin(z)) * vector->y + (cos(x) * sin(y) * cos(z) + sin(x) * sin(z)) * vector->z;
  vector->y = cos(y) * sin(z) * vector->x + (sin(x) * sin(y) * sin(z) + cos(x) * cos(z)) * vector->y + (cos(x) * sin(y) * sin(z) - sin(x) * cos(z)) * vector->z;
  vector->z = -sin(y) * vector->x + sin(x) * cos(y) * vector->y + cos(x) * cos(y) * vector->z;
}


//===== PID =======
float ApplyPID(float y, float yo, struct PIDSavings *savings, float pk, float ik, float dk, bool blockAbleWithHeight, float maxISum) {

  float delta = (yo - y);
  float finalValue = pk * delta; //declaring final value and applying the potential part

  if(!blockAbleWithHeight || !IblockedWithHeight)
    savings->iSum += ik * delta; //adding up the integral part

  savings->iSum = minMaxTheValue(maxISum, savings->iSum);
    
  finalValue += savings->iSum; // applying the integral part

  finalValue += dk * (delta / (millis() - savings->lastTimer));

  savings->lastTimer = millis();

  return finalValue;
}


//===== Value Adjustment methods =====

float roundToOneDecimal(float value)
{
  return roundf(value * 10.0) / 10.0; 
}

float minMaxTheValue(float maxV, float value)
{
  value = value > maxV ? maxV : value;
  value = value < -maxV ? -maxV : value;

  return value;
}

