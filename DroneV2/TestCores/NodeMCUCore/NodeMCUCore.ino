#include <Wire.h> //For I2C comm
#include <EasyTransferI2C.h>

#include <math.h>
// #include <printf.h>

#include <nRF24L01.h>
#include <RF24_config.h>
#include <RF24.h>

#include <SPI.h>

//===== Debug Settings =====
#define showsThrottle false
#define showsAngles true

//==== Loop settings ====
long pidTimer = 0;
long imuTimer = 0;
long evaluateImuTimestamp = 0;
long heightControlTimer = 0;
long deltaTime = 0;

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
RF24 radio(6, 7); // CE, CSN
const byte address[][6] = {"0", "1"};
int lastOperation = 0;

struct DataPackage
{
  int operation = 0;

  int rX = 0;
  int rY = 0;
  int lX = 0;
  int lY = 0;
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

//==== Settings ====
#define MIN_THROTTLE 1050 //250 standard
#define MAX_THROTTLE 1500 //350 good testing value  <->  900 flightable

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

int motorStartUpPhase = 2; //0 = not started ; 1 = should start; 2 started; -1 emergency stop

//===== Gyro variables =======

int16_t gyroX, gyroY, gyroZ;
double accX, accY, accZ, accTotalVector;
int temperature;
long gyroXcal, gyroYcal, gyroZcal;

long accXcal, accYcal, accZcal;

long loopTimer;
bool IMU_SetUpDone = false;

long accLoopTimer;
double accXAverageSum;
double accYAverageSum;
double accZAverageSum;
float finalAccX;
float finalAccY;
int accAverageSumCounter = 0;

float anglePitchGyro, angleRollGyro, angleYawGyro;
int angle_pitch_buffer, angle_roll_buffer;
boolean set_gyro_angles;
float angleRollAcc, anglePitchAcc;
float anglePitchOutput, angleRollOutput, angleYawOutput;

float accXOutput, accYOutput, accZOutput;

//====== Average Filter =====
int averageCounterMax = 10;
int averageCounter = 0;

float averageSumVectorX;
float finalValueVectorX; //bad idea?

float averageSumVectorY;
float finalValueVectorY; //bad idea?

float averageSumVectorZ;
float finalValueVectorZ; //bad idea?

float averageSumRoll;
float finalValueRoll; //bad idea?

float averageSumPitch;
float finalValuePitch; //bad idea?

float averageSumYaw;
float finalValueYaw; //bad idea?

//====== Structs ====== => 
//The PID regulator needs information from previous iterations. 
//Inorder to make it more reusable i created that data container
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
  Serial.begin(57600);
  Serial.println("--------------Drone is booting up-------------");
  Serial.println("--------------Establishing I2C Connection-------------");
  Wire.setClock(400000); //Must be called before wire begin
  Wire.begin();
  delay(1000); // giving rest of the modules time to start up

  ET.begin(details(nanoPWMData), &Wire); //Establish an i2c connection for Arduino

  //==== Gyro setup =====
  Serial.println("\n");
  Serial.println("--MPU Setup--");

  handleIMUSetup();
  Serial.println("--MPU Setup 2--");
  for (int cal_int = 0; cal_int < 2000 ; cal_int ++) {                 //Run this code 2000 times
    readAndCheckIMU();                                              //Read the raw acc and gyro data from the MPU-6050
    gyroXcal += gyroX;                                              //Add the gyro x-axis offset to the gyro_x_cal variable
    gyroYcal += gyroY;                                              //Add the gyro y-axis offset to the gyro_y_cal variable
    gyroZcal += gyroZ;                                              //Add the gyro z-axis offset to the gyro_z_cal variable
    delayMicroseconds(2500);                                        //Delay 2.5ms to simulate the 400Hz program loop
  }
  Serial.println("--MPU Setup 3--");

  gyroXcal /= 2000;                                                  //Divide the gyro_x_cal variable by 2000 to get the avarage offset
  gyroYcal /= 2000;                                                  //Divide the gyro_y_cal variable by 2000 to get the avarage offset
  gyroZcal /= 2000;                                                  //Divide the gyro_z_cal variable by 2000 to get the avarage offset

  calibrateGyroAngles();
  Serial.println("--MPU Setup 4--");
  IMU_SetUpDone = true; //this variable just stands for IMU consider renaming it
  //===== NRF24 Setup ====
  Serial.println("\n");
  Serial.println("--NRF24 Radio Setup--");
  // radio.begin();
  // //radio.openWritingPipe(address[1]);
  // radio.openReadingPipe(0, address[0]);
  // radio.setPALevel(RF24_PA_HIGH);
  // radio.startListening();

  //===== Timer Setup ====
  loopTimer = micros();
  pidTimer = micros();
  imuTimer = micros();
  deltaTime = micros();

  Serial.println("\n");
  Serial.println("--------------Set up - done-------------");
  delay(2000);
}

void loop()
{
  if (micros() - imuTimer >= 2500) // 3560
  {
    evaluateIMUData();
    imuTimer = micros();
  }

  showDebug();
}

//==================== Main Loop ============================

// void loop() {

//   currentMillisOfCylce = millis();


//   // handleRadioReceive();

//   //===== Measure & Control Loop =====

//   if(motorStartUpPhase == 2)
//   {
//     PIDHeightconstantAdjustFaktor = 1;
   
//     //=== Define Rotation values ===
//     currentYRotation = roundToOneDecimal(anglePitchOutput);
//     currentXRotation = roundToOneDecimal(angleRollOutput);
//     currentZRotation = roundToOneDecimal(angleYawOutput);


//     //currentHeight = currentHeight * 0.85 + (getDistance(triggerPin, echoPin) - heightOffset) * 0.15;

//     handleHeightControls();

//     // Serial.println("After Rounding & Height:");
//     // Serial.println(micros());

//     currentMicrosOfCylce = micros(); 
//     if(currentMicrosOfCylce - pidTimer > PID_INTERVAL) 
//     { 
//       pidTimer = currentMicrosOfCylce;
//       //handleHeightThrottle();

//       if(currentHeight > 1) IblockedWithHeight = false;

//       handleYawStablelisation();

//       handleAutoHover();
//       // Serial.println("After PID:");
//       // Serial.println(micros());

//     }
    
//     if (micros() - imuTimer > 2000) // 3560
//     {
//       evaluateIMUData();
//       imuTimer = micros();
//     }

//     // evaluateIMUData();
//     // Serial.println("After IMU:");
//     // Serial.println(micros());


//     limitThrottle();//important to limit all throttle values


//     //===== Send done data to Throttle Handling Nano ======
//     TransmitDataToNanoRight();

//     // Serial.println("End:");
//     // Serial.println(micros());

//     showDebug();

//   }
//   else if(motorStartUpPhase == 1)
//   {
//       //=== Engine Setup ===
//   }
// }

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
  if (radio.available())
  {
    radio.read(&data, sizeof(data));

    if (data.operation > 0 && lastOperation != data.operation)
    {
      lastOperation = data.operation;
      Serial.println("operration:");
      Serial.println(data.operation);

      if(data.operation == 1 && motorStartUpPhase == 0)
        motorStartUpPhase = 1; //set´s the start of the motors

      if (data.operation == 3)
        stopEngines();

      if (data.operation == 4)
        startEngines();
    }
  }
}

void handleHeightControls()
{
  if(currentMillisOfCylce - heightControlTimer > 10)
  {
    int t = -map(data.lX, -512, 512, -2, 3); // required because of poti errors
    heightThrottleAddition += t;
    heightControlTimer = currentMillisOfCylce;
    if(t < -1) t *= 10; // This is required because i want the motors to faster throttle down than up for safety reasons
    
    heightThrottleAddition = heightThrottleAddition < 0 ? 0 : heightThrottleAddition;
    heightThrottleAddition = heightThrottleAddition > MAX_THROTTLE - MIN_THROTTLE ? MAX_THROTTLE - MIN_THROTTLE : heightThrottleAddition; 
  }

  setThrottleForAll(MIN_THROTTLE + heightThrottleAddition);
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

void calculateFlightVector()
{

  struct Vector3 filterG;
  filterG.z = -1;

  //rolls at x & pitches with y

  //ApplyEulerMatrix(&filterG, angleRollOutput, anglePitchOutput, angleYawOutput);

  moveVector.x = accXOutput;
  moveVector.y = accYOutput;
  moveVector.z = accZOutput;

  //  Serial.print(moveVector.x);
  //  Serial.print("\t");
  //  Serial.println(moveVector.y );

  //==filtering vector

  //  moveVector.x -= filterG.x;
  //  moveVector.y -= filterG.y;
  //  moveVector.z -= filterG.z;

  //ApplyEulerMatrix(&filterG, -angleRollOutput, -anglePitchOutput, -angleYawOutput);
}



void evaluateIMUData()
{
  readAndCheckIMU();                                             //Read the raw acc and gyro data from the MPU-6050

  gyroX -= gyroXcal;                                                //Subtract the offset calibration value from the raw gyro_x value
  gyroY -= gyroYcal;                                                //Subtract the offset calibration value from the raw gyro_y value
  gyroZ -= gyroZcal;                                                //Subtract the offset calibration value from the raw gyro_z value

  //Gyro angle calculations
  //0.0000611 = 1 / (250Hz * 65.5)
  //float nv = 1 / ((1000000 / (micros() - deltaTime)) * 65.5);
  //double nv = (double)(1/65.5) * ((double)(micros() - deltaTime) / (double)1000000);
  // double nv = 0.0000611;
  double nv = 0.0000381; //fixed 400Hz 
  anglePitchGyro += gyroY * nv;                                   //Calculate the traveled pitch angle and add this to the angle_pitch variable
  angleRollGyro += gyroX * nv;                                    //Calculate the traveled roll angle and add this to the angle_roll variable
  angleYawGyro += gyroZ * nv;

  deltaTime = micros();
  
  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
  anglePitchGyro -= anglePitchGyro * sin(gyroZ * 0.000001066);               //If the IMU has yawed transfer the roll angle to the pitch angel
  angleRollGyro += angleRollGyro * sin(gyroZ * 0.000001066);               //If the IMU has yawed transfer the pitch angle to the roll angel

  calibrateGyroAngles();

  //To dampen the pitch and roll angles a complementary filter is used
  anglePitchOutput = anglePitchOutput * 0.99 + anglePitchGyro * 0.01;   //Take 90% of the output pitch value and add 10% of the raw pitch value
  angleRollOutput = angleRollOutput * 0.99 + angleRollGyro * 0.01;      //Take 90% of the output roll value and add 10% of the raw roll value
  angleYawOutput = angleYawOutput * 0.99 + angleYawGyro * 0.01;


//  accXOutput = accXOutput * 0.995 + ((accX) - accXcal) * 0.005;
//  accYOutput = accYOutput * 0.995 + ((accY) - accYcal) * 0.005;
//  accZOutput = accZOutput * 0.995 + ((accZ) - accZcal) * 0.005;

  //angle_pitch_output = angle_pitch;
  //angle_roll_output = angle_roll;

}

void calibrateGyroAngles()
{
  //Accelerometer angle calculations
  accTotalVector = sqrt((accX * accX) + (accY * accY) + (accZ * accZ)); //Calculate the total accelerometer vector
  //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
  anglePitchAcc = asin((float)accX / accTotalVector) * -57.296;     //Calculate the pitch angle
  angleRollAcc = asin((float)accY / accTotalVector) * 57.296;     //Calculate the roll angle

  //Place the MPU-6050 spirit level and note the values in the following two lines for calibration
  anglePitchAcc -= -2.30;                                              //Accelerometer calibration value for pitch
  angleRollAcc -= -0.49;                                               //Accelerometer calibration value for roll

  if (set_gyro_angles) {                                               //If the IMU is already started
    anglePitchGyro = anglePitchGyro * 0.995 + anglePitchAcc * 0.005;     //Correct the drift of the gyro pitch angle with the accelerometer pitch angle 0.9996 0.000
    angleRollGyro = angleRollGyro * 0.995 + angleRollAcc * 0.005;        //Correct the drift of the gyro roll angle with the accelerometer roll angle
    // anglePitchGyro = anglePitchGyro * 1 + anglePitchAcc * 0;     //Correct the drift of the gyro pitch angle with the accelerometer pitch angle 0.9996 0.000
    // anglePitchGyro = anglePitchGyro * 1 + anglePitchAcc * 0;     //Correct the drift of the gyro pitch angle with the accelerometer pitch angle 0.9996 0.000
  }
  else {                                                               //At first start
    anglePitchGyro = anglePitchAcc;                                     //Set the gyro pitch angle equal to the accelerometer pitch angle
    angleRollGyro = angleRollAcc;                                       //Set the gyro roll angle equal to the accelerometer roll angle
    set_gyro_angles = true;                                            //Set the IMU started flag
  }
}

void writeOnIMU(uint8_t regAdress, uint8_t data)
{
  Wire.beginTransmission(0x68); //Start Com. IMU on given Adress
  Wire.write(regAdress);
  Wire.write(data);
  Wire.endTransmission();
}

void readAndCheckIMU()
{
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x3B);                                                    //Send the requested starting register
  Wire.endTransmission();                                              //End the transmission
  Wire.requestFrom(0x68, (uint8_t)14);                                          //Request 14 bytes from the MPU-6050
      Serial.println("--MPU Setup 2.5--");
  while (Wire.available() < 14);                                       //Wait until all the bytes are received
  accX = (double)((Wire.read()<<8) | Wire.read());                             //Add the low and high byte to the acc_x variable
  accY = (double)((Wire.read()<<8) | Wire.read());                             //Add the low and high byte to the acc_y variable
  accZ = (double)((Wire.read()<<8) | Wire.read());                             //Add the low and high byte to the acc_z variable
  temperature = (((int16_t)Wire.read()<<8) | Wire.read());                       //Add the low and high byte to the temperature variable
  gyroX = (((int16_t)Wire.read()<<8) | Wire.read());                             //Add the low and high byte to the gyro_x variable
  gyroY = (((int16_t)Wire.read()<<8) | Wire.read());                             //Add the low and high byte to the gyro_y variable
  gyroZ = (((int16_t)Wire.read()<<8) | Wire.read());                             //Add the low and high byte to the gyro_z variable
}

void handleIMUSetup()
{
  writeOnIMU(0x6B, 0x00); //Handle Initial Starting register
  writeOnIMU(0x1C, 0x10); //Handle Accel Config
  writeOnIMU(0x1B, 0x08); //Handle Gyro Config
}



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
  motorStartUpPhase = 1;
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
    throttleA = 500;
    throttleB = 500;
    throttleC = 500;
    throttleD = 500;
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

void dampenWithAverage(float addValue, double *averageSum, float *finalValue)
{
  if (averageCounter < averageCounterMax)
  {
    *averageSum += addValue;
  }
  else
  {
    averageCounter = 0;
    *finalValue = *averageSum / averageCounterMax;
    averageSum = 0;
  }
}

