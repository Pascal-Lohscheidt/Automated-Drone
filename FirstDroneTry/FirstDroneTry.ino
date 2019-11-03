#include <Wire.h>
#include <math.h>
#include <printf.h>
#include <nRF24L01.h>
#include <RF24_config.h>
#include <RF24.h>
#include <SPI.h>


extern "C"
{
  //void SetThrottle(int port, int throttle);
  void start();
  void Transmit();
}

volatile unsigned int th = 0;
volatile byte chan = 2;

//==== Loop settings

long imuTimer = 0;
long deltaTime = 0;

//==== Echo sensor Settings ====

bool sentEchoAlready = false;
long lastTimeEchoSent = 0;

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


//==== Echo settings ===

int triggerPin = 30;
int echoPin = 31;
float heightOffset;


//==== Settings ====
int minThrottle = 450; //250 standard
int maxThrottle = 700; //350 good testing value  <->  900 flightable

bool stopThrottle = false;

//===== Set Points =====
float xAngleSetPoint = 0;
float yAngleSetPoint = 0;
float zAngleSetPoint = 0;

float xVectorSetPoint = 0;
float yVectorSetPoint = 0;

float heightSetPoint = 2; //height in cm

//======== PID Settings =========
float heightIFaktor = 0.01; //0.09//0.045

float zAnglePFaktor = 0.1;
float zAngleIFaktor = 0.000; //0.04
float zAngleDFaktor = 35;

float xyMovePFaktor = 3; //16
float xyMoveIFaktor = 0.00; //0.001
float xyMoveDFaktor = 0; //0.001

float xyAnglePFaktor = 0.8;//0.9//0.85;//0.4 //3
float xyAngleIFaktor = 0.015; //0.02 //0.035
float xyAngleDFaktor = 7;//6.7//6.2//5.5 //90

bool IblockedWithHeight = true; 
float PIDHeightconstantAdjustFaktor = 1;


//==== PID variables =====
long pidTimer = 0;
int pidInterval = 7000;//1250; //us -> 1 / 800 -> 800hz


//float accJitterFactor = 40;

//====Throttles====
int throttleA; //1 -> pin 8
int throttleB; //2 -> pin 9
int throttleC; //3 -> pin 10
int throttleD; //4 -> pin 11

//===== Drone Info ====
float currentXRotation;
float currentYRotation;
float currentZRotation;

float currentHeight;


//===== Gyro variables =======

int gyroX, gyroY, gyroZ;
double accX, accY, accZ, accTotalVector;
int temperature;
long gyroXcal, gyroYcal, gyroZcal;

long accXcal, accYcal, accZcal;

long loopTimer;
bool setUpDone = false;

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
  start();
  Wire.begin();
  Serial.begin(57600);
  Serial.println("--------------Drone is booting up-------------");

  //==== Gyro setup =====
  Serial.println("\n");
  Serial.println("--MPU Setup--");

  setup_MPU_registers();

  for (int cal_int = 0; cal_int < 2000 ; cal_int ++) {                 //Run this code 2000 times
    read_MPU_data();                                              //Read the raw acc and gyro data from the MPU-6050
    gyroXcal += gyroX;                                              //Add the gyro x-axis offset to the gyro_x_cal variable
    gyroYcal += gyroY;                                              //Add the gyro y-axis offset to the gyro_y_cal variable
    gyroZcal += gyroZ;                                              //Add the gyro z-axis offset to the gyro_z_cal variable
    delay(4);                                                       //Delay 4ms to simulate the 250Hz program loop
  }

  gyroXcal /= 2000;                                                  //Divide the gyro_x_cal variable by 2000 to get the avarage offset
  gyroYcal /= 2000;                                                  //Divide the gyro_y_cal variable by 2000 to get the avarage offset
  gyroZcal /= 2000;                                                  //Divide the gyro_z_cal variable by 2000 to get the avarage offset

  calibrateGyroAngles();

  //  for (int cal_int = 0; cal_int < 2000 ; cal_int ++) {                 //Run this code 2000 times
  //    read_MPU_data();                                              //Read the raw acc and gyro data from the MPU-6050
  //    accXcal += accX;
  //    accYcal += accY;
  //    accZcal += accZ;
  //    delay(3);                                                       //Delay 3us to simulate the 250Hz program loop
  //  }
  //
  //  accXcal /= 2000;
  //  accYcal /= 2000;
  //  accZcal /= 2000;

  setUpDone = true;

  //===== Echo Setup ===
  Serial.println("\n");
  Serial.println("--Echo sensor Setup--");
  pinMode(triggerPin, OUTPUT);
  pinMode(echoPin, INPUT);
  digitalWrite(triggerPin, HIGH);
  heightOffset = getDistance(triggerPin, echoPin);
  currentHeight = 0;

  Serial.println("Height is now zero");
  Serial.println("------------------\n\n");

  //===== NRF24 Setup ====
  Serial.println("\n");
  Serial.println("--NRF24 Radio Setup--");
  radio.begin();
  //radio.openWritingPipe(address[1]);
  radio.openReadingPipe(0, address[0]);
  radio.setPALevel(RF24_PA_HIGH);
  radio.startListening();

  //===== Timer Setup ====
  loopTimer = micros();
  pidTimer = micros();
  imuTimer = micros();
  deltaTime = micros();
  
  //=== Engine Setup ===
  fireUpEngines();
  applyThrottle();
  //Serial.println("\n");
  //Serial.println("--------------Set up - done-------------");
}

unsigned int CreateBitSignal(unsigned int throttle)
{
  throttle += 48; //adding the 48 to make sure the signal is correct
  int checksum = 0;
  int checksumData = throttle * 2;
  checksum = (checksumData ^ (checksumData >> 4) ^ (checksumData >> 8)) & 0xf;

  unsigned int finalSignal = (throttle << 5) + checksum;
  //Serial.println(finalSignal);
  return finalSignal;
}

//==================== Main Loop ============================

void loop() {

  handleRadioReceive();

  
  PIDHeightconstantAdjustFaktor = 1;
  
  //===== Control Loop =====

  //currentHeight = currentHeight * 0.85 + (getDistance(triggerPin, echoPin) - heightOffset) * 0.15;
  
  
  //=== Define Rotation values ===
  currentYRotation = anglePitchOutput;
  currentXRotation = angleRollOutput;
  currentZRotation = angleYawOutput;
  
  //=== Define move vector values ===
  //calculateFlightVector();
  
  if(micros() - pidTimer > pidInterval)
  { 
    pidTimer = micros();
    //handleHeightThrottle();

    setThrottleForAll(minThrottle);

    if(currentHeight > 1) IblockedWithHeight = false;
  
    handleYawStablelisation();

    handleAutoHover();
  }
 else if (micros() - imuTimer > 4000) // 3560
 {
   //Serial.println(long(micros() - imuTimer));
   evaluateIMUData();
   imuTimer = micros();
 }

  Serial.print(currentHeight);
  Serial.print("\t");
  Serial.print(currentXRotation);
  Serial.print("\t");
  Serial.print(currentYRotation);
  Serial.print("\t");
  Serial.println(currentZRotation);

  limitThrottle();//important to limit all throttle values
  applyThrottle();

  //Serial.println((long)(micros() - loopTimer));
}




//==== Echo Methods =====

float getDistance(int triggerPort, int echoPort)
{
  float distance;
  float deltaTime;

  digitalWrite(triggerPort, LOW);
  delayMicroseconds(3);
  noInterrupts();
  digitalWrite(triggerPort, HIGH); //Trigger Impuls 10 us
  delayMicroseconds(10);
  interrupts();
  digitalWrite(triggerPort, LOW);
  deltaTime = pulseIn(echoPort, HIGH); // Echo-Zeit messen

  deltaTime /= 2; // Zeit halbieren
  distance = deltaTime / 29.1; // Zeit in Zentimeter umrechnen
  return (distance);
}


float getHeightOfEcho(int triggerPort, int echoPort)
{
  //====== Creating the fixed Echo Pulse=========
  if(!sentEchoAlready)
  {
    sentEchoAlready = true;
    digitalWrite(triggerPort, LOW);
    delayMicroseconds(3);
    noInterrupts();
    digitalWrite(triggerPort, HIGH); //Trigger Impuls 10 us
    lastTimeEchoSent = micros();
    delayMicroseconds(10);
    interrupts();
    digitalWrite(triggerPort, LOW);
  }
  else if(digitalRead(echoPort) == LOW)
  {
    sentEchoAlready = false;
    float deltaTime = micros() - lastTimeEchoSent;
    deltaTime /= 2; // Zeit halbieren
    float distance = deltaTime / 29.1; // Zeit in Zentimeter umrechnen
    return distance;
  }

  return currentHeight;
  
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

      if (data.operation == 3)
        stopEngines();

      if (data.operation == 4)
        startEngines();
    }

    //    Serial.println("_____Joystick Signal_____");
    //    Serial.print("rX: ");
    //    Serial.print(data.rX);
    //
    //    Serial.print(" - rY: ");
    //    Serial.println(data.rY);
    //
    //    Serial.print(" - .lX: ");
    //    Serial.print(data.lX);
    //
    //    Serial.print(" - .lY: ");
    //    Serial.println(data.lY);
  }
}





//======= autonomous methods =========


void handleHeightThrottle()
{
  setThrottleForAll(minThrottle + ApplyPID(currentHeight, heightSetPoint, &heightSavings, 2, heightIFaktor, 0, false, 600));
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

  //  Serial.print(throttleA);
  //  Serial.print("\t");
  //  Serial.print(throttleB);
  //  Serial.print("\t");
  //  Serial.print(throttleC);
  //  Serial.print("\t");
  //  Serial.println(throttleD);


  //int x = ApplyPID(currentXRotation, xAngleSetPoint, &xSavings, xyAnglePFaktor, xyAngleIFaktor, 1);
  //int y = ApplyPID(currentYRotation, yAngleSetPoint, &ySavings, xyAnglePFaktor, xyAngleIFaktor, 1);
  //  throttleA += (-x + y);
  //  throttleB += (-x - y);
  //  throttleC += (x + y);
  //  throttleD += (x - y);
}





//========== Throttle Steering ============

void fireUpEngines()
{
  Serial.println("Fire up Engines!!");
  for (int i = 1; i <= 800; i++)
  {
    Serial.print("Engine boost: ");
    Serial.println(i);
    setThrottleForAll(minThrottle + 150);
    limitThrottle();
    applyThrottle();
    delay(1);
    Serial.print("\n");
  }
  Serial.println("Done \n\n");
}


void setThrottleForAll(int throttle)
{
  throttleA = throttle;
  throttleB = throttle;
  throttleC = throttle;
  throttleD = throttle;
}

float getAverageThrottle()
{
  return (float)(throttleA + throttleB + throttleC + throttleD) / 4;
}

void limitThrottle()
{
  throttleA = throttleA < minThrottle ? minThrottle : throttleA;
  throttleB = throttleB < minThrottle ? minThrottle : throttleB;
  throttleC = throttleC < minThrottle ? minThrottle : throttleC;
  throttleD = throttleD < minThrottle ? minThrottle : throttleD;

  throttleA = throttleA > maxThrottle ? maxThrottle : throttleA;
  throttleB = throttleB > maxThrottle ? maxThrottle : throttleB;
  throttleC = throttleC > maxThrottle ? maxThrottle : throttleC;
  throttleD = throttleD > maxThrottle ? maxThrottle : throttleD;

  throttleA = abs(throttleA);
  throttleB = abs(throttleB);
  throttleC = abs(throttleC);
  throttleD = abs(throttleD);

  if (stopThrottle)
  {
    throttleA = 1;
    throttleB = 1;
    throttleC = 1;
    throttleD = 1;
  }
}

void applyThrottle()
{
  if (stopThrottle) return;
  noInterrupts();
  th = CreateBitSignal(throttleA);
  chan = 1;
  Transmit();

  th = CreateBitSignal(throttleB);
  chan = 2;
  Transmit();

  th = CreateBitSignal(throttleC);
  chan = 4;
  Transmit();

  th = CreateBitSignal(throttleD);
  chan = 8;
  Transmit();
  interrupts();
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
  read_MPU_data();                                                //Read the raw acc and gyro data from the MPU-6050

  gyroX -= gyroXcal;                                                //Subtract the offset calibration value from the raw gyro_x value
  gyroY -= gyroYcal;                                                //Subtract the offset calibration value from the raw gyro_y value
  gyroZ -= gyroZcal;                                                //Subtract the offset calibration value from the raw gyro_z value

  //Gyro angle calculations
  //0.0000611 = 1 / (250Hz * 65.5)
  //float nv = 1 / ((1000000 / (micros() - deltaTime)) * 65.5);
  //double nv = (1/65.5) * ((micros() - deltaTime) / 1000000);
  double nv = 0.0000611;
  anglePitchGyro += gyroY * nv;                                   //Calculate the traveled pitch angle and add this to the angle_pitch variable
  angleRollGyro += gyroX * nv;                                    //Calculate the traveled roll angle and add this to the angle_roll variable
  angleYawGyro += gyroZ * nv;

  deltaTime = micros();
  
  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
  anglePitchGyro -= anglePitchGyro * sin(gyroZ * 0.000001066);               //If the IMU has yawed transfer the roll angle to the pitch angel
  angleRollGyro += angleRollGyro * sin(gyroZ * 0.000001066);               //If the IMU has yawed transfer the pitch angle to the roll angel

  calibrateGyroAngles();

  //To dampen the pitch and roll angles a complementary filter is used
  anglePitchOutput = anglePitchOutput * 0.90 + anglePitchGyro * 0.1;   //Take 90% of the output pitch value and add 10% of the raw pitch value
  angleRollOutput = angleRollOutput * 0.90 + angleRollGyro * 0.1;      //Take 90% of the output roll value and add 10% of the raw roll value
  angleYawOutput = angleYawOutput * 0.90 + angleYawGyro * 0.1;


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
    //anglePitchGyro = anglePitchGyro * 0.995 + anglePitchAcc * 0.005;     //Correct the drift of the gyro pitch angle with the accelerometer pitch angle 0.9996 0.000
    //angleRollGyro = angleRollGyro * 0.995 + angleRollAcc * 0.005;        //Correct the drift of the gyro roll angle with the accelerometer roll angle
    anglePitchGyro = anglePitchGyro * 1 + anglePitchAcc * 0;     //Correct the drift of the gyro pitch angle with the accelerometer pitch angle 0.9996 0.000
    anglePitchGyro = anglePitchGyro * 1 + anglePitchAcc * 0;     //Correct the drift of the gyro pitch angle with the accelerometer pitch angle 0.9996 0.000
  }
  else {                                                               //At first start
    anglePitchGyro = anglePitchAcc;                                     //Set the gyro pitch angle equal to the accelerometer pitch angle
    angleRollGyro = angleRollAcc;                                       //Set the gyro roll angle equal to the accelerometer roll angle
    set_gyro_angles = true;                                            //Set the IMU started flag
  }
}

void setup_MPU_registers() {
  //Activate the MPU-6050
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x6B);                                                    //Send the requested starting register
  Wire.write(0x00);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
  //Configure the accelerometer (+/-8g)
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1C);                                                    //Send the requested starting register
  Wire.write(0x10);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
  //Configure the gyro (500dps full scale)
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1B);                                                    //Send the requested starting register
  Wire.write(0x08);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
}

void read_MPU_data() {                                            //Subroutine for reading the raw gyro and accelerometer data
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x3B);                                                    //Send the requested starting register
  Wire.endTransmission();                                              //End the transmission
  Wire.requestFrom(0x68, 14);                                          //Request 14 bytes from the MPU-6050
  while (Wire.available() < 14);                                       //Wait until all the bytes are received
  accX = Wire.read() << 8 | Wire.read();                              //Add the low and high byte to the acc_x variable
  accY = Wire.read() << 8 | Wire.read();                              //Add the low and high byte to the acc_y variable
  accZ = Wire.read() << 8 | Wire.read();                              //Add the low and high byte to the acc_z variable
  temperature = Wire.read() << 8 | Wire.read();                        //Add the low and high byte to the temperature variable
  gyroX = Wire.read() << 8 | Wire.read();                             //Add the low and high byte to the gyro_x variable
  gyroY = Wire.read() << 8 | Wire.read();                             //Add the low and high byte to the gyro_y variable
  gyroZ = Wire.read() << 8 | Wire.read();                             //Add the low and high byte to the gyro_z variable
}






//===== Secure Functions ====

void stopEngines()
{
  stopThrottle = true;
  Serial.println("-------------------");
  Serial.println("Throttle stopped!!");
  Serial.println("-------------------");
}

void startEngines()
{
  stopThrottle = false;
  Serial.println("-------------------");
  Serial.println("Engines Started");
  Serial.println("-------------------");
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

  return *finalValue;
}

