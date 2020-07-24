#include "ESC.h"
#include <Wire.h> //For I2C comm
#include <EasyTransferI2C.h>

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

SEND_DATA_STRUCTURE myData;

//===== PWM Settings ======
#define SPEED_MIN (1004)                                  // Set the Minimum Speed in microseconds
#define SPEED_MAX (2004)                                  // Set the Minimum Speed in microseconds

ESC ESCA (8, SPEED_MIN, SPEED_MAX, 500);                 // ESC_Name (ESC PIN, Minimum Value, Maximum Value, Default Speed, Arm Value)
ESC ESCB (9, SPEED_MIN, SPEED_MAX, 500);
ESC ESCC (10, SPEED_MIN, SPEED_MAX, 500);
ESC ESCD (11, SPEED_MIN, SPEED_MAX, 500);

//==== Settings ====
#define MIN_THROTTLE (1100) //250 standard
#define MAX_THROTTLE (1300) //350 good testing value  <->  900 flightable

//====Throttles====
int throttleA; //1 -> pin 8
int throttleB; //2 -> pin 9
int throttleC; //3 -> pin 10
int throttleD; //4 -> pin 11

int currentMotor = 0;

int motorStartUpPhase = 1;
bool stopThrottle = false;

//==== Timers ====
long throttleTimer = 0;

void setup()
{
    Serial.begin(57600);

    Serial.println("--------------Drone is booting up - Throttle Nano V2-------------");
    
    //Wire.setClock(400000); //Must be called before wire begin
    Wire.begin(I2C_SLAVE_ADDRESS); //Slave musst tell adress

    ET.begin(details(myData), &Wire);
    Wire.onReceive(receive);

    Serial.println("--------------Waiting 5 Sec.-------------");

    pinMode(13, OUTPUT);                               // LED Visual Output
    ESCA.arm();                                            // Send the Arm value so the ESC will be ready to take commands
    ESCB.arm();
    ESCC.arm();
    ESCD.arm();
    digitalWrite(13, HIGH);                            // LED High Once Armed
    delay(5000);   
    Serial.println("--------------ESC Armed-------------");          

    Serial.println("--------------Setup - Done-------------");

}

void applyThrottle()
{ 
  if (stopThrottle)
  {
    stopMotors();
    return;
  }

  if(currentMotor == 0) ESCA.speed(throttleA);
  else if(currentMotor == 1) ESCB.speed(throttleB);
  else if(currentMotor == 2) ESCC.speed(throttleC);
  else if(currentMotor == 3) ESCD.speed(throttleD);

  currentMotor++;
  if(currentMotor > 3) currentMotor = 0;

}

void stopMotors()
{
    ESCA.stop();
    ESCB.stop();
    ESCC.stop();
    ESCD.stop();
}

void armMotors()
{
 
}


void loop() {

  //===== Data Receive ====
  if(ET.receiveData()) {Serial.println(myData.throttleA);} // just to make sure nothing weird happens
  
  applyReceivedThrottleValues();
  limitThrottle();

  applyThrottle();
}

void receive(int numBytes) {
}

void applyReceivedThrottleValues()
{
  throttleA = myData.throttleA;
  throttleB = myData.throttleB;
  throttleC = myData.throttleC;
  throttleD = myData.throttleD;

  // throttleA = 1150;
  // throttleB = 1150;
  // throttleC = 1150;
  // throttleD = 1150;
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

  if (stopThrottle) //must be called after limitation for saftey reasons
  {
    throttleA = 500;
    throttleB = 500;
    throttleC = 500;
    throttleD = 500;
  }
}




