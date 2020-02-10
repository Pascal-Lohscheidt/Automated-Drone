#include "ESC.h"

#define SPEED_MIN (1000)                                  // Set the Minimum Speed in microseconds
#define SPEED_MAX (2000)                                  // Set the Minimum Speed in microseconds

ESC ESC_A (8, SPEED_MIN, SPEED_MAX, 500);                 // ESC_Name (ESC PIN, Minimum Value, Maximum Value, Default Speed, Arm Value)
ESC ESC_B (9, SPEED_MIN, SPEED_MAX, 500);
ESC ESC_C (10, SPEED_MIN, SPEED_MAX, 500);
ESC ESC_D (11, SPEED_MIN, SPEED_MAX, 500);
//==== Settings ====
#define MIN_THROTTLE (1100) //250 standard
#define MAX_THROTTLE (1500) //350 good testing value  <->  900 flightable

//====Throttles====
int throttleA; //1 -> pin 8
int throttleB; //2 -> pin 9
int throttleC; //3 -> pin 10
int throttleD; //4 -> pin 11

int motorStartUpPhase = 1;
bool stopThrottle = false;

int oESC;


void setup()
{
    armMotors();
    Serial.begin(57600);
    Serial.println("--------------Drone is booting up - Throttle Nano-------------");
    Serial.println("--------------ESC Armed-------------");
    Serial.println("--------------Waiting 3 Sec.-------------");
    delay(3000);
    Serial.println("--------------Setup - Done-------------");
}

void loop() {
  setThrottleForAll(MIN_THROTTLE);
  limitThrottle();
  applyThrottle();
  Serial.println("doing smthg");
  delay(10);
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
    // throttleA = 500;
    // throttleB = 500;
    // throttleC = 500;
    // throttleD = 500;
  }
}

void applyThrottle()
{ 
  // if (stopThrottle)
  // {
  //   stopMotors();
  //   return;
  // } 

  ESC_A.speed(throttleA);
  ESC_B.speed(throttleB);
  ESC_C.speed(throttleC);
  ESC_D.speed(throttleD);


}

void stopMotors()
{
    // esc_A.stop();
    // ESC_B.stop();
    // ESC_C.stop();
    // ESC_D.stop();
}

void armMotors()
{
    ESC_A.arm();
    ESC_B.arm();
    ESC_C.arm();
    ESC_D.arm();
}


