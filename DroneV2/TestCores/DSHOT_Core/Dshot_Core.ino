#include <Wire.h>
#include <math.h>
#include <printf.h>
#include <SPI.h>

extern "C"
{
  //void SetThrottle(int port, int throttle);
  void start();
  void Transmit();
}

volatile unsigned int th = 0;
volatile byte chan = 2;

//==== Settings ====
#define MIN_THROTTLE = 450; //250 standard
#define MAX_THROTTLE = 700; //350 good testing value  <->  900 flightable

//====Throttles====
int throttleA; //1 -> pin 8
int throttleB; //2 -> pin 9
int throttleC; //3 -> pin 10
int throttleD; //4 -> pin 11

void setup()
{
  start();
  Serial.begin(57600);
}

void loop()
{

}


//========== Throttle Steering ============

void fireUpEngines()
{
  Serial.println("Fire up Engines!!");
  for (int i = 1; i <= 800; i++)
  {
    Serial.print("Engine boost: ");
    Serial.println(i);
    setThrottleForAll(MIN_THROTTLE + 30);
    limitThrottle();
    applyThrottle();
    delay(1);
    Serial.print("\n");
  }
  Serial.println("Done \n\n");
  motorStartUpPhase = 2;
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
    throttleA = 1;
    throttleB = 1;
    throttleC = 1;
    throttleD = 1;
  }
}

void applyThrottle()
{ 
  if (stopThrottle) return;

  //faster transmission of the whole package
  unsigned int thA = CreateBitSignal(throttleA);
  unsigned int thB = CreateBitSignal(throttleB);
  unsigned int thC = CreateBitSignal(throttleC);
  unsigned int thD = CreateBitSignal(throttleD);
  
  noInterrupts();
  th = thA;
  chan = 1;
  Transmit();

  th = thB;
  chan = 2;
  Transmit();

  th = thC;
  chan = 4;
  Transmit();

  th = thD;
  chan = 8;
  Transmit();
  interrupts();
}

//===== Secure Functions ====

void stopEngines()
{
  stopThrottle = true;
  motorStartUpPhase = 0; // to make the motors able to start again
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