#include <Wire.h>
#include <math.h>
#include <printf.h>
// #include <SPI.h>
#include <EasyTransferI2C.h>

extern "C"
{
  //void SetThrottle(int port, int throttle);
  void start();
  void TransmitThree();
  void TransmitThreeWO();
  void TransmitThreeWT();
}

volatile unsigned int th = 0;
volatile byte chan = 2;

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

long lastTimeHeardOfMaster = 0;

//==== Settings ====
#define MIN_THROTTLE 300 //250 standard
#define MAX_THROTTLE 800 //350 good testing value  <->  900 flightable

bool stopThrottle = false;

//====Throttles====
int throttleA; //1 -> pin 8
int throttleB; //2 -> pin 9
int throttleC; //3 -> pin 10
int throttleD; //4 -> pin 11

void setup()
{
  start();
  
  setThrottleForAll(MIN_THROTTLE);
  applyThrottle();
  
  Serial.begin(57600);
  Serial.println("--------------Drone is booting up - Throttle Nano V3-------------");
  Serial.println("--------------Establishing I2C connection to other cores-------------");
  
  // Wire.setClock(400000); //Must be called before wire begin
  Wire.begin(I2C_SLAVE_ADDRESS); //Slave musst tell adress
  ET.begin(details(myData), &Wire);
  Wire.onReceive(receive);
  
  Serial.println("--------------Waiting 5 Sec & initiate warm up-------------");
  
  delay(6000);
  // fireUpEngines();          
  Serial.println("--------------Setup - Done-------------");
  
  pinMode(13, OUTPUT);                               // LED Visual Output
  digitalWrite(13, HIGH);                            // LED High Once Done
}

int counter = 0;
long timer = 0;

void loop()
{
  //===== Data Receive ====
  if(ET.receiveData()) 
  {
    Serial.println(myData.throttleA);
    lastTimeHeardOfMaster = millis();
  }

  if(millis() - lastTimeHeardOfMaster > 1000) myData.motorStartUpPhase = -1;
  // else myData.motorStartUpPhase = -1;
  // handleThrottleSweep();
  // handleConstantThrottle();

  applyReceivedThrottleValues();

  Serial.println(getAverageThrottle());
  applyThrottle();
}

//====== Data ======
void receive(int numBytes) { }

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

//========== Throttle Steering ============

void handleConstantThrottle()
{
  setThrottleForAll(MIN_THROTTLE + (int)(sin(millis()) * 1));
  
  // if(counter >= -2)
  //   counter--;
  // else if(counter >= 2)
  //   counter++;
}

void handleThrottleSweep()
{  
  if(millis() - timer > 10)
  {
    timer = millis();
    counter++;
    setThrottleForAll(counter);
    if(counter >= MAX_THROTTLE) counter = 0;
  }
}

void fireUpEngines()
{
  Serial.println("Fire up Engines!!");
  for (int i = 0; i <= MIN_THROTTLE + 50; i++)
  {
    for(int p = 0; p <= 20; p++)
    {
      setThrottleForAll(i);
      applyThrottle();
    }

    Serial.print("Engine boost: ");
    Serial.print(i);
    Serial.print("\n");
  }
  Serial.println("Done \n\n");
  // motorStartUpPhase = 2;
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

  if (myData.motorStartUpPhase == -1) //must be called after limitation for saftey reasons
  {
    throttleA = 1;
    throttleB = 1;
    throttleC = 1;
    throttleD = 1;
  }
}

void applyThrottle()
{ 
  if (myData.motorStartUpPhase == -1) return;

  // limitThrottle();

  //faster transmission of the whole package
  unsigned int thA = optimizeThrottleSignal(throttleA);
  unsigned int thB = optimizeThrottleSignal(throttleB);
  unsigned int thC = optimizeThrottleSignal(throttleC);
  unsigned int thD = optimizeThrottleSignal(throttleD);
  
  unsigned int modeA = getThrottleMode(throttleA + 1500);
  unsigned int modeB = getThrottleMode(throttleB + 1500);
  unsigned int modeC = getThrottleMode(throttleC + 1500);
  unsigned int modeD = getThrottleMode(throttleD + 1500);

  noInterrupts();
  th = thA;
  chan = 1;
  Transmit(modeA);

  th = thB;
  chan = 2;
  Transmit(modeB);

  th = thC;
  chan = 4;
  Transmit(modeC);

  th = thD;
  chan = 8;
  Transmit(modeD);
  interrupts();
}

void Transmit(unsigned int mode)
{
  if(mode == 0) TransmitThree();
  else if(mode == 1) TransmitThreeWO();
  else TransmitThreeWT();
}


//0 = (3x); 1 = (3x + 1); 2 = (3x + 2); //This Throttle packaging is needed to maintain the 125us max length of oneshot125
unsigned int getThrottleMode(unsigned int throttle)
{
  if(throttle % 3 == 0) return 0;
  if(throttle % 3 - 1 == 0) return 1;
  if(throttle % 3 - 2 == 0) return 2;

}

unsigned int optimizeThrottleSignal(unsigned int throttle)
{
  throttle += 1500;
  throttle /= 3;
  return throttle;
}

//===== Secure Functions ====

// void stopEngines()
// {
//   stopThrottle = true;
//   motorStartUpPhase = 0; // to make the motors able to start again
//   Serial.println("-------------------");
//   Serial.println("Throttle stopped!!");
//   Serial.println("-------------------");
// }

// void startEngines()
// {
//   stopThrottle = false;
//   Serial.println("-------------------");
//   Serial.println("Engines Started");
//   Serial.println("-------------------");
// }