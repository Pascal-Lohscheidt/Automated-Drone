#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <RF24_config.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

//===== NRF24 settings ======
RF24 radio(9, 10);
const byte address[][6] = {"0", "1"};

//===== OLED stuff =======

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET 4
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

//===== Timer ====
long heightControlTimer = 0;
long clickTimeStampLeft = 0;
long pidValueAdjustTimer = 0;

//===== Encoder =====

int clkPort = 2;
int dtPort = 3;
int encoderBtnPin = 5;
int preCounter;
int preCounterSteps = 2;
int currentStateCLK;
int prevStateCLK;

//==== Joystick Settings ====

int rightJoystickBtnPort = 6;
int leftJoystickBtnPort = 7;

int rXOffset;
int rYOffset;
int lXOffset;
int lYOffset;

//===== Rest =====

int operation = 1; //1-> start; 2-> land; 3-> Emergency Stop; 4-> Restart Engines 
int approvedOperation = 0;
bool operationSent = true;
int maxOperationCount = 4;

//===== Data Types =====

struct DataPackage
{
  short int operation = 0;
  short int heightAddition = 0;
  float pk = 0.2f;
  float ik = 0.75f;
  float dk = 0;
  float comp = 0.9991f;
};


typedef struct DataPackage Data;
Data data;

//====== Menu Selection ======
int currentPIDMenuPoint = 0;

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

void setup() {
  Serial.begin(57600);
  Wire.begin();
  //Set up NRF24
  radio.begin();
  radio.openWritingPipe(address[0]);
  radio.openReadingPipe(0, address[1]);
  radio.setPALevel(RF24_PA_HIGH);
  //radio.stopListening();
  radio.startListening();

  // Encoder SetUP
  pinMode(dtPort, INPUT);
  pinMode(clkPort, INPUT);
  pinMode(encoderBtnPin, INPUT);
  digitalWrite(encoderBtnPin, HIGH);
  prevStateCLK = digitalRead(clkPort);

  attachInterrupt(digitalPinToInterrupt(clkPort), isr, LOW);

  // Joystick Set up
  rXOffset = analogRead(0);
  rYOffset = analogRead(1);
  lXOffset = analogRead(2);
  lYOffset = analogRead(3);

  pinMode(leftJoystickBtnPort, INPUT);
  digitalWrite(leftJoystickBtnPort, HIGH);

  pinMode(rightJoystickBtnPort, INPUT);
  digitalWrite(rightJoystickBtnPort, HIGH);

  // OLED Display Setup
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.clearDisplay();

  Serial.println("Boot up......");

}

void loop() {
  
  handleJoysticks();
  handleEncoderClick();
  handleDataTransmission();
  updateDisplay();  
  // handleDataReceive();
}

void handleJoysticks()
{
  // data.rX = analogRead(0) - rXOffset;
  // data.rY = analogRead(1) - rYOffset;
  // data.lY = analogRead(3) - lYOffset;

  // if(data.rX < 5 && data.rX > -5) data.rX = 0;
  // if(data.rY < 5 && data.rY > -5) data.rY = 0;
  // if(data.lY < 5 && data.lY > -5) data.lY = 0;

  if(millis() - heightControlTimer > 10)
  {
    int lx = analogRead(2) - lXOffset;
    if(lx < 3 && lx > -3) lx = 0;
    int t = -map(lx, -512, 512, -2, 3); // required because of poti errors
    heightControlTimer = millis();
    if(t < 0) t *= 6; // This is required because i want the motors to faster throttle down than up for safety reasons
    // if(t == 3) t *= 5;
    // if(t == -2) t *= 20;
    data.heightAddition += t * 5;
    data.heightAddition = data.heightAddition <= 5 ? 5 : data.heightAddition;
    data.heightAddition = data.heightAddition > 800 ? 800 : data.heightAddition; 
  }

  if(millis() - pidValueAdjustTimer > 50)
  {
    pidValueAdjustTimer = millis();

    int rx = analogRead(0) - rXOffset;
    if(rx < 3 && rx > -3) rx = 0;
    int t = -map(rx, -512, 512, -2, 2); // required because of poti errors
    float addition = (float)t * 0.01f;
    if(currentPIDMenuPoint == 0) data.pk += addition * 10;
    if(currentPIDMenuPoint == 1) data.ik += addition;
    if(currentPIDMenuPoint == 2) data.dk += addition;
    if(currentPIDMenuPoint == 3) data.comp += (addition / 10);

  }


  //btnclicks

  if(digitalRead(leftJoystickBtnPort) == LOW && (millis() - clickTimeStampLeft) > 250)
  {
    clickTimeStampLeft = millis();
    currentPIDMenuPoint++;
    if(currentPIDMenuPoint > 3) currentPIDMenuPoint = 0;
  }

  if(digitalRead(rightJoystickBtnPort) == LOW && (millis() - clickTimeStampLeft) > 250)
  {
    clickTimeStampLeft = millis();
    approvedOperation = 3;
    operationSent = false;
  }


}

void handleDataTransmission()
{
  radio.stopListening();
  if (!operationSent)
  {
    operationSent = true;
    data.operation = approvedOperation;
  }
  else
  {
    data.operation = 0;
  }

  radio.write(&data, sizeof(data));
  radio.startListening();
}

// void handleDataReceive()
// {
//   if (radio.available())
//   {
//     radio.read(&tData, sizeof(tData));
//   }
// }

void isr()
{
  static unsigned long lastInterruptTime = 0;
  unsigned long interruptTime = millis();

  // If interrupts come faster than 5ms, assume it's a bounce and ignore
  if (interruptTime - lastInterruptTime > 5) {
    if (digitalRead(dtPort) == LOW)
    {
      operation++;
    }
    else
    {
      operation--;
    }

    // Restrict value from 0 to +100
    //virtualPosition = min(100, max(0, virtualPosition));


  }
  // Keep track of when we were here last (no more than every 5ms)
  lastInterruptTime = interruptTime;


  operation = operation < 1 ? maxOperationCount : operation;
  operation = operation > maxOperationCount ? 1 : operation;
}


void handleEncoderClick()
{
  if (digitalRead(encoderBtnPin) == LOW)
  {
    approvedOperation = operation;
    operationSent = false;
  }
}

void updateDisplay()
{
  display.setCursor(0, 0);
  display.clearDisplay();
  display.println("Remote Control:");
  display.print("Operation: ");
  display.print(operation);
  display.print(" - ");
  display.print(approvedOperation);
  display.println(" sel");
  display.println("____________________");

  // display.print("Height ");
  // display.print(tData.height);
  // display.print("| Bty ");
  // display.println(tData.batteryVoltage);
  
  // display.print("H-Goal ");
  // display.println(tData.heightGoal);

  // display.print("roll   ");
  // display.println(tData.rollAngle);
  // display.print("pitch  ");
  // display.println(tData.pitchAngle);
  // display.print("yaw    ");
  // display.println(tData.yawAngle);

  //  display.print("rX: ");
  //  display.println(data.rX);

  //  display.print("rY: ");
  //  display.println(data.rY);

  display.print("h throttle: ");
  display.println(data.heightAddition);

  if(currentPIDMenuPoint == 0) display.print("-> ");
  display.print("pk: ");
  display.println(data.pk * 100);

  if(currentPIDMenuPoint == 1) display.print("-> ");
  display.print("ik: ");
  display.println(data.ik * 100);

  if(currentPIDMenuPoint == 2) display.print("-> ");
  display.print("dk: ");
  display.println(data.dk * 100);

  if(currentPIDMenuPoint == 3) display.print("-> "); 
  display.print("m_comp: ");
  display.println(data.comp * 1000);


//  display.print("lY: ");
//  display.println(data.lY);

  display.display();
}



