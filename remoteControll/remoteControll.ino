#include <RF24.h>
#include <SPI.h>
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

int operation = 1; //1-> start; 2-> land; 3-> Emergency Stop; 
int approvedOperation = 0;
bool operationSent = true;
int maxOperationCount = 4;

//===== Data Types =====

struct DataPackage
{
  int operation = 0;

  int rX = 10;
  int rY = 23;
  int lX = 27;
  int lY = -30;
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

void setup() {
  Serial.begin(9600);
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
  
  updateDisplay();
  
  handleDataTransmission();
  handleDataReceive();
}

void handleJoysticks()
{
  data.rX = analogRead(0) - rXOffset;
  data.rY = analogRead(1) - rYOffset;
  data.lX = analogRead(2) - lXOffset;
  data.lY = analogRead(3) - lYOffset;

  //btnclicks
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

void handleDataReceive()
{
  if (radio.available())
  {
    radio.read(&tData, sizeof(tData));
  }
}

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

  display.print("Height ");
  display.print(tData.height);
  display.print("| Bty ");
  display.println(tData.batteryVoltage);
  
  display.print("H-Goal ");
  display.println(tData.heightGoal);

  display.print("roll   ");
  display.println(tData.rollAngle);
  display.print("pitch  ");
  display.println(tData.pitchAngle);
  display.print("yaw    ");
  display.println(tData.yawAngle);

//  display.print("rX: ");
//  display.println(data.rX);
//
//  display.print("rY: ");
//  display.println(data.rY);
//
//  display.print("lX: ");
//  display.println(data.lX);
//
//  display.print("lY: ");
//  display.println(data.lY);

  display.display();
}



