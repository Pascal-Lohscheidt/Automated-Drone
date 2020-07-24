#include <RF24.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

RF24 radio(7, 8); // CE, CSN
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

void setup()
{
  Serial.begin(57600);
  Serial.println("--NRF24 Radio Setup--");
  radio.begin();
  //radio.openWritingPipe(address[1]);
  radio.openReadingPipe(0, address[0]);
  radio.setPALevel(RF24_PA_HIGH);
  radio.startListening();

}


void loop() {
  handleRadioReceive();

}

void handleRadioReceive()
{
  if(radio.available())
  {

    radio.read(&data, sizeof(data));

    if (data.operation > 0 && lastOperation != data.operation)
    {
      lastOperation = data.operation;
      Serial.println("operration:");
      Serial.println(data.operation);
      // if(data.operation == 1 && motorStartUpPhase == 0)
      //   motorStartUpPhase = 2; //set´s the start of the motors
      // if (data.operation == 3)
      //   stopEngines();
      // if (data.operation == 4)
      //   startEngines();
    }

    Serial.print(data.lX);
    Serial.print("\t");
    Serial.print(data.lY);
    Serial.print("\t");
    Serial.print(data.rX);
    Serial.print("\t");
    Serial.println(data.rY);
  }
}