#include <Wire.h> //For I2C comm

//===== I2C Adressing ======

#define SLAVE_ADRESS 9
#define ANSWER_SIZE 5


void setup()
{
    Wire.begin();
    Serial.begin(57600);
    Serial.println("First Test NODE MCU");
}

void loop()
{
    Serial.println("Sending Data to nano");
    Wire.beginTransmission(SLAVE_ADRESS);
    Wire.write(0);
    Wire.endTransmission();

    Serial.println("Receive Data from Nano");

    Wire.requestFrom(SLAVE_ADRESS, ANSWER_SIZE);

    String response = "";
    while (Wire.available())
    {
        char b = Wire.read();
        response += b;
    }

    Serial.println(response);
}