#include <Wire.h> //For I2C comm
#include <EasyTransferI2C.h>

//===== I2C Adressing ======

#define I2C_SLAVE_ADDRESS 9


EasyTransferI2C ET;

struct SEND_DATA_STRUCTURE {
    int16_t sex;
    int16_t doubleSex;
};

SEND_DATA_STRUCTURE myData;

void setup()
{
    Wire.begin();
    Serial.begin(57600);
    Serial.println("First Test NODE MCU");
    ET.begin(details(myData), &Wire);
    myData.sex = 10;
}

void loop()
{
    Serial.println("Sending Data to nano");
    ET.sendData(I2C_SLAVE_ADDRESS);
}