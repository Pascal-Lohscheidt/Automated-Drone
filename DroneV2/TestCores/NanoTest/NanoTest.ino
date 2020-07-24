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
    Wire.begin(I2C_SLAVE_ADDRESS); //Slave musst tell adress

    ET.begin(details(myData), &Wire);
    
    Wire.onReceive(receive);

    Serial.begin(57600);
    Serial.println("First Test Nano");
}

void loop()
{    
    if(ET.receiveData())
    {
        Serial.println(myData.sex);
    }
    
}


void receive(int numBytes) {
    Serial.println("hey at least something");
}