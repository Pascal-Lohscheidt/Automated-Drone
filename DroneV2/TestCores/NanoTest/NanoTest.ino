#include <Wire.h> //For I2C comm

//===== I2C Adressing ======

#define SLAVE_ADRESS 9
#define ANSWER_SIZE 5

String resp = "Fcku!";

void setup()
{
    Wire.begin(SLAVE_ADRESS); //Slave musst tell adress
    Serial.begin(57600);
    Serial.println("First Test Nano");

    //RequestEvent
    Wire.onRequest(requestEvent);

    //Wire Receive Event
    Wire.onReceive(requestEvent);
}

void loop()
{    
    delay(10);
}

void receiveEvent()
{
    while(0 < Wire.available())
    {
        byte x = Wire.read();
    }
    Serial.println("ReceiveEvent");
}

void requestEvent()
{
    byte response[ANSWER_SIZE];

    //Format Answer as array
    for (byte i = 0; i < ANSWER_SIZE; i++)
    {
        response[i] = (byte)resp.charAt(i);
    }
    Wire.write(sizeof(response));

    Serial.println("RequestEvent");
    

}