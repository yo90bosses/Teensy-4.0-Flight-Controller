#ifndef _COMMUNICATION_H_
#define _COMMUNICATION_H_

#include <Arduino.h>
#include <helper_data.h>
#include <helper_3dmath.h>



enum LINKDATASTAGE {
    LINK_STANDBY,
    LINK_START,
    LINK_PACKETPROPERTIES,
    LINK_PAYLOAD,
    LINK_END
};




void communicationLoop(SystemDataStruct *systemData) {

    static uint32_t lastSend = micros();
    static uint32_t linkLoopTime = micros();
    static bool firstRun = true;
    static bool linkWorking = false;
    static LINKDATASTAGE linkStage = LINK_STANDBY;
    static uint32_t linkByteCounter = 0;

    LinkDataStruct *linkData = &systemData->linkData;

    SensorDataStruct *sensorData = &systemData->sensorData;
    PPMDataStruct *PPMData = &systemData->IBUSData;


    if (firstRun) {
        firstRun = false;

        Serial5.begin(9600, SERIAL_8N1); //can only be setup at 9600 baud
        pinMode(22, OUTPUT);
        pinMode(23, OUTPUT);

        digitalWrite(23, LOW);

        delay(100);

        Serial5.print("AT+B115200");

        //Serial.println(Serial5.readString());

        /*Serial5.write('A');
        Serial5.write('T');

        String message;

        uint32_t start = millis();

        while(millis() - start < 100 && message.length() < 2) {
            if (Serial5.available() > 0) {
                message += String(Serial5.read());
            }
        }

        if (message.equals("OK")) {
            linkWorking = true;
            Serial.println("Link Started. Setting up...");

            sendStringToLink("AT+B115200");
            delay(10);

            sendStringToLink("AT+C001");
            delay(10);

            sendStringToLink("AT+FU3");
            delay(10);

            sendStringToLink("AT+P4");
            delay(10);

            Serial.println("Link was setup.");

        } else {
            Serial.println("Link Failed.");
        }*/

        digitalWrite(23, HIGH);

        delay(100);

        Serial5.begin(115200, SERIAL_8N1);

    }



    if (micros() - linkLoopTime >= 500) {
        linkLoopTime = micros();


        if (micros() - lastSend >= 200000) {
            lastSend = micros();

            if (linkStage == LINK_STANDBY) linkStage = LINK_START; // trigger data send

            linkData->attitude.w = (sensorData->attitude.getProductQuat(Quaternion(0,0,0,1)).getProductQuat(sensorData->attitude.getConjugate())).getAngleTo(Quaternion(0,0,0,1))*RAD_TO_DEG;
            linkData->attitude.x = sensorData->accel.getMagnitude();
            linkData->timestamp = systemData->timestamp;

        }


        switch (linkStage) {

            case LINK_START:
                
                Serial5.write(0xAA); //Packet start identifier 
                Serial5.write(0xFF);

                linkStage = LINK_PACKETPROPERTIES;

                break;

            case LINK_PACKETPROPERTIES:
                
                Serial5.write(0x01); //Transmitter ID 1
                Serial5.write(0x01); //Payload type 1 (experimental)

                linkByteCounter = 0;

                linkStage = LINK_PAYLOAD;
                
                break;

            case LINK_PAYLOAD:
                
                Serial5.write(*((uint8_t *)linkData+linkByteCounter));

                linkByteCounter++;

                if (linkByteCounter >= sizeof(*linkData)) linkStage = LINK_END;
                
                break;

            case LINK_END:
                
                Serial5.write(0xAF); //Packet end identifier

                linkStage = LINK_STANDBY;
                
                break;
            
            
            default:
                break;
        }

        if (micros() - linkLoopTime > 100) Serial.println("Com loop took: " + String(micros() - linkLoopTime));

    }

}

#endif