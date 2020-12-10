#include <Arduino.h>

#include <helper_3dmath.h>
#include <helper_data.h>
#include <definitions.h>

#include <hardware.h>
#include <control.h>
#include <LEDStuff.h>
#include <communication.h>



//Global variables
SystemDataStruct systemData;

float systemUsage = 0.0f;


//Tasks
void serialDataPrint();

void flightControllerLoop();

void systemPerformanceCalc();



void setup() {


    //Serial.begin(115200);

    systemData.startTimestamp = millis();


}




void loop() {
    
    static uint32_t lastRun = micros();

    if (micros() - lastRun >= 50) {
        lastRun = micros();
        flightControllerLoop();
    }

    systemPerformanceCalc();

    serialDataPrint();


}







void flightControllerLoop() { //IMPORTANT TEST ALL PERAMETERS BEFORE TESTING HEIGHT HOLD FUNCTION. NOT TESTED AT ALL!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

    static uint32_t calStart = 0;
    static bool calibrated = false;


    if (!calibrated) {
        calibrated = true;
        calStart = millis();
    }

    if (millis() > 2500 && millis() < 2600) calStart = millis();

    if (millis() - calStart < 100) {
        systemData.calibration = true;
    } else {
        systemData.calibration = false;
    }

    systemData.timestamp = micros();

    hardwareIBUSLoop(&systemData);

    hardwareSensorLoop(&systemData);

    controlLoop(&systemData);

    hardwarePPMLoop(&systemData);

    communicationLoop(&systemData);


}



void systemPerformanceCalc() {

    static uint32_t counter = 0;
    static uint32_t lastCalc = millis();

    if (millis() - lastCalc >= 500) {
        lastCalc += 500;

        systemUsage = (1.0f - float(counter)/SYSTEMUSAGE_MAX)*100.0f;
        counter = 0;

    }

    counter++;

}



void serialDataPrint() {

    static uint32_t printTimestamp = micros();
    static uint32_t lastDataID = 0;
    PPMDataStruct *IBUSData = &systemData.IBUSData;
    SensorDataStruct *sensorData = &systemData.sensorData;
    SetpointStruct *setpointData = &systemData.setpointData;


    if (micros() - printTimestamp >= 1000000/DEBUG_SAMPLINGRATE) {
        printTimestamp = micros();

        uint32_t sensorReads = sensorData->rawData.gyroDataID - lastDataID;
        lastDataID = sensorData->rawData.gyroDataID;

        Serial.print("System Usage: ");
        Serial.print(systemUsage);
        /*Serial.print(" Sampling Rate: ");
        Serial.print(sensorReads*DEBUG_SAMPLINGRATE);
        Serial.print("\tAttitude: ");
        Serial.print(sensorData->attitude.w);
        Serial.print(" ");
        Serial.print(sensorData->attitude.x);
        Serial.print(" ");
        Serial.print(sensorData->attitude.y);
        Serial.print(" ");
        Serial.print(sensorData->attitude.z);*/
        /*Serial.print(" AccRaw: ");
        Serial.print(sensorData->accel.x);
        Serial.print(" ");
        Serial.print(sensorData->accel.y);
        Serial.print(" ");
        Serial.print(sensorData->accel.z);
        Serial.print(" ");
        Serial.print(sensorData->accel.getMagnitude());*/
        /*Serial.print(" Sonic: ");
        Serial.print(sensorData->rawData.sonicDistance);
        Serial.print(" ");
        Serial.print(sensorData->rawData.sonicSpeed);*/
        /*Serial.print(" Acc: ");
        Serial.print(sensorData->rawData.accelRaw.x);
        Serial.print(" ");
        Serial.print(sensorData->rawData.accelRaw.y);
        Serial.print(" ");
        Serial.print(sensorData->rawData.accelRaw.z);
        Serial.print(" ");
        Serial.print(sensorData->rawData.accelRaw.getMagnitude());*/
        /*Serial.print(" Acc: ");
        Serial.print(sensorData->rawData.rateRaw.x);
        Serial.print(" ");
        Serial.print(sensorData->rawData.rateRaw.y);
        Serial.print(" ");
        Serial.print(sensorData->rawData.rateRaw.z);
        Serial.print(" ");
        Serial.print(sensorData->rawData.rateRaw.getMagnitude());*/
        Serial.print("AttitudeSet: ");
        Serial.print(setpointData->attitude.w);
        Serial.print(" ");
        Serial.print(setpointData->attitude.x);
        Serial.print(" ");
        Serial.print(setpointData->attitude.y);
        Serial.print(" ");
        Serial.print(setpointData->attitude.z);
        /*Serial.print("\tMotor power: ");
        Serial.print(systemData.setpointData.power);
        Serial.print("\tHeight: ");
        Serial.print(systemData.sensorData.position.getHeightMSL());
        Serial.print("\tSpeed Z: ");
        Serial.print(systemData.sensorData.speed.z);*/
        Serial.print("\tTorqe: ");
        Serial.print(systemData.outputTorqe.x);
        Serial.print(" ");
        Serial.print(systemData.outputTorqe.y);
        Serial.print(" ");
        Serial.print(systemData.outputTorqe.z);
        Serial.print("\tPower: ");
        Serial.print(systemData.setpointData.power);
        /*Serial.print("\tOut: ");
        Serial.print(systemData.servoData.data[MOTOR_CHANNEL_1]);
        Serial.print(" ");
        Serial.print(systemData.servoData.data[MOTOR_CHANNEL_2]);
        Serial.print(" ");
        Serial.print(systemData.servoData.data[MOTOR_CHANNEL_3]);
        Serial.print(" ");
        Serial.print(systemData.servoData.data[MOTOR_CHANNEL_4]);*/



        Serial.println();
        


    }


}
