#ifndef _GUIDANCE_H_
#define _GUIDANCE_H_

#include <Arduino.h>

#include <helper_3dmath.h>
#include <helper_data.h>

#include <definitions.h>



void positionHold(SystemDataStruct *systemData) {

    static bool wasArmed = true;
    static uint32_t lastDataTimestamp = 0;
    static float powerRampUp = 0.0f;
    static float powerSetpointBuf = 0.0f;

    PPMDataStruct *IBUSData = &systemData->IBUSData;
    SensorDataStruct *sensorData = &systemData->sensorData;
    SetpointStruct *setpointData = &systemData->setpointData;


    //if last run was long ago then reset settings to not unexpectedly jump
    if (systemData->flightMode != FLIGHT_MODE::FLIGHT_AUTOPILOT || wasArmed != systemData->systemArmed) {

        systemData->flightMode = FLIGHT_MODE::FLIGHT_AUTOPILOT;
        lastDataTimestamp = systemData->timestamp;
        wasArmed = systemData->systemArmed;
        powerRampUp = 0.0f;
        powerSetpointBuf = 0.0f;

        setpointData->attitude = sensorData->attitude;

        setpointData->attitude.x = 0.0f;
        setpointData->attitude.y = 0.0f;
        setpointData->attitude.normalize();

        setpointData->position = sensorData->position;
        setpointData->position.setHeightMSL(setpointData->position.getHeightMSL()+1.0f);

        return;

    }


    double dTime = double(systemData->timestamp - lastDataTimestamp)/1000000.0;
    lastDataTimestamp = systemData->timestamp;



    if (IBUSData->data[IBUS_CHANNEL_SWA] > 0.5f) {
        powerRampUp += 0.2*dTime;
        powerRampUp = constrain(powerRampUp, 0.0f, 1.0f);
    } else {
        powerRampUp = 0.0f;
    }



    //setpointData->power = IBUSData->data[IBUS_CHANNEL_POWER]*0.5f + 0.5f;

    /*
        IMPORTANT TEST ALL PERAMETERS BEFORE TESTING HEIGHT HOLD FUNCTION. NOT TESTED AT ALL!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        1. test if sensorData->postion gives correct fast and accurate height data also maybe during flight.
        2. check if rampup function works (SWE to activate heighthold)
        2. test ALL VARIABLES while going through different correction pases from position and tilt.

    */
    


    Quaternion direction = sensorData->position.getDistanceTo(setpointData->position);
    direction.w = 0.0f;
    direction.x = 0.0f;
    direction.y = 0.0f;
    direction.z = sensorData->rawData.sonicDistance - 1.0f; // JUST FOR TESING set height is at 1 meter
    
    float distance = direction.getMagnitude();
    direction.normalize();

    float speed = sensorData->speed*direction;
    speed = sensorData->rawData.sonicSpeed; // JUST FOR TESING

    Quaternion P = -direction*(distance*POS_HOLD_P);

    Quaternion D = direction*(speed*POS_HOLD_D);

    Quaternion force = (P+D)*DRONE_WEIGHT;

    if (force.getMagnitude() > 1.0f) {

        float xyMag = 1.0 - force.z*force.z;
        xyMag = sqrtf(max(0.0f, xyMag));

        Quaternion newForce = force;
        newForce.z = 0.0f;
        newForce.normalize();
        newForce *= xyMag;

        force = newForce + Quaternion(0,0,0,force.z);
        
    }


    Quaternion setpoint = sensorData->attitude;
    //remove tilt from attitude but keep current heading
    setpoint.x = 0.0f;
    setpoint.y = 0.0f;
    setpoint.normalize();


    //rest of code is just temporary to give manual control for altitude hold testing purposes


    setpointData->attitude.x = 0.0f;
    setpointData->attitude.y = 0.0f;
    setpointData->attitude.normalize();

    Quaternion rotation;
    rotation.x = IBUSData->data[IBUS_CHANNEL_ROLL];
    rotation.y = IBUSData->data[IBUS_CHANNEL_PITCH];

    float yawRotation = -IBUSData->data[IBUS_CHANNEL_YAW]*ACRO_RATE_Z*DEG_TO_RAD/2.0f;

    setpointData->attitude = setpointData->attitude.getProductQuat(Quaternion(Quaternion(0,0,0,1), yawRotation*dTime));

    setpointData->attitude = setpointData->attitude.getProductQuat(Quaternion(rotation.getNormalized(), rotation.getMagnitude()*MODE_LEVEL_MAXANGLE*DEG_TO_RAD)).getNormalized();

    //axis to rotate around
    Quaternion setpointRotationAxis = Quaternion(0,0,0,1).getCross(force).getNormalized();

    //angle to rotate
    float angle = Quaternion(0,0,0,1).getAngleTo(force);

    //Calculate new attitude and apply to setpointdata
    //setpointData->attitude = Quaternion(1,0,0,0); //Quaternion(setpointRotationAxis, angle).getProductQuat(setpoint);

    //angle error
    float angleError = sensorData->attitude.getProductQuat(setpointData->attitude.getConjugate()).w;

    angleError = constrain(angleError, 0.01f, 1.0f); // constrain to avoid 0 division in thrust correction

    if (angle > 0.5f*M_PI) angleError = -angleError;

    //Calculate force that is needed and apply to setpointdata
    powerSetpointBuf = powerSetpointBuf*AUTOPILOT_POWER_LPF + force.getMagnitude()*(1.0f - AUTOPILOT_POWER_LPF); // filter outputpower to keep spikes away

    setpointData->power = powerSetpointBuf/angleError*powerRampUp;

    if (powerRampUp == 0.0f) setpointData->power = IBUSData->data[IBUS_CHANNEL_POWER]*0.5f + 0.5f;

    //Serial.println(D.z/POS_HOLD_D);



    /*//Unit vector from position to setpoint (AB = B - A)
    Quaternion direction = sensorData->position.getDistanceTo(setpointData->position);

    //Distance (in meters) from postition to setpoint (AB = B - A)
    float distance = direction.getMagnitude();
    direction.normalize();

    //Speed in direction (speed dot direction)
    float speed = sensorData->speed*direction;

    //Speed vector orthogonal to direction with direction component removed in 
    Quaternion orthoSpeed = sensorData->speed - direction*speed;

    //P acceleration
    Quaternion P = direction*(distance*POS_HOLD_P);

    //D acceleration
    Quaternion D = -(orthoSpeed + direction*speed)*POS_HOLD_D;

    //Correctional force
    Quaternion force = (P+D)*DRONE_WEIGHT;

    //Prioritise Height correction over position
    if (force.getMagnitude() > 1.0f) {

        float xyMag = 1.0 - force.z*force.z;
        xyMag = sqrtf(max(0.0f, xyMag));

        Quaternion newForce = force;
        newForce.z = 0.0f;
        newForce.normalize();
        newForce *= xyMag;

        force = newForce + Quaternion(0,0,0,force.z);
        
    }


    //Needed attitude
    Quaternion setpoint = sensorData->attitude;
    //remove tilt from attitude and keep current heading
    setpoint.x = 0.0f;
    setpoint.y = 0.0f;
    setpoint.normalize();

    //axis to rotate around
    Quaternion setpointRotationAxis = Quaternion(0,0,0,1).getCross(force).getNormalized();

    //angle to rotate
    float angle = Quaternion(0,0,0,1).getAngleTo(force);

    //Calculate new attitude and apply to setpointdata
    setpointData->attitude = Quaternion(setpointRotationAxis, angle).getProductQuat(setpoint);

    //Calculate force that is needed and apply to setpointdata
    setpointData->power = force.getMagnitude();*/

}




void safeMode(SystemDataStruct *systemData) {


    systemData->setpointData.attitude = Quaternion(1,0,0,0);

    systemData->setpointData.power = 0;


}




void guidanceLoop(SystemDataStruct *systemData) {

    static uint32_t lastTimestamp = 0;
    static uint32_t lastDataID = -1;


    PPMDataStruct *IBUSData = &systemData->IBUSData;
    SensorDataStruct *sensorData = &systemData->sensorData;
    SetpointStruct *setpointData = &systemData->setpointData;


    if ((sensorData->dataID != lastDataID) && (micros() - lastTimestamp >= GUIDANCE_SAMPLINGRATE_LIMIT)) {
        lastTimestamp = micros();
        lastDataID = sensorData->dataID;

        if (systemData->flightMode == FLIGHT_AUTOPILOT) {

            switch (systemData->autopilotData.autopilotMode) {

                case AUTOPILOT_MODE::AUTOPILOT_POSITIONHOLD:
                    positionHold(systemData);
                    break;
                
                default:
                    safeMode(systemData);
                    break;

            }
        }

    }



}




#endif