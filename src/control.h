#ifndef _CONTROL_H_
#define _CONTROL_H_


#include <helper_3dmath.h>
#include <helper_data.h>

#include <definitions.h>
#include <guidance.h>



float rcExpoCalc(float x, float percent) {

    return (1.0 - percent)*x + percent*x*x*x;

}


float nonLinearPIDCalc(float in, float zeroKick, float zeroKickOffset, float exponent, float normalisedResult = 1.0f) {

    float k = normalisedResult + zeroKick/(1 + expf(zeroKickOffset*in*in)); //factor calc

    k = k*pow(abs(in), exponent); //unlinearise

    return k<0.0f ? -k : k; //sign function

}



void acroFlight(SystemDataStruct *systemData) {

    static uint32_t lastDataTimestamp = 0;
    static bool wasArmed = true;

    PPMDataStruct *IBUSData = &systemData->IBUSData;
    SensorDataStruct *sensorData = &systemData->sensorData;
    SetpointStruct *setpointData = &systemData->setpointData;


    setpointData->power = IBUSData->data[IBUS_CHANNEL_POWER]*0.5f + 0.5f;


    //if last run was long ago then reset settings to not unexpectedly jump
    if (systemData->flightMode != FLIGHT_ACRO || wasArmed != systemData->systemArmed) {

        systemData->flightMode = FLIGHT_ACRO;
        setpointData->attitude = sensorData->attitude;
        lastDataTimestamp = systemData->timestamp;
        wasArmed = systemData->systemArmed;

        return;

    }


    float dTime = double(systemData->timestamp - lastDataTimestamp)/1000000.0;
    lastDataTimestamp = systemData->timestamp;




    Quaternion rotation;
    rotation.x = rcExpoCalc(IBUSData->data[IBUS_CHANNEL_ROLL], ACRO_EXPO_X)*ACRO_RATE_X*DEG_TO_RAD;
    rotation.y = rcExpoCalc(IBUSData->data[IBUS_CHANNEL_PITCH], ACRO_EXPO_Y)*ACRO_RATE_Y*DEG_TO_RAD;
    rotation.z = rcExpoCalc(-IBUSData->data[IBUS_CHANNEL_YAW], ACRO_EXPO_Z)*ACRO_RATE_Z*DEG_TO_RAD;

    setpointData->rollRate = rotation;

    if (rotation.getMagnitude() > 0.0f) {
        setpointData->attitude = setpointData->attitude.getProductQuat(Quaternion(rotation.getNormalized(), rotation.getMagnitude()*dTime));
    }


    

}



void levelFlight(SystemDataStruct *systemData) {

    static uint32_t lastDataTimestamp = 0;
    static bool wasArmed = true;

    PPMDataStruct *IBUSData = &systemData->IBUSData;
    SensorDataStruct *sensorData = &systemData->sensorData;
    SetpointStruct *setpointData = &systemData->setpointData;


    setpointData->power = IBUSData->data[IBUS_CHANNEL_POWER]*0.5f + 0.5f;


    //if last run was long ago then reset settings to not unexpectedly jump
    if (systemData->flightMode != FLIGHT_ANGLE || wasArmed != systemData->systemArmed) {

        systemData->flightMode = FLIGHT_ANGLE;
        lastDataTimestamp = systemData->timestamp;
        wasArmed = systemData->systemArmed;

        setpointData->attitude = sensorData->attitude;

        setpointData->attitude.x = 0.0f;
        setpointData->attitude.y = 0.0f;
        setpointData->attitude.normalize();

        return;

    }


    float dTime = double(systemData->timestamp - lastDataTimestamp)/1000000.0;
    lastDataTimestamp = systemData->timestamp;

    setpointData->attitude.x = 0.0f;
    setpointData->attitude.y = 0.0f;
    setpointData->attitude.normalize();

    Quaternion rotation;
    rotation.x = IBUSData->data[IBUS_CHANNEL_ROLL];
    rotation.y = IBUSData->data[IBUS_CHANNEL_PITCH];

    float yawRotation = -IBUSData->data[IBUS_CHANNEL_YAW]*ACRO_RATE_Z*DEG_TO_RAD/2.0f;

    setpointData->attitude = setpointData->attitude.getProductQuat(Quaternion(Quaternion(0,0,0,1), yawRotation*dTime));

    if (IBUSData->data[FLIGHTMODE_SWITCH] > 0.5f && IBUSData->data[IBUS_CHANNEL_SWA] > 0.5f) {
        setpointData->attitude = setpointData->attitude.getProductQuat(Quaternion(Quaternion(0,1,0,0), 135.0f*DEG_TO_RAD));
    } else {
        setpointData->attitude = setpointData->attitude.getProductQuat(Quaternion(rotation.getNormalized(), rotation.getMagnitude()*MODE_LEVEL_MAXANGLE*DEG_TO_RAD)).getNormalized();
    }

}



void autopilotFlight(SystemDataStruct *systemData) {

    static uint32_t lastDataTimestamp = 0;
    static bool wasArmed = true;

    PPMDataStruct *IBUSData = &systemData->IBUSData;
    SensorDataStruct *sensorData = &systemData->sensorData;
    SetpointStruct *setpointData = &systemData->setpointData;


    //setpointData->power = IBUSData->data[IBUS_CHANNEL_POWER]*0.5f + 0.5f; //Remove when throttle control implimented


    //if coming from a different flight mode then reset settings to not unexpectedly jump
    if (systemData->flightMode != FLIGHT_AUTOPILOT || wasArmed != systemData->systemArmed) {

        systemData->flightMode = FLIGHT_AUTOPILOT;
        lastDataTimestamp = systemData->timestamp;
        wasArmed = systemData->systemArmed;

        return;

    }


    if (IBUSData->data[FLIGHTMODE_SWITCH] > 0.5f) {

        systemData->autopilotData.autopilotMode = AUTOPILOT_MODE::AUTOPILOT_RTH;

    } else if (IBUSData->data[FLIGHTMODE_SWITCH] > -0.5f) {

        systemData->autopilotData.autopilotMode = AUTOPILOT_MODE::AUTOPILOT_GOTOWAYPOINT;

    } else {

        systemData->autopilotData.autopilotMode = AUTOPILOT_MODE::AUTOPILOT_POSITIONHOLD;

    }


    systemData->autopilotData.autopilotMode = AUTOPILOT_MODE::AUTOPILOT_POSITIONHOLD; // for testing purposes and therfore safety because other function are not implemented

}



void systemControlLoop(SystemDataStruct *systemData) {

    static uint32_t lastIBUSDataID = -1;

    PPMDataStruct *IBUSData = &systemData->IBUSData;
    SetpointStruct *setpointData = &systemData->setpointData;

    //Check if Failsafe
    if (systemData->IBUSFailsafe) {

        systemData->enableMotors = false;
        systemData->systemArmed = false;

    } else if (IBUSData->dataID != lastIBUSDataID) { //Only run if new IBUS data
        lastIBUSDataID = IBUSData->dataID;

        //check system arming
        if (!systemData->systemArmed) {
            
            //Check arming criteria
            if (IBUSData->data[DISARMING_SWITCH] > 0.5f && IBUSData->data[IBUS_CHANNEL_POWER] < -0.90f) {
            
                systemData->systemArmed = true;
                systemData->enableMotors = true;
                
            }

        } 

        //Disarming
        if (IBUSData->data[DISARMING_SWITCH] <= 0.5f) {

            systemData->enableMotors = false;
            systemData->systemArmed = false;

        }

        //Flight mode selecting
        if (IBUSData->data[CONTROLMODE_SWITCH] > 0.5f) {

            autopilotFlight(systemData);

        } else if (IBUSData->data[CONTROLMODE_SWITCH] > -0.5f) {

            levelFlight(systemData);

        } else {
            
            acroFlight(systemData);

        }

    }

}



void stabilisationLoop(SystemDataStruct *systemData) {

    static uint32_t lastUpdateus = 0;
    static bool     initialized = false;
    static FLIGHT_MODE lastFlightMode = FLIGHT_MANUAL;
    static Quaternion i = Quaternion(0,0,0,0);
    static Quaternion d = Quaternion(0,0,0,0);
    static Quaternion lastSetpointError;
    static Quaternion lastRate;
    static float lastThrottle = 0.0f;


    PPMDataStruct *IBUSData = &systemData->IBUSData;
    SensorDataStruct *sensorData = &systemData->sensorData;
    SetpointStruct *setpointData = &systemData->setpointData;
    PIDSettingStruct *PIDSetting = &systemData->PIDSetting;


    if (systemData->systemArmed) {


        if (systemData->flightMode != lastFlightMode) {
            lastFlightMode = systemData->flightMode;

            PIDSetting->PF = Quaternion(0, PID_P_X, PID_P_Y, PID_P_Z);
            PIDSetting->IF = Quaternion(0, PID_I_X, PID_I_Y, PID_I_Z);
            PIDSetting->DF = Quaternion(0, PID_D_X, PID_D_Y, PID_D_Z);
            PIDSetting->FF = Quaternion(0, PID_F_X, PID_F_Y, PID_F_Z);
            PIDSetting->IL = Quaternion(0, PID_I_LIMIT, PID_I_LIMIT, PID_I_LIMIT);
                    
            i = Quaternion(0,0,0,0);
            d = Quaternion(0,0,0,0);

            lastThrottle = IBUSData->data[IBUS_CHANNEL_POWER];
            lastRate = sensorData->rawData.rateRaw;
            lastUpdateus = sensorData->rawData.timestampIMU - 10000;

        }


        double dTime = double(sensorData->rawData.timestampIMU - lastUpdateus)/1000000.0;
        lastUpdateus = sensorData->rawData.timestampIMU;


        //Serial.println(dTime*1000000.0);


        Quaternion r = setpointData->rollRate;
        Quaternion p = Quaternion(0,0,0,0);
        Quaternion f = Quaternion(0,0,0,0);


        if (systemData->flightMode != FLIGHT_MODE::FLIGHT_ACRO && systemData->flightMode != FLIGHT_MODE::FLIGHT_MANUAL) {

            Quaternion rp = setpointData->attitude.getNormalized(true).getProductQuat(sensorData->attitude.getConjugate().getNormalized(true)).getNormalized(true);
            rp = sensorData->attitude.getConjugate().getProductQuat(Quaternion(0, rp.x, rp.y, rp.z)).getProductQuat(sensorData->attitude);

            Quaternion rd = -sensorData->rawData.rateRaw;

            r.w = 0.0f;
            r.x = rp.x*PID_RP_X + rd.x*PID_RD_X;
            r.y = rp.y*PID_RP_Y + rd.y*PID_RD_Y;
            r.z = rp.z*PID_RP_Z + rd.z*PID_RD_Z;

            //Serial.println(systemData->flightMode);

        } 

            

        p = r - sensorData->rawData.rateRaw; // P value from rate setpoint - rate is
        p.w = 0.0f;


        float throttleChange = (IBUSData->data[IBUS_CHANNEL_POWER] - lastThrottle)/dTime;
        lastThrottle = IBUSData->data[IBUS_CHANNEL_POWER];

        if (IBUSData->data[IBUS_CHANNEL_POWER] > PID_I_THROTTLE_RESET) {

            float PIDIM = 1.0f;
            if (abs(throttleChange) >= PID_ANTIGRAVITY_THRESHOLD) PIDIM = PID_ANTIGRAVITY_MULTIPLIER;

            i.x += p.x*PIDSetting->IF.x*PIDIM;
            i.x = constrain(i.x, -PIDSetting->IL.x, PIDSetting->IL.x);

            i.y += p.y*PIDSetting->IF.y*PIDIM; 
            i.y = constrain(i.y, -PIDSetting->IL.y, PIDSetting->IL.y);

            i.z += p.z*PIDSetting->IF.z*PIDIM;
            i.z = constrain(i.z, -PIDSetting->IL.z, PIDSetting->IL.z);

        } else {

            i = Quaternion(0,0,0,0);

        }


        Quaternion dBuf = (p - lastRate)/dTime; // calculate D value
        lastRate = p;

        d = d*PID_D_LPF + dBuf*(1.0f-PID_D_LPF); // LPF for D Value

        // Feedforward
        f.x = IBUSData->data[IBUS_CHANNEL_ROLL];
        f.y = -IBUSData->data[IBUS_CHANNEL_PITCH];
        f.z = IBUSData->data[IBUS_CHANNEL_YAW];

        
        if (sensorData->sensorMode == SENSOR_MODE::SENSOR_TOTAL_FAILURE || systemData->flightMode == FLIGHT_MODE::FLIGHT_MANUAL) { // only manual input in case of sensor failure or complete manual flight mode

            systemData->outputTorqe.w = 0.0f;
            systemData->outputTorqe.x = setpointData->rollRate.x*PIDSetting->PF.x;
            systemData->outputTorqe.y = setpointData->rollRate.y*PIDSetting->PF.y;
            systemData->outputTorqe.z = setpointData->rollRate.z*PIDSetting->PF.z;

        } else { //Proper PID calculation for normal situation

            systemData->outputTorqe.w = 0.0f;
            systemData->outputTorqe.x = i.x + d.x*PIDSetting->DF.x + p.x*PIDSetting->PF.x + f.x*PIDSetting->FF.x;
            systemData->outputTorqe.y = i.y + d.y*PIDSetting->DF.y + p.y*PIDSetting->PF.y + f.y*PIDSetting->FF.y;
            systemData->outputTorqe.z = i.z + d.z*PIDSetting->DF.z + p.z*PIDSetting->PF.z + f.z*PIDSetting->FF.z;

            /*Serial.println();
            Serial.print("Stuff: ");
            Serial.print(" " + String(r.x));
            Serial.print(" " + String(r.y));
            Serial.print(" " + String(r.z));*/

            //Serial.println(dTime*1000000);


        }


    } else {

        systemData->outputTorqe = Quaternion(0,0,0,0);

    }

}



void forceTransformation(SystemDataStruct *systemData) {

    SensorDataStruct *sensorData = &systemData->sensorData;
    SetpointStruct *setpointData = &systemData->setpointData;
    PPMDataStruct *servoData = &systemData->servoData;


    if (systemData->enableMotors && systemData->systemArmed) {

        float force = map(setpointData->power, 0.0f, 1.0f, IDLE_POWER, MAX_POWER); //map throttle to idle to full
        
        force = constrain(setpointData->power, IDLE_POWER, MAX_POWER)*2.0f - 1.0f; //constrain and remap throttle

        float fl = (-systemData->outputTorqe.y + systemData->outputTorqe.x + systemData->outputTorqe.z + force);
        float fr = (-systemData->outputTorqe.y - systemData->outputTorqe.x - systemData->outputTorqe.z + force);
        float bl = (systemData->outputTorqe.y + systemData->outputTorqe.x - systemData->outputTorqe.z + force);
        float br = (systemData->outputTorqe.y - systemData->outputTorqe.x + systemData->outputTorqe.z + force);

        servoData->data[MOTOR_CHANNEL_1] = constrain(fl, MIN_POWER-1.0f, 1.0f);
        servoData->data[MOTOR_CHANNEL_2] = constrain(fr, MIN_POWER-1.0f, 1.0f);
        servoData->data[MOTOR_CHANNEL_3] = constrain(bl, MIN_POWER-1.0f, 1.0f);
        servoData->data[MOTOR_CHANNEL_4] = constrain(br, MIN_POWER-1.0f, 1.0f);

    } else {

        servoData->data[MOTOR_CHANNEL_1] = -1.0f;
        servoData->data[MOTOR_CHANNEL_2] = -1.0f;
        servoData->data[MOTOR_CHANNEL_3] = -1.0f;
        servoData->data[MOTOR_CHANNEL_4] = -1.0f;

    }

}



void controlLoop(SystemDataStruct *systemData) {

    static uint32_t lastSensorDataID = -1;
    static uint32_t lastPIDTimestamp = 0;


    //Handling of new IBUS data
    systemControlLoop(systemData);

    //Guidance loop for autopilot
    guidanceLoop(systemData);

    //Update at the lower rate of either sensor or sampling rate
    if (systemData->sensorData.rawData.gyroDataID != lastSensorDataID && micros() - lastPIDTimestamp >= 1000000/CONTROL_SAMPLINGRATE_LIMIT) {
        lastPIDTimestamp = micros();// += 1000000/PID_SAMPLINGRATE;

        stabilisationLoop(systemData);

        forceTransformation(systemData);

        systemData->servoData.dataID++;

        lastSensorDataID = systemData->sensorData.rawData.gyroDataID;

    }

}




#endif