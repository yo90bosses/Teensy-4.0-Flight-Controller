#ifndef _HELPER_DATA_H_
#define _HELPER_DATA_H_
#include <Arduino.h>
#include <helper_3dmath.h>


enum LINK_DATA_TYPE {
    DATATYPE_FULL
};



enum SENSOR_STATUS {
    SENSOR_NOT_STARTED,
    SENSOR_STARTED,
    SENSOR_RUNNING,
    SENSOR_FAILED
};



enum FLIGHT_MODE {
    FLIGHT_MANUAL,
    FLIGHT_RATES,
    FLIGHT_ACRO,
    FLIGHT_HORIZON,
    FLIGHT_ANGLE,
    FLIGHT_AUTOPILOT,
    FLIGHT_EXPERIMENTAL
};



enum AUTOPILOT_MODE {
    AUTOPILOT_POSITIONHOLD,
    AUTOPILOT_GOTOWAYPOINT,
    AUTOPILOT_FOLLOWLINE,
    /* Return to Home*/
    AUTOPILOT_RTH
};



enum AUTOPILOT_ACTION {
    ACTION_STANDBY,
    ACTION_TAKEOFF,
    ACTION_FLIGHT,
    ACTION_LAND
};



enum SENSOR_MODE {
    SENSOR_BOOT,
    SENSOR_CALIBRATION,
    SENSOR_OK,
    SENSOR_NO_POSITION,
    SENSOR_TOTAL_FAILURE
};



struct AutopilotDataStruct {

    Position positionSetpoint;

    Quaternion direction;

    AUTOPILOT_MODE autopilotMode = AUTOPILOT_POSITIONHOLD;

    AUTOPILOT_ACTION autopilotAction = ACTION_STANDBY;

};



struct SensorDataStruct {


    Quaternion              attitude;

    Quaternion              speed;
    Quaternion              accel;

    Position                position;

    float                   batteryVoltage;

    uint32_t                dataID;
    uint32_t                timestamp;

    SENSOR_MODE             sensorMode = SENSOR_MODE::SENSOR_BOOT;



    struct RawSensorData {

        //experimental stuff that might not be needed

        Quaternion              accelRaw;
        Quaternion              rateRaw;
        Quaternion              magRaw;

        Position                positionGNSS;
        Quaternion              speedGNSS;

        float                   barometricHeight;
        float                   airTemperature;
        float                   airPressure;
        float                   humidity;

        float                   sonicDistance;
        float                   sonicSpeed;
        uint32_t                sonicEchoTimestamp = 0;

        byte                    numberOfSatellites = 0;

        float                   seaLevelPressure = 100000.0f;
        float                   speedOfSound = 343.0f;

        uint32_t                gyroDataID = 0;
        uint32_t                accelDataID = 0;
        uint32_t                magDataID = 0;
        uint32_t                GNSSdataID = 0;
        uint32_t                EMUdataID = 0;
        uint32_t                sonicDataID = 0;

        uint32_t                timestampIMU = 0;
        uint32_t                timestampMag = 0;
        uint32_t                timestampEMU = 0;
        uint32_t                timestampGNSS = 0;
        uint32_t                timestampSonic = 0;

        SENSOR_STATUS           IMUStatus = SENSOR_NOT_STARTED;
        SENSOR_STATUS           EMUStatus = SENSOR_NOT_STARTED;
        SENSOR_STATUS           magStatus = SENSOR_NOT_STARTED;
        SENSOR_STATUS           GNSSStatus = SENSOR_NOT_STARTED;
        SENSOR_STATUS           SonicStatus = SENSOR_NOT_STARTED;

        float                   IMUAccelCorrectionFactor = 0.0f;
        float                   IMUMagCorrectionFactor = 0.0f;
        float                   IMUAccelLPF = 0.0f;
        float                   IMUGyroLPF = 0.0f;

        float                   EMUSpeedCorrectionFactor = 1.0f;
        float                   EMUHeightCorrectionFactor = 1.0f;
        float                   EMUPressureLPF = 0.0f;

        float                   GNSSPositionCorrectionFactor = 0.0f;
        float                   GNSSSpeedCorrectionFactor = 0.0f;
        float                   GNSSPressureCorrectionFactor = 0.0f;

        bool                    accelSpeedIntegration = false;
        bool                    speedPositionIntegration = false;

    };
    

    RawSensorData rawData;


};




struct PPMDataStruct {


    byte channels = 16;

    uint32_t dataID = 0;
    uint32_t timestamp = 0;

    float data[16];
    
    
};




struct SetpointStruct {

    Quaternion attitude;

    Position position;

    Position positionFrom;

    

    //Range: 0 to 1;
    float power = 0.0f;

    //Range: -1 to 1
    Quaternion rollRate;

};




struct PIDSettingStruct {

    //P factor
    Quaternion PF = Quaternion(1,0,0,0);
    //I Factor
    Quaternion IF = Quaternion(1,0,0,0);
    //D Factor
    Quaternion DF = Quaternion(1,0,0,0);
    //I Limit
    Quaternion IL = Quaternion(1,0,0,0);
    //FeedForward
    Quaternion FF = Quaternion(1,0,0,0);

};


struct LinkDataStruct {

        Quaternion attitude;

        Position position;

        Quaternion speed;

        uint32_t dataID = 0;

        uint32_t timestamp = 0;

    };


struct LinkStruct {

    struct LinkInfoStruct {

        uint8_t systemID = 0;

        uint16_t packetCheckSum = 0;

        uint8_t packetSize = 16;

        uint8_t packets = 0;

        LINK_DATA_TYPE dataType = LINK_DATA_TYPE::DATATYPE_FULL;

    };

    

    LinkInfoStruct info;
    LinkDataStruct data;


    const uint8_t lineStart1 = '#';
    const uint8_t lineStart2 = 'A';
    const uint8_t linkInfoIden = 'I';
    const uint8_t linkDataIden = 'D';

    uint32_t packetSequence = 0; 



};




struct SystemDataStruct {

    PPMDataStruct IBUSData;
    PPMDataStruct servoData;

    SensorDataStruct sensorData;

    SetpointStruct setpointData;

    PIDSettingStruct PIDSetting;

    Quaternion outputTorqe;

    LinkDataStruct linkData;

    AutopilotDataStruct autopilotData;

    uint32_t lostLinkPackets;
    uint32_t corruptedLinkPackets;

    int servoPin[16];

    bool systemArmed = false;

    bool enableMotors = false;

    bool IBUSFailsafe = true;

    bool calibration = false;

    //Timeframe of current loop
    uint32_t timestamp = 0;

    //Start timestamp of system in millis
    uint32_t startTimestamp = 0;

    FLIGHT_MODE flightMode = FLIGHT_MANUAL;

};




//Old stuff but still in use

enum ARM_MODE {
    ARMED_MANUAL,
    ARMED_AUTOPILOT,
    DISARMED_BOOTUP,
    DISARMED_FAILSAFE,
    DISARMED_USER,
    DISARMED_CRASH,
    DISARMED_AUTOPILOT,
    DISARMED_GROUNDED,
    DISARMED_NOSENSORDATA
};



enum SYSTEM_MODE {
    SYSTEM_BOOTUP,
    SYSTEM_MANUAL, 
    SYSTEM_AUTOPILOT
};



enum PID_MODE {
    PID_BOOTUP,
    PID_HOLD,
    PID_RATE
};



enum INERTIAL_STATUS {
    INERTIAL_BOOTUP,
    INERTIAL_STATIONARY,
    INERTIAL_LANDED,
    INERTIAL_FALLING,
    INERTIAL_FLYING
};





class hardwareDataStructSensors {

    public:

    Quaternion              attitude;
    uint32_t                attitudeID = 0;
    
    Quaternion              gravity;

    Quaternion              speedLocal;
    Quaternion              accelLocal;

    Quaternion              speedNED;
    Quaternion              accelNED;

    Position                positionINS;

    Quaternion              accelRaw;
    Quaternion              rateRaw;
    Quaternion              speedRaw;
    Quaternion              magRaw;
    Position                positionGNSS;

    float                   barometricHeight;
    float                   variometer;
    float                   airTemperature;
    float                   humidity;
    //Air pressure is in Pascal
    float                   airPressure;

    byte                    numberOfSatellites = 0;

    float                   seaLevelPressure = 100000.0f;

    float                   batteryVoltage;

    uint16_t                calibrationID = 0;

    uint32_t                dataID = 0;
    uint32_t                gyroDataID = 0;
    uint32_t                accelDataID = 0;
    uint32_t                magDataID = 0;

    uint32_t                timestampIMU = 0;
    uint32_t                timestampEMU = 0;
    uint32_t                timestampGNSS = 0;


    byte                    magCalibrationStatus = 0;
    byte                    accelCalibrationStatus = 0;
    byte                    gyroCalibrationStatus = 0;
    byte                    systemCalibration = 0;


    //Status
    bool                    positionDataValid = false;
    bool                    inertialDataValid = false;
    bool                    environmentDataValid = false;

    byte                    IMUGyroCalibration = 0;
    byte                    IMUAccelCalibration = 0;
    byte                    IMULinAccelCalibration = 0;
    byte                    IMUMagCalibration = 0;
    byte                    IMUQuatCalibration = 0;

    INERTIAL_STATUS         inertialStatus = INERTIAL_BOOTUP;


    private: 

    bool changed = false;
    
};



struct hardwareDataStructPPM {

    public:

    bool                    failsafe = false;

    float                   PPM[20]; //range is from 1 to -1

    uint32_t                dataID = 0;

    uint32_t                timeStamp = 0;

    
};




class hardwareDataStructSensorSetting {

    public:

    float absoluteIMUAccelFactor = 0.0002f;
    float absoluteIMUMagFactor = 0.0001f;

    float absoluteInertailFactor = 0.001f;
    float absoluteIMUBaroFactor = 0.001f;
    float absoluteIMUPositionFactor = 0.001f;


    bool enableIMUAccelCorrection = false;
    bool enableIMUMagCorrection = false;

    bool enableInertialIntegration = false;

    bool enableAbsolutePosition = false;

    bool enableBarometer = false;

    bool enableBarometerCalibration = false;

    bool calibrate = false;

    uint32_t dataID = 0;


};



#endif