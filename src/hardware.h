#ifndef _HARDWARE_H_
#define _HARDWARE_H_


#include <helper_3dmath.h>
#include <helper_data.h>

#include <definitions.h>

#include <I2Cdev.h>
#include <HMC5883L.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "FlySkyIBus.h"
#include <UBLOX.h>
#include <MPU9250.h>



volatile uint32_t echoStart = 0;
volatile uint32_t echoEnd = 0;
volatile bool newEcho = 0;



void attitudeFromData(SensorDataStruct *sensorData) {

    sensorData->rawData.magRaw.w = 0.0f;

    Quaternion gravityMeasured = sensorData->rawData.accelRaw.getNormalized();
    Quaternion reference = Quaternion(0,0,0,-1);
    Quaternion gravityRotationAxis = gravityMeasured.getCross(reference).getNormalized();
    gravityRotationAxis.w = 0.0f;
    float gravityAngle = gravityMeasured.getAngleTo(reference);

    sensorData->attitude = Quaternion(1,0,0,0).getProductQuat(Quaternion(gravityRotationAxis, gravityAngle));

    Quaternion yaw = Quaternion(1,0,0,0);
    Quaternion magneticRotationAxis = Quaternion(0,0,0,1);
    float magneticAngle = 0.0f;

    if (sensorData->rawData.magRaw.getMagnitude() > 1.0f) { // check if valid

        reference = Quaternion(0,1,0,0);

        Quaternion magWorld = sensorData->attitude.getProductQuat(sensorData->rawData.magRaw).getProductQuat(sensorData->attitude.getConjugate());
        yaw = Quaternion(magWorld.w, 0, 0, magWorld.z).getNormalized();

        magneticAngle = yaw.getAngleTo(reference);

        sensorData->attitude = Quaternion(magneticRotationAxis, magneticAngle).getProductQuat(sensorData->attitude);

    }

    sensorData->attitude.normalize();

    //sensorData->attitude = Quaternion(1,0,0,0);
    
}




void INSFilterUpdate(SensorDataStruct *sensorData) {

    static uint32_t lastGNSSdataID = -1;
    static uint32_t lastAccelDataID = -1;

    static uint32_t lastIMUTimestamp = sensorData->rawData.timestampIMU;



    //Accel and speed Integral
    if (sensorData->rawData.accelDataID != lastAccelDataID) {

        double dTime = double(sensorData->rawData.timestampIMU - lastIMUTimestamp)/1000000.0;
        lastIMUTimestamp = sensorData->rawData.timestampIMU;

        //Accel Integral
        if (sensorData->rawData.accelSpeedIntegration) {

            sensorData->speed += sensorData->accel*dTime;

        }

        //speed Integral
        if (sensorData->rawData.speedPositionIntegration) {

            sensorData->position.changePosFromSpeed(sensorData->speed, dTime);

        }

    }



    
    if (sensorData->rawData.GNSSdataID != lastGNSSdataID) {

        //Complimentary filter for GNSS speed
        if (sensorData->rawData.GNSSSpeedCorrectionFactor != 0.0f) {
            
            sensorData->speed = sensorData->speed*(1.0 - sensorData->rawData.GNSSSpeedCorrectionFactor) + sensorData->rawData.speedGNSS*sensorData->rawData.GNSSSpeedCorrectionFactor;

        }

        //complimentary Filter for GNSS position. Does not correct height if EMU is running
        if (sensorData->rawData.GNSSPositionCorrectionFactor != 0.0f) {

            sensorData->position.setPosFromGPS(
                sensorData->position.getLat()*(1.0f - sensorData->rawData.GNSSPositionCorrectionFactor) + sensorData->rawData.positionGNSS.getLat()*sensorData->rawData.GNSSPositionCorrectionFactor,
                sensorData->position.getLon()*(1.0f - sensorData->rawData.GNSSPositionCorrectionFactor) + sensorData->rawData.positionGNSS.getLon()*sensorData->rawData.GNSSPositionCorrectionFactor,
                (sensorData->rawData.EMUStatus == SENSOR_RUNNING) ? (sensorData->position.getHeightMSL()):(sensorData->position.getHeightMSL()*(1.0f - sensorData->rawData.GNSSPositionCorrectionFactor) + sensorData->rawData.positionGNSS.getHeightMSL()*sensorData->rawData.GNSSPositionCorrectionFactor)
            );

        }

    }


    sensorData->speed.w = 0.0f;
    sensorData->speed.x = 0.0f;
    sensorData->speed.y = 0.0f;



    lastAccelDataID = sensorData->rawData.accelDataID;
    lastGNSSdataID = sensorData->rawData.GNSSdataID;

    sensorData->dataID++;

}




void IMUFilterUpdate(SensorDataStruct *sensorData, double time = 1.0/IMU_SAMPLINGRATE_LIMIT, bool reset = false) {

    static uint32_t lastGyroID = -1;
    static uint32_t lastAccelID = -1;
    static uint32_t lastMagID = -1;


    if (sensorData->attitude.getMagnitude() < 0.1f) {
        attitudeFromData(sensorData);
    } else if (sensorData->attitude.getMagnitude() < 0.9f) {
        sensorData->attitude.normalize();
    }
    

    //integration of gyro data and apply to current rotation to predict state
    if (lastGyroID != sensorData->rawData.gyroDataID) {
        lastGyroID = sensorData->rawData.gyroDataID;

        sensorData->rawData.rateRaw.w = 0.0f;
        Quaternion rotationDelta = Quaternion(sensorData->rawData.rateRaw.getNormalized(), sensorData->rawData.rateRaw.getMagnitude()*time);
        sensorData->attitude = sensorData->attitude.getProductQuat(rotationDelta).getNormalized();

    }


    float accelMagnitude = sensorData->rawData.accelRaw.getMagnitude();
    //calculate linearaccel in world reference
    sensorData->accel = sensorData->attitude.getProductQuat(sensorData->rawData.accelRaw).getProductQuat(sensorData->attitude.getConjugate()) - Quaternion(0, 0, 0, -9.81f);



    //check if accel data is inbounds to remove non gravitational accelerations and correct predicted position
    if (sensorData->rawData.IMUAccelCorrectionFactor != 0.0f && lastAccelID != sensorData->rawData.accelDataID && accelMagnitude > 6.0f && accelMagnitude < 14.0f) {
        lastAccelID = sensorData->rawData.accelDataID;

        Quaternion gravityMeasured = sensorData->rawData.accelRaw/accelMagnitude;
        Quaternion gravityCurrent = sensorData->attitude.getConjugate().getProductQuat(Quaternion(0,0,0,-1)).getProductQuat(sensorData->attitude);
        Quaternion gravityRotation = gravityMeasured.getCross(gravityCurrent).getNormalized();
        float gravityAngle = gravityMeasured.getAngleTo(gravityCurrent);

        //apply correction to predicted attitude
        sensorData->attitude = sensorData->attitude.getProductQuat(Quaternion(gravityRotation, gravityAngle*sensorData->rawData.IMUAccelCorrectionFactor));

    }


    float magMagnitude = sensorData->rawData.magRaw.getMagnitude();

    //cheack if mag data is inbounds to remove non earth fields and correct heading
    if (sensorData->rawData.IMUMagCorrectionFactor != 0.0f && lastMagID != sensorData->rawData.magDataID && magMagnitude > 10.0f && magMagnitude < 80.0f) {
        lastMagID = sensorData->rawData.magDataID;

        Quaternion current = sensorData->attitude;
        current.x = 0.0f;
        current.y = 0.0f;
        current.normalize();

        Quaternion heading = sensorData->attitude.getProductQuat(sensorData->rawData.magRaw).getProductQuat(sensorData->attitude.getConjugate());
        heading = Quaternion(heading.w, 0, 0, heading.z).getNormalized();

        Quaternion errorAxis = heading.getProductQuat(current.getConjugate());
        float errorAngle = acos(errorAxis.w)*2.0f;
        errorAxis.w = 0.0f;
        errorAxis.normalize();

        sensorData->attitude = sensorData->attitude.getProductQuat(Quaternion(errorAxis, errorAngle*sensorData->rawData.IMUMagCorrectionFactor));

    }

    //Sanity calc
    sensorData->attitude.normalize();

    sensorData->dataID++;

}



void sonicInterruptRoutine() {

    if (digitalReadFast(ULTRASONIC_ECHO_PIN)) {
        echoStart = micros();
    } else {
        echoEnd = micros();
        newEcho = true;
    }

}



void sonicTrig() {

    //Serial.println("TRIG");

    digitalWrite(ULTRASONIC_TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(ULTRASONIC_TRIG_PIN, LOW);

}




void hardwareSensorLoop(SystemDataStruct *systemData) {

    static bool firstRun = true;

    static MPU9250FIFO IMU(SPI, 33);
    static byte IMUStartFailures = 0;
    static bool accelCal = false;
    static bool gyroCal = false;
    static uint32_t runsCal = 0;
    static bool IMUCalibrating = false;

    static Adafruit_BME280 EMU;
    static byte EMUStartFailures = 0;
    static bool EMUCalibrating = false;

    static UBLOX GNSS(Serial7, 115200);
    static byte GNSSStartFailures = 0;
    static bool GNSSCalibrating = false;

    static HMC5883L mag;
    static byte magStartFailures = 0;
    static bool magCal = false;
    static bool magCalibrating = false;


    static Quaternion magScale = Quaternion(0, 1, 1, 1);
    static Quaternion magBias = Quaternion(0, 4, 0, 0);
    static Quaternion magMin = Quaternion(0, 10000, 10000, 10000);
    static Quaternion magMax = Quaternion(0, -10000, -10000, -10000);

    static Quaternion gyroScale = Quaternion(0, 1, 1, 1);
    static Quaternion gyroBias = Quaternion(0, 0, 0, 0);

    static Quaternion accelScale = Quaternion(0, 0.9875f, 0.99383f, 0.98862f);
    static Quaternion accelBias = Quaternion(0, -0.055f, -0.1325f, -0.03f);
    static Quaternion accelMin = Quaternion(0, 10000, 10000, 10000);
    static Quaternion accelMax = Quaternion(0, -10000, -10000, -10000);


    SensorDataStruct *sensorData = &systemData->sensorData;



    {   

        if (firstRun) {
            firstRun = false;

            Wire.begin();

            sensorData->rawData.accelSpeedIntegration = true;
            sensorData->rawData.speedPositionIntegration = true;
            sensorData->rawData.GNSSSpeedCorrectionFactor = INS_FILTER_SPEED;
            sensorData->rawData.GNSSPositionCorrectionFactor = INS_FILTER_POSITION;
            sensorData->rawData.IMUAccelCorrectionFactor = IMU_FILTER_ACCEL;
            sensorData->rawData.IMUMagCorrectionFactor = IMU_FILTER_MAG;
            sensorData->rawData.IMUAccelLPF = IMU_FILTER_ACCEL_LPF;
            sensorData->rawData.IMUGyroLPF = IMU_FILTER_GYRO_LPF;
            sensorData->rawData.EMUSpeedCorrectionFactor = EMU_FILTER_SPEED;
            sensorData->rawData.EMUHeightCorrectionFactor = EMU_FILTER_HEIGHT;
        }


        if (sensorData->rawData.GNSSStatus == SENSOR_NOT_STARTED && GNSSStartFailures < SENSOR_MAX_RESTARTS) {

            GNSS.begin();
            sensorData->rawData.GNSSStatus = SENSOR_STARTED;

        } else if (GNSSStartFailures >= SENSOR_MAX_RESTARTS && sensorData->rawData.GNSSStatus != SENSOR_FAILED) {
            sensorData->rawData.GNSSStatus = SENSOR_FAILED;
        }

        
        if (sensorData->rawData.SonicStatus == SENSOR_NOT_STARTED) {

            pinMode(ULTRASONIC_ECHO_PIN, INPUT);
            pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
            digitalWrite(ULTRASONIC_TRIG_PIN, LOW);

            attachInterrupt(ULTRASONIC_ECHO_PIN, sonicInterruptRoutine, CHANGE);

            sensorData->rawData.SonicStatus = SENSOR_STARTED;

        } 


        if (sensorData->rawData.EMUStatus == SENSOR_NOT_STARTED && EMUStartFailures < SENSOR_MAX_RESTARTS) {

            Wire.setClock(100000);
            
            if (EMU.begin(0x76, &Wire)) {

                EMU.setSampling(
                    Adafruit_BME280::MODE_NORMAL,
                    Adafruit_BME280::SAMPLING_X2,
                    Adafruit_BME280::SAMPLING_X1,
                    Adafruit_BME280::SAMPLING_X1,
                    Adafruit_BME280::FILTER_X16,
                    Adafruit_BME280::STANDBY_MS_0_5
                );

                Serial.println("EMU Start Successful.");
                sensorData->rawData.EMUStatus = SENSOR_STARTED;


            } else {
                                
                Serial.println("EMU failure");
                EMUStartFailures++;
                                
            }

            Wire.setClock(400000);            


        } else if (EMUStartFailures >= SENSOR_MAX_RESTARTS && sensorData->rawData.EMUStatus != SENSOR_FAILED) {
            sensorData->rawData.EMUStatus = SENSOR_FAILED;
        }


        if (sensorData->rawData.magStatus == SENSOR_NOT_STARTED && magStartFailures < SENSOR_MAX_RESTARTS) {

            Wire.setClock(100000);   

            mag.initialize();
            
            if (mag.testConnection()) {

                Serial.println("Mag Start Success");

                mag.setDataRate(50);
                mag.setSampleAveraging(0);

                sensorData->rawData.magStatus = SENSOR_STARTED;

            } else {

                Serial.println("Mag Start Failure");
                magStartFailures++;

            }

            Wire.setClock(400000);            


        } else if (magStartFailures >= SENSOR_MAX_RESTARTS && sensorData->rawData.magStatus != SENSOR_FAILED) {
            sensorData->rawData.magStatus = SENSOR_FAILED;
        }


        if (sensorData->rawData.IMUStatus == SENSOR_NOT_STARTED && IMUStartFailures < SENSOR_MAX_RESTARTS) {

            if (IMU.begin() >= 0) {

                IMU.enableDataReadyInterrupt();

                //IMU.calibrateGyro();
                IMU.setSrd(0);
                IMU.enableFifo(true, true, false, false);
                //IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_SPECIAL);
                IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_250HZ);
                IMU.setGyroRange(MPU9250::GYRO_RANGE_2000DPS);
                IMU.setAccelRange(MPU9250::ACCEL_RANGE_16G);

                sensorData->rawData.IMUStatus = SENSOR_STARTED;

                Serial.println("IMU start Successful.");

            } else {

                Serial.println("IMU Failure");

                IMUStartFailures++;

            }


        } else if (IMUStartFailures >= SENSOR_MAX_RESTARTS && sensorData->rawData.IMUStatus != SENSOR_FAILED) {
            sensorData->rawData.IMUStatus = SENSOR_FAILED;
        }


    }


    {   



        {

            static uint32_t lastTimestamp = 0;
            static uint32_t lastTrig = 0;
            static float lastDistance = 0;

            if ((sensorData->rawData.SonicStatus == SENSOR_STARTED || sensorData->rawData.SonicStatus == SENSOR_RUNNING) && (micros() - sensorData->rawData.timestampSonic >= 1000000.0f/SONIC_SAMPLINGRATE_LIMIT)) {
                sensorData->rawData.timestampSonic = micros();

                if (newEcho) {
                    newEcho = false;

                    sensorData->rawData.sonicEchoTimestamp = echoEnd;

                    double dTime = (micros() - lastTimestamp)/1000000.0;
                    lastTimestamp = echoEnd;

                    float distance = double(echoEnd - echoStart)*sensorData->rawData.speedOfSound/2000000.0;

                    distance = constrain(distance, 0.0f, 3.0f);

                    float speed = (distance - lastDistance)/dTime;
                    lastDistance = distance;

                    sensorData->rawData.sonicDistance = sensorData->rawData.sonicDistance*SONIC_DISTANCE_LPF + distance*(1.0f - SONIC_DISTANCE_LPF);
                    sensorData->rawData.sonicSpeed = sensorData->rawData.sonicSpeed*SONIC_SPEED_LPF + speed*(1.0f - SONIC_SPEED_LPF);
                    sensorData->rawData.sonicDataID++;
                    sensorData->rawData.SonicStatus = SENSOR_STATUS::SENSOR_RUNNING;


                    sonicTrig();
                    lastTrig = micros();

                } else if (lastTrig - micros() >= 50*1000) {
                    lastTrig = micros();

                    sonicTrig();

                }

            }

        }



        {


            static uint32_t lastTimestamp = 0;
            static float lastAltitude = 0;


            if ((sensorData->rawData.EMUStatus == SENSOR_STARTED || sensorData->rawData.EMUStatus == SENSOR_RUNNING) && micros() - sensorData->rawData.timestampEMU >= 1000000/EMU_SAMPLINGRATE_LIMIT) {
                sensorData->rawData.timestampEMU = micros();

                float dTime = float(sensorData->rawData.timestampEMU - lastTimestamp)/1000000.0f;
                lastTimestamp = sensorData->rawData.timestampEMU;

                sensorData->rawData.barometricHeight = EMU.readAltitude(sensorData->rawData.seaLevelPressure/100.0f); // sealevel pressure should be calculated from the gps height when available.

                float EMUSpeed = double(sensorData->rawData.barometricHeight - lastAltitude)/dTime;
                lastAltitude = sensorData->rawData.barometricHeight;

                sensorData->rawData.airPressure = EMU.readPressure();
                sensorData->rawData.airTemperature = EMU.readTemperature();

                sensorData->speed.z = sensorData->speed.z*(1.0f - sensorData->rawData.EMUSpeedCorrectionFactor) - EMUSpeed*sensorData->rawData.EMUSpeedCorrectionFactor;
                sensorData->position.setHeightMSL(sensorData->position.getHeightMSL()*(1.0 - sensorData->rawData.EMUHeightCorrectionFactor) + sensorData->rawData.barometricHeight*sensorData->rawData.EMUHeightCorrectionFactor);

                sensorData->rawData.EMUdataID++;

                if (EMUSpeed < 0.5f) {
                    sensorData->rawData.EMUStatus = SENSOR_RUNNING;
                }

            }


        }



        {

            if ((sensorData->rawData.GNSSStatus == SENSOR_STARTED || sensorData->rawData.GNSSStatus == SENSOR_RUNNING) && GNSS.readSensor()) {
                sensorData->rawData.timestampGNSS = micros();

                sensorData->rawData.numberOfSatellites = GNSS.getNumSatellites();

                if (sensorData->rawData.numberOfSatellites >= GNSS_MIN_SATS) {
                    
                    sensorData->rawData.positionGNSS.setPosFromGPS(
                        GNSS.getLatitude_deg(),
                        GNSS.getLongitude_deg(),
                        GNSS.getMSLHeight_m()
                    );

                    sensorData->rawData.speedGNSS.x = GNSS.getNorthVelocity_ms();
                    sensorData->rawData.speedGNSS.y = -GNSS.getEastVelocity_ms();
                    sensorData->rawData.speedGNSS.z = -GNSS.getDownVelocity_ms();

                    sensorData->rawData.GNSSdataID++;
                    sensorData->rawData.GNSSStatus = SENSOR_RUNNING;

                    INSFilterUpdate(sensorData);

                } else {

                    sensorData->rawData.GNSSStatus = SENSOR_STARTED;

                }

            }

        }



        {

            if ((sensorData->rawData.magStatus == SENSOR_STARTED || sensorData->rawData.magStatus == SENSOR_RUNNING) && (micros() - sensorData->rawData.timestampMag >= 1000000.0f/MAG_SAMPLINGRATE_LIMIT)) {
                sensorData->rawData.timestampMag = micros();

                Quaternion m = Quaternion(0,0,0,0);

                float gain = 1090.0f/100.0f;

                int16_t x;
                int16_t y;
                int16_t z;
                mag.getHeading(&x, &y, &z);

                m.x = float(-y)/gain;
                m.y = float(-x)/gain;
                m.z = float(-z)/gain;

                /*//Calibration
                        if (sensorData->rawData.IMUStatus == SENSOR_RUNNING) {   

                            static Quaternion gyroBiasBuf = Quaternion(0, 0.0f, 0.0f, 0.0f);

                            if (systemData->calibration && !IMUCalibrating) {
                                IMUCalibrating = true;

                                magMin = Quaternion(0, 10000, 10000, 10000);
                                magMax = Quaternion(0, -10000, -10000, -10000);
                                magScale = Quaternion(0, 1.0f, 1.0f, 1.0f);
                                magBias = Quaternion(0, 0.0f, 0.0f, 0.0f);

                                runsCal = 0;

                            } else if (systemData->calibration && IMUCalibrating) {

                                static uint32_t lastMagDataID = -1;

                                if (sensorData->rawData.magDataID != lastMagDataID) {
                                    lastMagDataID = sensorData->rawData.magDataID;

                                    Quaternion mag = sensorData->rawData.magRaw;

                                    if (magMax.x > mag.x) {
                                        magMax.x = mag.x;
                                    } else if (magMin.x < mag.x) {
                                        magMin.x = mag.x;
                                    }

                                    if (magMax.y > mag.y) {
                                        magMax.y = mag.y;
                                    } else if (magMin.y < mag.y) {
                                        magMin.y = mag.y;
                                    }

                                    if (magMax.z > mag.z) {
                                        magMax.z = mag.z;
                                    } else if (magMin.z < mag.z) {
                                        magMin.z = mag.z;
                                    }

                                }


                            } else if (!systemData->calibration && IMUCalibrating) {
                                IMUCalibrating = false;

                                //magBias = (magMax - magMin)/2.0f;
                                //magScale = (magMax - magBias/2.0f);


                            }*/

                sensorData->rawData.magRaw = m - magBias;

                float fieldStrength = sensorData->rawData.magRaw.getMagnitude();

                if (fieldStrength > 5 && fieldStrength < 200) {
                    sensorData->rawData.magStatus = SENSOR_RUNNING;
                } else {
                    sensorData->rawData.magStatus = SENSOR_STARTED;
                }

                sensorData->rawData.magDataID++;

                IMUFilterUpdate(sensorData);

            }

        }



        {
            
            if ((sensorData->rawData.IMUStatus == SENSOR_STARTED || sensorData->rawData.IMUStatus == SENSOR_RUNNING) && !digitalReadFast(IMU_INTERRUPT_PIN)) {

                uint32_t fifoCount = IMU.getFifoCount();

                if (fifoCount >= 512) {
                    IMU.resetFifo();
                    Serial.println("Fifo Reset");
                } else if (fifoCount > 0) {

                    IMU.readFifo();

                    float ax[100], ay[100], az[100];
                    float gx[100], gy[100], gz[100];
                    float mx[100], my[100], mz[100];
                    size_t fifoSize;

                    IMU.getFifoAccelX_mss(&fifoSize,ax);
                    IMU.getFifoAccelY_mss(&fifoSize,ay);
                    IMU.getFifoAccelZ_mss(&fifoSize,az);
                    IMU.getFifoGyroX_rads(&fifoSize,gx);
                    IMU.getFifoGyroY_rads(&fifoSize,gy);
                    IMU.getFifoGyroZ_rads(&fifoSize,gz);

                    for (int n = 0; n < fifoSize; n++) {

                        sensorData->rawData.timestampIMU = micros();

                        float dTime = 1000000/IMU_SAMPLINGRATE_LIMIT;

                        Quaternion gyro;
                        gyro.x = (gy[n] - gyroBias.x)*gyroScale.x;
                        gyro.y = (gx[n] - gyroBias.y)*gyroScale.y;
                        gyro.z = (-gz[n] - gyroBias.z)*gyroScale.z;
                        sensorData->rawData.rateRaw = sensorData->rawData.rateRaw*sensorData->rawData.IMUGyroLPF + gyro*(1.0f - sensorData->rawData.IMUGyroLPF);
                        sensorData->rawData.rateRaw.w = 0.0f;
                        sensorData->rawData.gyroDataID++;

                        Quaternion accel;
                        accel.x = (-ay[n] - accelBias.x)*accelScale.x;
                        accel.y = (-ax[n] - accelBias.y)*accelScale.y;
                        accel.z = (az[n] - accelBias.z)*accelScale.z;
                        sensorData->rawData.accelRaw = sensorData->rawData.accelRaw*sensorData->rawData.IMUAccelLPF + accel*(1.0f - sensorData->rawData.IMUAccelLPF);
                        sensorData->rawData.accelRaw.w = 0.0f;
                        sensorData->rawData.accelDataID++;

                        float gl = sensorData->rawData.rateRaw.getMagnitude();
                        float al = sensorData->rawData.accelRaw.getMagnitude();

                        if (gl < GYRO_CALIBRATION_THRESHOLD && al > 9 && al < 11) {
                            sensorData->rawData.IMUStatus = SENSOR_RUNNING;
                        }

                        //Calibration
                        if (sensorData->rawData.IMUStatus == SENSOR_RUNNING) {   

                            static Quaternion gyroBiasBuf = Quaternion(0, 0.0f, 0.0f, 0.0f);
                            static uint32_t gyroCalSamples = 0;
                            static uint32_t calGyroTimestamp = 0;
                            static bool accelCalibrated = false;

                            if (systemData->calibration && !IMUCalibrating) {
                                IMUCalibrating = true;

                                gyroBiasBuf = Quaternion(0, 0.0f, 0.0f, 0.0f);

                                /*accelMin = Quaternion(0, 10000, 10000, 10000);
                                accelMax = Quaternion(0, -10000, -10000, -10000);
                                accelScale = Quaternion(0, 1.0f, 1.0f, 1.0f);
                                accelBias = Quaternion(0, 0.0f, 0.0f, 0.0f);*/

                                gyroCalSamples = 0;
                                calGyroTimestamp = millis();
                                accelCalibrated = false;

                            } else if ((systemData->calibration || millis() - calGyroTimestamp < GYRO_CALIBRATION_TIME || !accelCalibrated) && IMUCalibrating) {

                                static uint32_t lastGyroID = -1;

                                if (sensorData->rawData.rateRaw.getMagnitude() < GYRO_CALIBRATION_THRESHOLD*DEG_TO_RAD) {
                                    gyroBiasBuf += gyro;
                                    runsCal++;
                                }

                                accelCalibrated = true;


                            } else if (!systemData->calibration && IMUCalibrating) {
                                IMUCalibrating = false;

                                if (runsCal > 0) gyroBias += gyroBiasBuf/runsCal;


                            }

                        }

                        IMUFilterUpdate(sensorData);

                        INSFilterUpdate(sensorData);

                    }

                    

                }

            }

        }




        {   

            if (sensorData->rawData.IMUStatus == SENSOR_STATUS::SENSOR_FAILED) {
                sensorData->sensorMode = SENSOR_MODE::SENSOR_TOTAL_FAILURE;
            }

        }




    }

}




void hardwarePPMLoop(SystemDataStruct *systemData) {

    static bool firstRun = true;
    static uint32_t lastOutputDataID = 0;
    static bool enableMotors = false;
    static uint32_t lastNewDataIDTimestamp = millis();

    PPMDataStruct *PPMData = &systemData->servoData;


    if (firstRun) {
        firstRun = false;

        pinMode(SERVO_PIN_1, OUTPUT);
        pinMode(SERVO_PIN_2, OUTPUT);
        pinMode(SERVO_PIN_3, OUTPUT);
        pinMode(SERVO_PIN_4, OUTPUT);
        //pinMode(SERVO_PIN_5, OUTPUT);
        //pinMode(SERVO_PIN_6, OUTPUT);
        //pinMode(SERVO_PIN_7, OUTPUT);

        analogWriteFrequency(SERVO_PIN_1, PPM_SPEED_FAST);
        analogWriteFrequency(SERVO_PIN_2, PPM_SPEED_FAST);
        analogWriteFrequency(SERVO_PIN_3, PPM_SPEED_FAST);
        analogWriteFrequency(SERVO_PIN_4, PPM_SPEED_FAST);
        //analogWriteFrequency(SERVO_PIN_5, PPM_SPEED_SLOW);
        //analogWriteFrequency(SERVO_PIN_6, PPM_SPEED_SLOW);
        //analogWriteFrequency(SERVO_PIN_7, PPM_SPEED_SLOW);

        analogWriteResolution(PPM_RESOLUTION);

        analogWrite(SERVO_PIN_1, 0);
        analogWrite(SERVO_PIN_2, 0);
        analogWrite(SERVO_PIN_3, 0);
        analogWrite(SERVO_PIN_4, 0);
        //analogWrite(SERVO_PIN_5, 0);
        //analogWrite(SERVO_PIN_6, 0);
        //analogWrite(SERVO_PIN_7, 0);

        

    }


    if (PPMData->dataID != lastOutputDataID) {
        lastOutputDataID = PPMData->dataID;
        lastNewDataIDTimestamp = micros();

        float x;

        if (systemData->enableMotors) {

            x = (constrain(PPMData->data[MOTOR_CHANNEL_1]*0.5f + 0.5f, 0.0f, 1.0f)*(PPM_MAX_US - PPM_MIN_US) + PPM_MIN_US)/float(PPM_FAST_US_TO_BIT); // convert to 0 to 1
            if (x == x) analogWrite(SERVO_PIN_1, int(x));
            else analogWrite(SERVO_PIN_1, int(PPM_MIN_US/float(PPM_FAST_US_TO_BIT)));

            x = (constrain(PPMData->data[MOTOR_CHANNEL_2]*0.5f + 0.5f, 0.0f, 1.0f)*(PPM_MAX_US - PPM_MIN_US) + PPM_MIN_US)/float(PPM_FAST_US_TO_BIT); // convert to 0 to 1
            if (x == x) analogWrite(SERVO_PIN_2, int(x));
            else analogWrite(SERVO_PIN_2, int(PPM_MIN_US/float(PPM_FAST_US_TO_BIT)));
                
            x = (constrain(PPMData->data[MOTOR_CHANNEL_3]*0.5f + 0.5f, 0.0f, 1.0f)*(PPM_MAX_US - PPM_MIN_US) + PPM_MIN_US)/float(PPM_FAST_US_TO_BIT); // convert to 0 to 1
            if (x == x) analogWrite(SERVO_PIN_3, int(x));
            else analogWrite(SERVO_PIN_3, int(PPM_MIN_US/float(PPM_FAST_US_TO_BIT)));
                
            x = (constrain(PPMData->data[MOTOR_CHANNEL_4]*0.5f + 0.5f, 0.0f, 1.0f)*(PPM_MAX_US - PPM_MIN_US) + PPM_MIN_US)/float(PPM_FAST_US_TO_BIT); // convert to 0 to 1
            if (x == x) analogWrite(SERVO_PIN_4, int(x));
            else analogWrite(SERVO_PIN_4, int(PPM_MIN_US/float(PPM_FAST_US_TO_BIT)));

        } else {

            analogWrite(SERVO_PIN_1, int(PPM_MIN_US/float(PPM_FAST_US_TO_BIT)));
            analogWrite(SERVO_PIN_2, int(PPM_MIN_US/float(PPM_FAST_US_TO_BIT)));
            analogWrite(SERVO_PIN_3, int(PPM_MIN_US/float(PPM_FAST_US_TO_BIT)));
            analogWrite(SERVO_PIN_4, int(PPM_MIN_US/float(PPM_FAST_US_TO_BIT)));

        }

        /*x = ((PPMData->data[AUX_CHANNEL_1]*0.5f + 0.5f)*(2000 - 1000) + 1000)/float(PPM_SLOW_US_TO_BIT); // convert to 0 to 1
        analogWrite(SERVO_PIN_5, int(x));*/


    } else if (micros() - lastNewDataIDTimestamp >= 1000000/CONTROL_SAMPLINGRATE_LIMIT*10) {

        analogWrite(SERVO_PIN_1, int(PPM_MIN_US/float(PPM_FAST_US_TO_BIT)));
        analogWrite(SERVO_PIN_2, int(PPM_MIN_US/float(PPM_FAST_US_TO_BIT)));
        analogWrite(SERVO_PIN_3, int(PPM_MIN_US/float(PPM_FAST_US_TO_BIT)));
        analogWrite(SERVO_PIN_4, int(PPM_MIN_US/float(PPM_FAST_US_TO_BIT)));

        systemData->enableMotors = false;
        systemData->systemArmed = false;

    }

}




void hardwareIBUSLoop(SystemDataStruct *systemData) {

    static byte failureCounterIBUS = 0;
    static uint32_t lastIBUSupdateus = 0;  
    static bool IBUSStarted = false;

    PPMDataStruct *IBUSData = &systemData->IBUSData;

    if (!IBUSStarted) {

        IBus.begin(Serial4);

        systemData->IBUSFailsafe = true;
        IBUSData->channels = 14;

        IBUSStarted = true;

    }


    if (IBUSStarted && IBus.loop()) { 

        PPMDataStruct PPMData = *IBUSData;

        for (int n = 0; n < 14; n++) {
            PPMData.data[n] = float(IBus.readChannel(n)-1500)/500.0f;
        }

        if (abs(PPMData.data[6]) > 1.4f || abs(PPMData.data[7]) > 1.4f) {
            systemData->IBUSFailsafe = true;
        } else {
            systemData->IBUSFailsafe = false;
        }

        //*IBUSData = PPMData;
        for (int n = 0; n < IBUSData->channels; n++) {
            IBUSData->data[n] = IBUSData->data[n]*IBUSDATA_LPF + PPMData.data[n]*(1.0f - IBUSDATA_LPF);
        }

        IBUSData->timestamp = micros();
        IBUSData->dataID++;
        lastIBUSupdateus = micros();
        

    }

    if (micros() - lastIBUSupdateus >= IBUS_TIMEOUT*1000) {
        systemData->IBUSFailsafe = true;
    }

}


#endif