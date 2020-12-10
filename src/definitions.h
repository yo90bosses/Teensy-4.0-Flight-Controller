#ifndef _DEFINITIONS_H_
#define _DEFINITIONS_H_

#include <Arduino.h>


//Samplingrates
#define IMU_SAMPLINGRATE_LIMIT 4000
#define MAG_SAMPLINGRATE_LIMIT 50
#define SONIC_SAMPLINGRATE_LIMIT 50
#define EMU_SAMPLINGRATE_LIMIT 50
#define SYSTEM_SAMPLINGRATE 20000

#define GUIDANCE_SAMPLINGRATE_LIMIT 100
#define CONTROL_SAMPLINGRATE_LIMIT 8000

#define DEBUG_SAMPLINGRATE 30

#define IMU_FAILUREDETECT_SAMPLES 100 // if over 1oo samples were missed then assume IMU failure

//Tuning

//PID hold factors
//P Factor
#define PID_RP_X 15.0f
#define PID_RP_Y 15.0f
#define PID_RP_Z 30.0f

//I Factor
#define PID_RI_X 0.0f
#define PID_RI_Y 0.0f
#define PID_RI_Z 0.0f

#define PID_RI_LIMIT 0.0f//0.5f //PID I value Limit for Anti-Windup

//D Factor
#define PID_RD_X 0.1f//0.250f
#define PID_RD_Y 0.1f//0.250f
#define PID_RD_Z 0.0f//0.5f

//PID Rate factors
//P Factor
#define PID_P_X 0.28f
#define PID_P_Y 0.28f
#define PID_P_Z 0.70f

//I Factor
#define PID_I_X 0.0003f
#define PID_I_Y 0.0003f
#define PID_I_Z 0.0005f
#define PID_I_LIMIT 0.2f//0.5f //PID I value Limit for Anti-Windup

#define PID_I_THROTTLE_RESET -0.85f

#define PID_ANTIGRAVITY_MULTIPLIER 1.0f //3.0f
#define PID_ANTIGRAVITY_THRESHOLD 2.0f

//D Factor
#define PID_D_X 0.0003f
#define PID_D_Y 0.0003f
#define PID_D_Z 0.0f//0.0002f

#define PID_D_LPF 0.4f

#define PID_D_USE_SETPOINT false

//FeedForward
#define PID_F_X 0.0f
#define PID_F_Y 0.0f
#define PID_F_Z 0.0f


//Autopilot settings

//Auto position hold factors
#define POS_HOLD_P 0.8f
#define POS_HOLD_I 0.0f
#define POS_HOLD_D 0.4f

#define AUTOPILOT_POWER_LPF 0.8f

#define MAX_SETPOINT_DEVIATION 30.0f


//Complimentary filter settings

//IMU
#define IMU_FILTER_ACCEL 0.0002f
#define IMU_FILTER_MAG 0.0f
#define IMU_FILTER_ACCEL_LPF 0.95f
#define IMU_FILTER_GYRO_LPF 0.7f

//EMU
#define EMU_FILTER_PRESSURE_LPF 0.7f
#define EMU_FILTER_SPEED 0.4f//0.05f
#define EMU_FILTER_HEIGHT 0.5f//0.1f

//Ultrasonic
#define SONIC_DISTANCE_LPF 0.8f
#define SONIC_SPEED_LPF 0.8f

//INS Filter specifics
#define INS_FILTER_POSITION 0.4f
#define INS_FILTER_SPEED 0.4f


//IBUS data filtering
#define IBUSDATA_LPF 0.3f


//Settings
#define MODE_LEVEL_MAXANGLE 40.0f

#define ACRO_RATE_X 800.0f
#define ACRO_RATE_Y 800.0f
#define ACRO_RATE_Z 400.0f

#define ACRO_EXPO_X 0.60f
#define ACRO_EXPO_Y 0.60f
#define ACRO_EXPO_Z 0.4f

#define DISARM_CRASH_THRESHOLD_G 10.0f

#define PPM_MIN_US 5
#define PPM_MAX_US 25
#define PPM_SPEED_SLOW 50
#define PPM_SPEED_FAST 0008
#define PPM_RESOLUTION 14

#define MAX_POWER 0.95f
#define IDLE_POWER 0.095f
#define MIN_POWER 0.07f

#define SERVO_PIN_1 2
#define SERVO_PIN_2 3
#define SERVO_PIN_3 4
#define SERVO_PIN_4 5
#define SERVO_PIN_5 6
#define SERVO_PIN_6 7
#define SERVO_PIN_7 8
#define SERVO_PIN_8 9
#define SERVO_PIN_9 10
#define SERVO_COUNT 5

#define ULTRASONIC_TRIG_PIN 15
#define ULTRASONIC_ECHO_PIN 14

#define IMU_INTERRUPT_PIN 32

#define SENSOR_MAX_RESTARTS 5

#define GNSS_MIN_SATS 5

#define GYRO_CALIBRATION_THRESHOLD 5.0f
#define GYRO_CALIBRATION_TIME 1000


//constants
#define MOTOR_CHANNEL_1 0 //Front left, negative spin
#define MOTOR_CHANNEL_2 1 //Front right, positive spin
#define MOTOR_CHANNEL_3 2 //Back left, positive spin
#define MOTOR_CHANNEL_4 3 //Back right, negative spin
#define AUX_CHANNEL_1 4 //connected to camera servo
#define AUX_CHANNEL_2 5 //aux
#define AUX_CHANNEL_3 6 //aux

#define IBUS_CHANNEL_ROLL 0
#define IBUS_CHANNEL_PITCH 1
#define IBUS_CHANNEL_POWER 2
#define IBUS_CHANNEL_YAW 3
#define IBUS_CHANNEL_SWA 4
#define IBUS_CHANNEL_SWB 5
#define IBUS_CHANNEL_SWC 6
#define IBUS_CHANNEL_SWD 7
#define IBUS_CHANNEL_SWE 8
#define IBUS_CHANNEL_VRA 9
#define IBUS_CHANNEL_VRB 10
#define IBUS_CHANNEL_12 11
#define IBUS_CHANNEL_13 12
#define IBUS_CHANNEL_14 13
#define IBUS_CHANNEL_15 14
#define IBUS_CHANNEL_16 15

#define DISARMING_SWITCH IBUS_CHANNEL_SWD
#define CONTROLMODE_SWITCH IBUS_CHANNEL_SWC
#define FLIGHTMODE_SWITCH IBUS_CHANNEL_SWB
#define OHSHITMODE_SWITCH IBUS_CHANNEL_SWA

#define IBUS_TIMEOUT 100

#define PPM_FAST_US_TO_BIT 1000000.0f/PPM_SPEED_FAST/pow(2, PPM_RESOLUTION)
#define PPM_SLOW_US_TO_BIT 1000000.0f/PPM_SPEED_SLOW/pow(2, PPM_RESOLUTION)

#define RAD_DEG 57.29578f
#define DEG_RAD 0.017453f

#define SYSTEMUSAGE_MAX 1625953

#define DRONE_WEIGHT 0.8f // in kg
#define MOTOR_FORCE 7.8f // in Newton

#endif