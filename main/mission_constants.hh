#ifndef __MISSION_CONSTANTS_HH__
#define __MISSION_CONSTANTS_HH__

#include "matrix.hh"
#include <vector>
using namespace std;

//*****************************************************************************
//                             CONVERSION FACTORS
//*****************************************************************************
const float DEG_2_RAD = 0.017453292519943295;  // (rad/deg) conversion factor from degrees to radians
const float RAD_2_DEG = 57.29577951308232;  // (deg/rad) conversion factor from radians to degrees
const float G_2_MS2 = 9.80665;  // (m/s^2/gee) conversion factor from gee to m/s^2

const float MEGA = 1.0e9;  // (--) SI Mega prefix
const int MEGA_I=1e9;
const float KILO=1.0e3;
const float KILO_I=1e3;

//*****************************************************************************
//                                  MISSION
//*****************************************************************************

// Missing modes (controls what happens in the loop)
enum Mode {
    STARTUP_STABLE = 0,
    COUNTDOWN = 1,
    FINAL_COUNTDOWN = 2,
    PREP_TVC = 3,
    BURN_BABY_BURN = 4,
    SHUTDOWN_STABLE = 5
};

const bool TVC_TEST=false;
const bool TVC_ENABLE=true;
const bool KALMAN_ENABLED=false;
const bool NULL_U_X_AXIS=false;
const bool NULL_U_Y_AXIS=false;

//most modes change by time
//after a predefined time period we switch to the next mode
//MODE TIME PERIODS
const int STARTUP_STABLE_PERIOD=2; //15
const int COUNTDOWN_PERIOD=2; //20
const int FINAL_COUNTDOWN_PERIOD=2;
const int PREP_TVC_PERIOD=3;
const int BURN_BABY_BURN_PERIOD=300; //15

//*****************************************************************************
//                                  HARDWARE
//*****************************************************************************
/* LSB value for linear acceleration measurements.
 * See MPU6050::getAcceleration docstring for options.
 * Current setting is a full-scale range of +/- 8 gee,
 * in main::setup, in call to setFullScaleAccelRange.
 */
const float LSB_LINEAR = 2048.0;  // (milli-gee/count) note documentation gives incorrectly inverted units

/* LSB value for angular velocity measurements.
 * See MPU6050::getRotation docstring for options.
 * Current setting is a full-scale range of +/-m 250 deg/s,
 * in main::setup, in call to setFullScaleGyroRange.
 */
const float LSB_ANGULAR = 131.0;  // (deg/s/count) note documentation gives incorrectly inverted units

//PINOUTS
const int TVC_X_PIN = 20;  // pin for the servo that actuates TVC around x body axis
const int TVC_Y_PIN = 21;  // pin for the servo that actuates TVC around y body axis
const int SD_CS_PIN = 0;
const int FLASH_CS_PIN = 0;
const int IMU_1_PIN=0;
const int IMU_2_PIN=0;
const int R_LED_PIN=0;
const int B_LED_PIN=4;
const int G_LED_PIN=6;

// Thrust-Vector Controller (TVC)  //TODO: modify values
const float GEAR = 7.5;  // gearing ratio of the servo to the TVC //9
const float SERVO_SPEED=1.5;
const float MAX_U_DELTA=2;
const float TVC_LOOP=5;
const float MIN_STEP=DEG_2_RAD/GEAR;
const float TVC_X_OFFSET = 107.5; // TODO: add description
const float TVC_Y_OFFSET = 67.5;  // TODO: add description
const float MAX_U= 5*DEG_2_RAD;  //maximum gimbal angle
const float BETA = 135*DEG_2_RAD;  // angle (rad) that corrects for misalignment between body frame and TVC frame

// TODO: define these quaternions based on IMU installation in rocket
const float QR_IMU_TO_BODY[4] = {1.0, 0.0, 0.0, 0.0};  // (--) right, scalar-first, Hamiltonian quaternion transforming IMU frame to body frame
const float QR_BODY_TO_IMU[4] = {1.0, 0.0, 0.0, 0.0};  // (--) right, scalar-first, Hamiltonian quaternion transforming body frame to IMU frame

//IMU parameters
const float EULER_X_OFFSET;
const float EULER_Y_OFFSET;

//rocket physical parameters
const float THRUST=10; //TODO: make exact
const float MOMENT_ARM=.2413; //TODO: make exact
const float MOMENT_INERTIA_XX=.01984; //TODO: make exact
const float MOMENT_INERTIA_YY=.01669;//TODO: make exact
const float MASS=.611;

//control constants TODO: fill these out
const vector<float>  A_VALUES {0,THRUST/MASS,0,0,0,0,
                0,0,1,0,0,0,
                0,0,0,0,0,0,
                0,0,0,-THRUST/MASS,0,0,
                0,0,0,0,0,1,
                0,0,0,0,0,0}; //dynamics matrix
const Matrix A=Matrix(6, 6, A_VALUES);        
 
const vector<float> B_VALUES {THRUST/MASS, 0, 
                        0, 0,
                        -THRUST*MOMENT_ARM/MOMENT_INERTIA_YY, 0,
                        0, -THRUST/MASS,
                        0, 0,
                        0, -THRUST*MOMENT_ARM/MOMENT_INERTIA_XX}; //input matrix
const Matrix B=Matrix(6, 2, B_VALUES);
 
const vector<float> K_VALUES {-.6325,-5.5608,-1.4869,0,0,0,
                        0,0,0,.6325,-5.5608,-1.4869}; //controller gain
const Matrix KC=Matrix(2, 6, K_VALUES);

const vector<float> C_VALUES {1,0,0,0,0,0,
                        0,0,1,0,0,0,
                        0,0,0,1,0,0,
                        0,0,0,0,0,1}; //sensor matrix
const Matrix C=Matrix(4, 6, C_VALUES);
 
const vector<float> L_VALUES {13.824, 6.9192, 0, 0,
                        6.9192, 8.3015, 0, 0,
                        20.0079, 53.3949, 0, 0,
                        0, 0, 13.824, -6.9192,
                        0, 0, -6.9192, 8.3015,
                        0, 0, -20.0079, 53.3949}; //kalman gain
const Matrix L=Matrix(6, 4, L_VALUES);

#endif
