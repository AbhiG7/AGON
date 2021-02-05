#ifndef __MISSION_CONSTANTS_HH__
#define __MISSION_CONSTANTS_HH__

#include <cstdint>
#include <Eigen313.h>  // linear algebra library
#include <LU>  // needed for Eigen313.h to work


//*****************************************************************************
//                             CONVERSION FACTORS
//*****************************************************************************
const float DEG_2_RAD = 0.017453292519943295;  // (rad/deg) conversion factor from degrees to radians
const float RAD_2_DEG = 57.29577951308232;  // (deg/rad) conversion factor from radians to degrees
const float G_2_MS2 = 9.80665;  // (m/s^2/gee) conversion factor from gee to m/s^2

const float MEGA = 1.0e9;  // (--) SI Mega prefix

//*****************************************************************************
//                                  MISSION
//*****************************************************************************
//TODO: modify modes



const bool eraseFlash=false;

// Missing modes (controls what happens in the loop)
enum Mode {
    STARTUP_STABLE = 0,
    NAVIGATION = 1,
    BURN_BABY_BURN = 2,
    SHUTDOWN_STABLE = 3
};

// timing
const int COUNTDOWN_WAIT = 10;  // (TODO: units) TODO: what is this?
const int CALIBRATE_TIME = 0;  // (TODO: units) TODO: what is this?

// z-acceleration > fireGoal+fireBuffer continuously for fireTBuffer before switching to 
const float fireTBuffer = 0.01*MEGA;  // (us) TODO: what is this?
const float fireBuffer = 3276.0/2.0/16.0;  // (m/s^2) ~0.25 TODO: what is this
const float fireGoal = 0.0;  // -g TODO: what is this?

// |z-acceleration| < landBuffer+landGoal continuously for landTBuffer before switching to 
const float landTBuffer = 0.5*MEGA;  // (TODO: units) TODO: what is this? 
const float landBuffer = 3276.0/2.0/8.0; // (TODO: units) TODO: what is this?
const float landGoal = 0.0;  // (TODO: units) TODO: what is this?

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

/* These bias calibration values account for sensor bias
 * in linear acceleration and angular velocity measurements.
 * Can't be declared a const since it's calculated
 * on-the-fly every startup.
 */
int16_t IMU_LIN_ACC_BIAS[3];  // (count) IMU linear acceleration bias along each axis
int16_t IMU_ANG_VEL_BIAS[3];  // (count) IMU angular velocity bias about each axis

// LEDs 
//TODO:modify values
const int B_LED_1 = 6;  // Blue LED1 Pin
const int G_LED_1 = 4;  // Green LED1 Pin
const int B_LED_2 = 9;  // Blue LED2 Pin
const int G_LED_2 = 8;  // Green LED2 Pin

/* BLINK INDICES:  
 *   - 0: Rate (Hz)
 *   - 1-4: A, command for each of Green 1, Blue 1, Green 2, Blue 2
 *   - 5-8: B, command for each of Green 1, Blue 1, Green 2, Blue 2
 */
//TODO:debug on board
const int BLINK_0[9] = {1, 1, 0, 1, 0, 0, 0, 0, 0};
const int BLINK_1[9] = {1, 0, 1, 0, 1, 0, 1, 0, 1};
const int BLINK_2[9] = {1, 1, 0, 1, 0, 1, 0, 1, 0};
const int BLINK_3[9] = {1, 0, 1, 0, 1, 0, 1, 0, 1};

const int MOTOR_PIN = 22;  // Pin that signals motor to fire

// Thrust-Vector Controller (TVC)  //TODO: modify values
const int DROP_PIN = 5;  // pin for the servo in the drop mechanism
const int TVC_X_PIN = 20;  // pin for the servo that actuates TVC around x body axis
const int TVC_Y_PIN = 21;  // pin for the servo that actuates TVC around y body axis
const int GEAR = 9;  // gearing ratio of the servo to the TVC

const int drop_mechanism_hold=10; //TODO: modify after assembly
const int drop_mechanism_release=120; //TODO: modify after assembly

const int TVC_X_OFFSET = 85;  // TODO: add description
const int TVC_Y_OFFSET = 83;  // TODO: add description

const float BETA = 0.95;  // angle (rad) that corrects for misalignment between body frame and TVC frame
const int TVC_DELAY = 2;  // time (in miliseconds) for servo to move 1 deg--> how much delay between servo commands

// TODO: define these quaternions based on IMU installation in rocket
const float QR_IMU_TO_BODY[4] = {1.0, 0.0, 0.0, 0.0};  // (--) right, scalar-first, Hamiltonian quaternion transforming IMU frame to body frame
const float QR_BODY_TO_IMU[4] = {1.0, 0.0, 0.0, 0.0};  // (--) right, scalar-first, Hamiltonian quaternion transforming body frame to IMU frame


//control constants
MatrixXf A_template(36); //dynamics matrix
MatrixXf B_template(12); //input matrix
MatrixXf L_template(36); //Kalman gain
MatrixXf K_template(12); //LQR -> updates mid flight
MatrixXf C(36); //Sensor matrix
C=MatrixXd::Identity(6, 6);

//thrust curve
const float T={30, 9}; //Thrust values (in Newtons)
const float T_time={0, .33}; //time in the burn when we start to use the corresponding thrust value
const int T_intervals=2; //how many thrust data points do we have



#endif  // __MISSION_CONSTANTS_HH__