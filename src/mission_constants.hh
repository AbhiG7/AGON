#ifndef __MISSION_CONSTANTS_HH__
#define __MISSION_CONSTANTS_HH__

#include <cstdint>


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

// TODO: determine what all these are for
const float fireTBuffer = 0.01*MEGA;  // (TODO: units) TODO: what is this?
const float fireBuffer = 3276.0/2.0/16.0;  // (m/s^2) ~0.25 TODO: what is this
const float fireGoal = 0.0;  // -g TODO: what is this?

const float landTBuffer = 0.5*MEGA;  // (TODO: units) TODO: what is this? 
const float landBuffer = 3276.0/2.0/8.0; // (TODO: units) TODO: what is this?
const float landGoal = 0.0;  // (TODO: units) TODO: what is this?

// TODO: what are these?
long fireTime;
long liftoffTime;
long landTime;
bool buffStatus;
float buffCountDown;

//*****************************************************************************
//                                  HARDWARE
//*****************************************************************************
/* LSB value for linear acceleration measurements.
 * See MPU6050::getAcceleration docstring for options.
 * Current setting is a full-scale range of +/- 8 gee.
 */
const float LSB_LINEAR = 2048.0;  // (milli-gee/LSB) note documentation gives incorrectly inverted units

/* LSB value for angular velocity measurements.
 * See MPU6050::getRotation docstring for options.
 * Current setting is a full-scale range of +/-m 250 deg/s.
 */
const float LSB_ANGULAR = 131.0;  // (deg/s/LSB) note documentation gives incorrectly inverted units

/* These bias calibration values account for sensor bias
 * in linear acceleration measurements. Can't be declared
 * a const since it's calculated on the fly every startup.
 */
int16_t IMU_ACC_BIAS[3];  // (count) IMU linear acceleration along each axis

// LEDs
const int B_LED_1 = 6;  // TODO: add description
const int G_LED_1 = 4;  // TODO: add description
const int B_LED_2 = 9;  // TODO: add description
const int G_LED_2 = 8;  // TODO: add description

// Pyro TODO: more descriptive name?
const int PYRO_PIN = 22;  // TODO: add description

// Thrust-Vector Controller (TVC)
const int TVC_TOP_PIN = 5;  // TODO: add description
const int TVC_X_PIN = 20;  // TODO: add description
const int TVC_Y_PIN = 21;  // TODO: add description

const int TVC_X_OFFSET = 85;  // TODO: add description
const int TVC_Y_OFFSET = 83;  // TODO: add description

const float BETA = 0.95;  // TODO: add description
const int GEAR = 9;  // TODO: add description
const float DAMPER = 1.5;  // TODO: add description
const int TVC_DELAY = 10;  // TODO: add description

// TODO: define these quaternions based on IMU installation in rocket
const float QR_IMU_TO_BODY[4] = {1.0, 0.0, 0.0, 0.0};  // (--) right, scalar-first, Hamiltonian quaternion transforming IMU frame to body frame
const float QR_BODY_TO_IMU[4] = {1.0, 0.0, 0.0, 0.0};  // (--) right, scalar-first, Hamiltonian quaternion transforming body frame to IMU frame


#endif  // __MISSION_CONSTANTS_HH__