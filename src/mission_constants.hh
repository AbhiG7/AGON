#ifndef __MISSION_CONSTANTS_HH__
#define __MISSION_CONSTANTS_HH__

#include <cstdint>


//*****************************************************************************
//                                MISSION MODES
//*****************************************************************************
enum Mode {
    STARTUP_STABLE = 0,
    NAVIGATION = 1,
    BURN_BABY_BURN = 2,
    SHUTDOWN_STABLE = 3
};

//*****************************************************************************
//                              HARDWARE-SPECIFIC
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

// TODO: define these quaternions based on IMU installation in rocket
const float qr_imu_to_body[4] = {1.0, 0.0, 0.0, 0.0};  // (--) right, scalar-first, Hamiltonian quaternion transforming IMU frame to body frame
const float qr_body_to_imu[4] = {1.0, 0.0, 0.0, 0.0};  // (--) right, scalar-first, Hamiltonian quaternion transforming body frame to IMU frame


//*****************************************************************************
//                          PHYSICS CONVERSION FACTORS
//*****************************************************************************
const float DEG_2_RAD = 0.017453292519943295;  // (rad/deg) conversion factor from degrees to radians
const float RAD_2_DEG = 57.29577951308232;  // (deg/rad) conversion factor from radians to degrees
const float G_2_MS2 = 9.80665;  // (m/s^2/gee) conversion factor from gee to m/s^2

#endif  // __MISSION_CONSTANTS_HH__