#ifndef __WORKSPACE_HH__
#define __WORKSPACE_HH__

#include <cstdint>
#include "mission_constants.hh"
#include "moding.hh"
#include "MPU6050.h"  // MPU 6050 IMU Library
#include <Servo.h>  // TODO: what's this?
#include "Wire.h"  // Arduino library


class Workspace
{
    public:
        //*****************************************************************************
        //                              HARDWARE SETUP
        //*****************************************************************************
        MPU6050 imu;  // (--) the IMU to read data from

        //*****************************************************************************
        //                              LOOP VARIABLES
        //*****************************************************************************
        // clock time
        unsigned long t_prev_cycle = 0;  // (us) contains the time of the previous cycle at the start of each loop
        unsigned long dt = 0;  // (us) used to store time difference between t_prev_cycle and return of micros() at the start of each loop

        // sensor measurements
        float a[3] = {0.0, 0.0, 0.0};  // (m/s^2) linear acceleration, used for storing sensor measurements
        float w[3] = {0.0, 0.0, 0.0};  // (rad/s) angular velocity, used for storing sensor measurements

        // flight mode
        Mode mode = STARTUP_STABLE;

        // Thrust-Vector Controller (TVC)
        Servo tvc_top;  // TODO: add description
        Servo tvc_x;  // TODO: add description
        Servo tvc_y;  // TODO: add description

        // state
        float r_body[3] = {0.0, 0.0, 0.0};  // (m) position of the body frame origin TODO: define inertial frame
        float v_body_wrt_inertial_in_inertial[3] = {0.0, 0.0, 0.0};  // (m/s) velocity of body frame origin wrt inertial frame, components resolved in inertial frame
        float qr_body_wrt_inrt[4] = {1.0, 0.0, 0.0, 0.0};  // (--) right, scalar-first, Hamiltonian quaternion transforming body frame into inertial frame
        float qr_inrt_wrt_body[4] = {1.0, 0.0, 0.0, 0.0};  // (--) right, scalar-first, Hamiltonian quaternion transforming inertial frame into body frame
};

#endif  // __WORKSPACE_HH__