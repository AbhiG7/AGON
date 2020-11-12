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
        MPU6050 imu_0(0x68);  // (--) first IMU to read data from
        MPU6050 imu_1(0x69);  // (--) second IMU to read data from

        //*****************************************************************************
        //                              LOOP VARIABLES
        //*****************************************************************************
        // LED blinks TODO: add descriptions to these
        int blink_master[36];  // TODO: maybe delay
        long next_blink;  // the next time (us) when the led colors should switch
        bool blink_flag = false;  //led colors switch when blink_flag is true

        // clock time
        unsigned long calibrate_time = 0;  //momment main loop starts
        unsigned long t_prev_cycle = 0;  // (us) contains the time of the previous cycle at the start of each loop
        unsigned long dt = 0;  // (us) used to store time difference between t_prev_cycle and return of micros() at the start of each loop

        // sensor measurements
        float a_0[3] = {0.0, 0.0, 0.0};  // (m/s^2) linear acceleration, used for storing sensor measurements
        float a_1[3] = {0.0, 0.0, 0.0};  // (m/s^2) linear acceleration, used for storing sensor measurements
        float w_0[3] = {0.0, 0.0, 0.0};  // (rad/s) angular velocity, used for storing sensor measurements
        float w_1[3] = {0.0, 0.0, 0.0};  // (rad/s) angular velocity, used for storing sensor measurements

        // flight mode
        Mode mode = STARTUP_STABLE;

        // Servos
        Servo servo_top;  // servo in the drop mechanism
        Servo tvc_x;  // servo that actuates TVC around x body axis
        Servo tvc_y;  // servo that actuates TVC around x body axis

        // controller parameters
        // TODO: define parameters, move to mission constants
        MatrixXf K(2,6); //LQR -> updates mid flight
        VectorXi x(6); //state vector = {vx, theta_y, d_theta_y_dt, vy, theta_x, d_theta_x_dt} global frame and euler angles
        VectorXf u(2); //input vector
        MatrixXf A(36); //dynamics matrix
        MatrixXf B(12); //input matrix
        MatrixXf C(24); //Sensor matrix
        MatrixXf L(24); //Kalman gain
        float maxU = 5*3.14159/180;  //maximum gimbal angle
        long uLast[2] = {0, 0};  //commanded servo angle

        // state
        float r_body[3] = {0.0, 0.0, 0.0};  // (m) position of the body frame origin TODO: define inertial frame
        float v_body_wrt_inertial_in_inertial[3] = {0.0, 0.0, 0.0};  // (m/s) velocity of body frame origin wrt inertial frame, components resolved in inertial frame
        float qr_body_wrt_inrt[4] = {1.0, 0.0, 0.0, 0.0};  // (--) right, scalar-first, Hamiltonian quaternion transforming body frame into inertial frame
        float qr_inrt_wrt_body[4] = {1.0, 0.0, 0.0, 0.0};  // (--) right, scalar-first, Hamiltonian quaternion transforming inertial frame into body frame
};

#endif  // __WORKSPACE_HH__