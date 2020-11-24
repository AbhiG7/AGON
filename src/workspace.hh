#ifndef __WORKSPACE_HH__
#define __WORKSPACE_HH__

#include <cstdint>
#include "mission_constants.hh"
#include "moding.hh"
#include "MPU6050.h"  // MPU 6050 IMU Library
#include <Servo.h>  // TODO: what's this?
#include "Wire.h"  // Arduino library


/* Workspace
 *
 * This class acts as a container for all of the variables which change
 * throughout the flight. This keeps things tidy by keeping them out of
 * the top of main.ino. An instance of this class is created at the top
 * of main.ino.
 * 
 * Constant variables should be placed in mission_constants.hh.
 */
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

        //event times
        unsigned long drop_time=0;
        unsigned long fire_time=0;

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

        //control vectors
        VectorXi x(6); //state vector = {vx, theta_y, d_theta_y_dt, vy, theta_x, d_theta_x_dt} global frame and euler angles
        VectorXf u(2); //input vector
        float maxU = 5*DEG_2_RAD;  //maximum gimbal angle
        int uLast[2] = {0, 0};  //commanded servo angle

        //control constants -> these are variable throughout the flight -> multiples of the templates in mission constants
        MatrixXf A(36); //dynamics matrix
        MatrixXf B(12); //input matrix
        MatrixXf L(36); //Kalman gain
        MatrixXf K(12); //LQR gain
        int thrust_curve_count=0; //current index of the thrust curve array

        // state
        float r_body[3] = {0.0, 0.0, 0.0};  // (m) position of the body frame origin TODO: define inertial frame
        float v_body_wrt_inertial_in_inertial[3] = {0.0, 0.0, 0.0};  // (m/s) velocity of body frame origin wrt inertial frame, components resolved in inertial frame
        float qr_body_wrt_inrt[4] = {1.0, 0.0, 0.0, 0.0};  // (--) right, scalar-first, Hamiltonian quaternion transforming body frame into inertial frame
        float qr_inrt_wrt_body[4] = {1.0, 0.0, 0.0, 0.0};  // (--) right, scalar-first, Hamiltonian quaternion transforming inertial frame into body frame
};

#endif  // __WORKSPACE_HH__