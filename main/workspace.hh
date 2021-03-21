#ifndef __WORKSPACE_HH__
#define __WORKSPACE_HH__

#include <cstdint>
#include "mission_constants.hh"
#include "moding.hh"
#include "matrix.hh"
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
        struct bno055_t myBNO;
        struct bno055_euler myEulerData;

        //*****************************************************************************
        //                              LOOP VARIABLES
        //*****************************************************************************
        // clock time
        unsigned long calibrate_time = 0;  //momment main loop starts
        unsigned long t_prev_cycle = 0;  // (us) contains the time of the previous cycle at the start of each loop
        unsigned long dt = 0;  // (us) used to store time difference between t_prev_cycle and return of micros() at the start of each loop

        // sensor measurements
        float a_0[3] = {0.0, 0.0, 0.0};  // (m/s^2) linear acceleration, used for storing sensor measurements
        float a_1[3] = {0.0, 0.0, 0.0};  // (m/s^2) linear acceleration, used for storing sensor measurements
        float theta_0[3] = {0.0, 0.0, 0.0};  // (rad/s) angular velocity, used for storing sensor measurements
        float theta_1[3] = {0.0, 0.0, 0.0};  // (rad/s) angular velocity, used for storing sensor measurements

        // flight mode
        Mode mode = STARTUP_STABLE;
        int next_mode_time=0;

        // Servos
        Servo tvc_x;  // servo that actuates TVC around x body axis
        Servo tvc_y;  // servo that actuates TVC around x body axis

        //control vectors
        // set up controller
        float initialize_6 [6]={0, 0, 0, 0, 0, 0};
        float initialize_4 [4]={0, 0, 0, 0};
        float initialize_2 [2]={0, 0};
        Matrix x=Matrix(6, 1, initialize_6);
        Matrix y=Matrix(4, 1, initialize_4);
        Matrix u=Matrix(2, 1, initialize_2);
        Matrix last_u=Matrix(2, 1, initialize_2);
        float y_values [4]= {0, 0, 0, 0};
        float yaw;        

        // state
        float r_body[3] = {0.0, 0.0, 0.0};  // (m) position of the body frame origin TODO: define inertial frame
        float v_body_wrt_inertial_in_inertial[3] = {0.0, 0.0, 0.0};  // (m/s) velocity of body frame origin wrt inertial frame, components resolved in inertial frame
        float qr_body_wrt_inrt[4] = {1.0, 0.0, 0.0, 0.0};  // (--) right, scalar-first, Hamiltonian quaternion transforming body frame into inertial frame
        float qr_inrt_wrt_body[4] = {1.0, 0.0, 0.0, 0.0};  // (--) right, scalar-first, Hamiltonian quaternion transforming inertial frame into body frame

        void construct_y()
        {
            y_values[0]=0;
            y_values[1]=(theta_0[1]+theta_1[1])/2;
            y_values[2]= 0;
            y_values[3]= (theta_0[0]+theta_1[0])/2;
            y.redefine(y_values);
        }
};

#endif  // __WORKSPACE_HH__
