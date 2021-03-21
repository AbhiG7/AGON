#include "I2Cdev.h"
#include "moding.hh"
//#include <SPIFlash.h>  // flash chip library
#include "BNO055_support.h"		//Contains the bridge code between the API and Arduino
#include <Wire.h>
#include "workspace.hh"  // variable storage
#include <Servo.h>
#include "matrix.hh"
#include "mission_constants.hh"
#include <cmath>

using namespace std;

// declare flash chip 
//#define CHIPSIZE MB64
//SPIFlash flash(1);

// (--) instance of workspace class storing all the variables used in the loop
Workspace ws;


/* tvc_abs
 *
 * TODO: add docstring
 */
void send_tvc(Matrix u, Matrix * last_u, double yaw)
{
    Matrix last_u_now= *last_u;
    //scale down angle to physical tvc limit
    float input_magnitude=powf((pow(u.select(1, 1), 2)+pow(u.select(2, 1), 2)), 0.5);
    if (input_magnitude>MAX_U)
    {
        u.scale(MAX_U/input_magnitude);
    }
    //send 
    float travel_magnitude=powf((pow(u.select(1, 1)-last_u_now.select(1, 1), 2)+pow(u.select(2, 1)-last_u_now.select(2, 1), 2)), 0.5);
    if (travel_magnitude>SERVO_SPEED*ws.dt)
    {
        u=last_u_now+(u-last_u_now).scale(SERVO_SPEED*ws.dt/travel_magnitude);
    }
    *last_u=u;

    //rotate input are body z axis
    float gamma=yaw+BETA;
    float rotation_values [4]={cos(gamma), sin(gamma), -sin(gamma), cos(gamma)};
    Matrix R=Matrix(2, 2, rotation_values);
    u=R*u;

    //convert to degrees, gear the angle, and round
    ws.tvc_y.write(round(GEAR*RAD_2_DEG*u.select(2, 1))+TVC_Y_OFFSET);
    ws.tvc_x.write(round(GEAR*RAD_2_DEG*u.select(1, 1))+TVC_X_OFFSET);
}


/* setup
 * 
 * This is called once to initialize everything.
 */
void setup()
{
    // set up pins (OUTPUT and LOW are defined in Arduino.h)


    //TODO flash setup
    //flash.begin(9600);  // begins flash chip at specified baud rate

    // set initial mission mode
    ws.mode = STARTUP_STABLE;

    // set up Thrust-Vector Controller (TVC)
    ws.tvc_x.attach(TVC_X_PIN);  // attaches declared servo to specified pin
    ws.tvc_y.attach(TVC_Y_PIN);  // attaches declared servo to specified pin

    //Initialize I2C communication
    Wire.begin();

    //Initialization of the BNO055
    BNO_Init(&ws.myBNO); //Assigning the structure to hold information about the device

    //Configuration to NDoF mode
    bno055_set_operation_mode(OPERATION_MODE_NDOF);

    delay(1);

    ws.next_mode_time=millis()+STARTUP_STABLE_PERIOD*KILO_I;
}


/* loop
 *
 * This runs, well, on a loop every processing cycle. It'll just keep
 * going until it reaches an internal terminating condition or the
 * board stops receiving power.
 */
void loop()
{
    // update the clock
    ws.dt = micros() - ws.t_prev_cycle;  // (us) time step since previous loop
    ws.t_prev_cycle += ws.dt;  // (us) update time for next loop

    bno055_read_euler_hrp(&ws.myEulerData);
    ws.yaw=float(ws.myEulerData.h)/16.00*DEG_2_RAD;
    ws.theta_0[0]=ws.myEulerData.r;
    ws.theta_1[0]=ws.myEulerData.r;
    ws.theta_0[1]=ws.myEulerData.p;
    ws.theta_1[1]=ws.myEulerData.p;

    // check for moding change conditions
    switch (ws.mode)
    {
        case (STARTUP_STABLE):
        {
            if (change_mode_to_countdown(millis()>ws.next_mode_time))
            {
                transition_to_countdown();
                ws.next_mode_time=millis()+COUNTDOWN*KILO_I;
                ws.mode = COUNTDOWN;
            }
            else
            {
                
            }
            break;
        }
        case (COUNTDOWN):
        {
            if (change_mode_to_final_countdown(millis()>ws.next_mode_time))
            {
                transition_to_final_countdown();
                ws.next_mode_time=millis()+FINAL_COUNTDOWN_PERIOD*KILO_I;
                ws.mode = FINAL_COUNTDOWN;
            }
            else
            {
                ws.construct_y();
                ws.x=ws.x+(L*(ws.y-(C*ws.x))).scale(ws.dt/MEGA); //state estimation only using sensors
            }
            break;
        }
        case(FINAL_COUNTDOWN):
        {
            if (change_mode_to_prep_tvc(millis()>ws.next_mode_time))
            {
                transition_to_prep_tvc();
                ws.next_mode_time=millis()+PREP_TVC_PERIOD*KILO_I;
                ws.mode = PREP_TVC;
            }
            else
            {
                ws.construct_y();
                ws.x=ws.x+(L*(ws.y-(C*ws.x))).scale(ws.dt/MEGA); //state estimation only using sensors
            }
            break;
        }
        case(PREP_TVC):
        {
            if (change_mode_to_burn_baby_burn(millis()>ws.next_mode_time))
            {
                transition_to_burn_baby_burn();
                ws.next_mode_time=millis()+BURN_BABY_BURN*KILO_I;
                ws.mode = BURN_BABY_BURN;
            }
            else
            {
                ws.construct_y();
                ws.u=(KC*ws.x).scale(-1);//calculate input
                send_tvc(ws.u, &ws.last_u, ws.yaw);
                ws.x=ws.x+(L*(ws.y-(C*ws.x))).scale(ws.dt/MEGA); //state estimation only using sensors
            }
            break;
        }
        case (BURN_BABY_BURN):
        {
            if (change_mode_to_shutdown_stable(millis()>ws.next_mode_time))
            {
                transition_to_shutdown_stable();
                ws.mode = SHUTDOWN_STABLE;
            }
            else
            {
                ws.construct_y();
                ws.u=(KC*ws.x).scale(-1); //calculate input
                send_tvc(ws.u, &ws.last_u, ws.yaw);
                ws.x=ws.x+(A*ws.x+B*ws.last_u+L*(ws.y-(C*ws.x))).scale(ws.dt/MEGA); //state estimation using Kalman filter
            }
            break;
        }
        case (SHUTDOWN_STABLE):
        {
            // TODO: implement this block
            break;
        }
        display_matrix(ws.y);
    }
    // TODO: add data record
    // TODO: add (somewhere else) data struct
}
