#include <cstdint>  // C STD Int library, might not be needed
#include "I2Cdev.h"
#include "moding.hh"
#include <SH.h>  // TODO: what's this?
#include <SPIFlash.h>  // flash chip library
#include "Wire.h"  // Arduino library
#include "workspace.hh"  // variable storage
#include <Servo.h>
#include "matrix.hh"
#include "mission_constants.hh"

// declare flash chip 
#define CHIPSIZE MB64
SPIFlash flash(1);

// (--) instance of workspace class storing all the variables used in the loop
Workspace ws;


/* tvc_abs
 *
 * TODO: add docstring
 */
void send_tvc_signal(int x, int y, double gamma, int d)
{
    // cos and sin are native to arduino
    double u = x*cos(gamma) + y*sin(gamma);
    double v = y*cos(gamma) - x*sin(gamma);
    ws.tvc_y.write(v + TVC_Y_OFFSET);
    ws.tvc_x.write(u + TVC_X_OFFSET);
}


/* setup
 * 
 * This is called once to initialize everything.
 */
void setup()
{
    // set up pins (OUTPUT and LOW are defined in Arduino.h)

    //TODO do we still need this????
    // join I2C bus (I2Cdev library doesn't do this automatically)
    // TODO: do these need to be #if's, i.e. could they be regular C++ if's?
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    //TODO flash setup
    flash.begin(9600);  // begins flash chip at specified baud rate

    // set initial mission mode
    ws.mode = STARTUP_STABLE;

    // set up Thrust-Vector Controller (TVC)
    ws.tvc_x.attach(TVC_X_PIN);  // attaches declared servo to specified pin
    ws.tvc_y.attach(TVC_Y_PIN);  // attaches declared servo to specified pin

    // actuate servo along x and then along y axis at startup
    tvc_abs(  0,   0, BETA, TVC_DELAY*100);
    tvc_abs( 30,   0, BETA, TVC_DELAY*100);
    tvc_abs(-30,   0, BETA, TVC_DELAY*100);
    tvc_abs(  0,   0, BETA, TVC_DELAY*100);
    tvc_abs(  0,  30, BETA, TVC_DELAY*100);
    tvc_abs(  0, -30, BETA, TVC_DELAY*100);
    tvc_abs(  0,   0, BETA, TVC_DELAY*100);

    // set up IMUs

    // calibrate the IMU linear acceleration sensors
    calibrate_imu_linear_acceleration(*IMU_LIN_ACC_BIAS, *IMU_ANG_VEL_BIAS);

    // set up controller
    float initialize_6 [6]={0, 0, 0, 0, 0, 0};
    float initialize_4 [4]={0, 0, 0, 0};
    float initialize_2 [2]={0, 0};
    ws.x=Matrix(6, 1, initialize_6);
    ws.y=Matrix(4, 1, initialize_4);
    ws.u=Matrix(2, 1, initialize_2);
    ws.last_u=Matrix(2, 1, initialize_2);

    ws.calibrate_time = micros();
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

    // check for moding change conditions
    switch (ws.mode)
    {
        case (STARTUP_STABLE):
        {
            if (change_mode_to_navigation(ws.mode, false))
            {
                transition_to_navigation();
                ws.mode = NAVIGATION;
            }
            else
            {
                
            }
            break;
        }
        case (NAVIGATION):
        {
            if (change_mode_to_countdown(ws.mode, false))
            {
                transition_to_countdown();
                ws.mode = COUNTDOWN;
            }
            else
            {
                //TODO: construct y
                ws.x=ws.x+(L*(ws.y-(C*ws.x))).scale(ws.dt) //state estimation only using sensors
            }
            break;
        }
        case (COUNTDOWN)
        {
            if (change_mode_to_final_countdown(ws.mode, false))
            {
                transition_to_final_countdown();
                ws.mode = FINAL_COUNTDOWN;
            }
            else
            {
                //TODO: construct y
                ws.x=ws.x+(L*(ws.y-(C*ws.x))).scale(ws.dt) //state estimation only using sensors
            }
            break;
        }
        case(FINAL_COUNTDOWN)
        {
            if (change_mode_to_prep_tvc(ws.mode, false))
            {
                transition_to_prep_tvc();
                ws.mode = PREP_TVC;
            }
            else
            {
                //TODO: construct y
                ws.x=ws.x+(L*(ws.y-(C*ws.x))).scale(ws.dt) //state estimation only using sensors
            }
            break;
        }
        case(PREP_TVC)
        {
            if (change_mode_to_burn_baby_burn(ws.mode, false))
            {
                transition_to_burn_baby_burn();
                ws.mode = BURN_BABY_BURN;
            }
            else
            {
                //TODO: construct y
                ws.u=(K*ws.x).scale(-1);//calculate input
                //send signal to tvc
                ws.x=ws.x+(L*(ws.y-(C*ws.x))).scale(ws.dt) //state estimation only using sensors
            }
            break;
        }
        case (BURN_BABY_BURN):
        {
            if (change_mode_to_shutdown_stable(ws.mode, false))
            {
                transition_to_shutdown_stable();
                ws.mode = SHUTDOWN_STABLE;
            }
            else
            {
                //TODO: construct y
                ws.u=(K*ws.x).scale(-1); //calculate input
                ws.x=ws.x+(A*ws.x+B*ws.last_u+L*(ws.y-(C*ws.x))).scale(ws.dt) //state estimation using Kalman filter
            }
            break;
        }
        case (SHUTDOWN_STABLE):
        {
            // TODO: implement this block
            break;
        }
    }
    // TODO: add data record
    // TODO: add (somewhere else) data struct
}