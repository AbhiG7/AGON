#include "I2Cdev.h"
#include "moding.hh"
#include <SPIFlash.h>  // flash chip library
#include "BNO055_support.h"		//Contains the bridge code between the API and Arduino
#include <Wire.h>
#include "workspace.hh"  // variable storage
#include <Servo.h>
#include "matrix.hh"
#include "mission_constants.hh"
#include <cmath>
#include <vector>
using namespace std;

// declare flash chip 
//#define CHIPSIZE MB64
//SPIFlash flash(1);

// (--) instance of workspace class storing all the variables used in the loop
Workspace ws;

void main_display_matrix(Matrix param, float scale)
{
  for (int i=0; i<param.rows*param.columns; i++)
  {
     Serial.print(param.values[i]*scale, 4);
     Serial.print("    ");
  }
  Serial.println(" ");
}

/* tvc_abs
 *
 * TODO: add docstring
 */
Matrix send_tvc(Matrix u, Matrix last_u, float yaw)
{
    //scale down angle to physical tvc limit
    float input_magnitude=powf((powf(u.select(1, 1), 2)+powf(u.select(2, 1), 2)), 0.5);
    if (input_magnitude>MAX_U)
    {
        u.scale(MAX_U/input_magnitude);
    }
    main_display_matrix(u, RAD_2_DEG);
    //send 
    float travel_magnitude=powf((powf(u.select(1, 1)-last_u.select(1, 1), 2)+powf(u.select(2, 1)-last_u.select(2, 1), 2)), 0.5);
    if (travel_magnitude>SERVO_SPEED*ws.dt/MEGA)
    {
        u=last_u+(u-last_u).scale(SERVO_SPEED*ws.dt/MEGA/travel_magnitude);
    }
    main_display_matrix(u, RAD_2_DEG);

    //rotate input are body z axis
    float gamma=yaw+BETA;
    vector<float> rotation_values {cos(gamma), -sin(gamma), sin(gamma), cos(gamma)};
    Matrix R=Matrix(2, 2, rotation_values);
    Matrix new_u=R*u;
    main_display_matrix(u, RAD_2_DEG);
    Serial.println(" ");
    
    //convert to degrees, gear the angle, and round
    if (!NULL_U_X_AXIS)  {ws.tvc_x.write(round(GEAR*RAD_2_DEG*new_u.select(1, 1))+TVC_X_OFFSET);}
    if (!NULL_U_Y_AXIS)  {ws.tvc_y.write(round(GEAR*RAD_2_DEG*new_u.select(2, 1))+TVC_Y_OFFSET);}
    

    return u;
}


/* setup
 * 
 * This is called once to initialize everything.
 */
void setup()
{
    // set up pins (OUTPUT and LOW are defined in Arduino.h)
    pinMode(15, INPUT);
    digitalWrite(15, HIGH);

    pinMode(0, INPUT);
    digitalWrite(0, LOW);
    
    //TODO flash setup
    //flash.begin(9600);  // begins flash chip at specified baud rate

    // set initial mission mode
    ws.mode = STARTUP_STABLE;

    // set up Thrust-Vector Controller (TVC)
    ws.tvc_x.attach(TVC_X_PIN);  // attaches declared servo to specified pin
    ws.tvc_y.attach(TVC_Y_PIN);  // attaches declared servo to specified pin

    ws.tvc_x.write(TVC_X_OFFSET);
    ws.tvc_y.write(TVC_Y_OFFSET);

    //Initialize I2C communication
    Wire.begin();

    //Initialization of the BNO055
    BNO_Init(&ws.bno_1); //Assigning the structure to hold information about the device
    BNO_Init(&ws.bno_2); //Assigning the structure to hold information about the device

    bno055_set_axis_remap_value(0X06);
    delay(1);
    bno055_set_z_remap_sign(0x01);
    delay(1);
    bno055_set_z_remap_sign(0x01);
    delay(1);


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
    ws.dt = float(micros() - ws.t_prev_cycle);  // (us) time step since previous loop
    ws.t_prev_cycle += ws.dt;  // (us) update time for next loop

    bno055_read_euler_hrp(&ws.euler_1);
    bno055_read_euler_hrp(&ws.euler_2);

    // check for moding change conditions
    switch (ws.mode)
    {
        case (STARTUP_STABLE):
        {
            if (change_mode_to_countdown(millis()>ws.next_mode_time))
            {
                transition_to_countdown();
                ws.next_mode_time=millis()+COUNTDOWN_PERIOD*KILO_I;
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
            if (change_mode_to_prep_tvc(millis() > ws.next_mode_time))
            {
                digitalWrite(15, HIGH);
                delay(1);
                digitalWrite(15, HIGH);
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
                ws.next_mode_time=millis()+BURN_BABY_BURN_PERIOD*KILO_I;
                ws.mode = BURN_BABY_BURN;
            }
            else
            {
                ws.construct_y();
                ws.u=(KC*ws.x).scale(-1);//calculate input
                ws.last_u=send_tvc(ws.u, ws.last_u, ws.yaw);
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
                ws.last_u=send_tvc(ws.u, ws.last_u, ws.yaw);
                ws.x=ws.x+(A*ws.x+B*ws.last_u+L*(ws.y-(C*ws.x))).scale(ws.dt/MEGA); //state estimation using Kalman filter
            }
            break;
        }
        case (SHUTDOWN_STABLE):
        {
            // TODO: implement this block
            break;
        }
        
    }
    Serial.println(ws.mode);
    
    //Serial.print(millis());
    //main_display_matrix(ws.last_u, RAD_2_DEG);
    // TODO: add data record
    // TODO: add (somewhere else) data struct
}
