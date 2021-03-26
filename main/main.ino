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
Matrix send_tvc(Matrix u, Matrix last_u, float yaw, Servo x, Servo y)
{
    //scale down angle to physical tvc limit
    float input_magnitude=sqrtf(u.values[0]*u.values[0]+u.values[1]*u.values[1]);
    if (input_magnitude>MAX_U)
    {
        u.scale(MAX_U/input_magnitude);
    }

    if(NULL_U_X_AXIS){ u.values[0]=0; }
    if(NULL_U_Y_AXIS){ u.values[1]=0; }

    vector<float> rotation_values {cos(yaw+BETA), -sin(yaw+BETA), sin(yaw+BETA), cos(yaw+BETA)};
    Matrix R=Matrix(2, 2, rotation_values);
    u=R*u;
    u.values[0]=GEAR*RAD_2_DEG*u.values[0]+TVC_X_OFFSET;
    u.values[1]=GEAR*RAD_2_DEG*u.values[1]+TVC_Y_OFFSET;
    float delta=u.values[0]-last_u.values[0];
    if (abs(delta)>MAX_U_DELTA)
    {
      u.values[0]=last_u.values[0]+delta/abs(delta)*MAX_U_DELTA;
    }
    delta=u.values[1]-last_u.values[1];
    if (abs(delta)>MAX_U_DELTA)
    {
      u.values[1]=last_u.values[1]+delta/abs(delta)*MAX_U_DELTA;
    }
    Serial.print(u.values[0]);
    Serial.print("    ");
    Serial.println(u.values[1]);

    //convert to degrees, gear the angle, and round
    if (TVC_ENABLE)
    {
      
      x.write(round(u.values[0]));
      y.write(round(u.values[1]));
    }
    delay(1);

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

    pinMode(G_LED_PIN, OUTPUT);
    pinMode(R_LED_PIN, OUTPUT);
    pinMode(B_LED_PIN, OUTPUT);

    // LED - white, setup
    digitalWrite(R_LED_PIN, LOW);
    digitalWrite(G_LED_PIN, LOW);
    digitalWrite(B_LED_PIN, LOW);

    
    //TODO flash setup
    //flash.begin(9600);  // begins flash chip at specified baud rate

    // set initial mission mode
    ws.mode = STARTUP_STABLE;

    // set up Thrust-Vector Controller (TVC)
    if (TVC_ENABLE || TVC_TEST)
    {
      ws.tvc_x.attach(TVC_X_PIN);  // attaches declared servo to specified pin
      ws.tvc_y.attach(TVC_Y_PIN);  // attaches declared servo to specified pin
      ws.tvc_x.write(TVC_X_OFFSET);
      ws.tvc_y.write(TVC_Y_OFFSET);
    }

    //Initialize I2C communication
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    delay(1000);

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

    if(TVC_TEST)
    {
      Matrix test_u=Matrix(2, 1, ws.initialize_2);
      vector<float> test_us {MAX_U,-MAX_U,MAX_U, -MAX_U, 0};
      
      for (int i=0; i<10; i++)
      {
        if (i<5)
        {
          test_u.values[0]=test_us[i];
          test_u.values[1]=0;
        }
        else
        {
          test_u.values[1]=test_us[i-5];
          test_u.values[0]=0;
        }
        
        vector<float> test_rotation {cos(BETA), -sin(BETA), sin(BETA), cos(BETA)};
        Matrix test_R=Matrix(2, 2, test_rotation);
        test_u=test_R*test_u;
        ws.tvc_x.write(round(GEAR*RAD_2_DEG*test_u.values[0])+TVC_X_OFFSET);
        ws.tvc_y.write(round(GEAR*RAD_2_DEG*test_u.values[1])+TVC_Y_OFFSET);
        delay(1000);
      }
    }

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
    unsigned long current_time=micros();
    ws.dt = float((current_time - ws.t_prev_cycle)/MEGA);  // (us) time step since previous loop
    ws.t_prev_cycle = current_time;  // (us) update time for next loop

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

                // LED - magenta
                digitalWrite(R_LED_PIN, LOW);
                digitalWrite(G_LED_PIN, HIGH);
                digitalWrite(B_LED_PIN, LOW);

                ws.next_mode_time=millis()+COUNTDOWN_PERIOD*KILO_I;
                ws.mode = COUNTDOWN;
            }
            else
            {
                ws.construct_y();
                ws.construct_x(false);
            }
            break;
        }
        case (COUNTDOWN):
        {          
            if (change_mode_to_final_countdown(millis()>ws.next_mode_time))
            {
                transition_to_final_countdown();
                // LED - yellow
                digitalWrite(R_LED_PIN, LOW); // Turn the LED on
                digitalWrite(G_LED_PIN, LOW); // Turn the LED on
                digitalWrite(B_LED_PIN, HIGH); // Turn the LED off

                ws.next_mode_time=millis()+FINAL_COUNTDOWN_PERIOD*KILO_I;
                ws.mode = FINAL_COUNTDOWN;
            }
            else
            {
                ws.construct_y();
                ws.construct_x(false);
            }
            break;
        }
        case(FINAL_COUNTDOWN):
        {
            if (change_mode_to_prep_tvc(millis() > ws.next_mode_time))
            {
                transition_to_prep_tvc();
                // LED - red
                digitalWrite(R_LED_PIN, LOW);
                digitalWrite(G_LED_PIN, HIGH);
                digitalWrite(B_LED_PIN, HIGH);
                ws.next_mode_time=millis()+PREP_TVC_PERIOD*KILO_I;
                ws.mode = PREP_TVC;
            }
            else
            {
                ws.construct_y();
                ws.construct_x(false);
            }
            break;
        }
        case(PREP_TVC):
        {
            if (change_mode_to_burn_baby_burn(millis()>ws.next_mode_time))
            {
                transition_to_burn_baby_burn();
                // LED - green
                digitalWrite(R_LED_PIN, HIGH);
                digitalWrite(G_LED_PIN, LOW);
                digitalWrite(B_LED_PIN, HIGH);
                ws.next_mode_time=millis()+BURN_BABY_BURN_PERIOD*KILO_I;
                ws.mode = BURN_BABY_BURN;
                
            }
            else
            {
                ws.construct_y();
                ws.u=(KC*ws.x).scale(-1);//calculate input
                ws.last_u=send_tvc(ws.u, ws.last_u, ws.yaw, ws.tvc_x, ws.tvc_y);
                ws.construct_x(false);
            }
            break;
        }
        case (BURN_BABY_BURN):
        {
            if (change_mode_to_shutdown_stable(millis()>ws.next_mode_time))
            {
                transition_to_shutdown_stable();

                //sets motor off - blue
                digitalWrite(R_LED_PIN, HIGH);
                digitalWrite(G_LED_PIN, HIGH);
                digitalWrite(B_LED_PIN, LOW);

                // Hi Chris :)

                // LED - cyan
                digitalWrite(R_LED_PIN, HIGH);
                digitalWrite(G_LED_PIN, LOW);
                digitalWrite(B_LED_PIN, LOW);
                ws.mode = SHUTDOWN_STABLE;
            }
            else
            {
                ws.construct_y();
                ws.u=(KC*ws.x).scale(-1); //calculate input
                ws.last_u=send_tvc(ws.u, ws.last_u, ws.yaw, ws.tvc_x, ws.tvc_y);
                ws.construct_x(KALMAN_ENABLED);
            }
            break;
        }
        case (SHUTDOWN_STABLE):
        {
            ws.construct_y();
            ws.construct_x(false);
            break;
        }
        
    }

    Serial.println(ws.mode);
    main_display_matrix(ws.x, 1);
    //Serial.print(millis());
    //main_display_matrix(ws.last_u, RAD_2_DEG);
    // TODO: add data record
    // TODO: add (somewhere else) data struct
}
