#include <cstdint>  // C STD Int library, might not be needed
// #include <Eigen30.h>  // another of version of Eigen313.h
#include "I2Cdev.h"
#include "mission_constants.hh"
#include "moding.hh"
#include <SH.h>  // TODO: what's this?
#include <SPIFlash.h>  // flash chip library
#include "Wire.h"  // Arduino library
#include "workspace.hh"  // variable storage
#include "matrix.hh"


using namespace Eigen;

// declare flash chip 
#define CHIPSIZE MB64
SPIFlash flash(1);

// (--) instance of workspace class storing all the variables used in the loop
Workspace ws;

/* print_measurements
 * 
 * Print out the latest IMU measurements, formatted nicely.
 */
void print_measurements(float a[3], float w[3])
{
    Serial.print('a (m/s^2): '); Serial.print(a[0]);
    Serial.print('\t'); Serial.print(a[1]); 
    Serial.print('\t'); Serial.println(a[2]);
    
    Serial.print('w (rad/s): '); Serial.print(w[0]);
    Serial.print('\t'); Serial.print(w[1]); 
    Serial.print('\t'); Serial.println(w[2]);
}


/* calibrate_imu_linear_acceleration
 * 
 * Checks a sample of IMU measurements over a few seconds while the rocket
 * is ASSUMED to be still. Averages the sum of those measurements to get a
 * linear acceleration bias along each axis. This bias is stored and assumed
 * to be constant throughout the flight. Bias is used in get_latest_measurements
 * when converting sensor readings to engineering values.
 */
void calibrate_imu_linear_acceleration(int16_t *imu_lin_acc_bias[3], int16_t *imu_ang_vel_bias[3])
{
    int num_loops = 2500;  // number of cycles to check measurements for bias
    long a_sums[3] = {0, 0, 0};  // sum of linear acceleration measurements each loop to be averaged
    long w_sums[3] = {0, 0, 0};  // sum of angular velocity measurements each loop to be averaged
    
    int16_t ax, ay, az, wx, wy, wz;  // temp loop vars

    for (int i = 0; i < num_loops; i++)
    {
        imu.getMotion6(&ax, &ay, &az, &wx, &wy, &wz); //we want to label the vertical axis as z. the MPUs are orientated with y as vertical
        a_sums[0] += ax;
        a_sums[1] += ay;
        a_sums[2] += az;

        w_sums[0] += wx;
        w_sums[1] += wy;
        w_sums[2] += wz;
    }

    // find average bias
    a_sums[0] /= num_loops;
    a_sums[1] /= num_loops;
    a_sums[2] /= num_loops;

    w_sums[0] /= num_loops;
    w_sums[1] /= num_loops;
    w_sums[2] /= num_loops;

    // store biases for later use
    *imu_lin_acc_bias[0] = a_sums[0];
    *imu_lin_acc_bias[1] = a_sums[1];
    *imu_lin_acc_bias[2] = a_sums[2];

    *imu_ang_vel_bias[0] = w_sums[0];
    *imu_ang_vel_bias[1] = w_sums[1];
    *imu_ang_vel_bias[2] = w_sums[2];
}

/* get_latest_measurements
 * 
 * Wrapper around MPU6050::getMotion6 that converts the sensor counts into engineering values.
 * 
 * Inputs:
 *   - MPU6050 imu: instance of an MPU 6050 IMU connection to read measurements from
 *   - float *a[3]: address of 3-array to store converted linear acceleration measurements
 *   - float *w[3]: address of 3-array to store converted angular velocity measurements
 * 
 *  Modifies:
 *   - Sets values of passed arrays based on converted sensor measurement values
 *       - Linear Acceleration conversion: $a_i = a_{m,i}\cdot\text{LSB}_{\text{linear}}\cdot1000\frac{\text{milli-gee}}{\text{gee}}\cdot9.80665\frac{\text{m/s}^2}{gee}$
 *       - Angular Velocity conversion: $\omega_i = \omega_{m,i}\cdot\text{LSB}_{\text{angular}}\cdot0.0174...\frac{\text{rad}}{\text{deg}}$
 *     where $a_{m,i}$ denotes the linear acceleration measurement in direction $i$
 *     and $\omega_{m,i}$ denotes the angular velocity measurement about axis $i$.
 * 
 * Outputs:
 *   - return 0
 */ 
void get_latest_measurements(MPU6050 imu, float *a[3], float *w[3], bool debug=false)
{
    // READ DATA
    int16_t ax, ay, az, wx, wy, wz;  // temp vars to store raw measurements
    imu.getMotion6(&ax, &ay, &az, &wx, &wy, &wz);  // stores measurements in temp vars

    // CONVERT AND STORE DATA
    *a[0] = (ax - IMU_LIN_ACC_BIAS[0])*LSB_LINEAR*1000*G_2_MS2;  // (m/s^2) x-axis acceleration
    *a[1] = (ay - IMU_LIN_ACC_BIAS[1])*LSB_LINEAR*1000*G_2_MS2;  // (m/s^2) y-axis acceleration
    *a[2] = (az - IMU_LIN_ACC_BIAS[2])*LSB_LINEAR*1000*G_2_MS2;  // (m/s^2) z-axis acceleration

    *w[0] = (wx - IMU_ANG_VEL_BIAS)*LSB_ANGULAR*DEG_2_RAD;  // (rad/s) x-axis angular velocity
    *w[1] = (wy - IMU_ANG_VEL_BIAS)*LSB_ANGULAR*DEG_2_RAD;  // (rad/s) y-axis angular velocity
    *w[2] = (wz - IMU_ANG_VEL_BIAS)*LSB_ANGULAR*DEG_2_RAD;  // (rad/s) z-axis angular velocity

    if (debug) {
        print_measurements(a, w);
    }
}


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
    pinMode(B_LED_1, OUTPUT);
    pinMode(G_LED_1, OUTPUT);
    pinMode(B_LED_2, OUTPUT);
    pinMode(G_LED_2, OUTPUT);

    digitalWrite(B_LED_1, LOW);
    digitalWrite(G_LED_1, LOW);
    digitalWrite(B_LED_2, LOW);
    digitalWrite(G_LED_2, LOW);

    // join I2C bus (I2Cdev library doesn't do this automatically)
    // TODO: do these need to be #if's, i.e. could they be regular C++ if's?
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    flash.begin(9600);  // begins flash chip at specified baud rate
    if (eraseFlash)
        flash.eraseChip();  // erases flash chip

    // set initial mission mode
    ws.mode = STARTUP_STABLE;

    // set up Thrust-Vector Controller (TVC)
    ws.tvc_x.attach(TVC_X_PIN);  // attaches declared servo to specified pin
    ws.tvc_y.attach(TVC_Y_PIN);  // attaches declared servo to specified pin
    ws.tvc_top.attach(DROP_PIN);  // attaches declared servo to specified pin

    ws.tvc_top.write(drop_mechanism_hold); delay(1000);  // TODO: what is the "10" doing and can it be a mission constant?
    
    // actuate servo along x and then along y axis at startup
    tvc_abs(  0,   0, BETA, TVC_DELAY*100);
    tvc_abs( 30,   0, BETA, TVC_DELAY*100);
    tvc_abs(-30,   0, BETA, TVC_DELAY*100);
    tvc_abs(  0,   0, BETA, TVC_DELAY*100);
    tvc_abs(  0,  30, BETA, TVC_DELAY*100);
    tvc_abs(  0, -30, BETA, TVC_DELAY*100);
    tvc_abs(  0,   0, BETA, TVC_DELAY*100);

    // set up IMUs
    ws.imu_0.initialize();
    ws.imu_0.setFullScaleAccelRange(MPU6050_ACCEL_FS_8);  // WARNING: changing this will require changing LSB_LINEAR mission constant
    ws.imu_0.setFullScaleGyroRange(MPU6050_GYRO_FS_250);  // WARNING: changing this will require changing LSB_ANGULAR mission constant
    ws.imu_1.initialize();
    ws.imu_1.setFullScaleAccelRange(MPU6050_ACCEL_FS_8);  // WARNING: changing this will require changing LSB_LINEAR mission constant
    ws.imu_1.setFullScaleGyroRange(MPU6050_GYRO_FS_250);  // WARNING: changing this will require changing LSB_ANGULAR mission constant

    // calibrate the IMU linear acceleration sensors
    calibrate_imu_linear_acceleration(*IMU_LIN_ACC_BIAS, *IMU_ANG_VEL_BIAS);

    // TODO: finish pulling over the rest of AGON1a setup code below this point

    // TODO: (and not necessarily here) organize blink patterns by mode

    // set up pyro
    pinMode(MOTOR_PIN, OUTPUT);
    digitialWrite(MOTOR_PIN, LOW);

    // set up controller
    // TODO : move stuff below
    float * initialize={0, 0, 0, 0, 0, 0};
    ws.x=Matrix(6, 1, initialize);
    ws.y=Matrix(4, 1, initialize);
    ws.u=Matrix(2, 1, initialize);

    ws.uLast[0] = 0;  //last commanded tvc x input
    ws.uLast[1] = 0; //last commanded tvc y input

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
            if (change_mode_to_navigation(mode, false))
            {
                transition_to_navigation();
                ws.mode = NAVIGATION;
            }
            else
            {
                // TODO: implement this block
            }
            break;
        }
        case (NAVIGATION):
        {
            if (change_mode_to_burn_baby_burn(mode, false))
            {
                transition_to_burn_baby_burn();
                ws.mode = BURN_BABY_BURN;
            }
            else
            {
                //construct x
                //construct y
                ws.x+=ws.dt*(ws.A*ws.x+ws.B*ws.u+ws.L*(ws.y-ws.C*ws.x)) //calculate next state
            }
            break;
        }
        case (BURN_BABY_BURN):
        {
            if (change_mode_to_shutdown_stable(mode, false))
            {
                transition_to_shutdown_stable();
                ws.mode = SHUTDOWN_STABLE;
            }
            else
            {
                //checks whether the burn time has surpassed the pre-defined burn interval
                //if so, then updates the control constant by multiplying by the current thrust
                //construct x
                //construct y
                ws.u=sMult(mMult(), -1); //calculate input
                ws.x=mAdd(ws.x, sMult(mAdd(mAdd(), ), ws.dt));
                ws.x+=ws.dt*(ws.A*ws.x+ws.B*ws.u+ws.L*(ws.y-ws.C*ws.x)); //calculate next state
                //process u
                //send u to tvc
            }
            break;
        }
        case (SHUTDOWN_STABLE):
        {
            // TODO: implement this block
            break;
        }
    }
    // TODO: add blink
    // TODO: add data record
    // TODO: add (somewhere else) data struct
}