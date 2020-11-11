#include <cstdint>  // C STD Int library, might not be needed
// #include <Eigen30.h>  // TODO: what's this and why is it commented-out?
#include <Eigen313.h>  // TODO: what's this?
#include "I2Cdev.h"
#include <LU>  // TODO: what's this?
#include "mission_constants.hh"
#include "moding.hh"
#include "MPU6050.h"  // MPU 6050 IMU Library
#include <SH.h>  // TODO: what's this?
#include <SPIFlash.h>  // TODO: what's this?
#include "Wire.h"  // Arduino library
#include "workspace.hh"  // variable storage


using namespace Eigen;

// TODO: what do these do?
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
void calibrate_imu_linear_acceleration(int16_t *imu_acc_bias[3])
{
    int num_loops = 2500;  // number of cycles to check measurements for bias
    long a_sums[3] = {0, 0, 0};  // sum of measurements each loop to be averaged
    int16_t ax, ay, az, wx, wy, wz;  // temp loop vars

    for (int i = 0; i < num_loops; i++)
    {
        imu.getMotion6(&ax, &ay, &az, &wx, &wy, &wz);
        a_sums[0] += ax;
        a_sums[1] += ay;
        a_sums[2] += az;
        
        // TODO: do we need to zero the values of ax, ay, and az every loop?
        // TODO: why aren't we also calculating an agular velocity bias?
    }

    // find average bias
    a_sums[0] /= num_loops;
    a_sums[1] /= num_loops;
    a_sums[2] /= num_loops;

    // store biases for later use
    *imu_acc_bias[0] = a_sums[0];
    *imu_acc_bias[1] = a_sums[1];  // TODO: why does AGON1a have a_sums[1] - imuRange/2?
    *imu_acc_bias[2] = a_sums[2];
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
    *a[0] = (ax - IMU_ACC_BIAS[0])*LSB_LINEAR*1000*G_2_MS2;  // (m/s^2) x-axis acceleration
    *a[1] = (ay - IMU_ACC_BIAS[1])*LSB_LINEAR*1000*G_2_MS2;  // (m/s^2) y-axis acceleration
    *a[2] = (az - IMU_ACC_BIAS[2])*LSB_LINEAR*1000*G_2_MS2;  // (m/s^2) z-axis acceleration
    *w[0] = wx*LSB_ANGULAR*DEG_2_RAD;  // (rad/s) x-axis angular velocity
    *w[1] = wy*LSB_ANGULAR*DEG_2_RAD;  // (rad/s) y-axis angular velocity
    *w[2] = wz*LSB_ANGULAR*DEG_2_RAD;  // (rad/s) z-axis angular velocity

    if (debug) {
        print_measurements(a, w);
    }
}


/* tvc_abs
 *
 * TODO: add docstring
 */
void tvc_abs(int x, int y, double gamma, int d)
{
    // TODO: why are x and y in parentheses?
    // TODO: where are cos and sin defined?
    double u = (x)*cos(gamma) + (y)*sin(gamma);
    double v = (y)*cos(gamma) - (x)*sin(gamma);
    ws.tvc_y.write(v + TVC_Y_OFFSET);
    ws.tvc_x.write(u + TVC_X_OFFSET);
    delay(d);  // TODO: why is this delay here?
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

    flash.begin(9600);  // TODO: what does this do?
    // flash.eraseChip();  // TODO: what does this do and why is it commented-out?

    // set initial mission mode
    ws.mode = STARTUP_STABLE;

    // set up Thrust-Vector Controller (TVC)
    ws.tvc_x.attach(TVC_X_PIN);  // TODO: add description
    ws.tvc_y.attach(TVC_Y_PIN);  // TODO: add description
    ws.tvc_top.attach(TVC_TOP_PIN);  // TODO: add description

    ws.tvc_top.write(10); delay(1000);  // TODO: what is the "10" doing and can it be a mission constant?
    
    // TODO: what's going on in this block?
    tvc_abs(  0,   0, BETA, TVC_DELAY*100);
    tvc_abs( 30,   0, BETA, TVC_DELAY*100);
    tvc_abs(-30,   0, BETA, TVC_DELAY*100);
    tvc_abs(  0,   0, BETA, TVC_DELAY*100);
    tvc_abs(  0,  30, BETA, TVC_DELAY*100);
    tvc_abs(  0, -30, BETA, TVC_DELAY*100);
    tvc_abs(  0,   0, BETA, TVC_DELAY*100);

    // set up IMU
    ws.imu.initialize();
    ws.imu.setFullScaleAccelRange(MPU6050_ACCEL_FS_8);  // WARNING: changing this will require changing LSB_LINEAR mission constant
    ws.imu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);  // WARNING: changing this will require changing LSB_ANGULAR mission constant
    ws.imu.CalibrateAccel(30);  // TODO: What is "30"? Can this be made a mission constant?
    ws.imu.CalibrateGyro(30);  // TODO: What is "30"? Can this be made a mission constant?

    // calibrate the IMU linear acceleration sensors
    calibrate_imu_linear_acceleration(*IMU_ACC_BIAS);

    // TODO: finish pulling over the rest of AGON1a setup code below this point

    // blink for good luck
    for (int num_blink_loops = 0; num_blink_loops < 4; num_blink_loops++)
    {
        // TODO: what's going on here?
        ws.blink_master[i] = BLINK_0[i];
        ws.blink_master[i+9] = BLINK_1[i];
        ws.blink_master[i+18] = BLINK_2[i];
        ws.blink_master[i+27] = BLINK_3[i];
    }

    // set up pyro
    pinMode(PYRO_PIN, OUTPUT);
    digitialWrite(PYRO_PIN, LOW);

    // set up controller
    // TODO: where are these values from and can they be mission constants?
    ws.x << 0, 0, 0, 0, 0, 0;
    ws.xControl << 0, 0, 0, 0, 0, 0;
    ws.Ka << 0.34641, 1.72254, 0.32694, 0, 0, 0, 0, 0, 0, 0.34641, -1.88376, -0.3991;
    ws.uLast[0] = 0;
    ws.uLast[1] = 0;

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
                // TODO: implement this block
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
                // TODO: implement this block
            }
            break;
        }
        case (SHUTDOWN_STABLE):
        {
            // TODO: implement this block
            break;
        }
    }

}