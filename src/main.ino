#include <cstdint>  // C STD Int library, might not be needed
#include "mission_constants.hh"
#include "moding.hh"
#include "MPU6050.h"  // MPU 6050 IMU Library
#include "Wire.h"  // Arduino library
#include "workspace.hh"  // variable storage

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
int get_latest_measurements(MPU6050 imu, float *a[3], float *w[3], bool debug=false)
{
    // READ DATA
    int16_t ax, ay, az, wx, wy, wz;  // temp vars to store raw measurements
    imu.getMotion6(&ax, &ay, &az, &wx, &wy, &wz);  // stores measurements in temp vars

    // CONVERT AND STORE DATA
    *a[0] = ax*LSB_LINEAR*1000*G_2_MS2;  // (m/s^2) x-axis acceleration
    *a[1] = ay*LSB_LINEAR*1000*G_2_MS2;  // (m/s^2) y-axis acceleration
    *a[2] = az*LSB_LINEAR*1000*G_2_MS2;  // (m/s^2) z-axis acceleration
    *w[0] = wx*LSB_ANGULAR*DEG_2_RAD;    // (rad/s) x-axis angular velocity
    *w[1] = wy*LSB_ANGULAR*DEG_2_RAD;    // (rad/s) y-axis angular velocity
    *w[2] = wz*LSB_ANGULAR*DEG_2_RAD;    // (rad/s) z-axis angular velocity

    if (debug) {
        print_measurements(a, w);
    }

    return 0;
}


/* setup
 * 
 * This is called once to initialize everything.
 */
void setup()
{
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