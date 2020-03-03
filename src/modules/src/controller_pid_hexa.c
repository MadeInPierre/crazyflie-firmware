#include "estimator_kalman.h"
#include "pid.h"
#include "stabilizer.h"
#include "stabilizer_types.h"
#include <stdio.h>

#include "controller_pid_hexa.h"
#include "sensfusion6.h"

#include "debug.h"
#include "led.h"
#include "ledseq.h"
#include "log.h"
#include "math3d.h"
#include "param.h"
#define RATE_CONTROLLER_LOOP RATE_1000_HZ

#define UPDATE_DT (float)(1.0f / RATE_CONTROLLER_LOOP)

PidObject pidX;
PidObject pidY;
PidObject pidZ;
PidObject pidQX;
PidObject pidQY;
PidObject pidQZ;

#define Hexa_PID_X_KP 1.00
#define Hexa_PID_X_KI 0.0
#define Hexa_PID_X_KD 1.5
#define Hexa_PID_X_INTEGRATION_LIMIT 0.01

#define Hexa_PID_Y_KP 1.00
#define Hexa_PID_Y_KI 0.0
#define Hexa_PID_Y_KD 1.5
#define Hexa_PID_Y_INTEGRATION_LIMIT 0.01

#define Hexa_PID_Z_KP 19.0
#define Hexa_PID_Z_KI 1.0
#define Hexa_PID_Z_KD 15.0
#define Hexa_PID_Z_INTEGRATION_LIMIT 0.2

#define Hexa_PID_QX_KP 500.0
#define Hexa_PID_QX_KI 0.0
#define Hexa_PID_QX_KD 200.0
#define Hexa_PID_QX_INTEGRATION_LIMIT 0.4

#define Hexa_PID_QY_KP 300.0
#define Hexa_PID_QY_KI 0.0
#define Hexa_PID_QY_KD 100.0
#define Hexa_PID_QY_INTEGRATION_LIMIT 0.4

#define Hexa_PID_QZ_KP 300.0
#define Hexa_PID_QZ_KI 1.0
#define Hexa_PID_QZ_KD 100.0
#define Hexa_PID_QZ_INTEGRATION_LIMIT 0.4
#define Hexa_mass 0.045 //45g in kg
#define Hexa_Ixx 0.000016
#define Hexa_Iyy 0.000016
#define Hexa_Izz 0.000029
#define D_FILTER true
#define D_FILTER_ATTITUDE false
#define CUTOFF_FREQ 20.0f
#define CUTOFF_FREQ_ATTITUDE 0.0f
// acceleration control
static float ax;
static float ay;
static float az;
// angular acceleration control
static float wx;
static float wy;
static float wz;
// setpoint control
static float sx;
static float sy;
static float sz;
//current position
static float cx;
static float cy;
static float cz;
//current attitude
static float qw;
static float qx;
static float qy;
static float qz;
//setpoint attitude
static float sqw;
static float sqx;
static float sqy;
static float sqz;
static float t;
static float t_init;
static bool took_off = false;
static bool isInit = false;
static bool firstControllerLoop = true;
static bool shutdown_flag = true;
static bool test_flag = false;
static bool landing_flag = false;
void controllerPidHexaInit(void)
{
    if (isInit) {
        return;
    }

    pidInit(&pidQX, 0, Hexa_PID_QX_KP, Hexa_PID_QX_KI, Hexa_PID_QX_KD, UPDATE_DT, 1.0 / UPDATE_DT, CUTOFF_FREQ_ATTITUDE, D_FILTER_ATTITUDE);
    pidInit(&pidQY, 0, Hexa_PID_QY_KP, Hexa_PID_QY_KI, Hexa_PID_QY_KD, UPDATE_DT, 1.0 / UPDATE_DT, CUTOFF_FREQ_ATTITUDE, D_FILTER_ATTITUDE);
    pidInit(&pidQZ, 0, Hexa_PID_QZ_KP, Hexa_PID_QZ_KI, Hexa_PID_QZ_KD, UPDATE_DT, 1.0 / UPDATE_DT, CUTOFF_FREQ_ATTITUDE, D_FILTER_ATTITUDE);
    pidSetIntegralLimit(&pidQX, Hexa_PID_QX_INTEGRATION_LIMIT);
    pidSetIntegralLimit(&pidQY, Hexa_PID_QY_INTEGRATION_LIMIT);
    pidSetIntegralLimit(&pidQZ, Hexa_PID_QZ_INTEGRATION_LIMIT);

    pidInit(&pidX, 0, Hexa_PID_X_KP, Hexa_PID_X_KI, Hexa_PID_X_KD, UPDATE_DT, 1.0 / UPDATE_DT, CUTOFF_FREQ, D_FILTER);
    pidInit(&pidY, 0, Hexa_PID_Y_KP, Hexa_PID_Y_KI, Hexa_PID_Y_KD, UPDATE_DT, 1.0 / UPDATE_DT, CUTOFF_FREQ, D_FILTER);
    pidInit(&pidZ, 0, Hexa_PID_Z_KP, Hexa_PID_Z_KI, Hexa_PID_Z_KD, UPDATE_DT, 1.0 / UPDATE_DT, CUTOFF_FREQ, D_FILTER);
    pidSetIntegralLimit(&pidX, Hexa_PID_QX_INTEGRATION_LIMIT);
    pidSetIntegralLimit(&pidY, Hexa_PID_QY_INTEGRATION_LIMIT);
    pidSetIntegralLimit(&pidZ, Hexa_PID_QZ_INTEGRATION_LIMIT);
    t = 0;
    t_init = 0;
    isInit = true;
    DEBUG_PRINT("Initializing PID Hexa \n");
    ax = 0;
    ay = 0;
    az = 0;
    wx = 0;
    wy = 0;
    wz = 0;
    sx = 0;
    sy = 0;
}

bool controllerPidHexaTest(void)
{
    bool pass = isInit;

    return pass;
}
float transform_error(double error, double min_bound, double threshold, double max_bound)
{
    if(error<threshold && error > -threshold)
    {
        // value is to low to be used -> it is ignored.
        return 0;
    }
    else if(error > max_bound)
    {
        // value is too high
        return (float)max_bound;
    }
    else if(error < min_bound)
    {
        // value is too low 
        return (float)min_bound;
    }
    else
    {
        //value is within bounds
        return error;
    }
    

}
// Updates control to desired in drone frame accelerations that power distribution will need to apply
void controllerPidHexa(control_t* control, setpoint_t* setpoint,
    const sensorData_t* sensors,
    const state_t* state,
    const uint32_t tick)
{
    if (RATE_DO_EXECUTE(RATE_CONTROLLER_LOOP, tick)) {
        if (setpoint->position.z > 41.99 && setpoint->position.z < 42.01) {
            //Emergency stop
            shutdown_flag = true;
            landing_flag = false;
        }
        if (setpoint->position.z > 62.99 && setpoint->position.z < 63.01) {
            //Landing
            landing_flag = true;
            sz = sz * 0.995;
        }
        if (setpoint->position.z > 83.99 && setpoint->position.z < 84.01) {
            //Take off
            shutdown_flag = false;
            landing_flag = false;
            took_off = false;
            firstControllerLoop = true;
            sz = 0.8;
        }
        if (!(shutdown_flag)) {
            if (firstControllerLoop) {
                estimatorKalmanInit();
                sx = setpoint->position.x;
                sy = setpoint->position.y;
                firstControllerLoop = false;
                t_init = 0;
            }
            t_init = fmax(fmin(1, t_init + 1/RATE_CONTROLLER_LOOP), t_init);
            // computing state and setpoints
            cx = state->position.x;
            cy = state->position.y;
            cz = state->position.z;
            qw = state->attitudeQuaternion.w;
            qx = state->attitudeQuaternion.x;
            qy = state->attitudeQuaternion.y;
            qz = state->attitudeQuaternion.z;
            // sqw = setpoint->attitudeQuaternion.w;
            sqw = 1;
            sqx = setpoint->attitudeQuaternion.x;
            sqy = setpoint->attitudeQuaternion.y;
            sqz = setpoint->attitudeQuaternion.z;
            pidSetDesired(&pidX, sx);
            pidSetDesired(&pidY, sy);
            pidSetDesired(&pidZ, sz);
            pidSetDesired(&pidQX, 0);
            pidSetDesired(&pidQY, 0);
            pidSetDesired(&pidQZ, 0);
            //Get Quaternion error by multiplication rather than by substraction
            struct quat current_attitude = mkquat(qx, qy, qz, qw);
            struct quat setpoint_attitude = mkquat(sqx, sqy, sqz, sqw);
            struct quat inv_attitude = qinv(current_attitude);
            struct quat q_error = qqmul(setpoint_attitude, inv_attitude);
            if (q_error.w < 0) //rotation would be faster in the other direction
            {
                q_error = mkquat(-q_error.x, -q_error.y, -q_error.z, q_error.w);
            }
            //The values given to control should be taken as forces. The value given by the PID should be taken as accelerations.
            //Default control for attitude
            wx = pidUpdate(&pidQX, q_error.x, true) * Hexa_Ixx;
            wy = pidUpdate(&pidQY, q_error.y, true) * Hexa_Iyy;
            wz = pidUpdate(&pidQZ, q_error.z, true) * Hexa_Izz;
            //Default control for position
            // ax = Hexa_mass * pidUpdate(&pidX, cx, true);
            ax = 0;
            ay = 0;
            // ay = Hexa_mass * pidUpdate(&pidY, cy, true);
            // adding gravity to the error so that the drone is able to hover more easily.
            az = Hexa_mass * (pidUpdate(&pidZ, cz, true) + 9.81);
            if (cz < 0.15 && !(took_off)) {
                //taking off, warming the motors, ignoring the x,y controls.
                ax = 0;
                ay = 0;
                // az += 0.00025;
            }
            else {
                //took off, default control for altitude.
                took_off = true;
            }
            if (cz > 0.2) {
                //High enough to set a new altitude goal
                sz = 1.5;
            }
            if (took_off && !landing_flag && cz > 1.0) {
                // High enough to try special commands
                ax = (float)(0.05);
            }
            if (test_flag) {
                // Debug subpart of the controller.
                if (landing_flag) {
                    //Second phase using default controls
                    // ax = (float)0.00;
                    // ay = (float)0.00;
                    // az = (float)0.6;
                    // wx = 0.000;
                    // wy = 0.000;
                    // wz = 0.005;
                }
                else {
                    //First and default phase
                    ax = (float)0.00;
                    ay = (float)0.00;
                    az = (float)0.4;
                    wx = 0.000;
                    wy = 0.00;
                    wz = 0.00;
                }
            }
            //Truncating the force/torque setpoints inside bounds
            ax = transform_error(ax, -0.05, 0.02, 0.05);
            ay = transform_error(ay, -0.05, 0.02, 0.05);
            az = transform_error(az, 0.30, 0.01, 0.60);
            wx = transform_error(wx, -0.005, 0.00, 0.005);
            wy = transform_error(wy, -0.005, 0.00, 0.005);
            wz = transform_error(wz, -0.005, 0.00, 0.005);
        }
        else {
            //Shutting down
            ax = 0;
            ay = 0;
            az = 0;
            wx = 0;
            wy = 0;
            wz = 0;
        }
        //Updating the control setpoints
        control->ax = -ax;
        control->ay = -ay;
        control->az = az;
        control->roll = (int16_t)(wx * 10000);
        control->pitch = (int16_t)(wy * 10000);
        control->yaw = -(int16_t)(wz * 10000);
    }
}

LOG_GROUP_START(controller)
LOG_ADD(LOG_FLOAT, ax, &ax)
LOG_ADD(LOG_FLOAT, ay, &ay)
LOG_ADD(LOG_FLOAT, az, &az)
LOG_ADD(LOG_FLOAT, wx, &wx)
LOG_ADD(LOG_FLOAT, wy, &wy)
LOG_ADD(LOG_FLOAT, wz, &wz)
LOG_ADD(LOG_FLOAT, sx, &sx)
LOG_ADD(LOG_FLOAT, sy, &sy)
LOG_ADD(LOG_FLOAT, sz, &sz)
LOG_ADD(LOG_FLOAT, cx, &cx)
LOG_ADD(LOG_FLOAT, cy, &cy)
LOG_ADD(LOG_FLOAT, cz, &cz)
LOG_ADD(LOG_FLOAT, qw, &qw)
LOG_ADD(LOG_FLOAT, qx, &qx)
LOG_ADD(LOG_FLOAT, qy, &qy)
LOG_ADD(LOG_FLOAT, qz, &qz)
LOG_ADD(LOG_FLOAT, sqw, &sqw)
LOG_ADD(LOG_FLOAT, sqx, &sqx)
LOG_ADD(LOG_FLOAT, sqy, &sqy)
LOG_ADD(LOG_FLOAT, sqz, &sqz)
LOG_GROUP_STOP(controller)
