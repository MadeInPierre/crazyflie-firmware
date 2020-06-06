/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2016 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * power_distribution_stock.c - Crazyflie stock power distribution code
 */
#include "power_distribution.h"

#include "log.h"
#include "param.h"
#include "num.h"

#include "debug.h"

#include "motors.h"

static bool motorSetEnable = false;

static struct {
  uint32_t m1;
  uint32_t m2;
  uint32_t m3;
  uint32_t m4;
  uint32_t m5;
  uint32_t m6;
} motorPower;

static struct {
  uint16_t m1;
  uint16_t m2;
  uint16_t m3;
  uint16_t m4;
  uint16_t m5;
  uint16_t m6;
} motorPowerSet;

int16_t r; 
int16_t p; 
float t; 
int16_t y;
int16_t K_y = 1.0;
float K_t = 1.0;
int16_t K_p = 1.0;
int16_t K_r = 1.0;
void powerDistributionInit(void)
{

  DEBUG_PRINT("Initializing Hexa Plan\n");
  motorsInit(motorMapDefaultBrushed);
}

bool powerDistributionTest(void)
{
  bool pass = true;

  pass &= motorsTest();

  return pass;
}

#define limitThrust(VAL) limitUint16(VAL)

void powerStop()
{
  motorsSetRatio(MOTOR_M1, 0);
  motorsSetRatio(MOTOR_M2, 0);
  motorsSetRatio(MOTOR_M3, 0);
  motorsSetRatio(MOTOR_M4, 0);
  motorsSetRatio(MOTOR_M5, 0);
  motorsSetRatio(MOTOR_M6, 0);
}

void powerDistribution(const control_t *control)
{
    r = K_r * control->roll / 4.0f;
    p = K_p * control->pitch / 2.0f;
    t = K_t * control->az;
    y = K_y * control->yaw;
    p = p;
    r = r;
    y = y;
    motorPower.m6 = limitThrust(t - y - p - r);
    motorPower.m5 = limitThrust(t + y - p + r);
    motorPower.m4 = limitThrust(t - y + 2 * r);
    motorPower.m3 = limitThrust(t + y + p + r);
    motorPower.m2 = limitThrust(t - y + p - r);
    motorPower.m1 = limitThrust(t + y - 2 * r);

    if (motorSetEnable) {
        motorsSetRatio(MOTOR_M1, motorPowerSet.m1);
        motorsSetRatio(MOTOR_M2, motorPowerSet.m2);
        motorsSetRatio(MOTOR_M3, motorPowerSet.m3);
        motorsSetRatio(MOTOR_M4, motorPowerSet.m4);
        motorsSetRatio(MOTOR_M5, motorPowerSet.m5);
        motorsSetRatio(MOTOR_M6, motorPowerSet.m6);
    }
    else {
        motorsSetRatio(MOTOR_M1, motorPower.m1);
        motorsSetRatio(MOTOR_M2, motorPower.m2);
        motorsSetRatio(MOTOR_M3, motorPower.m3);
        motorsSetRatio(MOTOR_M4, motorPower.m4);
        motorsSetRatio(MOTOR_M5, motorPower.m5);
        motorsSetRatio(MOTOR_M6, motorPower.m6);
    }
}

PARAM_GROUP_START(motorPowerSet)
PARAM_ADD(PARAM_UINT8, enable, &motorSetEnable)
PARAM_ADD(PARAM_FLOAT, K_r, &K_r)
PARAM_ADD(PARAM_FLOAT, K_p, &K_p)
PARAM_ADD(PARAM_FLOAT, K_y, &K_y)
PARAM_ADD(PARAM_FLOAT, K_t, &K_t)
PARAM_ADD(PARAM_UINT16, m1, &motorPowerSet.m1)
PARAM_ADD(PARAM_UINT16, m2, &motorPowerSet.m2)
PARAM_ADD(PARAM_UINT16, m3, &motorPowerSet.m3)
PARAM_ADD(PARAM_UINT16, m4, &motorPowerSet.m4)
PARAM_ADD(PARAM_UINT16, m5, &motorPowerSet.m5)
PARAM_ADD(PARAM_UINT16, m6, &motorPowerSet.m6)
PARAM_GROUP_STOP(ring)

LOG_GROUP_START(motor)

LOG_ADD(LOG_INT16, r, &r)
LOG_ADD(LOG_INT16, p, &p)
LOG_ADD(LOG_INT16, y, &y)
LOG_ADD(LOG_FLOAT, t, &t)
LOG_ADD(LOG_INT32, m4, &motorPower.m4)
LOG_ADD(LOG_INT32, m1, &motorPower.m1)
LOG_ADD(LOG_INT32, m2, &motorPower.m2)
LOG_ADD(LOG_INT32, m3, &motorPower.m3)
LOG_ADD(LOG_INT32, m5, &motorPower.m5)
LOG_ADD(LOG_INT32, m6, &motorPower.m6)
LOG_GROUP_STOP(motor)
