/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
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
 *
 */
#include <math.h>

#include "FreeRTOS.h"
#include "task.h"

#include "config.h"
#include "system.h"
#include "pm.h"
#include "stabilizer.h"
#include "commander.h"
#include "attitude_controller.h"
#include "sensfusion6.h"
#include "imu.h"
#include "motors.h"
#include "log.h"
#include "pid.h"
#include "param.h"
#include "sitaw.h"
#ifdef PLATFORM_CF1
  #include "ms5611.h"
#else
  #include "lps25h.h"
#endif
#include "num.h"
#include "position_estimator.h"
#include "position_controller.h"
#include "altitude_hold.h"


/**
 * Defines in what divided update rate should the attitude
 * control loop run relative the rate control loop.
 */
#define ATTITUDE_UPDATE_RATE_DIVIDER  2
#define ATTITUDE_UPDATE_DT  (float)(1.0 / (IMU_UPDATE_FREQ / ATTITUDE_UPDATE_RATE_DIVIDER)) // 250hz

#define ALTHOLD_UPDATE_RATE_DIVIDER  5
#define ALTHOLD_UPDATE_DT  (float)(1.0 / (IMU_UPDATE_FREQ / ALTHOLD_UPDATE_RATE_DIVIDER))   // 100hz


// Barometer/ Altitude hold stuff
static float accWZ     = 0.0; // Acceleration Without gravity along Z axis (G).
static float accMAG    = 0.0; // Acceleration magnitude
static float velocityZ = 0.0; // Vertical speed (world frame) integrated from vertical acceleration (m/s)

static float vAccDeadband = 0.04; // Vertical acceleration deadband
static float velZAlpha = 0.995;   // Blending factor to avoid vertical speed to accumulate error


static Axis3f gyro; // Gyro axis data in deg/s
static Axis3f acc;  // Accelerometer axis data in mG
static Axis3f mag;  // Magnetometer axis data in testla

static float eulerRollActual;   // Measured roll angle in deg
static float eulerPitchActual;  // Measured pitch angle in deg
static float eulerYawActual;    // Measured yaw angle in deg

uint16_t actuatorThrust;  // Actuator output for thrust base

uint32_t motorPowerM1;  // Motor 1 power output (16bit value used: 0 - 65535)
uint32_t motorPowerM2;  // Motor 2 power output (16bit value used: 0 - 65535)
uint32_t motorPowerM3;  // Motor 3 power output (16bit value used: 0 - 65535)
uint32_t motorPowerM4;  // Motor 4 power output (16bit value used: 0 - 65535)

static bool isInit;
static int mode;

static uint16_t limitThrust(int32_t value);

xSemaphoreHandle(modeGatekeeper) = 0;
static void stabilizerTask(void* param)
{
  uint32_t lastWakeTime;

  //Wait for the system to be fully started to start stabilization loop
  systemWaitStart();
  lastWakeTime = xTaskGetTickCount ();
  int modeLocal = 0;
  while(1) {
	  	vTaskDelayUntil(&lastWakeTime, F2T(IMU_UPDATE_FREQ)); // 500Hz
	  	if(xSemaphoreTake(modeGatekeeper, M2T(1))) {
	  		modeLocal = mode;
	  		xSemaphoreGive(modeGatekeeper);
	  	}

	  	if(modeLocal == 1) {
	  	motorPowerM2 = limitThrust(fabs(32000*30/180.0));
	  	motorsSetRatio(MOTOR_M2, motorPowerM2);
        motorsSetRatio(MOTOR_M1, 0);
	  	}else {
	  	motorPowerM1 = limitThrust(fabs(32000*20/180.0));
        motorsSetRatio(MOTOR_M1, motorPowerM1);
	  	motorsSetRatio(MOTOR_M2, 0);
	  	}
   }
}

/*static void stabilizerTask(void* param)
{
  uint32_t lastWakeTime;

  //Wait for the system to be fully started to start stabilization loop
  systemWaitStart();
  lastWakeTime = xTaskGetTickCount ();
  while(1) {
	  	vTaskDelayUntil(&lastWakeTime, F2T(IMU_UPDATE_FREQ)); // 500Hz
	    imu9Read(&gyro, &acc, &mag);

	    if (imu6IsCalibrated()) {
	    	sensfusion6UpdateQ(gyro.x, gyro.y, gyro.z, acc.x, acc.y, acc.z, ATTITUDE_UPDATE_DT);
	    	sensfusion6GetEulerRPY(&eulerRollActual, &eulerPitchActual, &eulerYawActual);

			accWZ = sensfusion6GetAccZWithoutGravity(acc.x, acc.y, acc.z);

			// Estimate speed from acc (drifts)
        	velocityZ += deadband(accWZ, vAccDeadband) * FUSION_UPDATE_DT * G;
        	velocityZ *= velZAlpha;

	  	}
   }
}*/

static void modeswitchTask(void* param)
{
	systemWaitStart();		//Wait for the system to be fully started to start stabilization loop

  while(1)
  {
	  vTaskDelay(M2T(5000));
	  if(xSemaphoreTake(modeGatekeeper, M2T(1))){
		  mode = !mode;
		  xSemaphoreGive(modeGatekeeper);
	  }

  }
}

void stabilizerInit(void)
{
  if(isInit)
    return;

  motorsInit(motorMapDefaultBrushed);
  imu6Init();
  sensfusion6Init();
  attitudeControllerInit();
  mode = 0;
  modeGatekeeper = xSemaphoreCreateMutex();
  xTaskCreate(stabilizerTask, STABILIZER_TASK_NAME,
              STABILIZER_TASK_STACKSIZE, NULL, STABILIZER_TASK_PRI, NULL);
  xTaskCreate(modeswitchTask, MODESWITCH_TASK_NAME,
                MODESWITCH_TASK_STACKSIZE, NULL, MODESWITCH_TASK_PRI, NULL);

  isInit = true;
}

bool stabilizerTest(void)
{
  bool pass = true;

  pass &= motorsTest();
  pass &= imu6Test();
  pass &= sensfusion6Test();
  pass &= attitudeControllerTest();

  return pass;
}

static uint16_t limitThrust(int32_t value)
{
  return limitUint16(value);
}

/* Perform matrix multiplication in the LQ feedback Kx = K*x. */
void feedbackMultiply(double K[4][6], double x[6][1], double Kx[4][1]) {
    int i,j,k;
    for (i=0; i<4; i++) {
        for (j=0; j<1; j++) {
            for (k=0; k<6; k++) {
                Kx[i][j] += K[i][k] * x[k][j];
            }
        }
    }
}

/* Perform matrix multiplication for the reference gain Krr = Kr*r. */
void referenceMultiply(double Kr[3][3], double r[3][1], double Krr[3][1]) {
    int i,j,k;
    for (i=0; i<3; i++) {
        for (j=0; j<1; j++) {
            for (k=0; k<3; k++) {
                Krr[i][j] += Kr[i][k] * r[k][j];
            }
        }
    }
}

LOG_GROUP_START(stabilizer)
LOG_ADD(LOG_FLOAT, roll, &eulerRollActual)
LOG_ADD(LOG_FLOAT, pitch, &eulerPitchActual)
LOG_ADD(LOG_FLOAT, yaw, &eulerYawActual)
LOG_ADD(LOG_UINT16, thrust, &actuatorThrust)
LOG_GROUP_STOP(stabilizer)

LOG_GROUP_START(acc)
LOG_ADD(LOG_FLOAT, x, &acc.x)
LOG_ADD(LOG_FLOAT, y, &acc.y)
LOG_ADD(LOG_FLOAT, z, &acc.z)
LOG_GROUP_STOP(acc)

LOG_GROUP_START(gyro)
LOG_ADD(LOG_FLOAT, x, &gyro.x)
LOG_ADD(LOG_FLOAT, y, &gyro.y)
LOG_ADD(LOG_FLOAT, z, &gyro.z)
LOG_GROUP_STOP(gyro)

LOG_GROUP_START(mag)
LOG_ADD(LOG_FLOAT, x, &mag.x)
LOG_ADD(LOG_FLOAT, y, &mag.y)
LOG_ADD(LOG_FLOAT, z, &mag.z)
LOG_GROUP_STOP(mag)

LOG_GROUP_START(motor)
LOG_ADD(LOG_INT32, m4, &motorPowerM4)
LOG_ADD(LOG_INT32, m1, &motorPowerM1)
LOG_ADD(LOG_INT32, m2, &motorPowerM2)
LOG_ADD(LOG_INT32, m3, &motorPowerM3)
LOG_GROUP_STOP(motor)
