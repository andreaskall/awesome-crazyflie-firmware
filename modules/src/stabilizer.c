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
//static float accWZ     = 0.0; // Acceleration Without gravity along Z axis (G).
//static float accMAG    = 0.0; // Acceleration magnitude
//static float velocityZ = 0.0; // Vertical speed (world frame) integrated from vertical acceleration (m/s)

//static float vAccDeadband = 0.04; // Vertical acceleration deadband
//static float velZAlpha = 0.995;   // Blending factor to avoid vertical speed to accumulate error


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

float motor1_PWM;
float motor2_PWM;
float motor3_PWM;
float motor4_PWM;

static bool isInit;
static int mode;
static float referenceGlobal[3];
static float thrustArray[4] = {0,0,0,0};
static float states[6] = {0,0,0,0,0,0};
float K[4][6];
float Kr[4][3];
static float Kx[4] = {0,0,0,0};
static float Krr[4] = {0,0,0,0};

// K and Kr matrix values
static float roll_pitch = 0.07;
static float roll_pitch_dot = 0.03;
static float yaw_gain = 0.0005;
static float yaw_dot_gain = 0.02;

static uint16_t limitThrust(int32_t value);
void feedbackMultiply(float K[4][6], float x[6], float Kx[4]);
void referenceMultiply(float Kr[4][3], float r[3], float Krr[4]);

xSemaphoreHandle(modeGatekeeper) = 0;
xSemaphoreHandle(referenceGatekeeper) = 0;



static void stabilizerTask(void* param)
{
  uint32_t lastWakeTime;
  static float referenceLocal[3] = {0,0,0};


  //Wait for the system to be fully started to start stabilization loop
  systemWaitStart();
  lastWakeTime = xTaskGetTickCount ();
  while(1) {
	  	vTaskDelayUntil(&lastWakeTime, F2T(IMU_UPDATE_FREQ)); // 500Hz
	    imu9Read(&gyro, &acc, &mag);

	    if (imu6IsCalibrated()) {
	    	sensfusion6UpdateQ(gyro.x, gyro.y, gyro.z, acc.x, acc.y, acc.z, ATTITUDE_UPDATE_DT);
	    	sensfusion6GetEulerRPY(&eulerRollActual, &eulerPitchActual, &eulerYawActual);
	    	states[0] = eulerRollActual*(M_PI/180);
	    	states[1] = gyro.x*(M_PI/180);
	    	states[2] = eulerPitchActual*(M_PI/180);
	    	states[3] = gyro.y*(M_PI/180);
	    	states[4] =	eulerYawActual*(M_PI/180);
	    	states[5] =	gyro.z*(M_PI/180);


			if(xSemaphoreTake(referenceGatekeeper, M2T(0.5))){
				referenceLocal[0] = referenceGlobal[0];
				referenceLocal[1] = referenceGlobal[1];
				referenceLocal[2] = referenceGlobal[2];
				xSemaphoreGive(referenceGatekeeper);
			}

			if(xSemaphoreTake(modeGatekeeper, M2T(1))){
				feedbackMultiply(K, states, Kx);
				referenceMultiply(Kr, referenceLocal, Krr);
				xSemaphoreGive(modeGatekeeper);
			}
	    	thrustArray[0] = Krr[0] - Kx[0];
	    	thrustArray[1] = Krr[1] - Kx[1];
	    	thrustArray[2] = Krr[2] - Kx[2];
	    	thrustArray[3] = Krr[3] - Kx[3];

	    	motor1_PWM = -76124.69*thrustArray[0]*thrustArray[0] + 149054.6*thrustArray[0] + 1135.7;
	    	motor2_PWM = -76124.69*thrustArray[1]*thrustArray[1] + 149054.6*thrustArray[1] + 1135.7;
	    	motor3_PWM = -76124.69*thrustArray[2]*thrustArray[2] + 149054.6*thrustArray[2] + 1135.7;
	    	motor4_PWM = -76124.69*thrustArray[3]*thrustArray[3] + 149054.6*thrustArray[3] + 1135.7;

	    	motorPowerM1 = limitThrust(motor1_PWM + actuatorThrust);
		  	motorPowerM2 = limitThrust(motor2_PWM + actuatorThrust);
		  	motorPowerM3 = limitThrust(motor3_PWM + actuatorThrust);
		  	motorPowerM4 = limitThrust(motor4_PWM + actuatorThrust);

		  	motorsSetRatio(MOTOR_M1, motorPowerM1);
		  	motorsSetRatio(MOTOR_M2, motorPowerM2);
		  	motorsSetRatio(MOTOR_M3, motorPowerM3);
		  	motorsSetRatio(MOTOR_M4, motorPowerM4);

	    			//accWZ = sensfusion6GetAccZWithoutGravity(acc.x, acc.y, acc.z);

			// Estimate speed from acc (drifts)
        	//velocityZ += deadband(accWZ, vAccDeadband) * FUSION_UPDATE_DT * G;
        	//velocityZ *= velZAlpha;

	  	}
   }
}

static void modeswitchTask(void* param)
{
	systemWaitStart();		//Wait for the system to be fully started to start stabilization loop
	while(1)
	{
		vTaskDelay(M2T(2000));
		if(xSemaphoreTake(modeGatekeeper, M2T(1))){
			mode = !mode;
			K[0][0] = -roll_pitch; K[1][0] = -roll_pitch; K[2][0] = roll_pitch; K[3][0] = roll_pitch;
			K[0][1] = -roll_pitch_dot; K[1][1] = -roll_pitch_dot; K[2][1] = roll_pitch_dot; K[3][1] = roll_pitch_dot;
			K[0][2] = roll_pitch; K[1][2] = -roll_pitch; K[2][2] = -roll_pitch; K[3][2] = roll_pitch;
			K[0][3] = -roll_pitch_dot; K[1][3] = roll_pitch_dot; K[2][3] = roll_pitch_dot; K[3][3] = -roll_pitch_dot;
			K[0][4] = -yaw_gain; K[1][4] = yaw_gain; K[2][4] = -yaw_gain; K[3][4] = yaw_gain;
			K[0][5] = -yaw_dot_gain; K[1][5] = yaw_dot_gain; K[2][5] = -yaw_dot_gain; K[3][5] = yaw_dot_gain;

			Kr[0][0] = K[0][0]; Kr[1][0] = K[1][0]; Kr[2][0] = K[2][0]; Kr[3][0] = K[3][0];
			Kr[0][1] = K[0][2]; Kr[1][1] = K[1][2]; Kr[2][1] = K[2][2]; Kr[3][1] = K[3][2];
			Kr[0][2] = K[0][5]; Kr[1][2] = K[1][5]; Kr[2][2] = K[2][5]; Kr[3][2] = K[3][5];

			xSemaphoreGive(modeGatekeeper);
		}


	}
}


static void refgenTask(void* param) {
	systemWaitStart();		//Wait for the system to be fully started to start stabilization loop
		while(1)
		{
			vTaskDelay(F2T(250));
			if(xSemaphoreTake(referenceGatekeeper, M2T(1))){
				commanderGetRPY(&referenceGlobal[0], &referenceGlobal[1], &referenceGlobal[2]);
				referenceGlobal[0] *= (M_PI/180);
				referenceGlobal[1] *= (M_PI/180);
				referenceGlobal[2] *= (M_PI/180);
				referenceGlobal[2] = -referenceGlobal[2];

				commanderGetThrust(&actuatorThrust);
				xSemaphoreGive(referenceGatekeeper);
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
  referenceGlobal[0] = 0;
  referenceGlobal[1] = 0;
  referenceGlobal[2] = 0;
  modeGatekeeper = xSemaphoreCreateMutex();
  referenceGatekeeper = xSemaphoreCreateMutex();
  xTaskCreate(stabilizerTask, STABILIZER_TASK_NAME,
              STABILIZER_TASK_STACKSIZE, NULL, STABILIZER_TASK_PRI, NULL);
  xTaskCreate(modeswitchTask, MODESWITCH_TASK_NAME,
                MODESWITCH_TASK_STACKSIZE, NULL, MODESWITCH_TASK_PRI, NULL);
  xTaskCreate(refgenTask, REFGEN_TASK_NAME,
                REFGEN_TASK_STACKSIZE, NULL, REFGEN_TASK_PRI, NULL);

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
void feedbackMultiply(float K[4][6], float x[6], float Kx[4]) {
    int i,k;
    for (i=0; i<4; i++) {
    	Kx[i] = 0;
    	for (k=0; k<6; k++) {
			Kx[i] += K[i][k] * x[k];
		}
    }
}

/* Perform matrix multiplication for the reference gain Krr = Kr*r. */
void referenceMultiply(float Kr[4][3], float r[3], float Krr[4]) {
    int i,k;
    for (i=0; i<4; i++) {
    	Krr[i] = 0;
        for (k=0; k<3; k++) {
            Krr[i] += Kr[i][k] * r[k];
        }
    }
}


PARAM_GROUP_START(conroller)
PARAM_ADD(PARAM_FLOAT, roll_pitch, &roll_pitch)
PARAM_ADD(PARAM_FLOAT, roll_pitch_dot, &roll_pitch_dot)
PARAM_ADD(PARAM_FLOAT, yaw_gain, &yaw_gain)
PARAM_ADD(PARAM_FLOAT, yaw_dot_gain, &yaw_dot_gain)
PARAM_GROUP_STOP(controller)


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

LOG_GROUP_START(thrustArray)
LOG_ADD(LOG_FLOAT, t1, &thrustArray[0])
LOG_ADD(LOG_FLOAT, t2, &thrustArray[1])
LOG_ADD(LOG_FLOAT, t3, &thrustArray[2])
LOG_ADD(LOG_FLOAT, t4, &thrustArray[3])
LOG_ADD(LOG_FLOAT, ctrThrust, &actuatorThrust)
LOG_GROUP_STOP(thrustArray)

LOG_GROUP_START(Kr)
LOG_ADD(LOG_FLOAT, t1, &Krr[0])
LOG_ADD(LOG_FLOAT, t2, &Krr[1])
LOG_ADD(LOG_FLOAT, t3, &Krr[2])
LOG_ADD(LOG_FLOAT, t4, &Krr[3])
LOG_GROUP_STOP(Kr)

LOG_GROUP_START(K)
LOG_ADD(LOG_FLOAT, t1, &Kx[0])
LOG_ADD(LOG_FLOAT, t2, &Kx[1])
LOG_ADD(LOG_FLOAT, t3, &Kx[2])
LOG_ADD(LOG_FLOAT, t4, &Kx[3])
LOG_GROUP_STOP(K)

LOG_GROUP_START(PWM)
LOG_ADD(LOG_FLOAT, motor1, &motor1_PWM)
LOG_ADD(LOG_FLOAT, motor2, &motor2_PWM)
LOG_ADD(LOG_FLOAT, motor3, &motor3_PWM)
LOG_ADD(LOG_FLOAT, motor4, &motor4_PWM)
LOG_GROUP_STOP(K)

