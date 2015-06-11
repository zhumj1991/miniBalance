/**
 * Copyright (C) 2011-2012 Bitcraze AB
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
#include <stdlib.h>
#include <math.h>

#include "FreeRTOS.h"
#include "task.h"

#include "config.h"
#include "system.h"
//#include "pm.h"
#include "commander_balance.h"
#include "stabilizer_balance.h"
#include "controller_balance.h"
#include "sensfusion6.h"
#include "imu.h"
#include "motors.h"
#include "encoder.h"
//#include "log.h"
#include "pid.h"
#include "ledseq.h"
//#include "param.h"
////#include "ms5611.h"
//#include "lps25h.h"
//#include "debug.h"

#define ATTITUDE_UPDATE_RATE_DIVIDER  1
#define FUSION_UPDATE_DT  (float)(1.0 / (IMU_UPDATE_FREQ / ATTITUDE_UPDATE_RATE_DIVIDER))

#define	KALMAN_FILTER
//#define COMPLEMENTARY_FILTER

#define LOGGING_ENABLED
#ifdef LOGGING_ENABLED
  #define PRIVATE
#else
  #define PRIVATE static
#endif

PRIVATE Axis3f gyro; // Gyro axis data in deg/s
PRIVATE Axis3f acc;  // Accelerometer axis data in mG

PRIVATE float eulerRollActual;
PRIVATE float eulerPitchActual;
PRIVATE float eulerYawActual;

extern PidObject pidVelocity;
extern PidObject pidTurn;
	
	
int16_t motorLeftOutput, motorRightOutput;
int32_t encoderLeft, encoderRight;
float angleBalance, gyroBalance, gyroTurn;							//平衡倾角 平衡陀螺仪 转向陀螺仪
uint8_t flagForward, flagBackward, flagLeft, flagRight;	//蓝牙遥控相关的变量
uint8_t flagStop = 1;																		//停止标志位 默认停止
int32_t balancePWM, velocityPWM, dirPWM;
uint16_t batteryVoltage;

static bool isInit;

static void stabilizerTask(void* param);

static int32_t balanceControl(const float angle, const float gyro);
static int32_t velocityControl(const int32_t encoderLeft, const int32_t encoderRight);
static int32_t dirControl(const int32_t encoderLeft, const int32_t encoderRight, const float gyroTurn);
static uint8_t motorTurnOff(const float angle, const int voltage);
static uint16_t limitThrust(int32_t value);


void stabilizerInit(void)
{
	motorsInit();
	encoderInit();
	imu6Init();
	sensfusion6Init();
	controllerInit();
	
	xTaskCreate(stabilizerTask, (const char *)"STABILIZER",
              2*configMINIMAL_STACK_SIZE, NULL, /*Piority*/2, NULL);

  isInit = true;
}

bool stabilizerTest(void)
{
  bool pass = true;

	if (!isInit)
  {
    pass = false;
  }
	
  pass &= motorsTest();
	pass &= encoderTest();
  pass &= imu6Test();
  pass &= sensfusion6Test();
  pass &= controllerTest();

  return pass;
}

static void stabilizerTask(void* param)
{
	uint32_t attitudeCounter = 0;
  uint32_t lastWakeTime;
	
	vTaskSetApplicationTaskTag(0, (void *)TASK_STABILIZER_ID_NBR);
	
  /* Wait for the system to be fully started to start stabilization loop */
  systemWaitStart();
	
	lastWakeTime = xTaskGetTickCount();
	
	while (1)
	{
		vTaskDelayUntil(&lastWakeTime, F2T(IMU_UPDATE_FREQ));
		
		imu6Read(&gyro, &acc);
		if (imu6IsCalibrated())	
		{
			if (++attitudeCounter >= ATTITUDE_UPDATE_RATE_DIVIDER)
			{
				sensfusion6UpdateQ(gyro.x, gyro.y, gyro.z, acc.x, acc.y, acc.z, FUSION_UPDATE_DT);
				sensfusion6GetEulerRPY(&eulerRollActual, &eulerPitchActual, &eulerYawActual);
				attitudeCounter = 0;
			}
			#if defined(KALMAN_FILTER)
				angleBalance = Kalman_Filter(eulerPitchActual, gyro.y);
			#elif defined(COMPLEMENTARY_FILTER)
				angleBalance = Complementary_Filter(eulerPitchActual, gyro.y);	
			#else
				angleBalance = eulerPitchActual;
			#endif
			gyroBalance = gyro.y;
			gyroTurn = gyro.z;
			
			/* Direction Command */
			comamndGetControl(&flagForward, &flagBackward, &flagLeft, &flagRight);
			flagStop = ~(flagForward || flagBackward || flagLeft || flagRight);

			/* Balance Control */
			// encoder
			encoderCalculate(encoderLeft, encoderRight);			
			balancePWM = balanceControl(angleBalance, gyroBalance);
			velocityPWM = velocityControl(encoderLeft, encoderRight);
			dirPWM = dirControl(encoderLeft, encoderRight, gyroTurn);

			/* Motors Output */
			motorLeftOutput = limitThrust(balancePWM - velocityPWM + dirPWM);
			motorRightOutput = limitThrust(balancePWM - velocityPWM - dirPWM);

			if(motorTurnOff(angleBalance, batteryVoltage) == 0)
			{
				// motor direction
				if(motorLeftOutput < 0)
					motorsSetDir(MOTOR_LEFT, MOTOR_BACKWARD);
				else
					motorsSetDir(MOTOR_LEFT, MOTOR_FORWARD);
				
				if(motorRightOutput < 0)
					motorsSetDir(MOTOR_RIGHT, MOTOR_BACKWARD);
				else
					motorsSetDir(MOTOR_RIGHT, MOTOR_FORWARD);
				// PWM output
				motorsSetRatio(MOTOR_LEFT, abs(motorLeftOutput));
				motorsSetRatio(MOTOR_RIGHT, abs(motorRightOutput));
			}
		}	
	}
}

static int32_t balanceControl(const float angle,const float gyro)
{  
   static float bias;
	 int balance;
	 bias = angle + 3;																	//===求出平衡的角度中值 和机械相关
	 balance = 300*bias + gyro/2;												//===计算直立电机控制
	 return balance;
}

#if 1
static int32_t velocityControl(const int32_t encoderLeft, const int32_t encoderRight)
{
	static int Velocity, Encoder_Least, Encoder,Movement;
	static long Encoder_Integral;

	/*************** 遥控前进后退部 ***************/
	if(1 == flagForward) Movement = 70;								//===如果前进标志位置1 位移为负
	else if(1 == flagBackward) Movement = -70;				//===如果后退标志位置1 位移为正
	else  Movement = 0;

	/***************** 速度PI控制 ******************/	
	Encoder_Least = encoderLeft + encoderRight;				//===获取最新速度偏差
	Encoder *= 0.7;																		//===一阶低通滤波器       
	Encoder += Encoder_Least * 0.3;										//===一阶低通滤波器    

	Encoder_Integral += Encoder;                                  //===积分出位移 积分时间：5ms
	Encoder_Integral = Encoder_Integral + Movement;								//===接收遥控器数据，控制前进后退
	if(Encoder_Integral > 18000)  	Encoder_Integral = 18000;			//===积分限幅
	if(Encoder_Integral < -18000)	Encoder_Integral = -18000;			//===积分限幅	
	Velocity = Encoder * 40 + Encoder_Integral / 5;								//===速度控制	
	if(motorTurnOff(angleBalance, batteryVoltage) == 1)   Encoder_Integral = 0;    //===电机关闭后清除积分
	return Velocity;
}
#else 
static int32_t velocityControl(const int32_t encoderLeft, const int32_t encoderRight)
{

}


#endif
static int32_t dirControl(const int32_t encoderLeft, const int32_t encoderRight, const float gyroTurn)//转向控制
{
	static int Turn_Target,Turn,Encoder_temp,Turn_Convert=3,Turn_Count;
	int Turn_Bias, Turn_Amplitude = 110/2 + 20;     //===Way_Angle为滤波方法，当是1时，即由DMP获取姿态，Turn_Amplitude取大，卡尔曼和互补是，取小，因为这两种滤波算法效果稍差。
	static long Turn_Bias_Integral;
	/*************** 遥控左右旋转部 ***************/
	if(1==flagLeft || 1==flagRight)									//这一部分主要是根据旋转前的速度调整旋转的起始速度，增加小车的适应性
	{
		if(++Turn_Count == 1)
			Encoder_temp = abs(encoderLeft - encoderRight);

		Turn_Convert = 50 / Encoder_temp;
		if(Turn_Convert < 1)
			Turn_Convert = 1;
		if(Turn_Convert > 4)
			Turn_Convert = 4;
	}	
	else
	{
		Turn_Convert=1;
		Turn_Count=0;
		Encoder_temp=0;
	}
	
	if(1==flagLeft)	           Turn_Target += Turn_Convert;
	else if(1==flagRight)	     Turn_Target -= Turn_Convert;
	else Turn_Target=0;//
	
	if(Turn_Target>Turn_Amplitude)  Turn_Target=Turn_Amplitude;		//===转向速度限幅
	if(Turn_Target<-Turn_Amplitude) Turn_Target=-Turn_Amplitude;

	/*************** 转向PD控制器 ****************/
	Turn_Bias = encoderLeft - encoderRight - Turn_Target;					//===计算转向偏差  
	Turn = Turn_Bias*55 - gyroTurn/2;																//===结合Z轴陀螺仪进行PID控制
	if(motorTurnOff(angleBalance, batteryVoltage) == 1)   Turn_Bias_Integral=0;//===电机关闭后清除积分
	return Turn;
}

static uint8_t motorTurnOff(float angle, int voltage)
{
	u8 temp;

	/*
	 * 关闭电机: 倾角大于40度, Flag_Stop置1, 电压低于11.1V
	 */
	if(angle<-40 || angle>40 || 1==flagStop || voltage<1110)
	{
		temp=1;
		motorsSetDir(MOTOR_LEFT, MOTOR_STOP);
		motorsSetDir(MOTOR_RIGHT, MOTOR_STOP);
	}
	else
		temp=0;
	
	return temp;			
}

static uint16_t limitThrust(int32_t value)
{
  if(value > UINT16_MAX)
  {
    value = UINT16_MAX;
  }
  else if(value < -UINT16_MAX)
  {
    value = -UINT16_MAX;
  }

  return (int16_t)value;
}
