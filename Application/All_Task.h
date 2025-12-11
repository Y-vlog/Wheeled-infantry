#ifndef ALL_TASK_H
#define ALL_TASK_H

#include "main.h"
#include "arm_math.h"
#include "remote_control.h"
#include "Chassis_Control.h"
#include "GimbalControl.h"
#include "Shoot_Control.h"
#include "M2006_Driver.h"
#include "hipnuc_imu_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"


extern RC_ctrl_t rc_ctrl;

/**
	********************************************
	* @ControlMode        
	* 0-待机  1-陀螺  2-头追  3-分离  4-部署
	****************************
	* 
	****************************
	* 3-分离
	*   ch0-右侧左右摇杆 -- 底盘左右平移
	*   ch1-右侧前后摇杆 -- 底盘前后平移
	*   ch2-左侧左右摇杆 -- 云台yaw旋转
	*   ch3-左侧前后摇杆 -- 云台pitch俯仰
	*   ch4-左侧滚轮     -- 底盘yaw旋转
	********************************************
	*/

/**
  ********************************************
  * @ChassisMode  底盘驱动模式
	* @Control      摇杆 
	* 0-待机  1-陀螺  2-头追  3-分离  4-部署
  ********************************************
  */
extern void Chassis_Task(void const * argument);

/**
  ********************************************
  * @GimbalMode  云台驱动模式
	* @Control     摇杆 
	* 0-待机  1-陀螺  2-头追  3-分离  4-部署
  ********************************************
  */
extern void Gimbal_Task(void const * argument);

/**
  ********************************************
  * @FireMode  发射驱动模式
  * @Control   右侧拨杆  
	* 0-待机  1-停机  2-空转  3-发射
  ********************************************
  */
extern void Shoot_Task(void const * argument);

#endif
