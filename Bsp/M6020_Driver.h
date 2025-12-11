#ifndef M6020_DRIVER_H
#define M6020_DRIVER_H

#include "main.h"
#include "pid.h"
#include "CAN_receive.h"


//PID_Mode == PID_POSITION 位置式
/*YAW轴pid参数相关宏定义*/		//带裁判系统		//不带裁判系统
#define YAWM6020_POSITIONPID_POSITION_KP 20.0f//0.076f	//0.075f//0.07f//0.066f//0.065f  位置环PID
#define YAWM6020_POSITIONPID_POSITION_KI 0.0f//0.0f		//0.0015f//0.0013f//0.002f//0.002f
#define YAWM6020_POSITIONPID_POSITION_KD 0.0f//0.36f		//0.0f
#define YAWM6020_SPEEDPID_POSITION_KP    15.0f//600		//365.0f//360.0f//350.0f//300.0f  速度环PID
#define YAWM6020_SPEEDPID_POSITION_KI    0.0f//0.0f
#define YAWM6020_SPEEDPID_POSITION_KD    0.01f//0.0f

#define YAW_INTERGRAL_VALVE 50

/*PITCH轴pid参数相关宏定义*/			//带裁判系统		//不带裁判系统
#define PITCHM6020_POSITIONPID_POSITION_KP 10.5f//0.4f		//0.15f
#define PITCHM6020_POSITIONPID_POSITION_KI 0.005f//0.01f 	//0.0001f
#define PITCHM6020_POSITIONPID_POSITION_KD 4.0f//0.0f		//0.1f
#define PITCHM6020_SPEEDPID_POSITION_KP    8.0f//310.0f	//30.0f
#define PITCHM6020_SPEEDPID_POSITION_KI    0.02f//0.0f		//15.0f
#define PITCHM6020_SPEEDPID_POSITION_KD    0.002f//0.0f        //10.0f

#define PITCH_INTERGRAL_VALVE 60					

//PID_Mode == PID_DELTA 增量式
/*YAW轴pid参数相关宏定义*/	
#define YAWM6020_POSITIONPID_DELTA_KP 0.0f//0.076f	//0.075f//0.07f//0.066f//0.065f  位置环PID
#define YAWM6020_POSITIONPID_DELTA_KI 0.0f//0.0f		//0.0015f//0.0013f//0.002f//0.002f
#define YAWM6020_POSITIONPID_DELTA_KD 0.0f//0.36f		//0.0f
#define YAWM6020_SPEEDPID_DELTA_KP    0.0f//600		//365.0f//360.0f//350.0f//300.0f  速度环PID
#define YAWM6020_SPEEDPID_DELTA_KI    0.0f//0.0f
#define YAWM6020_SPEEDPID_DELTA_KD    0.0f//0.0f

/*PITCH轴pid参数相关宏定义*/			//带裁判系统		//不带裁判系统
#define PITCHM6020_POSITIONPID_DELTA_KP   0.0f//0.4f		//0.15f
#define PITCHM6020_POSITIONPID_DELTA_KI   0.001f//0.01f 	//0.0001f
#define PITCHM6020_POSITIONPID_DELTA_KD   0.0f//0.0f		//0.1f
#define PITCHM6020_SPEEDPID_DELTA_KP 	  0.0f//310.0f	//30.0f
#define PITCHM6020_SPEEDPID_DELTA_KI      0.0f//0.0f		//15.0f
#define PITCHM6020_SPEEDPID_DELTA_KD      0.0f//0.0f        //10.0f


enum CAN_CMD_MODE{
	 CAN_CMD_VOLTAGE=0xFF,
	 CAN_CMD_CURRENT=0xFE
};

typedef struct{
	
	uint8_t Motor_ID;//电机id
	
	enum CAN_CMD_MODE CMD_mode;
	
	const motor_measure_t *Basic_Data;//电流，速度，温度
	
	pid_type_def  PID_Position_Control_Data;//位置环pid相关参数
	
	pid_type_def  PID_Speed_Control_Data;   //速度环pid相关参数
	
	pid_type_def  PID_Current_Control_Data; //电流环pid相关参数
	
}M6020_HandleTypeDef;
/**
  * @func			void M6020_Init(uint8_t motorid,M6020_HandleTypeDef* device,enum PID_MODE pidmode,enum CAN_CMD_MODE cmdmode)
  * @brief          6020电机相关参数初始化
  * @param[in]      M6020_Motor:6020电机句柄
  * @param[in]      M6020_ID:初始化电机id
  * @param[in]      PID_Mode:pid的控制模式(增量式，位置式)
  * @param[in]      CMDmode:6020的控制模式( CAN_CMD_VOLTAGE：can发送电压控制信号，CAN_CMD_CURRENT：can发送电流控制信号)
  * @retval         none
  */
extern void M6020_Init(M6020_HandleTypeDef* M6020_Motor,uint8_t M6020_ID,enum PID_MODE PID_Mode,enum CAN_CMD_MODE CMDmode);

/**
  * @func			void M6020_GetBasicData(M6020_HandleTypeDef* device)
  * @brief          获取6020电机相关参数
  * @param[in]      device:6020电机句柄
  * @retval         none
  */
extern void M6020_GetBasicData(M6020_HandleTypeDef *M6020_Motor);

/**
  * @func			int16_t M6020_PositionLoopPIDController(pid_type_def *pid, fp32 ref, fp32 set)
  * @brief          6020位置环pid控制函数
  * @param[in]      pid:6020电机位置环pid句柄
  * @param[in]      ref:6020电机实际角度
  * @param[in]      set:6020电机目标角度
  * @retval         6020位置环pid输出值
  */
extern int16_t M6020_PositionLoopPIDController(pid_type_def *pid, fp32 ref, fp32 set);

/**
  * @func			int16_t M6020_SpeedLoopPIDController(pid_type_def *pid, fp32 ref, fp32 set)
  * @brief          2006速度环pid控制函数
  * @param[in]      pid:6020电机速度环pid句柄
  * @param[in]      ref:6020电机实际转速
  * @param[in]      set:6020电机目标转速
  * @retval         6020速度环pid输出值
  */
extern int16_t M6020_SpeedLoopPIDController(pid_type_def *pid, fp32 ref, fp32 set);

/**
  * @func			int16_t M6020_CurrentLoopPIDController(pid_type_def *pid, fp32 ref, fp32 set)
  * @brief          6020电流环pid控制函数
  * @param[in]      pid:6020电机电流环pid句柄
  * @param[in]      ref:6020电机实际电流
  * @param[in]      set:6020电机目标电流
  * @retval         6020电流环pid输出值
  */
extern int16_t M6020_CurrentLoopPIDController(pid_type_def *pid, fp32 ref, fp32 set);

#endif
