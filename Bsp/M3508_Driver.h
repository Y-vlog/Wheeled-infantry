#ifndef M3508_DRIVER_H
#define M3508_DRIVER_H

#include "main.h"
#include "pid.h"
#include "CAN_receive.h"
#include "M3508_Driver.h"

/*是否开启滑动平均滤波*/
#define USING_SMOOTHING_FILTER 0// 1为开启  0为关闭

#if (USING_SMOOTHING_FILTER == 1)
	/*位置PID*/							     //3
	#define M3508_ID1_SPEEDPID_POSITION_KP   10.0f
	#define M3508_ID1_SPEEDPID_POSITION_KI    0.1f
	#define M3508_ID1_SPEEDPID_POSITION_KD    0.0f

	#define M3508_ID2_SPEEDPID_POSITION_KP   13.5f
	#define M3508_ID2_SPEEDPID_POSITION_KI    1.1f
	#define M3508_ID2_SPEEDPID_POSITION_KD    0.0f

	#define M3508_ID3_SPEEDPID_POSITION_KP    10.0f
	#define M3508_ID3_SPEEDPID_POSITION_KI    0.1f
	#define M3508_ID3_SPEEDPID_POSITION_KD    0.0f

	#define M3508_ID4_SPEEDPID_POSITION_KP   10.0f
	#define M3508_ID4_SPEEDPID_POSITION_KI    0.1f
	#define M3508_ID4_SPEEDPID_POSITION_KD    0.0f
	
	/*增量PID*/
	#define M3508_ID1_SPEEDPID_DELTA_KP   10.0f
	#define M3508_ID1_SPEEDPID_DELTA_KI    0.1f
	#define M3508_ID1_SPEEDPID_DELTA_KD    0.0f

	#define M3508_ID2_SPEEDPID_DELTA_KP   13.5f
	#define M3508_ID2_SPEEDPID_DELTA_KI    1.1f
	#define M3508_ID2_SPEEDPID_DELTA_KD    0.0f

	#define M3508_ID3_SPEEDPID_DELTA_KP    10.0f
	#define M3508_ID3_SPEEDPID_DELTA_KI    0.1f
	#define M3508_ID3_SPEEDPID_DELTA_KD    0.0f

	#define M3508_ID4_SPEEDPID_DELTA_KP   10.0f
	#define M3508_ID4_SPEEDPID_DELTA_KI    0.1f
	#define M3508_ID4_SPEEDPID_DELTA_KD    0.0f	
#else
	//**************位置PID******************** */
	//				驱动轮
	#define M3508_ID1_SPEEDPID_POSITION_KP   3.8f//19.0f
	#define M3508_ID1_SPEEDPID_POSITION_KI   0.4f//0.02f
	#define M3508_ID1_SPEEDPID_POSITION_KD   0.01f//0.0f

	#define M3508_ID2_SPEEDPID_POSITION_KP   4.5f//21.0f//17.0f//15.0f
	#define M3508_ID2_SPEEDPID_POSITION_KI   0.5f//0.1f//0.009f//0.008f
	#define M3508_ID2_SPEEDPID_POSITION_KD   0.0f// 0.0f

	#define M3508_ID3_SPEEDPID_POSITION_KP   4.5f//17.0f
	#define M3508_ID3_SPEEDPID_POSITION_KI   0.5f//0.008f
	#define M3508_ID3_SPEEDPID_POSITION_KD   0.0f//0.0f

	#define M3508_ID4_SPEEDPID_POSITION_KP   3.8f//20.0f//19.0f
	#define M3508_ID4_SPEEDPID_POSITION_KI   0.4f//0.02f
	#define M3508_ID4_SPEEDPID_POSITION_KD   0.01f//0.0f

	//				摩擦轮
	#define FRICTION_ID1_SPEEDPID_POSITION_KP   5.4f//19.0f
	#define FRICTION_ID1_SPEEDPID_POSITION_KI   0.8f//0.02f
	#define FRICTION_ID1_SPEEDPID_POSITION_KD   0.05f//0.0f

	#define FRICTION_ID2_SPEEDPID_POSITION_KP   5.8f//21.0f//17.0f//15.0f
	#define FRICTION_ID2_SPEEDPID_POSITION_KI   0.8f//0.1f//0.009f//0.008f
	#define FRICTION_ID2_SPEEDPID_POSITION_KD   0.05f// 0.0f

	
	/***************增量PID***********************/
	//				驱动轮
	#define M3508_ID1_SPEEDPID_DELTA_KP    9.5f
	#define M3508_ID1_SPEEDPID_DELTA_KI    0.1f
	#define M3508_ID1_SPEEDPID_DELTA_KD    0.0f

	#define M3508_ID2_SPEEDPID_DELTA_KP    15.0f
	#define M3508_ID2_SPEEDPID_DELTA_KI    0.2f
	#define M3508_ID2_SPEEDPID_DELTA_KD    0.0f

	#define M3508_ID3_SPEEDPID_DELTA_KP    20.0f
	#define M3508_ID3_SPEEDPID_DELTA_KI    0.1f
	#define M3508_ID3_SPEEDPID_DELTA_KD    0.0f

	#define M3508_ID4_SPEEDPID_DELTA_KP    15.0f
	#define M3508_ID4_SPEEDPID_DELTA_KI    0.1f
	#define M3508_ID4_SPEEDPID_DELTA_KD    0.0f	
	// 			  摩擦轮
	#define FRICTION_ID1_SPEEDPID_DELTA_KP   2.5f//19.0f
	#define FRICTION_ID1_SPEEDPID_DELTA_KI   0.0f//0.02f
	#define FRICTION_ID1_SPEEDPID_DELTA_KD   0.001f//0.0fz

	#define FRICTION_ID2_SPEEDPID_DELTA_KP   2.5f//19.0f
	#define FRICTION_ID2_SPEEDPID_DELTA_KI   0.0f//0.02f
	#define FRICTION_ID2_SPEEDPID_DELTA_KD   0.001f//0.0f


#endif

typedef struct{
	
	uint8_t M3508_ID;//电机id
	
	const motor_measure_t *Basic_Data;//电流，速度，温度
	
	pid_type_def  PID_Speed_Control_Data;//速度环pid相关参数
	
	pid_type_def  PID_Currrent_Control_Data;//电流环pid相关参数
	
}M3508_HandleTypeDef;

/**
  * @func			void M3508_Init(uint8_t motorid,M3508_HandleTypeDef* device,enum PID_MODE mode)
  * @brief          3508电机相关参数初始化
  * @param[in]      device:3508电机句柄
  * @param[in]      M3508_ID:初始化电机id
  * @param[in]      PID_MODE:pid的控制模式(增量式，位置式) PID_DETAIL or PID_POSITION
  * @param[in]      选择初始化底盘M3508还是发射摩擦轮M3508
  * @retval         none
  */
extern void M3508_Init(M3508_HandleTypeDef* device,uint8_t M3508_ID,enum PID_MODE Mode, uint8_t ifWheel);
                
/**
  * @func			void M3508_GetBasicData(M3508_HandleTypeDef* device)
  * @brief          获取3508电机相关参数
  * @param[in]      device:3508电机句柄
  * @retval         none
  */
extern void M3508_GetBasicData(M3508_HandleTypeDef* device,uint8_t ifWheel);
/**
  * @func			int16_t M3508_SpeedLoopPIDController(pid_type_def *pid, fp32 ref, fp32 set)
  * @brief          3508速度环pid控制函数
  * @param[in]      pid:3508电机速度环pid句柄
  * @param[in]      ref:3508电机实际转速
  * @param[in]      set:3508电机目标转速
  * @retval         3508速度环pid输出值
  */
extern int16_t M3508_SpeedLoopPIDController(pid_type_def *pid, fp32 ref, fp32 set);

/**
  * @func			int16_t M3508_CurrentLoopPIDController(pid_type_def *pid, fp32 ref, fp32 set)
  * @brief          3508电流环pid控制函数
  * @param[in]      pid:3508电机电流环pid句柄
  * @param[in]      ref:3508电机实际电流
  * @param[in]      set:3508电机目标电流
  * @retval         3508电流环pid输出值
  */
extern int16_t M3508_CurrentLoopPIDController(pid_type_def *pid, fp32 ref, fp32 set);

#endif
