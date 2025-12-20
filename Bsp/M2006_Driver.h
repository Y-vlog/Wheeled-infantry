#ifndef M2006_DRIVER_H
#define M2006_DRIVER_H

#include "main.h"
#include "pid.h"
#include "CAN_receive.h"
#include "arm_math.h"
#include "math.h"
#include <stdlib.h>

#define M2006_REDUCTION_RATIO 36.000000 //减速比36:1
#define UP2006_SPEED -1980
#define UP2006_UNLOCK_CURRENT 5500     // 检测堵转中的电流保护阈值
#define UP2006_UNLOCKING_CURRENT 2500  // 解锁过程中的电流保护阈值

/*速度PID参数*/
#define UP2006_SPEEDPID_KP 8.0f
#define UP2006_SPEEDPID_KI 0.005f
#define UP2006_SPEEDPID_KD 0.001f

/*位置PID参数*/
#define UP2006_POSITIONPID_KP 3.0f
#define UP2006_POSITIONPID_KI 0.005f
#define UP2006_POSITIONPID_KD 0.0005f

/*增量式PID参数*/
#define UP2006_SPEEDPID_DELTA_KP 0.0f
#define UP2006_SPEEDPID_DELTA_KI 0.0f
#define UP2006_SPEEDPID_DELTA_KD 0.0f

#define UP2006_POSITIONPID_DELTA_KP 0.0f
#define UP2006_POSITIONPID_DELTA_KI 0.0f
#define UP2006_POSITIONPID_DELTA_KD 0.0f

extern int16_t UP_TRIGGER_SPEED;


typedef struct{
	
	uint8_t M2006_ID;//电机id
	
	int  Expand_ecd;//电机减速箱旋转轴 绝对编码器值
	
	int  Cylinder_Number;//旋转圈数
	
	int16_t Record_Last_ecd;//这个为转子的上一次的绝对编码器值
    
    uint8_t unlock_step;       // 解锁步骤: 0-未解锁 1~4:解锁步骤
    uint8_t is_unlocking;      // 解锁标志
    uint32_t unlock_start_time;// 每个步骤开始的时间戳
    uint16_t step_duration;    // 每个步骤持续时间(ms)
    int16_t unlock_speed;      // 解锁速度
	
//	int16_t record_exp_ecdboxs[M2006_RECORD_EXP_ECDBOSX_LENGTH];//记录转子绝对编码器值
	
	const motor_measure_t *Basic_Data;//电流，速度，温度
	
    pid_type_def  PID_Position_Control_Data;//位置环pid相关参数
	
	pid_type_def  PID_Speed_Control_Data;//速度环pid相关参数
	
	pid_type_def  PID_Currrent_Control_Data;//电流环pid相关参数
	
}M2006_HandleTypeDef;


/**
  * @func			void M2006_Init(uint8_t motorid,M2006_HandleTypeDef* device,enum PID_MODE pidmode)
  * @brief          2006电机相关参数初始化
  * @param[in]      device:2006电机句柄
  * @param[in]      M2006_ID:初始化电机id  7
  * @param[in]      PID_MODE:pid的控制模式(增量式，位置式) PID_DETAIL or PID_POSITION
  * @retval         none
  */
extern void M2006_Init(M2006_HandleTypeDef* device,uint8_t M2006_ID,enum PID_MODE PID_Mode);

/**
  * @func			void M2006_GetBasicData(M2006_HandleTypeDef* device)
  * @brief          获取2006电机相关参数
  * @param[in]      device:2006电机句柄
  * @retval         none
  */
extern void M2006_GetBasicData(M2006_HandleTypeDef* device);

/**
  * @func			int16_t M2006_GetExpandECDData(M2006_HandleTypeDef* device)
  * @brief          获取2006电机编码器绝对角度(对编码器角度范围进行了扩充)
  * @brief          该函数对低速旋转的电机有效，高速旋转可能会存在丢圈问题
  * @param[in]      device:2006电机句柄
  * @retval         扩充后的编码器角度值
  */
extern int16_t M2006_GetExpandECDData(M2006_HandleTypeDef* device);

/**
  * @func			float M2006_AbsoluteEncoderToAngle(int absoluteencoder)
  * @brief          将2006电机(转子)编码器绝对角度值 转换为 (减速箱)编码器绝对角度值
  * @param[in]      absoluteencoder:2006电机(转子)编码器绝对角度值
  * @retval         (减速箱)编码器绝对角度值
  */
extern float M2006_AbsoluteEncoderToAngle(int absoluteencoder);

/**
  * @func			M2006_LockedMotorDetertion(M2006_HandleTypeDef* device)
  * @brief          2006电机堵转检测
  * @param[in]      device:2006电机句柄
  * @retval         Stop_Signal为1堵转，反则为之
  */
extern uint8_t M2006_LockedMotorDetertion(M2006_HandleTypeDef* device);
/**
  * @func			M2006_LockedMotorDetertion(M2006_HandleTypeDef* device)
  * @brief          2006电机解锁堵转
  * @param[in]      device:2006电机句柄
  * @retval         在原来位置来回滑动两次
  */
extern void M2006_UnlockMotor(M2006_HandleTypeDef* device);

/**
  * @func			int16_t M2006_PositionLoopPIDController(pid_type_def *pid, fp32 ref, fp32 set)
  * @brief          2006位置环pid控制函数
  * @param[in]      pid:2006电机位置环pid句柄
  * @param[in]      ref:2006电机实际角度
  * @param[in]      set:2006电机目标角度
  * @retval         2006位置环pid输出值
  */
extern int16_t M2006_PositionLoopPIDController(pid_type_def *pid, fp32 ref, fp32 set);
/**
  * @func			int16_t M2006_SpeedLoopPIDController(pid_type_def *pid, fp32 ref, fp32 set)
  * @brief          2006速度环pid控制函数
  * @param[in]      pid:2006电机速度环pid句柄
  * @param[in]      ref:2006电机实际转速
  * @param[in]      set:2006电机目标转速
  * @retval         2006速度环pid输出值
  */
extern int16_t M2006_SpeedLoopPIDController(pid_type_def *pid, fp32 ref, fp32 set);

/**
  * @func			int16_t M2006_CurrentLoopPIDController(pid_type_def *pid, fp32 ref, fp32 set)
  * @brief          2006电流环pid控制函数
  * @param[in]      pid:2006电机电流环pid句柄
  * @param[in]      ref:2006电机实际电流
  * @param[in]      set:2006电机目标电流
  * @retval         2006电流环pid输出值
  */
extern int16_t M2006_CurrentLoopPIDController(pid_type_def *pid, fp32 ref, fp32 set);

/**
  * @func			int16_t int16_abs(int16_t value)
  * @brief          返回int16_t数据的绝对值
  * @param[in]      value ： 需要取绝对值的数据
  * @retval         返回int16_t数据的绝对值
  */
extern int16_t int16_abs(int16_t value);

#endif
