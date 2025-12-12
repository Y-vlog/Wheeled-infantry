#ifndef GIMBALCONTEOL_H
#define GIMBALCONTEOL_H

#include "M6020_driver.h"
#include "chassis_control.h"

#define Kff_Yaw_Speed 1.2f       //Yaw轴M6020PID前馈
#define Kff_Pitch_Speed 1.2f     //Pitch轴M6020PID前馈

#define ENCODER_MAX_VALUE 8192
#define ENCODER_HALF_VALUE 4096

#define PITCH_MIN_ecd 3729    // 最小允许编码器
#define PITCH_MAX_ecd 4525    // 最大允许编码器

#define PITCH_MIN_Angle -18    // 最小允许角度
#define PITCH_MAX_Angle 28    // 最大允许角度

#define Pitch_Began_ecd 4150              //机械装配正方向编码器值
#define YAW_Began_ecd 723                 //机械装配正方向编码器值
#define MOTOR_CURRENT_LIMIT 4000          //6020电流最大值

#define YAW_Sensitivity 1.0f
#define Pitch_Sensitivity 0.5f

/*云台电机相关句柄*/
extern M6020_HandleTypeDef   Yaw_Gimbal_Motor;
extern M6020_HandleTypeDef Pitch_Gimbal_Motor;

/*重力补偿相关参数*/
extern int Compensation_Factor;
extern float Pitch_cos;
extern float Pitch_sin;

///*云台yaw轴相关参数*/
//extern float Yaw_Beganangle;
//extern int16_t Yaw_Beganecd;  //以编码器为参考
//extern char first_run_Yaw;
//extern int32_t target_position_Yaw;

///*云台ptich轴相关参数*/
//extern int16_t Pitch_Beganecd;

void Gimbal_Init(void);

/**
  * @func			int16_t Pitch_Gimbal_Task(int16_t targetpostion)
  * @brief          pitch轴串级pid控制任务
  * @param[in]      targetpostion：pitch轴目标绝对编码器值（0-8191） 以编码器为绝对
  * @retval         pitch轴串级pid输出
  */
extern int16_t Pitch_Gimbal_ControlOfECD(int16_t Remota_Control_ch3);

/**
  * @func			int16_t Pitch_Gimbal_Contro_Angle(int16_t Remota_Control_ch3)
  * @brief          小陀螺时候yaw移动角度
  * @param[in]      rc_2：遥控器数值 
  * @retval     
  */
extern int16_t Pitch_Gimbal_Contro_Angle(int16_t Remota_Control_ch3);

/**
  * @func			int16_t Pitch_Gimbal_ControlOfIMU(int16_t real_angle ,int16_t target_angle)
  * @brief          Pitch轴超核(陀螺仪位于云台)
  * @brief          以陀螺仪角度(世界坐标)为参考
  * @param[in]      realangle：Pitch轴实时角度，范围[-180,180]
  * @param[in]      targetpostion：Pitch轴目标角度[-180,180]
  * @retval         yaw轴串级pid输出
  */
extern int16_t Pitch_Gimbal_ControlOfIMU(int16_t real_angle ,int16_t target_angle);

/**
  * @func			int16_t Yaw_Gimbal_Contro_angle(int16_t rc_2)
  * @brief          小陀螺时候yaw移动角度
  * @param[in]      rc_2：遥控器数值 
  * @retval         yaw轴角度
  */
extern int16_t Yaw_Gimbal_Contro_Angle(int16_t Remota_Control_ch2);

/**
  * @func			int16_t Yaw_Gimbal_ControlOfIMU(float realangle ,float targetangle)
  * @brief          yaw轴超核陀螺仪(陀螺仪位于云台)
  * @brief          以陀螺仪角度(世界坐标)为参考
  * @param[in]      realangle：yaw轴实时角度，范围[-180,180]
  * @param[in]      targetpostion：yaw轴目标角度[-180,180]
  * @retval         yaw轴串级pid输出
  */
extern int16_t Yaw_Gimbal_ControlOfIMU(int16_t real_angle ,int16_t target_angle);

/**
  * @func			int16_t Yaw_Gimbal_6020_angle(M6020_HandleTypeDef * device)
  * @brief          yaw轴6020编码器角度
  * @param[in]      6020_HandleTypeDef * device：YAW轴电机数据
  * @retval         yaw轴转化[-180,180]
  */
extern int16_t Yaw_Gimbal_6020_angle(M6020_HandleTypeDef * device);


/**
  * @func			int16_t Update_ReferenceBeganeECD(M6020_HandleTypeDef* device)
  * @brief          更新参考角度
  * @brief          以绝对编码值为参考
  * @param[in]      device：6020电机句柄
  * @retval         yaw轴更新后的参考角度(绝对编码器值)
  */
extern int16_t Update_ReferenceBeganECD(M6020_HandleTypeDef* device);

/**
  * @func			int16_t GravityCompensation(const M6020_HandleTypeDef* device)
  * @brief          pitch轴重力补偿
  * @param[in]      device：6020电机句柄
  * @retval         补偿值
  */
extern int16_t GravityCompensation(const M6020_HandleTypeDef* device);


#endif
