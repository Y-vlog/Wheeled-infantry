#ifndef Chassis_Control_H
#define Chassis_Control_H

#include "main.h"
#include "M3508_driver.h"
#include "arm_math.h"
#include "ins_task.h"
#include "Power_distribution.h"
#include "remote_control.h"
#include "gimbalControl.h"

#define CHASSiS_SPEED_PID_MODE 0 //PID_POSITION->0//PID_DELTA->1

#define CHASSIS_ROTATE_K  1.0f 

// 弧度转换系数：π/180 ≈ 0.0174533，替代 1/57.295779513
#define RADIAN_COEF_CHASSIS 0.017453292519943295f

#define Speed_K 4
/*小陀螺旋转速度*/
#define CHASSIS_GYRO_ROTATE_SPEED 2500
#define CHASSIS_MAX_XVECTOR_SPEED 4500
#define CHASSIS_MAX_YVECTOR_SPEED 4500

/*底盘pid相关宏*/
#define CHASSIS_ANGLEPID_KP 7.0f   //12
#define CHASSIS_ANGLEPID_KI 0.0f  //0.003
#define CHASSIS_ANGLEPID_KD 0.0f   //0.06
#define CHASSIS_SPEEDPID_KP 5.0f 
#define CHASSIS_SPEEDPID_KI 0.0f 
#define CHASSIS_SPEEDPID_KD 0.002f 


typedef struct {
    float sin;         // 夹角正弦值
    float cos;         // 夹角余弦值
    int16_t c_x;       // 底盘坐标系X方向速度（左右）
    int16_t c_y;       // 底盘坐标系Y方向速度（前后）
} ChassisVectorsTypeDef;


extern M3508_HandleTypeDef Wheel_M3508Handle[4];	//底盘四个电机
extern INS_t INS;

extern int16_t set_currentMA;
extern int16_t set_currentMB;
extern int16_t set_currentMC;
extern int16_t set_currentMD;

/**
  * @func			void Chassis_Init(void)
  * @brief          底盘驱动初始化
  * @param[in]      none
  * @retval         none
  */
extern void Chassis_Init(void);

/**
  * @func			void Chassis_WheatWheel_Solution(
  *						int16_t chassisXvector,
  *						int16_t chassisYvectory,
  *						int16_t chassisRotatevector,
  *						int rotateK)
  * @brief          麦轮底盘运动学解算
  * @brief          底盘坐标系(俯视图)：
	/\底盘正方向		 /\x(roll)
	||					||
	||					||
	||					||
		y(pitch)《========
		 
  * @param[in]      chassisXvector:底盘坐标系下，x方向下的速度
  * @param[in]      chassisYvectory:底盘坐标系下，y方向下的速度
  * @param[in]      chassisRotatevector:底盘(世界)坐标系下，底盘旋转的速度
  * @param[in]      rotateK:旋转系数(与底盘长款轴距有关k=a+b)
  * @retval         none
  */
extern void Chassis_WheatWheel_Solution(int16_t ChassisXvector, int16_t ChassisYvectory, int16_t ChassisRotatevector, float rotateK);

/**
  * @func			void ChassisGyro_Task(
  *						int16_t worldXvector,
  *						int16_t worldYvector,
  *						int rotateK,
  *						float c_angle)
  * @brief          底盘小陀螺任务
  * @param[in]      worldXvector：世界/云台坐标系下，x方向速度
  * @param[in]      worldYvector：世界/云台坐标系下，y方向速度
  * @param[in]      rotateK：旋转系数(与底盘长款轴距有关k=a+b)
  * @param[in]      c_angle：世界/云台坐标系与底盘坐标系的夹角编码器值 一般为云台与底盘
  * @retval         none
  */
extern void ChassisGyro_Task(int16_t World_Xvector,int16_t World_Yvector,int16_t World_Zrotate, float rotateK,int16_t C_Angle);

/**
  * @func			void ChassisPositionControl_Task(
  *						int16_t worldXvector,
  *						int16_t worldYvector,
  *						int rotateK,
  *						float32_t realangle,
  *						float32_t targetangle)
  * @brief          底盘跟随云台任务
  * @param[in]      worldXvector：世界/云台坐标系下，x方向速度
  * @param[in]      worldYvector：世界/云台坐标系下，y方向速度
  * @param[in]      rotateK：旋转系数(与底盘长款轴距有关k=a+b)
  * @param[in]      realangle：  世界/云台坐标系与底盘坐标系的实时夹角(yaw轴电机绝对编码器值)
  * @param[in]      targetangle：世界/云台坐标系与底盘坐标系的目标夹角(yaw轴电机绝对编码器值) 旋转后
  * @retval         none
  */
extern void ChassisPositionControl_Task(int16_t World_Xvector,int16_t World_Yvector,float rotateK,int16_t realangle,int16_t targetangle);

/**
  * @func		void Chassis_Follow(int16_t chassisXvector,int16_t chassisYvectory,int16_t chassisRotatevector,int16_t rotateK)
  * @brief      底盘跟随云台任务
  * @param[in]  chassisXvector:底盘坐标系下，x方向下的速度
  * @param[in]  chassisYvectory:底盘坐标系下，y方向下的速度
  * @param[in]  INS:底盘陀螺仪数据包
  * @param[in]  init_gambal_yaw:云台目标角度
  * @retval     void
  */
extern void Chassis_Follow(int16_t ChassisXvector, int16_t ChassisYvectory, float rotateK, INS_t* INS, float init_gambal_yaw);

#endif
