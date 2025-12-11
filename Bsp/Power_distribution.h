#ifndef POWER_DISTRIBUTION_H
#define POWER_DISTRIBUTION_H

#include "main.h"
#include <math.h>
#include "Chassis_Control.h"

//#define CHASSIS_POWER_MAX 60.0f       // 最大允许功率60W
//#define POWER_ESTIMATE_K 0.0733f      // 功率估算系数:(电机效率) × (单位转换系数)
//#define POWER_LIMIT_SMOOTHING 0.2f    // 功率限制平滑系数

//#define CURRENT_RAW_MIN -16384       // 数据电流最小值
//#define CURRENT_RAW_MAX 16384        // 数据电流最大值
//#define CURRENT_ACTUAL_MIN -20.0f    // 实际电流最小值(A)
//#define CURRENT_ACTUAL_MAX 20.0f     // 实际电流最大值(A)

//#define POWER_PID_KP 5.0f
//#define POWER_PID_KI 0.2f
//#define POWER_PID_KD 0.1f
//#define POWER_PID_MAX_OUTPUT 1.0f    // 最大输出限制为1.0（不衰减）
//#define POWER_PID_MIN_OUTPUT 0.3f    // 最小输出限制为0.3（最大衰减）


//typedef struct {
//    float total_power;       // 总功率估算值
//    float attenuation_factor; // 电流衰减系数
//    float filtered_factor;    // 滤波后的衰减系数
//    uint32_t last_time;      // 上次计算时间
//    
//    // PID控制相关变量
//    float error;             // 当前误差
//    float last_error;        // 上次误差
//    float integral;          // 积分项
//    float derivative;        // 微分项
//} PowerControl_t;

//static PowerControl_t power_ctrl = {0}; // 功率控制实例


//extern float Power_PID_Calculate(float setpoint, float actual);

//extern void Power_all_Calculate(void);


typedef struct {
    float k1;          // 转速二次项系数
    float k2;          // 力矩二次项系数
    float a;           // 常数项
    float Ct;          // 力矩电流系数 (1.996e-6)
    float max_power;   // 单个电机最大功率限制
    float Kt;
} MotorPowerModel_t;

extern float CalculateMotorPower(int16_t I_cmd, int16_t speed_rpm);

extern int16_t SolveForTargetCurrent(float target_power, int16_t speed_rpm, int16_t original_I_cmd);

extern void PowerDistribution(int16_t* currents, int16_t* speeds);

#endif
