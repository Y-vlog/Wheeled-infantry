#include "gimbalControl.h"
#include "arm_math.h"

/*云台电机相关句柄*/
M6020_HandleTypeDef   Yaw_Gimbal_Motor;
M6020_HandleTypeDef   Pitch_Gimbal_Motor;

/*重力补偿相关参数*/
int Compensation_Factor;
float Pitch_cos;
float Pitch_sin;

///*云台yaw轴相关参数*/
//float Yaw_Beganangle;//以陀螺仪为参考

//int16_t d_ecd;
//float d_angle;

///*云台ptich轴相关参数*/
//int16_t Pitch_Beganecd;




/**
  * @func			void Gimbal_Init(void)
  * @brief          云台驱动初始化
  * @param[in]      none
  * @retval         none
  */
void Gimbal_Init(void)
{
	/*初始化云台相关电机(注意更改ID号)，PID电压控制*/
	M6020_Init(&Yaw_Gimbal_Motor, 0x01, PID_POSITION, CAN_CMD_VOLTAGE);
	M6020_Init(&Pitch_Gimbal_Motor, 0x02, PID_POSITION, CAN_CMD_VOLTAGE);
	
//	/*初始化pitch重力补偿相关参数*/
	Compensation_Factor = 4000;
	Pitch_cos = 0;
	Pitch_sin = 0;
//	
//	/*初始化云台yaw轴相关参数*/
//	Yaw_Beganangle=0.0f;
//	d_ecd=0;
//	d_angle=0;
//    
//	/*初始化云台pitch相关参数*/
//	Pitch_Beganecd=0;
}

/**
  * @func			int16_t Pitch_Gimbal_ControlOfECD(int16_t Remota_Control_ch3)
  * @brief          pitch轴串级pid控制任务,俯仰角限制解算
  * @param[in]      targetpostion：pitch轴遥控器
  * @retval         pitch轴串级pid输出
  */
int16_t Pitch_Gimbal_ControlOfECD(int16_t rc_3)
{
	M6020_GetBasicData(&Pitch_Gimbal_Motor);//先获取数据
    
   // 转换为0~8191的环形范围（处理负值和超范围值）
    int16_t encoder_val = rc_3 * Pitch_Sensitivity + Pitch_Began_Meddle_ecd;
    
    encoder_val %= ENCODER_MAX_VALUE;  // 取模运算缩小到-8192~8192
    if (encoder_val < 0)
    {
        encoder_val += ENCODER_MAX_VALUE;  // 负值转为正向环形值（如-1 → 8191）
    }
        
    int16_t Diff = encoder_val - Pitch_Gimbal_Motor.Basic_Data->ecd;
    
    // 处理超过半圈的情况，选择最短路径
    if (Diff > ENCODER_HALF_VALUE) //4096
    {
        Diff -= ENCODER_MAX_VALUE;
    } 
    
    else if (Diff < -ENCODER_HALF_VALUE)
    {
        Diff += ENCODER_MAX_VALUE;
    }
    
    // 注意：这里将目标值转换为基于当前位置的相对目标，而非绝对目标
    int16_t adjusted_target = Pitch_Gimbal_Motor.Basic_Data->ecd + Diff;  // 计算经过最短路径调整后的目标值
    
    if(adjusted_target > PITCH_MAX_ecd)
    {
        adjusted_target = PITCH_MAX_ecd;
    }
    
    else if(adjusted_target < PITCH_MIN_ecd)
    {
        adjusted_target = PITCH_MIN_ecd;
    }
    
    int16_t target_rpm = M6020_PositionLoopPIDController(
        &Pitch_Gimbal_Motor.PID_Position_Control_Data,
        Pitch_Gimbal_Motor.Basic_Data->ecd, 
        adjusted_target  // 使用调整后的目标值，确保PID按最短路径计算
    );
    
    int16_t output = M6020_SpeedLoopPIDController(
        &Pitch_Gimbal_Motor.PID_Speed_Control_Data, 
        Pitch_Gimbal_Motor.Basic_Data->speed_rpm, 
        target_rpm
    );

//    //电流限幅（保护电机）
//    if (output > MOTOR_CURRENT_LIMIT)
//    {
//        output = MOTOR_CURRENT_LIMIT;
//    }
//    
//    else if (output < -MOTOR_CURRENT_LIMIT)
//    {
//        output = -MOTOR_CURRENT_LIMIT;
//    }
	
    return output;
}

/**
  * @func			int16_t Pitch_Gimbal_Contro_Angle(int16_t Remota_Control_ch3)
  * @brief          小陀螺时候yaw移动角度
  * @param[in]      rc_2：遥控器数值 
  * @retval     
  */
static float angle_acc_Pitch = 0.0f;
int16_t Pitch_Gimbal_Contro_Angle(int16_t Remota_Control_ch3)
{

    // 遥控器死区
    const int deadzone = 30;

    // 有输入（不在死区）
    if(Remota_Control_ch3 > deadzone || Remota_Control_ch3 < -deadzone)
    {
        // 输入范围 -660~660 映射为增量
        float delta = (float)Remota_Control_ch3 * Pitch_Sensitivity * 0.005f;  

        angle_acc_Pitch = (int16_t)(delta + angle_acc_Pitch);
    }
    
    if(angle_acc_Pitch > PITCH_MAX_Angle)
    {
        angle_acc_Pitch = PITCH_MAX_Angle;
    }
    
    else if(angle_acc_Pitch < PITCH_MIN_Angle)
    {
        angle_acc_Pitch = PITCH_MIN_Angle;
    }

    // 没有输入 → 保持累积角度，不动
    return -angle_acc_Pitch;
}

/**
  * @func			int16_t Pitch_Gimbal_ControlOfIMU(int16_t real_angle ,int16_t target_angle)
  * @brief          Pitch轴超核(陀螺仪位于云台)
  * @brief          以陀螺仪角度(世界坐标)为参考
  * @param[in]      realangle：Pitch轴实时角度，范围[-180,180]
  * @param[in]      targetpostion：Pitch轴目标角度[-180,180]
  * @retval         yaw轴串级pid输出
  */
static int16_t Last_Output_PITCH = 0;
int16_t Pitch_Gimbal_ControlOfIMU(int16_t real_angle ,int16_t target_angle)
{
    M6020_GetBasicData(&Pitch_Gimbal_Motor);
    
    int16_t target_rpm = M6020_PositionLoopPIDController(
    &Pitch_Gimbal_Motor.PID_Position_Control_Data,
    real_angle, 
    target_angle
    );
    
    int16_t output = M6020_SpeedLoopPIDController(
    &Pitch_Gimbal_Motor.PID_Speed_Control_Data, 
    Pitch_Gimbal_Motor.Basic_Data->speed_rpm, 
    target_rpm
    );
    
    int16_t error_taget = (int16_t)M6020_Forcast_PID * (output - Last_Output_PITCH);
    
    Last_Output_PITCH = output;
    
    output = error_taget + output;
    
    //    //电流限幅（保护电机）
//    if (output > MOTOR_CURRENT_LIMIT)
//    {
//        output = MOTOR_CURRENT_LIMIT;
//    }
//    
//    else if (output < -MOTOR_CURRENT_LIMIT)
//    {
//        output = -MOTOR_CURRENT_LIMIT;
//    }
    
	return output;
}


/**
  * @func			int16_t Yaw_Gimbal_Contro_angle(int16_t Remota_Control_ch2)
  * @brief          小陀螺时候yaw移动角度
  * @param[in]      rc_2：遥控器数值 
  * @retval     
  */
static float angle_acc_Yaw = 0.0f;
int16_t Yaw_Gimbal_Contro_Angle(int16_t Remota_Control_ch2)
{
    // 静态变量：累积角度（可无限转）

    // 遥控器死区
    const int deadzone = 30;


    // 有输入（不在死区）
    if(Remota_Control_ch2 > deadzone || Remota_Control_ch2 < -deadzone)
    {
        // 输入范围 -660~660 映射为增量
        float delta = (float)Remota_Control_ch2 * YAW_Sensitivity * 0.005f;  

        angle_acc_Yaw += delta;
    }

    // 没有输入 → 保持累积角度，不动
    return -(int16_t)angle_acc_Yaw;
}

/**
  * @func			int16_t Yaw_Gimbal_ControlOfIMU(float realangle ,float targetangle)
  * @brief          yaw轴超核陀螺仪(陀螺仪位于云台)
  * @brief          以陀螺仪角度(世界坐标)为参考
  * @param[in]      realangle：yaw轴实时角度，范围[-180,180]
  * @param[in]      targetpostion：yaw轴目标角度[-180,180]
  * @retval         yaw轴串级pid输出
  */
static int16_t Last_Output_YAW = 0;
int16_t Yaw_Gimbal_ControlOfIMU(int16_t real_angle ,int16_t target_angle)
{
    M6020_GetBasicData(&Yaw_Gimbal_Motor);
    
    int16_t target_rpm = M6020_PositionLoopPIDController(
    &Yaw_Gimbal_Motor.PID_Position_Control_Data,
    real_angle, 
    target_angle
    );
    
    int16_t output = M6020_SpeedLoopPIDController(
    &Yaw_Gimbal_Motor.PID_Speed_Control_Data, 
    Yaw_Gimbal_Motor.Basic_Data->speed_rpm, 
    target_rpm
    );
    
    int16_t error_taget = (int16_t)M6020_Forcast_PID * (output - Last_Output_YAW);
    
    Last_Output_YAW = output;
    
    output = error_taget + output;
    
//    //电流限幅（保护电机）
//    if (output > MOTOR_CURRENT_LIMIT)
//    {
//        output = MOTOR_CURRENT_LIMIT;
//    }
//    
//    else if (output < -MOTOR_CURRENT_LIMIT)
//    {
//        output = -MOTOR_CURRENT_LIMIT;
//    }
    
	return output;
}

/**
  * @func			int16_t Yaw_Gimbal_6020_angle(M6020_HandleTypeDef * device)
  * @brief          yaw轴6020编码器角度
  * @param[in]      6020_HandleTypeDef * device：YAW轴电机数据
  * @retval         yaw轴转化[-180,180]
  */
int16_t Yaw_Gimbal_6020_angle(M6020_HandleTypeDef *device)
{
    if (device == NULL || device->Basic_Data == NULL)
    {
        return 0;
    }
    
    M6020_GetBasicData(device);
    
    int32_t beganecd = (int32_t)device->Basic_Data->ecd - (int32_t)YAW_Began_ecd;
    
    beganecd = beganecd % 8192;
    if (beganecd < 0) beganecd += 8192;
    
    int32_t angle = (beganecd * 360) / 8192;

    if (angle > 180)
    {
        angle = angle - 360;
    }
    
    angle = -angle;
    
    return (int16_t)angle;
}

/**
  * @func			int16_t GravityCompensation(const M6020_HandleTypeDef* device)
  * @brief          pitch轴重力补偿
  * @param[in]      device：6020电机句柄
  * @retval         补偿值
  */
int16_t GravityCompensation(const M6020_HandleTypeDef* device)
{
	float angle = ((float)device->Basic_Data->ecd - (float)Pitch_Began_Meddle_ecd) * 360 / 8191  * RADIAN_COEF_CHASSIS;
	float cos = 0.0f;
	float sin = 0.0f;
       
    cos = arm_cos_f32(angle);
    sin = arm_sin_f32(angle);
    
	Pitch_cos = cos;
	Pitch_sin = sin;

	return Compensation_Factor * Pitch_cos;
}

