#include "M2006_Driver.h"

int Stop_Signal = 0;

int16_t int16_abs(int16_t value);
/**
  * @func			void M2006_Init(uint8_t motorid,M2006_HandleTypeDef* device,enum PID_MODE pidmode)
  * @brief          2006电机相关参数初始化
  * @param[in]      device:2006电机句柄
  * @param[in]      M2006_ID:初始化电机id  7
  * @param[in]      PID_MODE:pid的控制模式(增量式，位置式) PID_DETAIL or PID_POSITION
  * @retval         none
  */
void M2006_Init(M2006_HandleTypeDef* device,uint8_t M2006_ID,enum PID_MODE PID_Mode)
{
	/*初始化电机相关参数*/
	device->M2006_ID = M2006_ID;
	device->Expand_ecd = 0;
	device->Cylinder_Number = 0;
	device->Record_Last_ecd = 0;
    
    device->unlock_step = 0;
    device->is_unlocking = 0;
	
	/*初始化pid相关参数*/
	fp32 PID_Position[3] = {0.0f};
	fp32 PID_Speed[3]    = {0.0f};
//  fp32 PID_Current[3]  = {0.0f};
    
	if(PID_Mode == PID_POSITION)//位置式
	{
		switch(0x200 + M2006_ID)
		{
            
			case CAN_TRIGGER_UP_MOTOR_ID:
            PID_Position[0] = UP2006_POSITIONPID_KP;
            PID_Position[1] = UP2006_POSITIONPID_KI;
            PID_Position[2] = UP2006_POSITIONPID_KD;
            
			PID_Speed[0] = UP2006_SPEEDPID_KP;
			PID_Speed[1] = UP2006_SPEEDPID_KI;
			PID_Speed[2] = UP2006_SPEEDPID_KD;
			break;
            
			default : break;
		}
	}
	else if(PID_Mode == PID_DELTA)//增量式
	{
		switch(0x200 + M2006_ID)
		{          
			case CAN_TRIGGER_UP_MOTOR_ID:
            PID_Position[0] = UP2006_POSITIONPID_DELTA_KP;
            PID_Position[1] = UP2006_POSITIONPID_DELTA_KI;
            PID_Position[2] = UP2006_POSITIONPID_DELTA_KD;
            
			PID_Speed[0] = UP2006_SPEEDPID_DELTA_KP;
			PID_Speed[1] = UP2006_SPEEDPID_DELTA_KI;
			PID_Speed[2] = UP2006_SPEEDPID_DELTA_KD;
			break;
            
			default : break;
		}		
	}
    PID_init(&(device->PID_Position_Control_Data),PID_Mode, PID_Position, 16384,16384);//位置环
    PID_init(&(device->PID_Speed_Control_Data),   PID_Mode, PID_Speed,    16384,16384);//速度环
//  PID_init(&(device->PID_Currrent_Control_Data),PID_Mode, PID_Current,  16384,16384);
}

/**
  * @func			void M2006_GetBasicData(M2006_HandleTypeDef* device)
  * @brief          获取2006电机相关参数
  * @param[in]      device:2006电机句柄
  * @retval         none
  */
void M2006_GetBasicData(M2006_HandleTypeDef* device)
{
	if(device == NULL)
	{
		return;
	}
	switch(device->M2006_ID + 0x200)
	{
        case CAN_TRIGGER_UP_MOTOR_ID:
            device->Basic_Data = get_trigger_motor_measure_point();
        break;
		default : break;
			
	}
}

/**
  * @func			int16_t M2006_GetExpandECDData(M2006_HandleTypeDef* device)
  * @brief          获取2006电机编码器绝对角度(对编码器角度范围进行了扩充)
  * @brief          该函数对低速旋转的电机有效，高速旋转可能会存在丢圈问题
  * @param[in]      device:2006电机句柄
  * @retval         扩充后的编码器角度值
  */
int16_t M2006_GetExpandECDData(M2006_HandleTypeDef* device)
{
	if(device == NULL)
	{
		return 0;
	}
	//用上一次位置-这一次位置判断是否有突变
	int16_t d_ecd = device->Basic_Data->ecd - device->Record_Last_ecd;
    
	/*有突变记录一圈*/
	if(d_ecd > 8000)//反转突变
	{
		device->Cylinder_Number--;
	}
    
	else if(d_ecd < -8000)//正转突变
	{
		device->Cylinder_Number++;
	}
    
	else//未突变
	{
		/*计算范围拓展后的绝对编码器值*/ 
		if(device->Cylinder_Number >= 0)
		{
			device->Expand_ecd = device->Cylinder_Number*8191+device->Basic_Data->ecd;
		}
        
		else
		{
			device->Expand_ecd = device->Cylinder_Number*8191+8191-device->Basic_Data->ecd+8191;
		}                
	}
	/*更新上一次绝对编码器值*/
	device->Record_Last_ecd = device->Basic_Data->ecd;
	
	return device->Expand_ecd;
}

/**
  * @func			float M2006_AbsoluteEncoderToAngle(int absoluteencoder)
  * @brief          将2006电机(转子)编码器绝对角度值 转换为 (减速箱)编码器绝对角度值
  * @param[in]      absoluteencoder:2006电机(转子)编码器绝对角度值
  * @retval         (减速箱)编码器绝对角度值
  */
float M2006_AbsoluteEncoderToAngle(int absoluteencoder)
{
	//转子角度转换
	float rev_angle = (float)absoluteencoder * 360.0f / 8191;
	//减速箱角度转换
	rev_angle /= (float)M2006_REDUCTION_RATIO;
	
	return rev_angle;
}

//后续需要不断优化编写堵住程序
/**
  * @func			M2006_LockedMotorDetertion(M2006_HandleTypeDef* device)
  * @brief          2006电机堵转检测
  * @param[in]      device:2006电机句柄
  * @retval         Stop_Signal为1堵转，反则为之
  */
uint8_t M2006_LockedMotorDetertion(M2006_HandleTypeDef* device)
{
    M2006_GetBasicData(device);
    
    if(device == NULL || device->is_unlocking)
    {
        return 0;
    }
    
    static uint32_t last_stall_time = 0;
    uint32_t current_time = HAL_GetTick();
    
    // 防止频繁触发：两次堵转检测至少间隔2秒
    if(current_time - last_stall_time < 2000)
    {
        return 0;
    }
    
    // 更严格的堵转判据：大电流 + 低速度 + 持续一段时间
    static uint8_t stall_counter = 0;
    if(int16_abs(device->Basic_Data->given_current) > UP2006_UNLOCK_CURRENT && 
       int16_abs(device->Basic_Data->speed_rpm) < 1500 && 
       device->PID_Speed_Control_Data.set != 0)
    {
        stall_counter++;
        if(stall_counter >= 3) // 连续3次检测到堵转才确认
        {
            device->is_unlocking = 1;
            last_stall_time = current_time;
            stall_counter = 0;
            return 1;
        }
    }
    else
    {
        stall_counter = 0; // 重置计数器
    }
    
    return 0;
}
/**
  * @func			_userbool M2006_LockedMotorDetertion(M2006_HandleTypeDef* device)
  * @brief          2006电机解锁堵转
  * @param[in]      device:2006电机句柄
  * @retval         在原来位置来回滑动两次
  */
void M2006_UnlockMotor(M2006_HandleTypeDef* device) 
{
    if(device == NULL || !device->is_unlocking) 
    {
        return;
    }
    
    M2006_GetBasicData(device);
    
    static uint8_t is_initialized = 0;
    static uint32_t step_start_time = 0;
    static uint8_t current_protection_triggered = 0;
    
    // 初始化解锁过程
    if(!is_initialized)
    {
        step_start_time = HAL_GetTick();
        device->unlock_step = 1;
        device->unlock_speed = 2000;      // 使用较低速度避免电流过大
        device->step_duration = 500;      // 每个步骤持续时间500ms
        current_protection_triggered = 0;
        is_initialized = 1;
        
        // 首次进入时先停顿一下，让系统稳定
        UP_TRIGGER_SPEED = 0;
        return;
    }
    
    uint32_t current_time = HAL_GetTick();
    uint32_t elapsed_time = current_time - step_start_time;
    
    // 电流保护检测：如果在解锁过程中电流过大，提前结束当前步骤
    if(device->Basic_Data->given_current > UP2006_UNLOCKING_CURRENT && elapsed_time > 50) 
    {
        if(!current_protection_triggered)
        {
            current_protection_triggered = 1;
            // 电流过大，立即减速并提前进入下一步
            UP_TRIGGER_SPEED = 0;
            device->unlock_step++;
            step_start_time = current_time;
            
            // 如果已经完成所有步骤
            if(device->unlock_step > 4)
            {
                device->is_unlocking = 0;
                device->unlock_step = 0;
                is_initialized = 0;
                UP_TRIGGER_SPEED = -UP2006_SPEED;
            }
            return;
        }
    }
    else
    {
        current_protection_triggered = 0; // 重置电流保护标志
    }
    
    // 根据步骤设置运动方向
    int16_t target_speed = 0;
    switch(device->unlock_step)
    {
        case 1: // 第一步：反向运动（与正常运动方向相反）
            target_speed = device->unlock_speed;
            break;
            
        case 2: // 第二步：正向运动（回到正常运动方向）
            target_speed = -device->unlock_speed;
            break;
            
        case 3: // 第三步：再次反向运动
            target_speed = device->unlock_speed;
            break;
            
        case 4: // 第四步：再次正向运动
            target_speed = -device->unlock_speed;
            break;
            
        default:
            target_speed = 0;
            break;
    }
    
    // 速度渐变，避免突变导致电流冲击
    static int16_t current_speed = 0;
    if(abs(target_speed - current_speed) > 100)
    {
        // 速度渐变，每次变化不超过100
        if(target_speed > current_speed)
            current_speed += 100;
        else
            current_speed -= 100;
    }
    else
    {
        current_speed = target_speed;
    }
    
    UP_TRIGGER_SPEED = current_speed;
    
    // 检查当前步骤是否完成（时间到达）
    if(elapsed_time >= device->step_duration)
    {
        device->unlock_step++;
        step_start_time = current_time;
        current_speed = 0; // 重置速度渐变
        
        // 步骤间添加短暂停顿
        UP_TRIGGER_SPEED = 0;
        
        if(device->unlock_step > 4)
        {
            // 解锁完成
            device->is_unlocking = 0;
            device->unlock_step = 0;
            is_initialized = 0;
            UP_TRIGGER_SPEED = -UP2006_SPEED;
        }
    }
}
/**
  * @func			int16_t M2006_PositionLoopPIDController(pid_type_def *pid, fp32 ref, fp32 set)
  * @brief          2006位置环pid控制函数
  * @param[in]      pid:2006电机位置环pid句柄
  * @param[in]      ref:2006电机实际角度
  * @param[in]      set:2006电机目标角度
  * @retval         2006位置环pid输出值
  */
int16_t M2006_PositionLoopPIDController(pid_type_def *pid, fp32 ref, fp32 set)
{
	if (pid == NULL)
    {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = set - ref;
    if (pid->mode == PID_POSITION)
    {
        pid->Pout = pid->Kp * pid->error[0];
	    pid->Iout += pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        LimitMax(pid->Iout, pid->max_iout);
        pid->out = pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
		
		return pid->out;
    }
    else if (pid->mode == PID_DELTA)
    {
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        pid->out += pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
		
		return pid->out;
    }
	else
	{
		return 0;
	}
}

/**
  * @func			int16_t M2006_SpeedLoopPIDController(pid_type_def *pid, fp32 ref, fp32 set)
  * @brief          2006速度环pid控制函数
  * @param[in]      pid:2006电机速度环pid句柄
  * @param[in]      ref:2006电机实际转速
  * @param[in]      set:2006电机目标转速
  * @retval         2006速度环pid输出值
  */
int16_t M2006_SpeedLoopPIDController(pid_type_def *pid, fp32 ref, fp32 set)
{
	if (pid == NULL)
    {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = set - ref;
    if (pid->mode == PID_POSITION)
    {
        pid->Pout = pid->Kp * pid->error[0];
        pid->Iout += pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        LimitMax(pid->Iout, pid->max_iout);
        pid->out = pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
		
		return pid->out;
    }
    else if (pid->mode == PID_DELTA)
    {
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        pid->out += pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
		
		return pid->out;
    }
	else
	{
		return 0;
	}
}

/**
  * @func			int16_t M2006_CurrentLoopPIDController(pid_type_def *pid, fp32 ref, fp32 set)
  * @brief          2006电流环pid控制函数
  * @param[in]      pid:2006电机电流环pid句柄
  * @param[in]      ref:2006电机实际电流
  * @param[in]      set:2006电机目标电流
  * @retval         2006电流环pid输出值
  */
int16_t M2006_CurrentLoopPIDController(pid_type_def *pid, fp32 ref, fp32 set)
{
	if (pid == NULL)
    {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = set - ref;
    if (pid->mode == PID_POSITION)
    {
        pid->Pout = pid->Kp * pid->error[0];
		pid->Iout += pid->Ki * pid->error[0];	
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        LimitMax(pid->Iout, pid->max_iout);
        pid->out = pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
		
		return pid->out;
    }
    else if (pid->mode == PID_DELTA)
    {
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        pid->out += pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
		
		return pid->out;
    }
	else
	{
		return 0;
	}
}

/**
  * @func			int16_t int16_abs(int16_t value)
  * @brief          返回int16_t数据的绝对值
  * @param[in]      value ： 需要取绝对值的数据
  * @retval         返回int16_t数据的绝对值
  */
int16_t int16_abs(int16_t value) 
{
    // 对于普通值，直接通过符号判断返回绝对值
    return (value < 0) ? -value : value;
}
