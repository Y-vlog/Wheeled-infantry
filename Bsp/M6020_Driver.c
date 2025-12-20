#include "M6020_driver.h"

/**
  * @func			void M6020_Init(uint8_t motorid,M6020_HandleTypeDef* device,enum PID_MODE pidmode,enum CAN_CMD_MODE cmdmode)
  * @brief          6020电机相关参数初始化
  * @param[in]      M6020_Motor:6020电机句柄
  * @param[in]      M6020_ID:初始化电机id
  * @param[in]      PID_Mode:pid的控制模式(增量式，位置式)
  * @param[in]      CMDmode:6020的控制模式( CAN_CMD_VOLTAGE：can发送电压控制信号，CAN_CMD_CURRENT：can发送电流控制信号)
  * @retval         none
  */
void M6020_Init(M6020_HandleTypeDef* M6020_Motor,uint8_t M6020_ID,enum PID_MODE PID_Mode,enum CAN_CMD_MODE CMDmode)
{
    if(M6020_Motor == NULL)
	{
		return;
	}
	/*初始化电机相关参数*/
	M6020_Motor->Motor_ID = M6020_ID;
	M6020_Motor->CMD_mode = CMDmode;
    
	/*初始化pid相关参数*/
	fp32 PID_Position[3] = {0.0f};
	fp32 PID_Speed[3]    = {0.0f};
    //fp32 PID_Current[3]  = {0.0f};
	fp32 Integralvalve   = 0;
    
	if(PID_Mode == PID_POSITION)
	{
		switch(0x200 + M6020_ID)
		{
			case CAN_YAW_MOTOR_ID:
			{	
                PID_Position[0] = YAWM6020_POSITIONPID_POSITION_KP;
                PID_Position[1] = YAWM6020_POSITIONPID_POSITION_KI;
                PID_Position[2] = YAWM6020_POSITIONPID_POSITION_KD;
                
                PID_Speed[0] = YAWM6020_SPEEDPID_POSITION_KP;
                PID_Speed[1] = YAWM6020_SPEEDPID_POSITION_KI;
                PID_Speed[2] = YAWM6020_SPEEDPID_POSITION_KD;
                /*初始化积分分离阀门值*/
                Integralvalve = YAW_INTERGRAL_VALVE;
                break;
            }
			case  CAN_PIT_MOTOR_ID:
			{	
                PID_Position[0] = PITCHM6020_POSITIONPID_POSITION_KP;
                PID_Position[1] = PITCHM6020_POSITIONPID_POSITION_KI;
                PID_Position[2] = PITCHM6020_POSITIONPID_POSITION_KD;
                
                PID_Speed[0] = PITCHM6020_SPEEDPID_POSITION_KP;
                PID_Speed[1] = PITCHM6020_SPEEDPID_POSITION_KI;
                PID_Speed[2] = PITCHM6020_SPEEDPID_POSITION_KD;
                /*初始化积分分离阀门值*/
                Integralvalve = PITCH_INTERGRAL_VALVE;
                    break;
            } 
            default :break;
            
		}
	}
    
    else if(PID_Mode == PID_DELTA)
    {
		switch(0x200 + M6020_ID)
		{
			case CAN_YAW_MOTOR_ID:
			{	
                PID_Position[0] = YAWM6020_POSITIONPID_DELTA_KP;
                PID_Position[1] = YAWM6020_POSITIONPID_DELTA_KI;
                PID_Position[2] = YAWM6020_POSITIONPID_DELTA_KD;
                
                PID_Speed[0] = YAWM6020_SPEEDPID_DELTA_KP;
                PID_Speed[1] = YAWM6020_SPEEDPID_DELTA_KI;
                PID_Speed[2] = YAWM6020_SPEEDPID_DELTA_KD;
                /*初始化积分分离阀门值*/
                Integralvalve = YAW_INTERGRAL_VALVE;
                break;
            }
			case  CAN_PIT_MOTOR_ID:
			{	
                PID_Position[0] = PITCHM6020_POSITIONPID_DELTA_KP;
                PID_Position[1] = PITCHM6020_POSITIONPID_DELTA_KI;
                PID_Position[2] = PITCHM6020_POSITIONPID_DELTA_KD;
                
                PID_Speed[0] = PITCHM6020_SPEEDPID_DELTA_KP;
                PID_Speed[1] = PITCHM6020_SPEEDPID_DELTA_KI;
                PID_Speed[2] = PITCHM6020_SPEEDPID_DELTA_KD;
                /*初始化积分分离阀门值*/
                Integralvalve = PITCH_INTERGRAL_VALVE;
                    break;
            } 
            default :break;
            
		}        
    }
	
	if(CMDmode == CAN_CMD_CURRENT)  //电流PID
	{
		PID_init(&(M6020_Motor->PID_Position_Control_Data), PID_Mode, PID_Position, 300,300);
		PID_init(&(M6020_Motor->PID_Speed_Control_Data),    PID_Mode, PID_Speed,    5000,5000);
	}
	else if(CMDmode == CAN_CMD_VOLTAGE)  //电压PID
	{
		PID_init(&(M6020_Motor->PID_Position_Control_Data), PID_Mode, PID_Position, 350,350);
		PID_init(&(M6020_Motor->PID_Speed_Control_Data),    PID_Mode, PID_Speed,    25000,25000);
	}
    
	//PID_init(&(M6020_Motor->PID_Current_Control_Data), PID_Mode, PID_Current,  16384,16384);
	M6020_Motor->PID_Position_Control_Data.integral_valve = Integralvalve;       /*积分分离阀门值*/
	
}

/**
  * @func			void M6020_GetBasicData(M6020_HandleTypeDef* device)
  * @brief          获取6020电机相关参数
  * @param[in]      device:6020电机句柄
  * @retval         none
  */
void M6020_GetBasicData(M6020_HandleTypeDef *M6020_Motor)
{
	if(M6020_Motor == NULL)
	{
		return;
	}
	/*电机id判断*/
	switch(M6020_Motor->Motor_ID + 0x202)
	{
	  case CAN_YAW_MOTOR_ID:
		M6020_Motor->Basic_Data = get_yaw_gimbal_motor_measure_point();
	  break;
      
      case CAN_PIT_MOTOR_ID:
		M6020_Motor->Basic_Data = get_pitch_gimbal_motor_measure_point();
	  break;
		default:break;
			
	}
}

/**
  * @func			int16_t M6020_PositionLoopPIDController(pid_type_def *pid, fp32 ref, fp32 set)
  * @brief          6020位置环pid控制函数
  * @param[in]      pid:6020电机位置环pid句柄
  * @param[in]      ref:6020电机实际角度
  * @param[in]      set:6020电机目标角度
  * @retval         6020位置环pid输出值
  */
int16_t M6020_PositionLoopPIDController(pid_type_def *pid, fp32 ref, fp32 set)
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
		//积分分离
		if( (pid->error[0]) < (pid->integral_valve) )
		{
			pid->Iout += pid->Ki * pid->error[0];
		}
		pid->Iout += pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
		//积分限幅
        LimitMax(pid->Iout, pid->max_iout);
        pid->out = pid->Pout + pid->Iout + pid->Dout;
		//输出限幅
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
  * @func			int16_t M6020_SpeedLoopPIDController(pid_type_def *pid, fp32 ref, fp32 set)
  * @brief          2006速度环pid控制函数
  * @param[in]      pid:6020电机速度环pid句柄
  * @param[in]      ref:6020电机实际转速
  * @param[in]      set:6020电机目标转速
  * @retval         6020速度环pid输出值
  */
int16_t M6020_SpeedLoopPIDController(pid_type_def *pid, fp32 ref, fp32 set)
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
  * @func			int16_t M6020_CurrentLoopPIDController(pid_type_def *pid, fp32 ref, fp32 set)
  * @brief          6020电流环pid控制函数
  * @param[in]      pid:6020电机电流环pid句柄
  * @param[in]      ref:6020电机实际电流
  * @param[in]      set:6020电机目标电流
  * @retval         6020电流环pid输出值
  */
int16_t M6020_CurrentLoopPIDController(pid_type_def *pid, fp32 ref, fp32 set)
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


	
