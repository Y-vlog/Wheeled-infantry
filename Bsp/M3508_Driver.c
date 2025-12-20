#include "M3508_Driver.h"



/**
  * @func			void M3508_Init(uint8_t motorid,M3508_HandleTypeDef* device,enum PID_MODE mode)
  * @brief          3508电机相关参数初始化
  * @param[in]      device:3508电机句柄
  * @param[in]      M3508_ID:初始化电机id
  * @param[in]      PID_MODE:pid的控制模式(增量式，位置式) PID_DETAIL or PID_POSITION
  * @param[in]      选择初始化底盘M3508 1 还是发射摩擦轮M3508 0
  * @retval         none
  */
void M3508_Init(M3508_HandleTypeDef* device,uint8_t M3508_ID,enum PID_MODE Mode, uint8_t ifWheel)
{
    if(device == NULL)
    {
        return ;
    }
	//初始化电机id
	device->M3508_ID = M3508_ID;
    
	//初始化pid
	fp32 PID_SPEED[3]   = {0};         //速度环pid参数
	fp32 PID_CURRENT[3] = {0};         //电流环pid参数
    
    if(ifWheel == 1) //底盘M3508
    {
        if(Mode == PID_POSITION)
        {
          switch(0x200 + M3508_ID)
          {
            case CAN_3508_M1_ID:
            PID_SPEED[0] = M3508_ID1_SPEEDPID_POSITION_KP;
            PID_SPEED[1] = M3508_ID1_SPEEDPID_POSITION_KI;
            PID_SPEED[2] = M3508_ID1_SPEEDPID_POSITION_KD;
              break;

            case CAN_3508_M2_ID:
            PID_SPEED[0] = M3508_ID2_SPEEDPID_POSITION_KP;
            PID_SPEED[1] = M3508_ID2_SPEEDPID_POSITION_KI;
            PID_SPEED[2] = M3508_ID2_SPEEDPID_POSITION_KD;
              break;

            case CAN_3508_M3_ID:
            PID_SPEED[0] = M3508_ID3_SPEEDPID_POSITION_KP;
            PID_SPEED[1] = M3508_ID3_SPEEDPID_POSITION_KI;
            PID_SPEED[2] = M3508_ID3_SPEEDPID_POSITION_KD;
              break;

            case CAN_3508_M4_ID:
            PID_SPEED[0] = M3508_ID4_SPEEDPID_POSITION_KP;
            PID_SPEED[1] = M3508_ID4_SPEEDPID_POSITION_KI;
            PID_SPEED[2] = M3508_ID4_SPEEDPID_POSITION_KD;
              break;
            
            case CAN_3508_M5_ID:break;
            case CAN_3508_M6_ID:break;
            case CAN_3508_M7_ID:break;
            case CAN_3508_M8_ID:break;
            
            default :break;
          }
        }
    else if(Mode == PID_DELTA)
    {
      switch(0x200 + M3508_ID)
      {
        case CAN_3508_M1_ID:
        PID_SPEED[0] = M3508_ID1_SPEEDPID_DELTA_KP;
        PID_SPEED[1] = M3508_ID1_SPEEDPID_DELTA_KI;
        PID_SPEED[2] = M3508_ID1_SPEEDPID_DELTA_KD;
          break;

        case CAN_3508_M2_ID:
        PID_SPEED[0] = M3508_ID2_SPEEDPID_DELTA_KP;
        PID_SPEED[1] = M3508_ID2_SPEEDPID_DELTA_KI;
        PID_SPEED[2] = M3508_ID2_SPEEDPID_DELTA_KD;
          break;

        case CAN_3508_M3_ID:
        PID_SPEED[0] = M3508_ID3_SPEEDPID_DELTA_KP;
        PID_SPEED[1] = M3508_ID3_SPEEDPID_DELTA_KI;
        PID_SPEED[2] = M3508_ID3_SPEEDPID_DELTA_KD;
          break;

        case CAN_3508_M4_ID:
        PID_SPEED[0] = M3508_ID4_SPEEDPID_DELTA_KP;
        PID_SPEED[1] = M3508_ID4_SPEEDPID_DELTA_KI;
        PID_SPEED[2] = M3508_ID4_SPEEDPID_DELTA_KD;
          break;
        
        case CAN_3508_M5_ID:break;
        case CAN_3508_M6_ID:break;
        case CAN_3508_M7_ID:break;
        case CAN_3508_M8_ID:break;
        
        default :break;
      }		
    }
  }
    
  else  //摩擦轮M3508
  {
    if(Mode == PID_POSITION)
    {
      switch(0x200 + M3508_ID)
      {
        case CAN_TRIGGER_FIRE_MOTOR_Lift:
        PID_SPEED[0] = FRICTION_ID1_SPEEDPID_POSITION_KP;
        PID_SPEED[1] = FRICTION_ID1_SPEEDPID_POSITION_KI;
        PID_SPEED[2] = FRICTION_ID1_SPEEDPID_POSITION_KD;
          break;
        
        case CAN_TRIGGER_FIRE_MOTOR_Right:
        PID_SPEED[0] = FRICTION_ID2_SPEEDPID_POSITION_KP;
        PID_SPEED[1] = FRICTION_ID2_SPEEDPID_POSITION_KI;
        PID_SPEED[2] = FRICTION_ID2_SPEEDPID_POSITION_KD;
          break;
        
        default :break;
      }
    }
    else if(Mode == PID_DELTA)
    {
      switch(0x200 + M3508_ID)
      {
        case CAN_TRIGGER_FIRE_MOTOR_Lift:
        PID_SPEED[0] = FRICTION_ID1_SPEEDPID_DELTA_KP;
        PID_SPEED[1] = FRICTION_ID1_SPEEDPID_DELTA_KI;
        PID_SPEED[2] = FRICTION_ID1_SPEEDPID_DELTA_KD;
          break;
        
        case CAN_TRIGGER_FIRE_MOTOR_Right:
        PID_SPEED[0] = FRICTION_ID2_SPEEDPID_DELTA_KP;
        PID_SPEED[1] = FRICTION_ID2_SPEEDPID_DELTA_KI;
        PID_SPEED[2] = FRICTION_ID2_SPEEDPID_DELTA_KD;
          break;
        
        default :break;
      }		
    }    
  }
	PID_init(&(device->PID_Speed_Control_Data), Mode, PID_SPEED, 16384,16384);       //速度环PID初始化
	PID_init(&(device->PID_Currrent_Control_Data), Mode, PID_CURRENT, 16384,16384);  //电流环PID初始化
	
}
/**
  * @func			void M3508_GetBasicData(M3508_HandleTypeDef* device,uint8_t ifWheel)
  * @brief          获取3508电机相关参数
  * @param[in]      device:3508电机句柄 
  * @param[in]      选择读取底盘M3508 1 还是发射摩擦轮M3508 0
  * @retval         none
  */
void M3508_GetBasicData(M3508_HandleTypeDef* device,uint8_t ifWheel)
{
	if(device==NULL)
	{
		return;
	}
	/*电机id判读底盘还是摩擦轮*/
    if(ifWheel == 1)
    {
        switch(device->M3508_ID+0x200)
        {
            case CAN_3508_M1_ID:
            case CAN_3508_M2_ID:
            case CAN_3508_M3_ID:
            case CAN_3508_M4_ID: 
            {
                device->Basic_Data= get_chassis_motor_measure_point(device->M3508_ID - 1);
            }break;
            default:break;
        }   
    }

    else
    {
        switch(device->M3508_ID+0x200)
        {
            case CAN_TRIGGER_FIRE_MOTOR_Lift:
            case CAN_TRIGGER_FIRE_MOTOR_Right:
            {
                device->Basic_Data = get_Fire_motor_measure_point(device->M3508_ID + 2);
            }break;
            default:break;
        }
    }
}


/**
  * @func			int16_t M3508_SpeedLoopPIDController(pid_type_def *pid, fp32 ref, fp32 set)
  * @brief          3508速度环pid控制函数
  * @param[in]      pid:3508电机速度环pid句柄
  * @param[in]      ref:3508电机实际转速
  * @param[in]      set:3508电机目标转速
  * @retval         3508速度环pid输出值
  */
int16_t M3508_SpeedLoopPIDController(pid_type_def *pid, fp32 ref, fp32 set)
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
  * @func			int16_t M3508_CurrentLoopPIDController(pid_type_def *pid, fp32 ref, fp32 set)
  * @brief          3508电流环pid控制函数
  * @param[in]      pid:3508电机电流环pid句柄
  * @param[in]      ref:3508电机实际电流
  * @param[in]      set:3508电机目标电流
  * @retval         3508电流环pid输出值
  */
int16_t M3508_CurrentLoopPIDController(pid_type_def *pid, fp32 ref, fp32 set)
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

