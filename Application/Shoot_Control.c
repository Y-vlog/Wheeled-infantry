#include "Shoot_Control.h"

/*拨盘电机句柄*/
M2006_HandleTypeDef up_trigger_motor;
/*摩擦轮电机句柄 */
M3508_HandleTypeDef FrictionWheel_motorHandle[2];
/*射击模式相关参数*/
Shoot_ModeMessageTypedef shoot_mode_message;

/**
  * @func			void Shoot_Init(void)
  * @brief          射击驱动初始化
  * @param[in]      none
  * @retval         none
  */
void Shoot_Init(void)
{
	/*摩擦轮电机初始化.拨弹电机初始化*/
    M2006_Init(&up_trigger_motor,0x06, PID_POSITION);
	M3508_Init(&FrictionWheel_motorHandle[0],0x07, PID_POSITION, 0);
	M3508_Init(&FrictionWheel_motorHandle[1],0x08, PID_POSITION, 0);
    
	/*拨弹信息初始化*/
	shoot_mode_message.valve = VALVE_CLOSE;
	shoot_mode_message.mode = -1;
}

void Shoot_FrictionWheelControl(int16_t setSpeed_trigger, int16_t setSpeed_Left, int16_t setSpeed_Right)
{
    /*读取摩擦轮和拨弹电机数据*/
    M2006_GetBasicData(&up_trigger_motor);
	M3508_GetBasicData(&FrictionWheel_motorHandle[0], 0);
	M3508_GetBasicData(&FrictionWheel_motorHandle[1], 0);

    int16_t setCurrent_Low   = M2006_SpeedLoopPIDController(&up_trigger_motor.PID_Speed_Control_Data, up_trigger_motor.Basic_Data->speed_rpm, setSpeed_trigger);
	int16_t setCurrent_Left  = M3508_SpeedLoopPIDController(&(FrictionWheel_motorHandle[0].PID_Speed_Control_Data), FrictionWheel_motorHandle[0].Basic_Data->speed_rpm, setSpeed_Left);
	int16_t setCurrent_Right = M3508_SpeedLoopPIDController(&(FrictionWheel_motorHandle[1].PID_Speed_Control_Data), FrictionWheel_motorHandle[1].Basic_Data->speed_rpm, setSpeed_Right);	

	CAN_cmd_gimbal_Frie(setCurrent_Low, setCurrent_Left, setCurrent_Right);	
}



