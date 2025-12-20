#include "All_Task.h"

/**
	********************************************
	* @ControlMode        
	* 0-待机  1-陀螺  2-头追  3-分离  4-部署
	****************************
	* 
	****************************
	* 3-分离
	*   ch0-右侧左右摇杆 -- 底盘左右平移
	*   ch1-右侧前后摇杆 -- 底盘前后平移
	*   ch2-左侧左右摇杆 -- 云台yaw旋转
	*   ch3-左侧前后摇杆 -- 云台pitch俯仰
	*   ch4-左侧滚轮     -- 底盘yaw旋转
	********************************************
	*/

/**
  ********************************************
  * @ChassisMode  底盘驱动模式
	* @Control      摇杆 
	* 0-待机  1-陀螺  2-头追  3-分离  4-部署
  ********************************************
  */
#define ChassisWait   0
#define ChassisGroy   1
#define ChassisHead   3
#define ChassisSepa   2
#define ChassisDepoly 4

uint8_t ChassisMode = ChassisWait;

static float prev_xSpeed = 0;
static float prev_ySpeed = 0;
static float prev_zSpeed = 0;

float INS_Total = 0;

// 滤波系数 (0.1-0.3之间，值越小滤波越强)
const float alpha = 0.003f;

void Chassis_Task(void const * argument)
{
    
    float raw_xSpeed = 0;
    float raw_ySpeed = 0;
    float raw_zSpeed = 0;
    
    int16_t setChassis_xSpeed = 0;
    int16_t setChassis_ySpeed = 0;
    int16_t setChassis_zSpeed = 0;
    
    // 根据遥控器开关位置更新底盘模式
    for(;;)
    {
        switch(rc_ctrl.rc.s[1])
        {
            case ChassisGroy:  // 陀螺模式
                ChassisMode = ChassisGroy;
                
                raw_xSpeed = rc_ctrl.rc.ch[0] * Speed_K;
                raw_ySpeed = rc_ctrl.rc.ch[1] * Speed_K;
                raw_zSpeed = CHASSIS_GYRO_ROTATE_SPEED;
         
                // 应用低通滤波
//                setChassis_xSpeed = alpha * raw_xSpeed + (1 - alpha) * prev_xSpeed;
//                setChassis_ySpeed = alpha * raw_ySpeed + (1 - alpha) * prev_ySpeed;
//                setChassis_zSpeed = alpha * raw_zSpeed + (1 - alpha) * prev_zSpeed;
//                // 更新上一次的值
//                prev_xSpeed = setChassis_xSpeed;
//                prev_ySpeed = setChassis_ySpeed;
//                prev_zSpeed = setChassis_zSpeed;   
                 INS_Total = Yaw_Gimbal_6020_angle(&Yaw_Gimbal_Motor);                               // INS_Total[-180,180]
                       
                 ChassisGyro_Task(raw_xSpeed, raw_ySpeed, raw_zSpeed, CHASSIS_ROTATE_K, INS_Total); 
                
                break;
            
            case ChassisHead:  // 底盘跟随云台模式
                ChassisMode = ChassisHead;
            
                raw_xSpeed = rc_ctrl.rc.ch[0] * Speed_K;
                raw_ySpeed = rc_ctrl.rc.ch[1] * Speed_K;

//                // 应用低通滤波
//                setChassis_xSpeed = alpha * raw_xSpeed + (1 - alpha) * prev_xSpeed;
//                setChassis_ySpeed = alpha * raw_ySpeed + (1 - alpha) * prev_ySpeed;
//                setChassis_zSpeed = alpha * raw_zSpeed + (1 - alpha) * prev_zSpeed;

//                // 更新上一次的值
//                prev_xSpeed = setChassis_xSpeed;
//                prev_ySpeed = setChassis_ySpeed;
//                prev_zSpeed = setChassis_zSpeed;
            
            
                // 底盘跟随云台功能需要实现
                INS_Total = Yaw_Gimbal_6020_angle(&Yaw_Gimbal_Motor);
                ChassisPositionControl_Task(raw_xSpeed,raw_ySpeed, CHASSIS_ROTATE_K, INS_Total, 0);
               break;
            
            case ChassisSepa:  // 分离模式
                ChassisMode = ChassisSepa;
            
                raw_xSpeed = rc_ctrl.rc.ch[0] * Speed_K;
                raw_ySpeed = rc_ctrl.rc.ch[1] * Speed_K;
                raw_zSpeed = rc_ctrl.rc.ch[4] * Speed_K;

                // 应用低通滤波
//                setChassis_xSpeed = alpha * raw_xSpeed + (1 - alpha) * prev_xSpeed;
//                setChassis_ySpeed = alpha * raw_ySpeed + (1 - alpha) * prev_ySpeed;
//                setChassis_zSpeed = alpha * raw_zSpeed + (1 - alpha) * prev_zSpeed;

//                // 更新上一次的值
//                prev_xSpeed = setChassis_xSpeed;
//                prev_ySpeed = setChassis_ySpeed;
//                prev_zSpeed = setChassis_zSpeed;
            
                Chassis_WheatWheel_Solution(raw_xSpeed, raw_ySpeed, raw_zSpeed, CHASSIS_ROTATE_K);
                break;
            case ChassisWait:
                ChassisMode = ChassisWait;
            
                CAN_cmd_chassis(0, 0, 0, 0);
                
            default:
                ChassisMode = ChassisWait;
                break;
        }         
        osDelay(1);
    }
}
/**
  ********************************************
  * @GimbalMode  云台驱动模式
	* @Control     摇杆 
	* 0-待机  1-陀螺  2-头追  3-分离  4-部署
  ********************************************
  */
#define GIMBAL_MODE_WAIT    0   // 待机模式
#define GIMBAL_MODE_GYRO    1   // 陀螺模式(小陀螺，云台稳定)
#define GIMBAL_MODE_SEPA    2   // 分离模式
#define GIMBAL_MODE_HEAD    3   // 头追模式
#define GIMBAL_MODE_DEPLOY  4   // 部署模式

uint8_t GimbalMode = GIMBAL_MODE_WAIT;
int16_t pitch_output = 0;
int16_t pitch_angle_output = 0;
int16_t pitch_GravityCompensation = 0;

int16_t yaw_angle_output = 0;
int16_t yaw_output = 0;

//Pitch轴由摇杆控制（根据实际需求确认是否需要限制范围）
int16_t set_pitch = 0;
int16_t set_yaw = 0;

static float cam_yaw_err_f = 0.0f;
static float cam_pitch_err_f = 0.0f;
const float VISUAL_FILTER = 0.3f;
int16_t vision_yaw_diff = 0;
int16_t vision_pitch_diff = 0;


void Gimbal_Task(void const * argument)
{
    for(;;)
    {
        switch(rc_ctrl.rc.s[1])
        {
            case GIMBAL_MODE_GYRO:  // 陀螺模式(小陀螺)
            {
                GimbalMode = GIMBAL_MODE_GYRO; 
                
                cam_yaw_err_f   = cam_yaw_err_f   * (1.0f - VISUAL_FILTER) + vision_yaw_diff   * VISUAL_FILTER;
                cam_pitch_err_f = cam_pitch_err_f * (1.0f - VISUAL_FILTER) + vision_pitch_diff * VISUAL_FILTER;
                
                set_yaw = Yaw_Gimbal_Contro_Angle(rc_ctrl.rc.ch[2] + cam_yaw_err_f);
                yaw_angle_output = set_yaw;
                yaw_output = Yaw_Gimbal_ControlOfIMU(INS.YawTotalAngle, yaw_angle_output);

                //pitch_GravityCompensation = GravityCompensation(&Pitch_Gimbal_Motor);
                set_pitch = Pitch_Gimbal_Contro_Angle(rc_ctrl.rc.ch[3] + cam_pitch_err_f);
                pitch_angle_output = set_pitch + pitch_GravityCompensation;
                pitch_output = Pitch_Gimbal_ControlOfIMU(INS.Roll, pitch_angle_output);
//                set_pitch = rc_ctrl.rc.ch[3] + pitch_GravityCompensation;
//                pitch_output = Pitch_Gimbal_ControlOfECD(set_pitch);
                
                CAN_cmd_gimbal(yaw_output, pitch_output);
                
                cam_yaw_err_f  = vision_yaw_diff;
                cam_pitch_err_f = vision_pitch_diff;

                break;
            }
            
            case GIMBAL_MODE_HEAD:  // 底盘跟随云台模式
            {
                GimbalMode = GIMBAL_MODE_HEAD;
                
                cam_yaw_err_f   = cam_yaw_err_f   * (1.0f - VISUAL_FILTER) + vision_yaw_diff   * VISUAL_FILTER;
                cam_pitch_err_f = cam_pitch_err_f * (1.0f - VISUAL_FILTER) + vision_pitch_diff * VISUAL_FILTER;
                
                set_yaw = Yaw_Gimbal_Contro_Angle(rc_ctrl.rc.ch[2] + cam_yaw_err_f);
                yaw_angle_output = set_yaw;
                yaw_output = Yaw_Gimbal_ControlOfIMU(INS.YawTotalAngle, yaw_angle_output);

                //pitch_GravityCompensation = GravityCompensation(&Pitch_Gimbal_Motor);
                set_pitch = Pitch_Gimbal_Contro_Angle(rc_ctrl.rc.ch[3] + cam_pitch_err_f);
                pitch_angle_output = set_pitch + pitch_GravityCompensation;
                pitch_output = Pitch_Gimbal_ControlOfIMU(INS.Roll, pitch_angle_output);
//                set_pitch = rc_ctrl.rc.ch[3] + pitch_GravityCompensation;
//                pitch_output = Pitch_Gimbal_ControlOfECD(set_pitch);
                
                CAN_cmd_gimbal(yaw_output, pitch_output);
                
                cam_yaw_err_f  = vision_yaw_diff;
                cam_pitch_err_f = vision_pitch_diff;
                
                break;
            }
            
            case GIMBAL_MODE_SEPA:  // 分离模式
            {
                GimbalMode = GIMBAL_MODE_SEPA;
                
                cam_yaw_err_f   = cam_yaw_err_f   * (1.0f - VISUAL_FILTER) + vision_yaw_diff   * VISUAL_FILTER;
                cam_pitch_err_f = cam_pitch_err_f * (1.0f - VISUAL_FILTER) + vision_pitch_diff * VISUAL_FILTER;
                
                set_yaw = Yaw_Gimbal_Contro_Angle(rc_ctrl.rc.ch[2] + cam_yaw_err_f);
                yaw_angle_output = set_yaw;
                yaw_output = Yaw_Gimbal_ControlOfIMU(INS.YawTotalAngle, yaw_angle_output);

                //pitch_GravityCompensation = GravityCompensation(&Pitch_Gimbal_Motor);
                set_pitch = Pitch_Gimbal_Contro_Angle(rc_ctrl.rc.ch[3] + cam_pitch_err_f);
                pitch_angle_output = set_pitch + pitch_GravityCompensation;
                pitch_output = Pitch_Gimbal_ControlOfIMU(INS.Roll, pitch_angle_output);
//                set_pitch = rc_ctrl.rc.ch[3] + pitch_GravityCompensation;
//                pitch_output = Pitch_Gimbal_ControlOfECD(set_pitch);
                
                CAN_cmd_gimbal(yaw_output, pitch_output);
                
                cam_yaw_err_f  = vision_yaw_diff;
                cam_pitch_err_f = vision_pitch_diff;
                
                break;
            }

            case GIMBAL_MODE_WAIT:  // 待机模式
            default:
            {
                CAN_cmd_gimbal(0, 0);
                break;
            }
        }
        osDelay(10);      
    }
}

/**
  ********************************************
  * @FireMode  发射驱动模式
  * @Control   右侧拨杆  
	* 0-待机  1-停机  2-空转  3-发射
  ********************************************
  */
#define GunWait   0
#define GunStart  1
#define GunStop   3
#define GunFire   2

/*拨弹速度*/
int16_t UP_TRIGGER_SPEED = -1980;
extern M2006_HandleTypeDef up_trigger_motor;

void Shoot_Task(void const * argument)
{
    for(;;)
    {
        switch(rc_ctrl.rc.s[0])
        {   
            case GunStart:
            {
                //空转后续编写
                Shoot_FrictionWheelControl(-1000,500,-500);
            }break;
            
            case GunStop:
            {
                Shoot_FrictionWheelControl(0, 0, 0);
                cam_yaw_err_f = 0;
                cam_pitch_err_f = 0;
                
            }break;            
            case GunFire:
            {
                //UP_TRIGGER_SPEED根据情况判断是否堵转
                if(M2006_LockedMotorDetertion(&up_trigger_motor))
                {
                    // 检测到堵转，触发解锁
                    M2006_UnlockMotor(&up_trigger_motor);
                }
                else if(up_trigger_motor.is_unlocking)
                {
                    // 正在解锁过程中，继续执行解锁动作
                    M2006_UnlockMotor(&up_trigger_motor);
                } 
                else
                {
                    // 未堵转且不在解锁，使用正常速度
                    UP_TRIGGER_SPEED = UP2006_SPEED;
                }
                // 最终输出控制信号
                Shoot_FrictionWheelControl(UP_TRIGGER_SPEED, LEFT_SHOT_SPEED, RIGHT_SHOT_SPEED);
                
            }break;
            
            default : break;
        }
        osDelay(10);
    }
}
