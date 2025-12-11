/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             这里是CAN中断接收函数，接收电机数据,CAN发送函数发送电机电流控制电机.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "struct_typedef.h"
#include "can.h"

#define CHASSIS_CAN hcan1
#define GIMBAL_CAN hcan2

/* CAN send and receive ID */
typedef enum
{
    //can1
    CAN_CHASSIS_ALL_ID = 0x200,
    CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,
    CAN_3508_M3_ID = 0x203,
    CAN_3508_M4_ID = 0x204,
    
    CAN_GIMBAL_ALL_ID = 0x1FF,
    
    //can1
    CAN_YAW_MOTOR_ID = 0x205,
    CAN_PIT_MOTOR_ID = 0x206,
    //can2
    CAN_TRIGGER_UP_MOTOR_ID = 0x206,
    
    CAN_TRIGGER_FIRE_MOTOR_ID1 = 0x207,
    CAN_TRIGGER_FIRE_MOTOR_ID2 = 0x208,   
    
    Send_Baseboard_Data_ID = 0x3FF,

} can_msg_id_e;

typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
} motor_measure_t;

typedef struct
{
    int16_t YAW_Total;
    int16_t Remote_Pitch_ch2;
    int16_t Date3;
    int16_t Date4;
    
} CAN_TX_RX_t;

/**
  * @brief          发送电机控制电流(0x205,0x206)
  * @param[in]      yaw: (0x205) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      pitch: (0x206) 6020电机控制电流, 范围 [-30000,30000]
  * @retval         none
  */
extern void CAN_cmd_gimbal(int16_t yaw, int16_t pitch);

/**
  * @brief          发送电机控制电流(0x205,0x206)
  * @param[in]      yaw: (0x205) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      pitch: (0x206) 6020电机控制电流, 范围 [-30000,30000]
  * @retval         none
  */
extern void CAN_cmd_gimbal_Frie(int16_t trigger, int16_t shoot_Left, int16_t shoot_Right);
/**
  * @brief          发送ID为0x700的CAN包,它会设置3508电机进入快速设置ID
  * @param[in]      none
  * @retval         none
  */
extern void CAN_cmd_chassis_reset_ID(void);

/**
  * @brief          发送电机控制电流(0x201,0x202,0x203,0x204)
  * @param[in]      motor1: (0x201) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor2: (0x202) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor3: (0x203) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor4: (0x204) 3508电机控制电流, 范围 [-16384,16384]
  * @retval         none
  */
extern void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

/**
  * @brief          返回yaw 6020电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
extern const motor_measure_t *get_yaw_gimbal_motor_measure_point(void);

/**
  * @brief          返回pitch 6020电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
extern const motor_measure_t *get_pitch_gimbal_motor_measure_point(void);

/**
  * @brief          返回拨弹电机 2006电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
extern const motor_measure_t *get_trigger_motor_measure_point(void);


/**
  * @brief          返回底盘电机 3508电机数据指针
  * @param[in]      i: 电机编号,范围[0,3]
  * @retval         电机数据指针
  */
extern const motor_measure_t *get_chassis_motor_measure_point(uint8_t i);

/**
  * @brief          返回摩擦轮电机 3508电机数据指针
  * @param[in]      i: 电机编号,范围[7，8]
  * @retval         电机数据指针
  */
extern const motor_measure_t *get_Fire_motor_measure_point(uint8_t i);

/**
  * @brief  接受数据
  * @func	void Get_CAN2_RX_Data(CAN_TX_RX_t *Ptrr,uint8_t *Data)
  * @param  CAN_TX_RX_t *Ptrr：CAN_TX_RX_t的句柄
  * @param  Data：接受的数据，并进行赋值
  * @retval none
**/
extern void Get_CAN2_RX_Data(CAN_TX_RX_t *Ptrr,uint8_t *Date);

/**
  * @brief  发送底板数据到上板
  * @func	void Send_Baseboard_Data(const uint8_t *Remote_Data)
  * @param  Datax 传输数据
  * @retval none
**/
extern void Send_Baseboard_Data(int16_t Date1,int16_t Date2,int16_t Date3,int16_t Date4);
#endif
