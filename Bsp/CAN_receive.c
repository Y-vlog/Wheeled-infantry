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

#include "CAN_receive.h"
#include "main.h"
#include "string.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

//读取电机数据
#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }

void Get_CAN2_RX_Data(CAN_TX_RX_t *Ptrr,uint8_t *Date);

/*
motor data,  0:chassis motor1 3508;1:chassis motor3 3508;2:chassis motor3 3508;3:chassis motor4 3508;
4:yaw gimbal motor 6020;5:pitch gimbal motor 6020;6:trigger motor 2006;
电机数据, 0:底盘电机1 3508电机,   1:底盘电机2 3508电机,     2:底盘电机3 3508电机, 3:底盘电机4 3508电机;
          4:yaw云台电机 6020电机; 5:pitch云台电机 6020电机; 6:拨弹电机 2006电机 ; 7:摩擦轮1 3508电机;
          8:摩擦轮2 3508电机
*/
static motor_measure_t motor_chassis[9];

static CAN_TxHeaderTypeDef  gimbal_tx_message;
static uint8_t              gimbal_can_send_data[8];
static CAN_TxHeaderTypeDef  gimbal_tx_message_Fire;
static uint8_t              gimbal_can_send_data_Fire[8];
static CAN_TxHeaderTypeDef  chassis_tx_message;
static uint8_t              chassis_can_send_data[8];

CAN_TX_RX_t CAN2_RX;
static CAN_TxHeaderTypeDef	CAN_TxStructure;
static uint8_t              CAN2_Tx_Data[8];

/**
  * @brief          hal库CAN回调函数,接收电机数据  邮箱消息挂号中断
  * @param[in]      hcan:CAN句柄指针
  * @retval         none
  */

/************************** CAN1-FIFO0中断回调（处理电机数据） **************************/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    // 仅处理CAN1的FIFO0中断，避免与CAN2混淆
    if (hcan != &hcan1) return;

    CAN_RxHeaderTypeDef CAN1_RX_header; // 仅CAN1使用的接收头
    uint8_t CAN1_RX_data[8];            // 仅CAN1使用的接收数据
    HAL_StatusTypeDef read_status;

    /************************** 关键：中断异常处理 **************************/
    // 1. 检查FIFO0溢出（若溢出，消息已丢失，需清除标志）
    if (__HAL_CAN_GET_FLAG(hcan, CAN_FLAG_FOV0) != RESET)
    {
        __HAL_CAN_CLEAR_FLAG(hcan, CAN_FLAG_FOV0); // 清除溢出标志
        return;
    }

    // 2. 读取FIFO0消息（检查返回值，避免读取失败）
    read_status = HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &CAN1_RX_header, CAN1_RX_data);
    if (read_status != HAL_OK)
    {
        return;
    }

    /************************** 正常处理电机数据 **************************/
    // 仅处理预设的电机ID（避免无效消息占用资源）
    switch (CAN1_RX_header.StdId)
    {
        case CAN_3508_M1_ID:
        case CAN_3508_M2_ID:
        case CAN_3508_M3_ID:
        case CAN_3508_M4_ID:
            
        case CAN_YAW_MOTOR_ID:
        case CAN_PIT_MOTOR_ID:
        {
            static uint8_t can_1_motor_idx = 0;
            // 计算电机索引（确保ID连续，若ID不连续需单独赋值）
            can_1_motor_idx = CAN1_RX_header.StdId - CAN_3508_M1_ID;
            // 若电机索引超出范围，跳过（避免数组越界）
            if (can_1_motor_idx >= 9) break;
            // 读取电机数据（建议此函数仅做数据解析，无阻塞/耗时操作）
            get_motor_measure(&motor_chassis[can_1_motor_idx], CAN1_RX_data);
            break;
        }
        default:
            break;
    }
}
/************************** CAN2-FIFO1中断回调（处理底板数据） **************************/
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    // 仅处理CAN2的FIFO1中断，避免与CAN1混淆
    if (hcan != &hcan2){ return;}

    CAN_RxHeaderTypeDef CAN2_RX_header; // 仅CAN2使用的接收头
    uint8_t CAN2_RX_data[8];            // 仅CAN2使用的接收数据
    HAL_StatusTypeDef read_status;

    /************************** 关键：中断异常处理 **************************/
    // 1. 检查FIFO1溢出（CAN2-FIFO1专属溢出标志）
    if (__HAL_CAN_GET_FLAG(hcan, CAN_FLAG_FOV1) != RESET)
    {
        __HAL_CAN_CLEAR_FLAG(hcan, CAN_FLAG_FOV1);
        return;
    }

    // 2. 读取FIFO1消息（检查返回值）
    read_status = HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &CAN2_RX_header, CAN2_RX_data);
    if (read_status != HAL_OK)
    {
        return;
    }

    /************************** 正常处理底板数据 **************************/
    switch (CAN2_RX_header.StdId)
    {
        case CAN_TRIGGER_UP_MOTOR_ID:
            
        case CAN_TRIGGER_FIRE_MOTOR_ID1:
        case CAN_TRIGGER_FIRE_MOTOR_ID2:
        {
            static uint8_t can_2_motor_idx = 0;
            // 计算电机索引（确保ID连续，若ID不连续需单独赋值）
            can_2_motor_idx = CAN2_RX_header.StdId;
            // 若电机索引超出范围，跳过（避免数组越界）
            if (can_2_motor_idx >= 9) break;
            // 读取电机数据（建议此函数仅做数据解析，无阻塞/耗时操作）
            get_motor_measure(&motor_chassis[can_2_motor_idx], CAN2_RX_data);
            break;
        }
        default:
            break;
    }
}


/**
  * @brief          发送电机控制电流(0x205,0x206,0x207,0x208)
  * @param[in]      yaw: (0x205) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      pitch: (0x206) 6020电机控制电流, 范围 [-30000,30000]
  * @retval         none
  */
void CAN_cmd_gimbal(int16_t yaw, int16_t pitch)
{
    uint32_t send_mail_box;
    gimbal_tx_message.StdId = CAN_GIMBAL_ALL_ID;         //YAW轴ID Pitch轴ID 
    gimbal_tx_message.IDE = CAN_ID_STD;
    gimbal_tx_message.RTR = CAN_RTR_DATA;
    gimbal_tx_message.DLC = 0x08;
    
    gimbal_can_send_data[0] = (yaw >> 8);
    gimbal_can_send_data[1] = yaw;
    gimbal_can_send_data[2] = (pitch >> 8);
    gimbal_can_send_data[3] = pitch;
    gimbal_can_send_data[4] = (0 >> 8);
    gimbal_can_send_data[5] = 0;
    gimbal_can_send_data[6] = (0 >> 8);
    gimbal_can_send_data[7] = 0;
    

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box); //hcan1
}

/**
  * @brief          发送电机控制电流(0x205,0x206,0x207,0x208)
  * @param[in]      shoot1: (0x205) 3508电机控制电流, 范围 [-30000,30000]
  * @param[in]      shoot2: (0x206) 3508电机控制电流, 范围 [-30000,30000]
  * @param[in]      trigger: (0x206) 2006电机控制电流, 范围 [-30000,30000]
  * @retval         none
  */
void CAN_cmd_gimbal_Frie(int16_t trigger, int16_t shoot_Left, int16_t shoot_Right)
{
    uint32_t send_mail_box;
    gimbal_tx_message_Fire.StdId = CAN_GIMBAL_ALL_ID;//拨弹和两个摩擦轮的id
    gimbal_tx_message_Fire.IDE = CAN_ID_STD;
    gimbal_tx_message_Fire.RTR = CAN_RTR_DATA;
    gimbal_tx_message_Fire.DLC = 0x08;

    gimbal_can_send_data_Fire[0] = (0 >> 8);
    gimbal_can_send_data_Fire[1] = 0;
    gimbal_can_send_data_Fire[2] = (trigger >> 8);    //0x206
    gimbal_can_send_data_Fire[3] = trigger;           
    gimbal_can_send_data_Fire[4] = (shoot_Left >> 8); //0x207
    gimbal_can_send_data_Fire[5] = shoot_Left;
    gimbal_can_send_data_Fire[6] = (shoot_Right >> 8);//0x208
    gimbal_can_send_data_Fire[7] = shoot_Right;
    
    HAL_CAN_AddTxMessage(&GIMBAL_CAN, &gimbal_tx_message_Fire, gimbal_can_send_data_Fire, &send_mail_box); //hcan2
}

/**
  * @brief          发送ID为0x700的CAN包,它会设置3508电机进入快速设置ID
  * @param[in]      none
  * @retval         none
  */
void CAN_cmd_chassis_reset_ID(void)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = 0x700;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = 0;
    chassis_can_send_data[1] = 0;
    chassis_can_send_data[2] = 0;
    chassis_can_send_data[3] = 0;
    chassis_can_send_data[4] = 0;
    chassis_can_send_data[5] = 0;
    chassis_can_send_data[6] = 0;
    chassis_can_send_data[7] = 0;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

/**
  * @brief          发送电机控制电流(0x201,0x202,0x203,0x204)
  * @param[in]      motor1: (0x201) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor2: (0x202) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor3: (0x203) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor4: (0x204) 3508电机控制电流, 范围 [-16384,16384]
  * @retval         none
  */
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = CAN_CHASSIS_ALL_ID;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = motor1 >> 8;
    chassis_can_send_data[1] = motor1;
    chassis_can_send_data[2] = motor2 >> 8;
    chassis_can_send_data[3] = motor2;
    chassis_can_send_data[4] = motor3 >> 8;
    chassis_can_send_data[5] = motor3;
    chassis_can_send_data[6] = motor4 >> 8;
    chassis_can_send_data[7] = motor4;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);//hcan1
}

/**
  * @brief          返回yaw 6020电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_yaw_gimbal_motor_measure_point(void)
{
    return &motor_chassis[4];
}

/**
  * @brief          返回pitch 6020电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_pitch_gimbal_motor_measure_point(void)
{
    return &motor_chassis[5];
}


/**
  * @brief          返回拨弹电机 2006电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_trigger_motor_measure_point(void)
{
    return &motor_chassis[6];
}


/**
  * @brief          返回底盘电机 3508电机数据指针
  * @param[in]      i: 电机编号,范围[0,3]
  * @retval         电机数据指针
  */
const motor_measure_t *get_chassis_motor_measure_point(uint8_t i)
{
    return &motor_chassis[(i & 0x03)];
}

/**
  * @brief          返回摩擦轮电机 3508电机数据指针
  * @param[in]      i: 电机编号,范围[7，8]
  * @retval         电机数据指针
  */
const motor_measure_t *get_Fire_motor_measure_point(uint8_t i)
{
    return &motor_chassis[(i & 0x0F)];
}

/**
  * @brief  接受数据
  * @func	void Get_CAN2_RX_Data(CAN_TX_RX_t *Ptrr,uint8_t *Data)
  * @param  CAN_TX_RX_t *Ptrr：CAN_TX_RX_t的句柄
  * @param  Data：接受的数据，并进行赋值
  * @retval none
**/

void Get_CAN2_RX_Data(CAN_TX_RX_t *Ptrr,uint8_t *Data)
{
    Ptrr->YAW_Total = Data[0] << 8 | Data[1];   
    Ptrr->Remote_Pitch_ch2 = Data[2] << 8 | Data[3];
    Ptrr->Date3 = Data[4] << 8 | Data[5];
    Ptrr->Date4 = Data[6] << 8 | Data[7];
}

/**
  * @brief  发送底板数据到上板
  * @func	void Send_Baseboard_Data(const uint8_t *Remote_Data)
  * @param  Datax：传输数据
  * @retval none
**/
void Send_Baseboard_Data(int16_t Data1,int16_t Data2,int16_t Data3,int16_t Data4)
{
    uint32_t CAN2_TxMailbox;
     
    CAN_TxStructure.StdId = Send_Baseboard_Data_ID;  	 /*发送数据ID号*/
    CAN_TxStructure.IDE   = CAN_ID_STD;   				 /*标准帧,CAN_ID_EXT扩展帧*/
    CAN_TxStructure.RTR   = CAN_RTR_DATA;  				 /*数据帧,CAN_RTR_REMOTE遥控帧*/
    CAN_TxStructure.DLC   = 0x08;    					 /*数据长度为8字节*/
    
    CAN2_Tx_Data[0] = Data1 >> 8;
    CAN2_Tx_Data[1] = Data1;
    CAN2_Tx_Data[2] = Data2 >> 8;
    CAN2_Tx_Data[3] = Data2;
    CAN2_Tx_Data[4] = Data3 >> 8;
    CAN2_Tx_Data[5] = Data3;
    CAN2_Tx_Data[6] = Data4 >> 8;
    CAN2_Tx_Data[7] = Data4;
    
    HAL_CAN_AddTxMessage(&hcan2,&CAN_TxStructure,CAN2_Tx_Data,&CAN2_TxMailbox);
    
}
