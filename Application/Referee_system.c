#include "Referee_system.h"
#include "stdio.h"
#include "string.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "semphr.h"

uint8_t Turn_Finsh_Flag;
Usart_Referee_data Referee_data;//串口接收数据相关结构体
RobotInfo Robot_All_Info; //裁判系统相关的结构体，直接用结构体里面的数据就可以
extern osSemaphoreId_t  refereeSystem_semaphoreHandle;
uint8_t Test_S[UART6_RX_BUF_SIZE];
uint16_t Com_Test;
uint8_t Valll;
uint16_t test_Break;
/**
 * @brief   用于通知任务处理裁判系统数据
 * 
 * @return  无
 * 
 * @note    被下面函数调用
 */
void Referee_Data_Send_Msg()
{
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
         xSemaphoreGiveFromISR(refereeSystem_semaphoreHandle,&xHigherPriorityTaskWoken);
}




/**
 * @brief   校验帧头5byte
 * 
 * @return  无
 * 
 * @note    在任务中调用，检验帧头是否正确再确定是否丢包
 */

uint8_t frame_header_crc_check(const uint8_t *data) {
    uint8_t Frame_Data[5];
    memcpy(Frame_Data, data, 5 * sizeof(uint8_t));
    
    // 直接调用官方校验函数
    return Verify_CRC8_Check_Sum(Frame_Data, 5);
}


/**
 * @brief   同上
 * 
 * @return  无
 * 
 * @note    无
 */
uint16_t Referee_data_check(uint8_t *data) {
    if (data == NULL || Referee_data.Usart6_Rx_Index < 2) {
        return 0;  // 参数无效或数据长度不足
    }

    uint16_t Crc16_Val;         // 读出来的 CRC 校验值
    uint16_t Crc16_Cal = 0xFFFF; // CRC16 校验的初始值

    // 提取 CRC 校验值，假设 CRC 存储在最后两个字节
    Crc16_Val = (data[Referee_data.Usart6_Rx_Index - 2] << 8) | data[Referee_data.Usart6_Rx_Index - 1];

    // 计算 CRC16 校验值
    for (int i = 0; i < Referee_data.Usart6_Rx_Index - 2; i++) {
        Crc16_Cal ^= data[i]; // 将数据的每个字节与 CRC 做异或   
        // 对每个字节的每一位进行计算
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (Crc16_Cal & 0x8000) { // 如果最高位为 1
                Crc16_Cal = (Crc16_Cal << 1) ^ CRC16_POLYNOMIAL; // 左移并异或多项式
            } else {
                Crc16_Cal <<= 1; // 否则只是左移
            }
        }
    }

    // 返回结果，判断计算的 CRC16 是否与读取的 CRC16 匹配
    return (Crc16_Cal == Crc16_Val) ? 1 : 0;
}













/**
 * @brief   初始化裁判系统串口
 * 
 * @return  无
 * 
 * @note    上电初始化使用 Referee_system_Usart-可在宏更改自己串口
 */
void Referee_Usart_Init(){
    __HAL_UART_ENABLE_IT(Referee_system_Usart, UART_IT_IDLE);              /* 使能UART2总线空闲中断 */
    HAL_UART_Receive_DMA(Referee_system_Usart,Referee_data.Usart6_Rx_Buff ,UART6_RX_BUF_SIZE);//开启DMA接收，用于处理裁判系统数据
}



/**
 * @brief  检测空闲中断，发送信号量通知任务
 * 
 * @return  无
 * 
 * @note    双缓存区，避免数据覆盖，处理copy数据，清空DNA缓存数据
 */

void Referee_Usart_Data_Deal(){
     if (__HAL_UART_GET_FLAG(Referee_system_Usart, UART_FLAG_IDLE) != RESET){           //获取接收IDLE标志位是否被置位
        __HAL_UART_CLEAR_IDLEFLAG(Referee_system_Usart);
       HAL_UART_DMAStop(Referee_system_Usart); 			 //停止DMA传输，防止干扰
			 if(Turn_Finsh_Flag==0){
//      Referee_data.Usart6_Rx_Index = UART6_RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);  //获取接收到的数据长度//改了			 
			Referee_data.Usart6_Rx_Copy_Index=Referee_data.Usart6_Rx_Index;
      memcpy(Referee_data.Usart6_Rx_Buff_Copy, Referee_data.Usart6_Rx_Buff, sizeof(Referee_data.Usart6_Rx_Buff));	
       Referee_Data_Send_Msg();//给任务发通知，处理数据				 
			 }

      memset(Referee_data.Usart6_Rx_Buff,0,sizeof(Referee_data.Usart6_Rx_Buff));//清除缓存数组 
     

      HAL_UART_Receive_DMA(Referee_system_Usart,Referee_data.Usart6_Rx_Buff ,UART6_RX_BUF_SIZE);     
    }   
}

/**
 * @brief   解裁判系统包函数，开一个任务放这个函数就行
 * 
 * @return  无
 * 
 * @note    主要要先校验（校验帧头即可，CRC16没太大必要），注意根据自己任务优先级判断是否需要0sdely
 */

//void Referee_Task(void)
//{
//      uint16_t Com_Val;
//      xSemaphoreTake(Needed_Referee_dataHandle,portMAX_DELAY);
//      Com_Val=((uint16_t)Referee_data.Usart6_Rx_Buff_Copy[5] << 8) | (uint16_t)Referee_data.Usart6_Rx_Buff_Copy[6];
//      if(frame_header_crc_check(Referee_data.Usart6_Rx_Buff_Copy))
//      Que_Extract_Referee_Info(Referee_data.Usart6_Rx_Buff_Copy,Com_Val);
//      memset(Referee_data.Usart6_Rx_Buff_Copy, 0, sizeof(Referee_data.Usart6_Rx_Buff_Copy));
//      //osDelay(2);
// }


void Referee_Task(void)
{
         xSemaphoreTake(refereeSystem_semaphoreHandle,portMAX_DELAY);
         process_packet(Referee_data.Usart6_Rx_Buff_Copy,Referee_data.Usart6_Rx_Copy_Index,handle_frame);
	      memset(Referee_data.Usart6_Rx_Buff_Copy, 0, sizeof(Referee_data.Usart6_Rx_Buff_Copy));
				Turn_Finsh_Flag=0;
//      osDelay(2);
}

/**
 * @brief   解包数据放到对应结构体
 * 
 * @return  无
 * 
 * @note    这个暂时不用，用下面的传数组更方便
 */ 
void Extract_Referee_Info(Com_Id Cmd)
{
//    uint8_t index;
//    switch(Cmd)
//    {
//        case game_robot_state:
//            index = 7;
//            // 使用强制类型转换以确保按位操作不会发生类型不匹配
//            Robot_All_Info.Robot_Data.red_1_robot_HP = ((uint16_t)Referee_data.Usart6_Rx_Buff[index++] << 8) | (uint16_t)Referee_data.Usart6_Rx_Buff[index++];
//            Robot_All_Info.Robot_Data.red_2_robot_HP = ((uint16_t)Referee_data.Usart6_Rx_Buff[index++] << 8) | (uint16_t)Referee_data.Usart6_Rx_Buff[index++];
//            Robot_All_Info.Robot_Data.red_3_robot_HP = ((uint16_t)Referee_data.Usart6_Rx_Buff[index++] << 8) | (uint16_t)Referee_data.Usart6_Rx_Buff[index++];
//            Robot_All_Info.Robot_Data.red_4_robot_HP = ((uint16_t)Referee_data.Usart6_Rx_Buff[index++] << 8) | (uint16_t)Referee_data.Usart6_Rx_Buff[index++];
//            Robot_All_Info.Robot_Data.reserved1 = ((uint16_t)Referee_data.Usart6_Rx_Buff[index++] << 8) | (uint16_t)Referee_data.Usart6_Rx_Buff[index++];
//            Robot_All_Info.Robot_Data.red_7_robot_HP = ((uint16_t)Referee_data.Usart6_Rx_Buff[index++] << 8) | (uint16_t)Referee_data.Usart6_Rx_Buff[index++];
//            Robot_All_Info.Robot_Data.red_outpost_HP = ((uint16_t)Referee_data.Usart6_Rx_Buff[index++] << 8) | (uint16_t)Referee_data.Usart6_Rx_Buff[index++];
//            Robot_All_Info.Robot_Data.red_base_HP = ((uint16_t)Referee_data.Usart6_Rx_Buff[index++] << 8) | (uint16_t)Referee_data.Usart6_Rx_Buff[index++];
//            Robot_All_Info.Robot_Data.blue_1_robot_HP = ((uint16_t)Referee_data.Usart6_Rx_Buff[index++] << 8) | (uint16_t)Referee_data.Usart6_Rx_Buff[index++];
//            Robot_All_Info.Robot_Data.blue_2_robot_HP = ((uint16_t)Referee_data.Usart6_Rx_Buff[index++] << 8) | (uint16_t)Referee_data.Usart6_Rx_Buff[index++];
//            Robot_All_Info.Robot_Data.blue_3_robot_HP = ((uint16_t)Referee_data.Usart6_Rx_Buff[index++] << 8) | (uint16_t)Referee_data.Usart6_Rx_Buff[index++];
//            Robot_All_Info.Robot_Data.blue_4_robot_HP = ((uint16_t)Referee_data.Usart6_Rx_Buff[index++] << 8) | (uint16_t)Referee_data.Usart6_Rx_Buff[index++];
//            Robot_All_Info.Robot_Data.reserved2 = ((uint16_t)Referee_data.Usart6_Rx_Buff[index++] << 8) | (uint16_t)Referee_data.Usart6_Rx_Buff[index++];
//            Robot_All_Info.Robot_Data.blue_7_robot_HP = ((uint16_t)Referee_data.Usart6_Rx_Buff[index++] << 8) | (uint16_t)Referee_data.Usart6_Rx_Buff[index++];
//            Robot_All_Info.Robot_Data.blue_outpost_HP = ((uint16_t)Referee_data.Usart6_Rx_Buff[index++] << 8) | (uint16_t)Referee_data.Usart6_Rx_Buff[index++];
//            Robot_All_Info.Robot_Data.blue_base_HP = ((uint16_t)Referee_data.Usart6_Rx_Buff[index++] << 8) | (uint16_t)Referee_data.Usart6_Rx_Buff[index++];
//            break;

//        case event_data:
//            index = 7;
//            Robot_All_Info.event_data.event_data = ((uint32_t)Referee_data.Usart6_Rx_Buff[index++] << 24) |
//                                                    ((uint32_t)Referee_data.Usart6_Rx_Buff[index++] << 16) |
//                                                    ((uint32_t)Referee_data.Usart6_Rx_Buff[index++] << 8) |
//                                                    (uint32_t)Referee_data.Usart6_Rx_Buff[index++];
//            break;

//        case dart_info:
//            index = 7;
//            Robot_All_Info.dart_info.dart_remaining_time = Referee_data.Usart6_Rx_Buff[index++];
//            Robot_All_Info.dart_info.dart_info = ((uint16_t)Referee_data.Usart6_Rx_Buff[index++] << 8) | (uint16_t)Referee_data.Usart6_Rx_Buff[index++];
//            break;

//        case robot_status:
//            index = 7;
//            Robot_All_Info.robot_status.robot_id = Referee_data.Usart6_Rx_Buff[index++];
//            Robot_All_Info.robot_status.robot_level = Referee_data.Usart6_Rx_Buff[index++];
//            Robot_All_Info.robot_status.current_HP = ((uint16_t)Referee_data.Usart6_Rx_Buff[index++] << 8) | (uint16_t)Referee_data.Usart6_Rx_Buff[index++];
//            Robot_All_Info.robot_status.maximum_HP = ((uint16_t)Referee_data.Usart6_Rx_Buff[index++] << 8) | (uint16_t)Referee_data.Usart6_Rx_Buff[index++];
//            Robot_All_Info.robot_status.shooter_barrel_cooling_value = ((uint16_t)Referee_data.Usart6_Rx_Buff[index++] << 8) | (uint16_t)Referee_data.Usart6_Rx_Buff[index++];
//            Robot_All_Info.robot_status.shooter_barrel_heat_limit = ((uint16_t)Referee_data.Usart6_Rx_Buff[index++] << 8) | (uint16_t)Referee_data.Usart6_Rx_Buff[index++];
//            Robot_All_Info.robot_status.chassis_power_limit = ((uint16_t)Referee_data.Usart6_Rx_Buff[index++] << 8) | (uint16_t)Referee_data.Usart6_Rx_Buff[index++];
//            Robot_All_Info.robot_status.power_management_gimbal_output = Referee_data.Usart6_Rx_Buff[index++];
//            Robot_All_Info.robot_status.power_management_chassis_output = Referee_data.Usart6_Rx_Buff[index++];
//            Robot_All_Info.robot_status.power_management_shooter_output = Referee_data.Usart6_Rx_Buff[index++];
//            break;

//        case power_heat_data:
//            index = 7;
//            Robot_All_Info.power_heat_data.reserved1 = ((uint16_t)Referee_data.Usart6_Rx_Buff[index++] << 8) | (uint16_t)Referee_data.Usart6_Rx_Buff[index++];
//            Robot_All_Info.power_heat_data.reserved2 = ((uint16_t)Referee_data.Usart6_Rx_Buff[index++] << 8) | (uint16_t)Referee_data.Usart6_Rx_Buff[index++];
//            Robot_All_Info.power_heat_data.reserved3 = ((uint32_t)Referee_data.Usart6_Rx_Buff[index++] << 24) |
//                                                        ((uint32_t)Referee_data.Usart6_Rx_Buff[index++] << 16) |
//                                                        ((uint32_t)Referee_data.Usart6_Rx_Buff[index++] << 8) |
//                                                        (uint32_t)Referee_data.Usart6_Rx_Buff[index++];
//            Robot_All_Info.power_heat_data.buffer_energy = ((uint16_t)Referee_data.Usart6_Rx_Buff[index++] << 8) | (uint16_t)Referee_data.Usart6_Rx_Buff[index++];
//            Robot_All_Info.power_heat_data.shooter_17mm_1_barrel_heat = ((uint16_t)Referee_data.Usart6_Rx_Buff[index++] << 8) | (uint16_t)Referee_data.Usart6_Rx_Buff[index++];
//            Robot_All_Info.power_heat_data.shooter_17mm_2_barrel_heat = ((uint16_t)Referee_data.Usart6_Rx_Buff[index++] << 8) | (uint16_t)Referee_data.Usart6_Rx_Buff[index++];
//            Robot_All_Info.power_heat_data.shooter_42mm_barrel_heat = ((uint16_t)Referee_data.Usart6_Rx_Buff[index++] << 8) | (uint16_t)Referee_data.Usart6_Rx_Buff[index++];
//            break;

//        case robot_pos:
//            index = 7;
//            Robot_All_Info.robot_pos.x = ((uint32_t)Referee_data.Usart6_Rx_Buff[index++] << 24) |
//                                          ((uint32_t)Referee_data.Usart6_Rx_Buff[index++] << 16) |
//                                          ((uint32_t)Referee_data.Usart6_Rx_Buff[index++] << 8) |
//                                          (uint32_t)Referee_data.Usart6_Rx_Buff[index++];
//            Robot_All_Info.robot_pos.y = ((uint32_t)Referee_data.Usart6_Rx_Buff[index++] << 24) |
//                                          ((uint32_t)Referee_data.Usart6_Rx_Buff[index++] << 16) |
//                                          ((uint32_t)Referee_data.Usart6_Rx_Buff[index++] << 8) |
//                                          (uint32_t)Referee_data.Usart6_Rx_Buff[index++];
//            Robot_All_Info.robot_pos.angle = ((uint32_t)Referee_data.Usart6_Rx_Buff[index++] << 24) |
//                                             ((uint32_t)Referee_data.Usart6_Rx_Buff[index++] << 16) |
//                                             ((uint32_t)Referee_data.Usart6_Rx_Buff[index++] << 8) |
//                                             (uint32_t)Referee_data.Usart6_Rx_Buff[index++];
//            break;

//        case hurt_data:
//            index = 7;
//            // 强制转换成uint32_t以避免类型不匹配
//            Robot_All_Info.hurt_data.armor_id = Referee_data.Usart6_Rx_Buff[index++];
//            Robot_All_Info.hurt_data.HP_deduction_reason = Referee_data.Usart6_Rx_Buff[index++];
//            break;

//        case projectile_allowance:
//            index = 7;
//            // 强制转换成uint32_t以避免类型不匹配
//            Robot_All_Info.projectile_allowance.projectile_allowance_17mm = ((uint16_t)Referee_data.Usart6_Rx_Buff[index++] << 8) | (uint16_t)Referee_data.Usart6_Rx_Buff[index++];
//            Robot_All_Info.projectile_allowance.projectile_allowance_42mm = ((uint16_t)Referee_data.Usart6_Rx_Buff[index++] << 8) | (uint16_t)Referee_data.Usart6_Rx_Buff[index++];
//            Robot_All_Info.projectile_allowance.remaining_gold_coin = ((uint16_t)Referee_data.Usart6_Rx_Buff[index++] << 8) | (uint16_t)Referee_data.Usart6_Rx_Buff[index++];
//            break;

//        case rfid_status:
//            index = 7;
//            // 强制转换成uint32_t以避免类型不匹配
//            Robot_All_Info.rfid_status.rfid_status = ((uint32_t)Referee_data.Usart6_Rx_Buff[index++] << 24) |
//                                                    ((uint32_t)Referee_data.Usart6_Rx_Buff[index++] << 16) |
//                                                    ((uint32_t)Referee_data.Usart6_Rx_Buff[index++] << 8) |
//                                                    (uint32_t)Referee_data.Usart6_Rx_Buff[index++];
//            break;

//        case dart_client_cmd:
//            index = 7;
//            // 强制转换成uint32_t以避免类型不匹配
//            Robot_All_Info.dart_client_cmd.dart_launch_opening_status = Referee_data.Usart6_Rx_Buff[index++];
//            Robot_All_Info.dart_client_cmd.reserved = Referee_data.Usart6_Rx_Buff[index++];
//            Robot_All_Info.dart_client_cmd.target_change_time = ((uint16_t)Referee_data.Usart6_Rx_Buff[index++] << 8) | (uint16_t)Referee_data.Usart6_Rx_Buff[index++];
//            Robot_All_Info.dart_client_cmd.latest_launch_cmd_time = ((uint16_t)Referee_data.Usart6_Rx_Buff[index++] << 8) | (uint16_t)Referee_data.Usart6_Rx_Buff[index++];
//            break;
//             
//        case ground_robot_position:
//            index = 7;
//            // 解包 hero_x 位置
//            Robot_All_Info.ground_robot_position.hero_x = ((uint32_t)Referee_data.Usart6_Rx_Buff[index++] << 24) |
//                                                           ((uint32_t)Referee_data.Usart6_Rx_Buff[index++] << 16) |
//                                                           ((uint32_t)Referee_data.Usart6_Rx_Buff[index++] << 8) |
//                                                           (uint32_t)Referee_data.Usart6_Rx_Buff[index++];
//            // 解包 hero_y 位置
//            Robot_All_Info.ground_robot_position.hero_y = ((uint32_t)Referee_data.Usart6_Rx_Buff[index++] << 24) |
//                                                           ((uint32_t)Referee_data.Usart6_Rx_Buff[index++] << 16) |
//                                                           ((uint32_t)Referee_data.Usart6_Rx_Buff[index++] << 8) |
//                                                           (uint32_t)Referee_data.Usart6_Rx_Buff[index++];
//            // 解包 engineer_x 位置
//            Robot_All_Info.ground_robot_position.engineer_x = ((uint32_t)Referee_data.Usart6_Rx_Buff[index++] << 24) |
//                                                              ((uint32_t)Referee_data.Usart6_Rx_Buff[index++] << 16) |
//                                                              ((uint32_t)Referee_data.Usart6_Rx_Buff[index++] << 8) |
//                                                              (uint32_t)Referee_data.Usart6_Rx_Buff[index++];
//            // 解包 engineer_y 位置
//            Robot_All_Info.ground_robot_position.engineer_y = ((uint32_t)Referee_data.Usart6_Rx_Buff[index++] << 24) |
//                                                              ((uint32_t)Referee_data.Usart6_Rx_Buff[index++] << 16) |
//                                                              ((uint32_t)Referee_data.Usart6_Rx_Buff[index++] << 8) |
//                                                              (uint32_t)Referee_data.Usart6_Rx_Buff[index++];
//            // 解包 standard_3_x 位置
//            Robot_All_Info.ground_robot_position.standard_3_x = ((uint32_t)Referee_data.Usart6_Rx_Buff[index++] << 24) |
//                                                                 ((uint32_t)Referee_data.Usart6_Rx_Buff[index++] << 16) |
//                                                                 ((uint32_t)Referee_data.Usart6_Rx_Buff[index++] << 8) |
//                                                                 (uint32_t)Referee_data.Usart6_Rx_Buff[index++];
//            // 解包 standard_3_y 位置
//            Robot_All_Info.ground_robot_position.standard_3_y = ((uint32_t)Referee_data.Usart6_Rx_Buff[index++] << 24) |
//                                                                 ((uint32_t)Referee_data.Usart6_Rx_Buff[index++] << 16) |
//                                                                 ((uint32_t)Referee_data.Usart6_Rx_Buff[index++] << 8) |
//                                                                 (uint32_t)Referee_data.Usart6_Rx_Buff[index++];
//            // 解包 standard_4_x 位置
//            Robot_All_Info.ground_robot_position.standard_4_x = ((uint32_t)Referee_data.Usart6_Rx_Buff[index++] << 24) |
//                                                                 ((uint32_t)Referee_data.Usart6_Rx_Buff[index++] << 16) |
//                                                                 ((uint32_t)Referee_data.Usart6_Rx_Buff[index++] << 8) |
//                                                                 (uint32_t)Referee_data.Usart6_Rx_Buff[index++];
//            // 解包 standard_4_y 位置
//            Robot_All_Info.ground_robot_position.standard_4_y = ((uint32_t)Referee_data.Usart6_Rx_Buff[index++] << 24) |
//                                                                 ((uint32_t)Referee_data.Usart6_Rx_Buff[index++] << 16) |
//                                                                 ((uint32_t)Referee_data.Usart6_Rx_Buff[index++] << 8) |
//                                                                 (uint32_t)Referee_data.Usart6_Rx_Buff[index++];
//            // 解包 reserved1 值
//            Robot_All_Info.ground_robot_position.reserved1 = ((uint32_t)Referee_data.Usart6_Rx_Buff[index++] << 24) |
//                                                              ((uint32_t)Referee_data.Usart6_Rx_Buff[index++] << 16) |
//                                                              ((uint32_t)Referee_data.Usart6_Rx_Buff[index++] << 8) |
//                                                              (uint32_t)Referee_data.Usart6_Rx_Buff[index++];
//            // 解包 reserved2 值
//            Robot_All_Info.ground_robot_position.reserved2 = ((uint32_t)Referee_data.Usart6_Rx_Buff[index++] << 24) |
//                                                              ((uint32_t)Referee_data.Usart6_Rx_Buff[index++] << 16) |
//                                                              ((uint32_t)Referee_data.Usart6_Rx_Buff[index++] << 8) |
//                                                              (uint32_t)Referee_data.Usart6_Rx_Buff[index++];
//            break;
//        case radar_mark_data:
//            index = 7;
//            // 强制转换成uint32_t以避免类型不匹配
//            Robot_All_Info.radar_mark_data.mark_progress = Referee_data.Usart6_Rx_Buff[index++];
//            break;            

//        case sentry_info:
//            index = 7;
//            // 强制转换成uint32_t以避免类型不匹配
//            Robot_All_Info.sentry_info.sentry_info = ((uint32_t)Referee_data.Usart6_Rx_Buff[index++] << 24) |
//                                                           ((uint32_t)Referee_data.Usart6_Rx_Buff[index++] << 16) |
//                                                           ((uint32_t)Referee_data.Usart6_Rx_Buff[index++] << 8) |
//                                                           (uint32_t)Referee_data.Usart6_Rx_Buff[index++];
//            Robot_All_Info.sentry_info.sentry_info_2 = ((uint16_t)Referee_data.Usart6_Rx_Buff[index++] << 8) | (uint16_t)Referee_data.Usart6_Rx_Buff[index++];
//            break;
//        
//        case radar_info:
//            index = 7;
//            // 强制转换成uint32_t以避免类型不匹配
//            Robot_All_Info.radar_info.radar_info = Referee_data.Usart6_Rx_Buff[index++];
//            break;
//    }
}



/**
 * @brief   同上
 * 
 * @return  无
 * 
 * @note    双缓存区，避免数据覆盖
 */ 
void Que_Extract_Referee_Info(uint8_t *Data, Com_Id Cmd) {
    uint8_t index;
    switch (Cmd) {
        case game_status:
            index = 0;
            Robot_All_Info.game_status.game_type = Data[0] & 0x0f;
            Robot_All_Info.game_status.game_progress = (Data[0] >> 4) & 0x0f;
            // 修正 uint16_t 字段：低位先行
            Robot_All_Info.game_status.stage_remain_time = (uint16_t)Data[1] | ((uint16_t)Data[2] << 8);
            // 修正 uint64_t 字段：低位先行（依次读取 Byte0-Byte7，左移 0-56 位）
            Robot_All_Info.game_status.SyncTimeStamp = 
                (uint64_t)Data[3] | 
                ((uint64_t)Data[4] << 8) |
                ((uint64_t)Data[5] << 16) |
                ((uint64_t)Data[6] << 24) |
                ((uint64_t)Data[7] << 32) |
                ((uint64_t)Data[8] << 40) |
                ((uint64_t)Data[9] << 48) |
                ((uint64_t)Data[10] << 56);
            break;

        case game_robot_state:
            index = 0;
            // 所有 uint16_t 字段修正为低位先行
            Robot_All_Info.Robot_Data.red_1_robot_HP = (uint16_t)Data[0] | ((uint16_t)Data[1] << 8);
            Robot_All_Info.Robot_Data.red_2_robot_HP = (uint16_t)Data[2] | ((uint16_t)Data[3] << 8);
            Robot_All_Info.Robot_Data.red_3_robot_HP = (uint16_t)Data[4] | ((uint16_t)Data[5] << 8);
            Robot_All_Info.Robot_Data.red_4_robot_HP = (uint16_t)Data[6] | ((uint16_t)Data[7] << 8);
            Robot_All_Info.Robot_Data.reserved1 = (uint16_t)Data[8] | ((uint16_t)Data[9] << 8);
            Robot_All_Info.Robot_Data.red_7_robot_HP = (uint16_t)Data[10] | ((uint16_t)Data[11] << 8);
            Robot_All_Info.Robot_Data.red_outpost_HP = (uint16_t)Data[12] | ((uint16_t)Data[13] << 8);
            Robot_All_Info.Robot_Data.red_base_HP = (uint16_t)Data[14] | ((uint16_t)Data[15] << 8);
            Robot_All_Info.Robot_Data.blue_1_robot_HP = (uint16_t)Data[16] | ((uint16_t)Data[17] << 8);
            Robot_All_Info.Robot_Data.blue_2_robot_HP = (uint16_t)Data[18] | ((uint16_t)Data[19] << 8);
            Robot_All_Info.Robot_Data.blue_3_robot_HP = (uint16_t)Data[20] | ((uint16_t)Data[21] << 8);
            Robot_All_Info.Robot_Data.blue_4_robot_HP = (uint16_t)Data[22] | ((uint16_t)Data[23] << 8);
            Robot_All_Info.Robot_Data.reserved2 = (uint16_t)Data[24] | ((uint16_t)Data[25] << 8);
            Robot_All_Info.Robot_Data.blue_7_robot_HP = (uint16_t)Data[26] | ((uint16_t)Data[27] << 8);
            Robot_All_Info.Robot_Data.blue_outpost_HP = (uint16_t)Data[28] | ((uint16_t)Data[29] << 8);
            Robot_All_Info.Robot_Data.blue_base_HP = (uint16_t)Data[30] | ((uint16_t)Data[31] << 8);
            break;

        case event_data:
            index = 0;
            // 修正 uint32_t 字段：低位先行
            Robot_All_Info.event_data.event_data = 
                (uint32_t)Data[0] | 
                ((uint32_t)Data[1] << 8) |
                ((uint32_t)Data[2] << 16) |
                ((uint32_t)Data[3] << 24);
            break;

        case dart_info:
            index = 0;
            Robot_All_Info.dart_info.dart_remaining_time = Data[0];
            // 修正 uint16_t 字段
            Robot_All_Info.dart_info.dart_info = (uint16_t)Data[1] | ((uint16_t)Data[2] << 8);
            break;

        case robot_status:
            index = 0;
            Robot_All_Info.robot_status.robot_id = Data[0];
            Robot_All_Info.robot_status.robot_level = Data[1];
            // 修正所有 uint16_t 字段
            Robot_All_Info.robot_status.current_HP = (uint16_t)Data[2] | ((uint16_t)Data[3] << 8);
            Robot_All_Info.robot_status.maximum_HP = (uint16_t)Data[4] | ((uint16_t)Data[5] << 8);
            Robot_All_Info.robot_status.shooter_barrel_cooling_value = (uint16_t)Data[6] | ((uint16_t)Data[7] << 8);
            Robot_All_Info.robot_status.shooter_barrel_heat_limit = (uint16_t)Data[8] | ((uint16_t)Data[9] << 8);
            Robot_All_Info.robot_status.chassis_power_limit = (uint16_t)Data[10] | ((uint16_t)Data[11] << 8);
            Robot_All_Info.robot_status.power_management_gimbal_output = Data[12];
            Robot_All_Info.robot_status.power_management_chassis_output = Data[13];
            Robot_All_Info.robot_status.power_management_shooter_output = Data[14];
            break;

        case power_heat_data:
            index = 0;
            // 修正 uint16_t 字段
            Robot_All_Info.power_heat_data.reserved1 = (uint16_t)Data[0] | ((uint16_t)Data[1] << 8);
            Robot_All_Info.power_heat_data.reserved2 = (uint16_t)Data[2] | ((uint16_t)Data[3] << 8);
            // 修正 uint32_t 字段
            Robot_All_Info.power_heat_data.reserved3 = 
                (uint32_t)Data[4] | 
                ((uint32_t)Data[5] << 8) |
                ((uint32_t)Data[6] << 16) |
                ((uint32_t)Data[7] << 24);
            Robot_All_Info.power_heat_data.buffer_energy = (uint16_t)Data[8] | ((uint16_t)Data[9] << 8);
            Robot_All_Info.power_heat_data.shooter_17mm_1_barrel_heat = (uint16_t)Data[10] | ((uint16_t)Data[11] << 8);
            Robot_All_Info.power_heat_data.shooter_17mm_2_barrel_heat = (uint16_t)Data[12] | ((uint16_t)Data[13] << 8);
            Robot_All_Info.power_heat_data.shooter_42mm_barrel_heat = (uint16_t)Data[14] | ((uint16_t)Data[15] << 8);
            break;

        case robot_pos:
            index = 0;
            // 修正 uint32_t 字段
            Robot_All_Info.robot_pos.x = 
                (uint32_t)Data[0] | 
                ((uint32_t)Data[1] << 8) |
                ((uint32_t)Data[2] << 16) |
                ((uint32_t)Data[3] << 24);
            Robot_All_Info.robot_pos.y = 
                (uint32_t)Data[4] | 
                ((uint32_t)Data[5] << 8) |
                ((uint32_t)Data[6] << 16) |
                ((uint32_t)Data[7] << 24);
            Robot_All_Info.robot_pos.angle = 
                (uint32_t)Data[8] | 
                ((uint32_t)Data[9] << 8) |
                ((uint32_t)Data[10] << 16) |
                ((uint32_t)Data[11] << 24);
            break;

        case hurt_data:
            index = 0;
            Robot_All_Info.hurt_data.armor_id = Data[0] & 0x0f;
            Robot_All_Info.hurt_data.HP_deduction_reason = (Data[0]>>4) & 0x0f;
            break;

        case projectile_allowance:
            index = 0;
            // 修正 uint16_t 字段
            Robot_All_Info.projectile_allowance.projectile_allowance_17mm = (uint16_t)Data[0] | ((uint16_t)Data[1] << 8);
            Robot_All_Info.projectile_allowance.projectile_allowance_42mm = (uint16_t)Data[2] | ((uint16_t)Data[3] << 8);
            Robot_All_Info.projectile_allowance.remaining_gold_coin = (uint16_t)Data[4] | ((uint16_t)Data[5] << 8);
            break;

        case rfid_status:
            index = 0;
            // 修正 uint32_t 字段
            Robot_All_Info.rfid_status.rfid_status = 
                (uint32_t)Data[0] | 
                ((uint32_t)Data[1] << 8) |
                ((uint32_t)Data[2] << 16) |
                ((uint32_t)Data[3] << 24);
            break;

        case dart_client_cmd:
            index = 0;
            Robot_All_Info.dart_client_cmd.dart_launch_opening_status = Data[0];
            Robot_All_Info.dart_client_cmd.reserved = Data[1];
            // 修正 uint16_t 字段
            Robot_All_Info.dart_client_cmd.target_change_time = (uint16_t)Data[2] | ((uint16_t)Data[3] << 8);
            Robot_All_Info.dart_client_cmd.latest_launch_cmd_time = (uint16_t)Data[4] | ((uint16_t)Data[5] << 8);
            break;

        case ground_robot_position:
            index = 0;
            // 修正所有 uint32_t 字段
            Robot_All_Info.ground_robot_position.hero_x = 
                (uint32_t)Data[0] | 
                ((uint32_t)Data[1] << 8) |
                ((uint32_t)Data[2] << 16) |
                ((uint32_t)Data[3] << 24);
            Robot_All_Info.ground_robot_position.hero_y = 
                (uint32_t)Data[4] | 
                ((uint32_t)Data[5] << 8) |
                ((uint32_t)Data[6] << 16) |
                ((uint32_t)Data[7] << 24);
            Robot_All_Info.ground_robot_position.engineer_x = 
                (uint32_t)Data[8] | 
                ((uint32_t)Data[9] << 8) |
                ((uint32_t)Data[10] << 16) |
                ((uint32_t)Data[11] << 24);
            Robot_All_Info.ground_robot_position.engineer_y = 
                (uint32_t)Data[12] | 
                ((uint32_t)Data[13] << 8) |
                ((uint32_t)Data[14] << 16) |
                ((uint32_t)Data[15] << 24);
            Robot_All_Info.ground_robot_position.standard_3_x = 
                (uint32_t)Data[16] | 
                ((uint32_t)Data[17] << 8) |
                ((uint32_t)Data[18] << 16) |
                ((uint32_t)Data[19] << 24);
            Robot_All_Info.ground_robot_position.standard_3_y = 
                (uint32_t)Data[20] | 
                ((uint32_t)Data[21] << 8) |
                ((uint32_t)Data[22] << 16) |
                ((uint32_t)Data[23] << 24);
            Robot_All_Info.ground_robot_position.standard_4_x = 
                (uint32_t)Data[24] | 
                ((uint32_t)Data[25] << 8) |
                ((uint32_t)Data[26] << 16) |
                ((uint32_t)Data[27] << 24);
            Robot_All_Info.ground_robot_position.standard_4_y = 
                (uint32_t)Data[28] | 
                ((uint32_t)Data[29] << 8) |
                ((uint32_t)Data[30] << 16) |
                ((uint32_t)Data[31] << 24);
            Robot_All_Info.ground_robot_position.reserved1 = 
                (uint32_t)Data[32] | 
                ((uint32_t)Data[33] << 8) |
                ((uint32_t)Data[34] << 16) |
                ((uint32_t)Data[35] << 24);
            Robot_All_Info.ground_robot_position.reserved2 = 
                (uint32_t)Data[36] | 
                ((uint32_t)Data[37] << 8) |
                ((uint32_t)Data[38] << 16) |
                ((uint32_t)Data[39] << 24);
            break;

        case radar_mark_data:
            index = 0;
            Robot_All_Info.radar_mark_data.mark_progress = Data[0];
            break;

        case sentry_info:
            index = 0;
            // 修正 uint32_t 和 uint16_t 字段
            Robot_All_Info.sentry_info.sentry_info = 
                (uint32_t)Data[0] | 
                ((uint32_t)Data[1] << 8) |
                ((uint32_t)Data[2] << 16) |
                ((uint32_t)Data[3] << 24);
            Robot_All_Info.sentry_info.sentry_info_2 = (uint16_t)Data[4] | ((uint16_t)Data[5] << 8);
            break;

        case radar_info:
            index = 0;
            Robot_All_Info.radar_info.radar_info = Data[index++];
            break;
    }
}




/**
 * @brief   以下均为CRC校验内容，无需理会
 * 
 * @return  无
 * 
 * @note    
 */ 


//void Deal_Refere_System(){
//    static int Data_Z;
//    static uint8_t Com_ID;
//    static Data_Len;
//    for(int i=0;i<Referee_data.Usart6_Rx_Index;i++)
//    {
//        if(Referee_data.Usart6_Rx_Buff_Copy[i]==0xA5)
//            Data_Z++;
//    }
//    for(int j=0;j<Data_Z;j++){
//        static int count=0;
//        Data_Len = Referee_data.Usart6_Rx_Buff_Copy[count+1]<<8|Referee_data.Usart6_Rx_Buff_Copy[count+2];
//        Com_ID=Referee_data.Usart6_Rx_Buff_Copy[count+5]<<8||Referee_data.Usart6_Rx_Buff_Copy[count+6];
//        Data_Len=Data_Len+9;
//        count=count+Data_Len;
//    }


//}


const unsigned char CRC8_TAB[256] = 
{ 
0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41, 
0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc, 0x23, 
0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62, 0xbe, 0xe0, 
0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff, 0x46, 0x18, 0xfa, 
0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07, 0xdb, 0x85, 0x67, 0x39, 
0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a, 0x65, 0x3b, 0xd9, 0x87, 0x04, 
0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24, 0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 
0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9, 
0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd, 0x11, 
0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50, 0xaf, 0xf1, 
0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee, 0x32, 0x6c, 0x8e, 
0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73, 0xca, 0x94, 0x76, 0x28, 
0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b, 0x57, 0x09, 0xeb, 0xb5, 0x36, 
0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16, 0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 
0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8, 
0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35, 
};

// 计算CRC8校验值
uint8_t Get_CRC8_Check_Sum(uint8_t *pchMessage, uint32_t dwLength, uint8_t ucCRC8) {
    while (dwLength--) {
        ucCRC8 = CRC8_TAB[ucCRC8 ^ *pchMessage];
        pchMessage++;
    }
    return ucCRC8;
}

// 校验数据CRC8是否合法
uint32_t Verify_CRC8_Check_Sum(uint8_t *pchMessage, uint32_t dwLength) {
    if (pchMessage == NULL || dwLength <= 2) return 0;
    uint8_t ucExpected = Get_CRC8_Check_Sum(pchMessage, dwLength - 1, CRC8_INIT);
    return (ucExpected == pchMessage[dwLength - 1]);
}

// 追加CRC8校验码到数据末尾
void Append_CRC8_Check_Sum(uint8_t *pchMessage, uint32_t dwLength) {
    if (pchMessage == NULL || dwLength <= 2) return;
    uint8_t ucCRC = Get_CRC8_Check_Sum(pchMessage, dwLength - 1, CRC8_INIT);
    pchMessage[dwLength - 1] = ucCRC;
}


/**
 * @brief 从缓冲区解析一帧数据
 * @param buffer    输入缓冲区
 * @param buffer_len 缓冲区长度
 * @param frame     输出帧结构体
 * @param frame_len 输出完整帧长度（用于移动缓冲区偏移）
 * @return true     解析成功
 * @return false    解析失败（数据不完整或校验错误）
 */
unsigned char parse_frame(const uint8_t *buffer, size_t buffer_len, Frame *frame, size_t *frame_len) {
    // 检查最小帧长度：帧头(5) + cmd_id(2) + crc16(2) = 9字节
    if (buffer_len < 9) return 0;

		memcpy(Test_S,buffer,sizeof(Test_S));
	
    // 1. 检查SOF（假设SOF固定为0xA5）
    if (buffer[0] != 0xA5) return 0;
	
   test_Break++;
	if(test_Break>50000)test_Break=0;
    // 2. 解析帧头
    frame->header.SOF         = buffer[0];
    frame->header.data_length = (buffer[2] << 8) | buffer[1];  // 小端序转主机序
    frame->header.seq         = buffer[3];
    frame->header.crc8        = buffer[4];
	

//	Valll=

    // 3. 校验帧头CRC8（校验范围：SOF + data_length + seq）
    uint8_t header_crc = frame_header_crc_check(buffer);  // 计算前4字节的CRC8
//    if (header_crc != frame->header.crc8) return 0;

    // 4. 计算完整帧长度：帧头(5) + cmd_id(2) + data(N) + crc16(2)
    *frame_len = 5 + 2 + frame->header.data_length + 2;
    if (buffer_len < *frame_len) return 0;  // 数据不完整
    // 5. 解析cmd_id（假设小端序）
    frame->cmd_id = (buffer[6] << 8) | buffer[5];

	Com_Test= frame->cmd_id;
    // 6. 解析数据部分
    frame->data = (uint8_t*)(buffer + 7);  // 数据起始位置：buffer[5+2=7]

    // 7. 解析帧尾CRC16（小端序）
    //frame->crc16 = (buffer[*frame_len - 1] << 8) | buffer[*frame_len - 2];

    // 8. 校验整帧CRC16（校验范围：从SOF到数据结束，不包含帧尾CRC16）
   // uint16_t calc_crc = crc16(buffer, *frame_len - 2);
    //if (calc_crc != frame->crc16) return false;

    return 1;
}



/**
 * @brief 处理接收到的数据包（可能包含多帧或残帧）
 * @param input_buffer  输入数据缓冲区
 * @param input_len     输入数据长度
 * @param process_frame 帧处理回调函数
 */
void process_packet(uint8_t *input_buffer, size_t input_len, void (*process_frame)(Frame*)) {
    static uint8_t buffer[2048];       // 静态缓冲区处理残帧
    static size_t buffer_len = 0;      // 当前缓冲区数据长度
    
    // 1. 将新数据追加到缓冲区
size_t free_space = sizeof(buffer) - buffer_len;
if (input_len > free_space) {
// 处理溢出：丢弃旧数据（保留最新数据）或直接清空缓冲区
// 示例：保留足够空间存放新数据（需根据协议调整）
if (input_len <= sizeof(buffer)) {
memmove(buffer, buffer + (buffer_len - input_len), input_len); // 丢弃旧数据
buffer_len = input_len;
} else {
buffer_len = 0; // 直接清空
}
}

// 2. 安全追加新数据
memcpy(buffer + buffer_len, input_buffer, input_len);
buffer_len += input_len;




    size_t offset = 0;
    while (offset < buffer_len) {
        Frame frame;
        size_t frame_len;

        // 2. 尝试解析一帧
//			while(*(buffer + offset) != 0xa5)offset++;if(offset>25)break;
        if (parse_frame(buffer + offset, buffer_len - offset, &frame, &frame_len)) {
            // 3. 调用回调函数处理有效帧
            process_frame(&frame);
            offset += frame_len;  // 移动偏移到下一帧
        } else {
            // 4. 解析失败：可能是残帧，保留未处理数据
            if (offset > 0) {
                memmove(buffer, buffer + offset, buffer_len - offset);
                buffer_len -= offset;
                offset = 0;
            }
            break;
        }
    }
    // 5. 清空已处理数据
    if (offset >= buffer_len) {
        buffer_len = 0;
    }
}

void handle_frame(Frame *frame) { 
    Que_Extract_Referee_Info(frame->data, frame->cmd_id);
}