#include "bsp_can.h"
#include "main.h"


extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
void can_filter_init(void)
{
    CAN_FilterTypeDef can_filter_st;
    HAL_StatusTypeDef status;             // 用于检查HAL函数执行结果

    /************************** CAN1 配置（FIFO0中断接收电机数据） **************************/
    // 过滤器核心配置：32位掩码模式，接收所有标准ID（需与发送端ID类型匹配）
    can_filter_st.FilterActivation = ENABLE;          // 启用过滤器
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK; // 掩码匹配模式（灵活过滤ID）
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;// 32位过滤器宽度
    
    can_filter_st.FilterIdHigh = 0x0000;              // 标准ID高16位（11位标准ID存于高11位）
    can_filter_st.FilterIdLow = 0x0000;               // 标准ID标识（必须添加，否则消息被过滤）
    can_filter_st.FilterMaskIdHigh = 0x0000;          // 掩码全0 = 接收所有标准ID
    can_filter_st.FilterMaskIdLow = 0x0000;           // 掩码匹配ID类型（标准ID）
    // CAN1专属过滤器银行（0~13），消息存入FIFO0
    can_filter_st.FilterBank = 0;                     // CAN1使用过滤器银行0
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;// 匹配消息存入FIFO0
    can_filter_st.SlaveStartFilterBank = 14;          // 划分CAN2过滤器起始银行（固定14）
    
    // 配置CAN1过滤器并检查结果
    status = HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    if (status != HAL_OK) { while(1); } // 配置失败（如过滤器银行冲突），需排查

    // 启动CAN1 + 使能FIFO0消息待处理中断（触发HAL_CAN_RxFifo0MsgPendingCallback）
    status = HAL_CAN_Start(&hcan1);
    if (status != HAL_OK) { while(1); }
    status = HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    if (status != HAL_OK) { while(1); }


    /************************** CAN2 配置（FIFO1中断接收底板数据） **************************/
    // 复用过滤器结构体，仅修改CAN2专属参数
    can_filter_st.FilterBank = 14;                    // CAN2专属过滤器银行（14~27）
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO1;// 匹配消息存入FIFO1
    
    // 配置CAN2过滤器并检查结果
    status = HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
    if (status != HAL_OK) { while(1); }

    // 启动CAN2 + 使能FIFO1消息待处理中断（触发HAL_CAN_RxFifo1MsgPendingCallback）
    status = HAL_CAN_Start(&hcan2);
    if (status != HAL_OK) { while(1); }
    status = HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);
    if (status != HAL_OK) { while(1); }
}
