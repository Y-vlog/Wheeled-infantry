#ifndef __REFEREE_SYSTEM_H__
#define __REFEREE_SYSTEM_H__

#include "stdint.h"
#include "dma.h"

#define Referee_system_Usart &huart1
#define UART6_RX_BUF_SIZE 1024

#define CRC8_INIT 0xFF  // 初始值
#define CRC8_POLY 0x31  // 多项式 x^8 + x^5 + x^4 + 1 (对应生成多项式表)
#define CRC16_POLYNOMIAL 0x1021  // 标准CRC16多项式

typedef struct {
    uint8_t Usart6_Rx_Buff[UART6_RX_BUF_SIZE];  // 接收缓冲区
    uint8_t Usart6_Rx_Index;                    // 接收数据索引
    uint8_t Usart6_Rx_Buff_Copy[UART6_RX_BUF_SIZE]; // 数据副本缓冲区
	  uint8_t Usart6_Rx_Copy_Index; // 数据副本缓冲区
} Usart_Referee_data;// USART接收数据缓冲区结构


typedef enum {
    game_status=0x0001,
    game_robot_state = 0x0003,  
    event_data = 0x0101,  
    dart_info = 0x0105,    
    robot_status = 0x0201,    
    power_heat_data = 0x0202,
    robot_pos = 0x0203,
    hurt_data = 0x0206,
    projectile_allowance = 0x0208,
    rfid_status = 0x0209,
    dart_client_cmd = 0x020A,
    ground_robot_position = 0x020B,
    radar_mark_data = 0x020C,
    sentry_info = 0x020D,
    radar_info = 0x020E
} Com_Id;//裁判系统发来数据相关指令码


typedef  struct 
{ 
 uint8_t game_type; 
 uint8_t game_progress; 
 uint16_t stage_remain_time; 
 uint64_t SyncTimeStamp; 
}game_status_t;


// 游戏机器人血量数据结构
typedef struct {
    uint16_t red_1_robot_HP; 
    uint16_t red_2_robot_HP; 
    uint16_t red_3_robot_HP; 
    uint16_t red_4_robot_HP; 
    uint16_t reserved1; 
    uint16_t red_7_robot_HP; 
    uint16_t red_outpost_HP; 
    uint16_t red_base_HP; 
    uint16_t blue_1_robot_HP; 
    uint16_t blue_2_robot_HP; 
    uint16_t blue_3_robot_HP; 
    uint16_t blue_4_robot_HP; 
    uint16_t reserved2; 
    uint16_t blue_7_robot_HP; 
    uint16_t blue_outpost_HP; 
    uint16_t blue_base_HP; 
} game_robot_HP_t;

// 事件数据结构
typedef struct { 
    uint32_t event_data; 
} event_data_t;

// 飞镖信息结构
typedef struct { 
    uint8_t dart_remaining_time; 
    uint16_t dart_info; 
} dart_info_t;

// 机器人状态结构
typedef struct { 
    uint8_t robot_id; 
    uint8_t robot_level; 
    uint16_t current_HP; 
    uint16_t maximum_HP; 
    uint16_t shooter_barrel_cooling_value; 
    uint16_t shooter_barrel_heat_limit; 
    uint16_t chassis_power_limit; 
    uint8_t power_management_gimbal_output; 
    uint8_t power_management_chassis_output; 
    uint8_t power_management_shooter_output; 
} robot_status_t;

// 能量与热量数据结构
typedef struct { 
    uint16_t reserved1;
    uint16_t reserved2; 
    float reserved3; 
    uint16_t buffer_energy; 
    uint16_t shooter_17mm_1_barrel_heat; 
    uint16_t shooter_17mm_2_barrel_heat; 
    uint16_t shooter_42mm_barrel_heat; 
} power_heat_data_t;

// 机器人位置结构
typedef struct { 
    float x; 
    float y; 
    float angle; 
} robot_pos_t;

// 受伤数据结构
typedef struct { 
    uint8_t armor_id;
    uint8_t HP_deduction_reason;
} hurt_data_t;

// 弹药允许量数据结构
typedef struct { 
    uint16_t projectile_allowance_17mm; 
    uint16_t projectile_allowance_42mm; 
    uint16_t remaining_gold_coin; 
} projectile_allowance_t;

// RFID状态数据结构
typedef struct { 
    uint32_t rfid_status; 
} rfid_status_t;

// 飞镖客户端命令数据结构
typedef struct { 
    uint8_t dart_launch_opening_status; 
    uint8_t reserved; 
    uint16_t target_change_time; 
    uint16_t latest_launch_cmd_time; 
} dart_client_cmd_t;

// 地面机器人位置数据结构
typedef struct { 
    float hero_x; 
    float hero_y; 
    float engineer_x; 
    float engineer_y; 
    float standard_3_x; 
    float standard_3_y; 
    float standard_4_x; 
    float standard_4_y; 
    float reserved1; 
    float reserved2; 
} ground_robot_position_t;

// 雷达标记数据结构
typedef struct { 
    uint8_t mark_progress; 
} radar_mark_data_t;

// 哨兵信息数据结构
typedef struct { 
    uint32_t sentry_info; 
    uint16_t sentry_info_2; 
} sentry_info_t;

// 雷达信息数据结构
typedef struct { 
    uint8_t radar_info; 
} radar_info_t;

// 机器人所有信息结构
typedef struct { 
    game_status_t game_status;
    game_robot_HP_t Robot_Data;
    event_data_t event_data;
    dart_info_t dart_info;
    robot_status_t robot_status;
    power_heat_data_t power_heat_data;
    robot_pos_t robot_pos;
    hurt_data_t hurt_data;
    projectile_allowance_t projectile_allowance;
    rfid_status_t rfid_status;
    dart_client_cmd_t dart_client_cmd;
    ground_robot_position_t ground_robot_position;
    radar_mark_data_t radar_mark_data;
    sentry_info_t sentry_info;
    radar_info_t radar_info;
} RobotInfo;

extern RobotInfo Robot_All_Info;
extern Usart_Referee_data Referee_data;
extern DMA_HandleTypeDef hdma_usart6_rx;
// 函数声明：CRC8校验
//uint8_t frame_header_crc_check(uint8_t *data);
uint8_t frame_header_crc_check(const uint8_t *data);

// 函数声明：CRC16校验
uint16_t Referee_data_check(uint8_t *data);

// 函数声明：提取裁判信息
void Extract_Referee_Info(Com_Id Cmd);

// 函数声明，将数据存在相关结构体
void Que_Extract_Referee_Info(uint8_t *Data,Com_Id Cmd);

//  函数声明，初始化裁判系统串口
void Referee_Usart_Init(void);


void Referee_Usart_Data_Deal(void);



// CRC8计算函数
uint8_t Get_CRC8_Check_Sum(uint8_t *pchMessage, uint32_t dwLength, uint8_t ucCRC8);

// CRC8校验函数
uint32_t Verify_CRC8_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);

// 追加CRC8校验码到数据末尾
void Append_CRC8_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);

//在任务中会调用
void Referee_Task(void);




// 帧头定义
typedef struct {
    uint8_t   SOF;          // 帧起始符 (1字节)
    uint16_t  data_length;  // 数据长度 (2字节，小端序)
    uint8_t   seq;          // 包序号 (1字节)
    uint8_t   crc8;         // 帧头CRC8校验 (1字节)
} FrameHeader;

// 完整帧定义
typedef struct {
    FrameHeader header;      // 帧头
    uint16_t    cmd_id;      // 命令ID (2字节)
    uint8_t    *data;        // 数据部分 (长度由header.data_length决定)
    uint16_t    crc16;       // 整帧CRC16校验 (2字节，小端序)
} Frame;
void process_packet(uint8_t *input_buffer, size_t input_len, void (*process_frame)(Frame*));
void handle_frame(Frame *frame);


extern uint8_t Turn_Finsh_Flag;
#endif

