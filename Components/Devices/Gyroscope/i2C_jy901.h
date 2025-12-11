/**
  ****************************(C) COPYRIGHT 2024 征途****************************
  * @file       i2C_jy901.c/h
  * @brief      
  *             这里是jy901的驱动函数，通过dmaI2C中断接收和处理数据包.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-24-2024    liujiajian    		901数据处理的编写
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 征途****************************
  */
#ifndef I2C_JY901_H__
#define I2C_JY901_H__

#include "main.h"
#include "dma.h"
//#include "Simulate_IIC.h"
#include "i2c.h"
//#include "control.h"
#include "tim.h"


/*jy901寄存器地址宏定义*/
//#define SAVE 			0x00
//#define CALSW 		0x01
//#define RSW 			0x02
//#define RRATE			0x03
//#define BAUD 			0x04
//#define AXOFFSET	0x05
//#define AYOFFSET	0x06
//#define AZOFFSET	0x07
//#define GXOFFSET	0x08
//#define GYOFFSET	0x09
//#define GZOFFSET	0x0a
//#define HXOFFSET	0x0b
//#define HYOFFSET	0x0c
//#define HZOFFSET	0x0d
//#define D0MODE		0x0e
//#define D1MODE		0x0f
//#define D2MODE		0x10
//#define D3MODE		0x11
//#define D0PWMH		0x12
//#define D1PWMH		0x13
//#define D2PWMH		0x14
//#define D3PWMH		0x15
//#define D0PWMT		0x16
//#define D1PWMT		0x17
//#define D2PWMT		0x18
//#define D3PWMT		0x19
//#define IICADDR		0x1a
//#define LEDOFF 		0x1b
//#define GPSBAUD		0x1c

//#define YYMM				0x30
//#define DDHH				0x31
//#define MMSS				0x32
//#define MS					0x33
//#define AX					0x34
//#define AY					0x35
//#define AZ					0x36
//#define GX					0x37
//#define GY					0x38
//#define GZ					0x39
//#define HX					0x3a
//#define HY					0x3b
//#define HZ					0x3c			
//#define Roll				0x3d
//#define Pitch				0x3e
//#define Yaw					0x3f
//#define TEMP				0x40
//#define D0Status		0x41
//#define D1Status		0x42
//#define D2Status		0x43
//#define D3Status		0x44
//#define PressureL		0x45
//#define PressureH		0x46
//#define HeightL			0x47
//#define HeightH			0x48
//#define LonL				0x49
//#define LonH				0x4a
//#define LatL				0x4b
//#define LatH				0x4c
//#define GPSHeight   0x4d
//#define GPSYAW      0x4e
//#define GPSVL				0x4f
//#define GPSVH				0x50
//#define q0          0x51
//#define q1          0x52
//#define q2          0x53
//#define q3          0x54

#define ADDRESS (0x50<<1)
/*接收包长度宏定义*/
#define RXBUFFER_LEN 33		//接收3类数据，一共33位

/*  JY901数据结构体的定义  */
struct SAcc
{
	short a[3];
	short T;
};
struct SGyro
{
	short w[3];
	short T;
};
struct SAngle
{
	short Angle[3];
	short T;
};

 
//重定义结构体和JY901数据结构体
typedef struct
{
	float angle[3];
}Angle;

typedef struct
{
	float a[3];
}Acc;

typedef struct
{
	float w[3];
}SGyro;


typedef struct User_IIC
{
		uint8_t Rx_flag;											
		uint8_t Rx_len;												
		uint8_t frame_head;					//帧头
		uint8_t RxBuffer[RXBUFFER_LEN];		//接收缓冲
		Angle angle;						//角度
		Angle expand_angle;     //拓展角度
		Acc acc;							//加速度
		SGyro w;							//角速度
}User_IIC;

/**
	*@func void JY901_Init()
	*@brief 开启JY901的I2CDMA传输,并关闭半传输中断
	*@param  none
	*/
void JY901_Init();


/**
	*@func     uint8_t IICwriteBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data)
	*@brief    MCU从JY901读数据
	*@param    JY901_data     JY901的数据存放数组指针
	*@retval   none
	*/
void JY901_GetData(User_IIC* JY901_data);

/**
  * @func			void IDLE_IMU_Handler(void)
  * @brief          JY901接收空闲中断处理
  * @brief          该函数在stm32f4xx_it.c中被相应串口中断处理函数调用
  * @param[in]      none
  * @retval         none
  */
void IDLE_IMU_Handler(void);

/**
  * @func			void JY901_RC(User_USART* JY901_data,RC* data)
  * @brief          JY901数据包发送给遥控器
  * @param[in]      JY901_data：JY901句柄
	* @param[in]      data:遥控器数组指针
  * @retval         none
  */
void JY901_RC(User_IIC* JY901_data,RC* data);

/**
  * @func			void JY901_ExpandAngle(User_IIC* JY901_data)
  * @brief          拓展JY901角度范围
  * @param[in]      JY901_data：JY901句柄
  * @retval         none
  */
void JY901_ExpandAngle(User_IIC* JY901_data);

/**
  * @func						void I2C_DMA_Detect(void)
  * @brief          JY901的数据接收失败处理
  * @brief          在定时器里面检测DMA是否空闲
  * @param[in]      none
  * @retval         none
  */
void I2C_DMA_Detect(void);

#endif