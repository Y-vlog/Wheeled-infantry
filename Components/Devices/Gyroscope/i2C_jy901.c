/**
  ****************************(C) COPYRIGHT 2025 征途****************************
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
  ****************************(C) COPYRIGHT 2025 征途****************************
  */
#include "i2C_jy901.h"
#include "gpio.h"
#include "stm32f4xx_hal_dma.h"

/*定义jy901句柄*/

User_IIC      JY901_data;
extern RC* Receive;
extern DMA_HandleTypeDef hdma_i2c2_rx;
struct SAcc 	stcAcc;
struct SGyro 	stcGyro;
struct SAngle 	stcAngle;



#define Cale 32678

User_IIC   JY901_data;

/**
	*@func void JY901_Init()
	*@brief 开启JY901的I2CDMA传输,并关闭半传输中断
	*@param  none
	*/
void JY901_Init()
{
		HAL_I2C_Mem_Read_DMA(&hi2c2,ADDRESS,AX,I2C_MEMADD_SIZE_8BIT,JY901_data.RxBuffer,24);
		__HAL_DMA_DISABLE_IT(&hdma_i2c2_rx,DMA_IT_HT);
		//HAL_TIM_Base_Start_IT(&htim2);
}


/**
	*@func short PackToBytes(unsigned char cData[])
	*@brief 将高位和低位的数据拼在一起
	*@param  cData[] 拼接的数据
	*/
short PackToBytes(unsigned char cData[])
{
	return ((short)cData[1] << 8) | cData[0];
}

/**
	*@func     uint8_t IICwriteBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data)
	*@brief    MCU从JY901读数据
	*@param    JY901_data     JY901的数据存放数组指针
	*@retval   none
	*/
void JY901_GetData(User_IIC* JY901_data)
{
	//IICreadBytes(0x50,AX,18,JY901_data->ReceiveBuffer);
	if(JY901_data==NULL)
	{
		return;
	}
	
	/*  解析加速度数据包  */
	JY901_data->acc.a[0] = (float)PackToBytes(&JY901_data->RxBuffer[0])/Cale*16;
	JY901_data->acc.a[1] = (float)PackToBytes(&JY901_data->RxBuffer[2])/Cale*16;
	JY901_data->acc.a[2] = (float)PackToBytes(&JY901_data->RxBuffer[4])/Cale*16;
	
	/*  解析角速度数据包  */
	JY901_data->w.w[0]   = (float)PackToBytes(&JY901_data->RxBuffer[6])/Cale*2000;
	JY901_data->w.w[1]   = (float)PackToBytes(&JY901_data->RxBuffer[8])/Cale*2000;
	JY901_data->w.w[2]   = (float)PackToBytes(&JY901_data->RxBuffer[10])/Cale*2000;
	
	/*  解析角度数据包  */
	JY901_data->angle.angle[0] = (float)PackToBytes(&JY901_data->RxBuffer[18])/Cale*180;
	JY901_data->angle.angle[1] = (float)PackToBytes(&JY901_data->RxBuffer[20])/Cale*180;
	JY901_data->angle.angle[2] = (float)PackToBytes(&JY901_data->RxBuffer[22])/Cale*180;
}

/**
  * @func			void JY901_ExpandAngle(User_IIC* JY901_data)
  * @brief          拓展JY901角度范围
  * @param[in]      JY901_data：JY901句柄
  * @retval         none
  */
void JY901_ExpandAngle(User_IIC* JY901_data)
{
	if(JY901_data==NULL)
	{
		return;
	}
	static float last_angle=0;
	static int temp=0;//判断是否是第一次进入
	if(temp==0)
	{	
		last_angle=JY901_data->angle.angle[2];
		temp=1;
	}
	else
	{
		float d_angle=JY901_data->angle.angle[2]-last_angle;
		//printf("%f,%f,%f\n",JY901_data->angle.angle[2],last_angle,d_angle);
		/*判断角度是否发生突变*/
		if(d_angle<-340||temp==2)//正方向+突变
		{
			if(d_angle>340)
			{
				temp=1;
			}
			else
			{
				temp=2;
				JY901_data->expand_angle.angle[2]=360+JY901_data->angle.angle[2];
			}
			
		}
		else if(d_angle>340||temp==3)//负方向+突变
		{
			if(d_angle<-340)
			{
				temp=2;
			}
			else
			{
				temp=3;
				JY901_data->expand_angle.angle[2]=-360+JY901_data->angle.angle[2];
			}
		}
		else//未突变
		{
			temp=1;
			JY901_data->expand_angle.angle[2]=JY901_data->angle.angle[2];
		}
		JY901_data->expand_angle.angle[0]=JY901_data->angle.angle[0];
		JY901_data->expand_angle.angle[1]=JY901_data->angle.angle[1];
		last_angle=JY901_data->angle.angle[2];
	}
}

/**
  * @func			void JY901_RC(User_USART* JY901_data,RC* data)
  * @brief          JY901数据包发送给遥控器
  * @param[in]      JY901_data：JY901句柄
	* @param[in]    data:遥控器数组指针
  * @retval         none
  */
void JY901_RC(User_IIC* JY901_data,RC* data)
{
	//陀螺仪的加速度
	data->gyro.gimbal_acc_x = (int16_t) JY901_data->acc.a[0];
	data->gyro.gimbal_acc_y = (int16_t) JY901_data->acc.a[1];
	data->gyro.gimbal_acc_z = (int16_t) JY901_data->acc.a[2];
	
	//陀螺仪的角速度
	data->gyro.gimbal_vel_x = (int16_t) JY901_data->w.w[0];
	data->gyro.gimbal_vel_y = (int16_t) JY901_data->w.w[1];
	data->gyro.gimbal_vel_z = (int16_t) JY901_data->w.w[2];
	
	//陀螺仪的角度
	data->gyro.gimbal_gyro_x = (int16_t) JY901_data->angle.angle[0];
	data->gyro.gimbal_gyro_y = (int16_t) JY901_data->angle.angle[1];
	data->gyro.gimbal_gyro_z = (int16_t) JY901_data->angle.angle[2];
}

/**
  * @func			void IDLE_IMU_Handler(void)
  * @brief          JY901接收空闲中断处理
  * @brief          该函数在stm32f4xx_it.c中被相应串口中断处理函数调用,没有被使用
  * @param[in]      none
  * @retval         none
  */
void IDLE_IMU_Handler(void)
{
	
	 uint32_t Data_lave,Data_exist; 
	if (__HAL_DMA_GET_FLAG(&hdma_i2c2_rx,DMA_FLAG_TCIF2_6) == SET)//判断i2cdma接收是否处于空闲状态
	{
		//__HAL_UART_CLEAR_IDLEFLAG(&huart1);//清除DMA空闲中断标志位
		//__HAL_I2C_CLEAR_FLAG(&hi2c2,I2C_FLAG_AF);
		__HAL_DMA_CLEAR_FLAG(&hdma_i2c2_rx, DMA_FLAG_TCIF2_6);
		
		//HAL_UART_DMAStop(&huart1); //停止串口dma接收
		HAL_DMA_Abort(&hdma_i2c2_rx);
		
//		Data_lave = __HAL_DMA_GET_COUNTER(&hdma_i2c2_rx); //获取接收长度

//		Data_exist = 33-Data_lave; //计算剩余长度

//		if(Data_exist == 0)  //判断是否接收到正确长度数据
//		{
			/*接收成功后，处理相应接收数据*/
			JY901_GetData(&JY901_data);
			JY901_ExpandAngle(&JY901_data);
			JY901_RC(&JY901_data,Receive);
//		}
		HAL_I2C_Mem_Read_DMA(&hi2c2,ADDRESS,AX,I2C_MEMADD_SIZE_8BIT,JY901_data.RxBuffer,24);
		__HAL_DMA_DISABLE_IT(&hdma_i2c2_rx,DMA_IT_HT);
	}
}



/**
  * @func			void I2C_DMA_Detect(void)
  * @brief          JY901的数据接收失败处理
  * @brief          在定时器里面检测DMA是否空闲
  * @param[in]      none
  * @retval         none
  */
void I2C_DMA_Detect(void)
{
	if(HAL_DMA_GetState(&hdma_i2c2_rx) == HAL_DMA_STATE_READY)
	{
			MX_I2C2_Init();
			JY901_Init();
	}
	
}
