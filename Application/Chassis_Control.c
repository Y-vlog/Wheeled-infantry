#include "chassis_control.h"

M3508_HandleTypeDef Wheel_M3508Handle[4];	// define chassis-wheeljoint motor handle

/*底盘坐标系*/
ChassisVectorsTypeDef chassis_vectors;

/*底盘相关pid*/
pid_type_def Chassis_Angle_PID;
pid_type_def Chassis_Speed_PID;

int16_t set_currentMA = 0;
int16_t set_currentMB = 0;
int16_t set_currentMC = 0;
int16_t set_currentMD = 0;



/**
  * @func			void Chassis_Init(void)
  * @brief          底盘驱动初始化
  * @param[in]      none
  * @retval         none
  */
void Chassis_Init(void)
{
	/*底盘电机初始化*/
	#if (CHASSiS_SPEED_PID_MODE == 0)
	M3508_Init(&Wheel_M3508Handle[0], 0x01, PID_POSITION, 1);
	M3508_Init(&Wheel_M3508Handle[1], 0x02, PID_POSITION, 1);
	M3508_Init(&Wheel_M3508Handle[2], 0x03, PID_POSITION, 1);
	M3508_Init(&Wheel_M3508Handle[3], 0x04, PID_POSITION, 1);
	#else
	M3508_Init(&Wheel_M3508Handle[0], 0x01, PID_DELTA, 1);
	M3508_Init(&Wheel_M3508Handle[1], 0x02, PID_DELTA, 1);
	M3508_Init(&Wheel_M3508Handle[2], 0x03, PID_DELTA, 1);
	M3508_Init(&Wheel_M3508Handle[3], 0x04, PID_DELTA, 1);
	#endif
    
	/*底盘坐标系初始化*/
	chassis_vectors.c_x=0;
	chassis_vectors.c_y=0;
	chassis_vectors.cos=0.0f;
	chassis_vectors.sin=0.0f;
	
	// chassis_vectors.beganangle_cwtw=0;//获取开机角度
	// chassis_vectors.angle_cmtw=0;
	
	/*底盘相关pid初始化*/
	fp32 pid_Angle[3] =
    {
        CHASSIS_ANGLEPID_KP,
        CHASSIS_ANGLEPID_KI,
        CHASSIS_ANGLEPID_KD
	};
    fp32 chassis_whellspeed_pid[3] = 
    {
        CHASSIS_SPEEDPID_KP,
        CHASSIS_SPEEDPID_KI,
        CHASSIS_SPEEDPID_KD
    };

	PID_init(&Chassis_Angle_PID, PID_POSITION, pid_Angle, 5000, 5000);
    PID_init(&Chassis_Speed_PID, PID_POSITION, chassis_whellspeed_pid, 5000, 5000);
}

/**
  * @func			void Chassis_WheatWheel_Solution(
  *						int16_t chassisXvector,
  *						int16_t chassisYvectory,
  *						int16_t chassisRotatevector,
  *						int rotateK)
  * @brief          麦轮底盘运动学解算
  * @brief          底盘坐标系(俯视图)：
	/\底盘正方向		 /\x(roll)
	||					||
	||					||
	||					||
		y(pitch)《========
		 
  * @param[in]      chassisXvector:底盘坐标系下，x方向下的速度
  * @param[in]      chassisYvectory:底盘坐标系下，y方向下的速度
  * @param[in]      chassisRotatevector:底盘(世界)坐标系下，底盘旋转的速度
  * @param[in]      rotateK:旋转系数(与底盘长款轴距有关k=a+b)
  * @retval         none
  */
void Chassis_WheatWheel_Solution(int16_t ChassisXvector, int16_t ChassisYvectory, int16_t ChassisRotatevector, float rotateK)
{
	/*获取底盘电机基础数据*/
	M3508_GetBasicData(&Wheel_M3508Handle[0],1);
	M3508_GetBasicData(&Wheel_M3508Handle[1],1);
	M3508_GetBasicData(&Wheel_M3508Handle[2],1);
	M3508_GetBasicData(&Wheel_M3508Handle[3],1);
    
//    int16_t current_speeds[4] = {0};  //功率计算存储电流值

    /*全向轮解算*/
//	int16_t targetspeedMA = ChassisXvector + ChassisYvectory + ChassisRotatevector * rotateK;
//	int16_t targetspeedMB = ChassisXvector - ChassisYvectory - ChassisRotatevector * rotateK;
//	int16_t targetspeedMC = ChassisXvector + ChassisYvectory - ChassisRotatevector * rotateK;
//    int16_t targetspeedMD = ChassisXvector - ChassisYvectory + ChassisRotatevector * rotateK;
    
    /*麦轮解算*/
    int16_t targetspeedMA =  ChassisXvector + ChassisYvectory - ChassisRotatevector * rotateK;
    int16_t targetspeedMB =  ChassisXvector - ChassisYvectory - ChassisRotatevector * rotateK;
    int16_t targetspeedMC = -(ChassisXvector + ChassisYvectory + ChassisRotatevector * rotateK);
    int16_t targetspeedMD = -(ChassisXvector - ChassisYvectory + ChassisRotatevector * rotateK);
    
	#if (USING_SMOOTHING_FILTER == 1)
	/*创建平滑滤波器*/
	static uint8_t getlengthMA=0;
	static int16_t filter_boxsMA[M3508_ID1_DATALENGTH];
	static uint8_t getlengthMB=0;
	static int16_t filter_boxsMB[M3508_ID2_DATALENGTH];
	static uint8_t getlengthMC=0;
	static int16_t filter_boxsMC[M3508_ID3_DATALENGTH];
	static uint8_t getlengthMD=0;
	static int16_t filter_boxsMD[M3508_ID4_DATALENGTH];
	/*平滑滤波*/
	int16_t filter_rpmMA=Smoothing_filter(wheelJoint_motorHandle[0].basic_data.speed_rpm,&getlengthMA,filter_boxsMA,M3508_ID1_DATALENGTH);
	int16_t filter_rpmMB=Smoothing_filter(wheelJoint_motorHandle[1].basic_data.speed_rpm,&getlengthMB,filter_boxsMB,M3508_ID2_DATALENGTH);
	int16_t filter_rpmMC=Smoothing_filter(wheelJoint_motorHandle[2].basic_data.speed_rpm,&getlengthMC,filter_boxsMC,M3508_ID3_DATALENGTH);
	int16_t filter_rpmMD=Smoothing_filter(wheelJoint_motorHandle[3].basic_data.speed_rpm,&getlengthMD,filter_boxsMD,M3508_ID4_DATALENGTH);
	/*pid运算*/
	int16_t set_currentMA=M3508_SpeedLoopPIDController(&(wheelJoint_motorHandle[0].speed_control_data), filter_rpmMA,  targetspeedMA);
	int16_t set_currentMB=M3508_SpeedLoopPIDController(&(wheelJoint_motorHandle[1].speed_control_data), filter_rpmMB, -targetspeedMB);
	int16_t set_currentMC=M3508_SpeedLoopPIDController(&(wheelJoint_motorHandle[2].speed_control_data), filter_rpmMC, -targetspeedMC);
	int16_t set_currentMD=M3508_SpeedLoopPIDController(&(wheelJoint_motorHandle[3].speed_control_data), filter_rpmMD,  targetspeedMD);	
	#else
	/*pid运算*/
	set_currentMA=M3508_SpeedLoopPIDController(&(Wheel_M3508Handle[0].PID_Speed_Control_Data), Wheel_M3508Handle[0].Basic_Data->speed_rpm, targetspeedMA);
	set_currentMB=M3508_SpeedLoopPIDController(&(Wheel_M3508Handle[1].PID_Speed_Control_Data), Wheel_M3508Handle[1].Basic_Data->speed_rpm, targetspeedMB);
	set_currentMC=M3508_SpeedLoopPIDController(&(Wheel_M3508Handle[2].PID_Speed_Control_Data), Wheel_M3508Handle[2].Basic_Data->speed_rpm, targetspeedMC);
    set_currentMD=M3508_SpeedLoopPIDController(&(Wheel_M3508Handle[3].PID_Speed_Control_Data), Wheel_M3508Handle[3].Basic_Data->speed_rpm, targetspeedMD);
	#endif
    
    //(方法一)PID功率系数进行滤波功率
//    Power_all_Calculate();
//    
//    power_ctrl.attenuation_factor = Power_PID_Calculate(CHASSIS_POWER_MAX, power_ctrl.total_power);
//    
//    //一阶滤波 平滑滤波
//    power_ctrl.filtered_factor = POWER_LIMIT_SMOOTHING * power_ctrl.attenuation_factor + (1 - POWER_LIMIT_SMOOTHING) * power_ctrl.filtered_factor;

//    set_currentMA = (int16_t)(set_currentMA * power_ctrl.filtered_factor);
//    set_currentMB = (int16_t)(set_currentMB * power_ctrl.filtered_factor);
//    set_currentMC = (int16_t)(set_currentMC * power_ctrl.filtered_factor);
//    set_currentMD = (int16_t)(set_currentMD * power_ctrl.filtered_factor);
    
    //(方法二)功率限制算法
//    current_speeds[0] =  Wheel_M3508Handle[0].Basic_Data->speed_rpm;
//    current_speeds[1] = -Wheel_M3508Handle[1].Basic_Data->speed_rpm;
//    current_speeds[2] = -Wheel_M3508Handle[2].Basic_Data->speed_rpm;
//    current_speeds[3] =  Wheel_M3508Handle[3].Basic_Data->speed_rpm;

//    // 准备电流数组用于功率分配
//    int16_t currents[4] = {set_currentMA, -set_currentMB, -set_currentMC, set_currentMD};

//    // 执行功率分配算法
//    PowerDistribution(currents, current_speeds);

//    // 更新分配后的电流值
//    set_currentMA = currents[0];
//    set_currentMB = -currents[1];
//    set_currentMC = -currents[2];
//    set_currentMD = currents[3];
    
	CAN_cmd_chassis(set_currentMA, set_currentMB, set_currentMC, set_currentMD);
}

/**
  * @func			void ChassisGyro_Task(
  *						int16_t worldXvector,
  *						int16_t worldYvector,
  *						int rotateK,
  *						float c_angle)
  * @brief          底盘小陀螺任务
  * @param[in]      worldXvector：世界/云台坐标系下，x方向速度
  * @param[in]      worldYvector：世界/云台坐标系下，y方向速度
  * @param[in]      rotateK：旋转系数(与底盘长款轴距有关k=a+b)
  * @param[in]      c_angle：世界/云台坐标系与底盘坐标系的夹角编码器值 一般为云台与底盘
  * @retval         none
  */
void ChassisGyro_Task(int16_t World_Xvector,int16_t World_Yvector,int16_t World_Zrotate, float rotateK,int16_t C_Angle)
{
    //角度直接转弧度
    float angle_rad = (float)C_Angle * RADIAN_COEF_CHASSIS;
    
    //计算正弦和余弦值（使用ARM数学库提高精度）
    chassis_vectors.cos = arm_cos_f32(angle_rad);
    chassis_vectors.sin = arm_sin_f32(angle_rad);
    
    //世界坐标系→底盘坐标系转换（X左右/Y前后专用逻辑）
    //公式推导：基于旋转矩阵
    chassis_vectors.c_x = (int16_t)( World_Xvector * chassis_vectors.cos + World_Yvector * chassis_vectors.sin );  // 底盘X正=左边
    chassis_vectors.c_y = (int16_t)( -World_Xvector * chassis_vectors.sin + World_Yvector * chassis_vectors.cos ); // 底盘Y正=后方
    
    Chassis_WheatWheel_Solution(chassis_vectors.c_x, chassis_vectors.c_y, World_Zrotate, rotateK);
}

/**
  * @func			void ChassisPositionControl_Task(
  *						int16_t worldXvector,
  *						int16_t worldYvector,
  *						int rotateK,
  *						float32_t realangle,
  *						float32_t targetangle)
  * @brief          底盘跟随云台任务 底盘PID运算
  * @param[in]      worldXvector：世界/云台坐标系下，x方向速度
  * @param[in]      worldYvector：世界/云台坐标系下，y方向速度
  * @param[in]      rotateK：旋转系数(与底盘长款轴距有关k=a+b)
  * @param[in]      realangle：  世界/云台坐标系与底盘坐标系的实时夹角(yaw轴电机绝对编码器值)
  * @param[in]      targetangle：世界/云台坐标系与底盘坐标系的目标夹角(yaw轴电机绝对编码器值) 旋转后
  * @retval         none
  */
void ChassisPositionControl_Task(int16_t World_Xvector,int16_t World_Yvector,float rotateK,int16_t realangle,int16_t targetangle)
{
	/*串级pid*/
	int16_t targetspeed = PID_calc(&Chassis_Angle_PID, realangle, targetangle);//角度环
    
	/*角加速度环*/
	int16_t output = PID_calc(&Chassis_Speed_PID, Yaw_Gimbal_Motor.Basic_Data->speed_rpm, targetspeed); 
	
	Chassis_WheatWheel_Solution(World_Xvector,World_Yvector,output,rotateK);//速度环
}

/**
  * @func		void Chassis_Follow(int16_t chassisXvector,int16_t chassisYvectory,int16_t chassisRotatevector,int16_t rotateK)
  * @brief      底盘跟随云台任务 无底盘PID运算
  * @param[in]  chassisXvector:底盘坐标系下，x方向下的速度
  * @param[in]  chassisYvectory:底盘坐标系下，y方向下的速度
  * @param[in]  INS:底盘陀螺仪数据包[-180,180]
  * @param[in]  init_gambal_yaw:云台目标角度[-180,180]
  * @retval     void
  */
void Chassis_Follow(int16_t ChassisXvector, int16_t ChassisYvectory, float rotateK, INS_t* INS, float init_gambal_yaw)
{
	/*串级pid*/
	int16_t targetspeed = PID_calc(&Chassis_Angle_PID, INS->Yaw, init_gambal_yaw);//角度环
    
	/*角加速度环*/
	int16_t output = PID_calc(&Chassis_Speed_PID, Yaw_Gimbal_Motor.Basic_Data->speed_rpm, targetspeed); 
	
	Chassis_WheatWheel_Solution(ChassisXvector, ChassisYvectory, output, rotateK);//速度环
    
}

