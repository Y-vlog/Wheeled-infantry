#ifndef SHOOT_CONTROL_H
#define SHOOT_CONTROL_H

#include "M3508_Driver.h"
#include "M2006_Driver.h"

/*摩擦轮发射速度宏*/
#define LEFT_SHOT_SPEED   8000
#define RIGHT_SHOT_SPEED  -8000


/*保险宏*/
#define VALVE_OPEN  1
#define VALVE_CLOSE 0

//后续编写单发 循环
enum Shoot_Mode{
	
	CYCLE_FIRE_MODE, //循环发射
	SINGLE_SHOT_MODE,//单发
};
typedef struct{
	
	int valve;//射击阀门
	int mode;//发射控制模式

}Shoot_ModeMessageTypedef;

extern void Shoot_Init(void);
extern void Shoot_FrictionWheelControl(int16_t setSpeed_trigger, int16_t setSpeed_Left, int16_t setSpeed_Right);

#endif
