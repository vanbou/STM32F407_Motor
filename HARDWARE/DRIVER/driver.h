#ifndef __DRIVER_H
#define __DRIVER_H
#include "sys.h"
#include "stdlib.h"	

#define DRIVER_DIR_LR   PGout(9) // 旋转方向
#define DRIVER_OE_LR    PGout(8) // 使能脚 低电平有效 

#define DRIVER_DIR_UD   PGout(7) // 旋转方向
#define DRIVER_OE_UD    PGout(6) // 使能脚 低电平有效 

#define RCR_VAL    255  //每计数（RCR_VAL+1）次，中断一次，这个值（0~255）设置大一些可以降低中断频率
#define RCR_VAL1    255  //每计数（RCR_VAL+1）次，中断一次，这个值（0~255）设置大一些可以降低中断频率


#define TAGGLE 1
#define NORMAL 0

typedef enum
{
	CW = 1,//高电平顺时针
	CCW = 0,//低电平逆时针
}DIR_Type;//运行方向

extern long target_pos;//有符号方向
extern long current_pos;//有符号方向

void Driver_InitLR(void);//驱动器初始化
void Driver_InitUD(void);//驱动器初始化


void TIM8_OPM_RCR_InitLR(u16 arr,u16 psc);//TIM8_CH2 单脉冲输出+重复计数功能初始化
void TIM8_OPM_RCR_InitUD(u16 arr,u16 psc);//TIM8_CH1 单脉冲输出+重复计数功能初始化

void TIM8_StartupLR(u32 frequency);   //启动定时器8
void TIM8_StartupUD(u32 frequency);

void Locate_RleLR(long num,u32 frequency,DIR_Type dir); //相对定位函数
void Locate_AbsLR(long num,u32 frequency);//绝对定位函数

void Locate_RleUD(long num,u32 frequency,DIR_Type dir); //相对定位函数
void Locate_AbsUD(long num,u32 frequency);//绝对定位函数


u8 Check_Sensor0_State(void);//水平方向左极限传感器
u8 Check_Sensor1_State(void);//水平方向右极限传感器
u8 Check_Sensor2_State(void);//竖直方向下极限传感器
u8 Check_Sensor3_State(void);//竖直方向上极限传感器

void turnUp(void);
void turnDown(void);
void turnLeft(void);
void turnRight(void);

#endif


