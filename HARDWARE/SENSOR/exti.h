//#ifndef _GUA_LIMIT_SWITCH_H_
//#define _GUA_LIMIT_SWITCH_H_
// 
///*********************宏定义************************/   
//#ifndef GUA_U8        
//typedef unsigned char GUA_U8;        
//#endif    
// 
//#ifndef GUA_8        
//typedef signed char GUA_8;        
//#endif      
//      
//#ifndef GUA_U16        
//typedef unsigned short GUA_U16;        
//#endif 
// 
//#ifndef GUA_16        
//typedef signed short GUA_16;        
//#endif         
//      
//#ifndef GUA_U32        
//typedef unsigned long GUA_U32;        
//#endif 
// 
//#ifndef GUA_32        
//typedef signed long GUA_32;       
//#endif
// 
//#ifndef GUA_U64    
//typedef unsigned long long GUA_U64;  
//#endif
// 
//#ifndef GUA_64    
//typedef signed long long GUA_64;  
//#endif
// 
////可读的限位开关
//#define GUA_LIMIT_SWITCH_STATUS_UP                    0            //UP限位开关
//#define GUA_LIMIT_SWITCH_STATUS_DOWN                  1            //DWON限位开关
// 
////限位开关的触发状态
//#define GUA_LIMIT_SWITCH_STATUS_TRIGGER               0            //限位开关触发
//#define GUA_LIMIT_SWITCH_STATUS_IDLE                  1            //D位开关没触发
// 
///*********************外部函数声明************************/ 
//GUA_U8 GUA_Limit_Switch_Check_Pin(GUA_U8 nGUA_Limit_Switch_Status);  
//void GUA_Limit_Switch_Init(void);
// 
//#endif





#ifndef __EXTI_H
#define __EXIT_H	 
#include "sys.h"  	
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F407开发板
//外部中断 驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2014/5/4
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	 

#define TAGGLE 1
#define NORMAL 0

void EXTIX_Init(void);	//外部中断初始化		 
#endif

























