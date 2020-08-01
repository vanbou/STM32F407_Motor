#include "exti.h"
#include "delay.h" 
#include "led.h" 
#include "key.h"
#include "driver.h"
#include "sys.h"
#include "usart.h"


//外部中断0服务程序 右边传感器中断
void EXTI0_IRQHandler(void)
{
  if(EXTI_GetITStatus(EXTI_Line0) == 1)
  {
		printf("--------exit0-------------");
		Locate_RleLR(200,200,CW);
		
    EXTI_ClearITPendingBit(EXTI_Line0);
  }//清除LINE0上的中断标志位 
}


//外部中断1服务程序 左边传感器中断
void EXTI1_IRQHandler(void)
{
	 if(EXTI_GetITStatus(EXTI_Line1) == 1)
	  {
		printf("--------exit1-------------");
			Locate_RleLR(200,200,CW);
		
    EXTI_ClearITPendingBit(EXTI_Line1);
  }//清除LINE1上的中断标志位 
}


//外部中断2服务程序 下方传感器中断
void EXTI2_IRQHandler(void)
{

	 if(EXTI_GetITStatus(EXTI_Line2) == 1)
	  {
			printf("--------exit2-------------");
		Locate_RleUD(10000,10000,CW);
		
    EXTI_ClearITPendingBit(EXTI_Line2);
  } //清除LINE2上的中断标志位  
}
//外部中断3服务程序 上方传感器中断
void EXTI3_IRQHandler(void)
{

 if(EXTI_GetITStatus(EXTI_Line3) == 1)
	 {
		 printf("--------exit3-------------");
		Locate_RleUD(10000,10000,CW);
		
    EXTI_ClearITPendingBit(EXTI_Line3);
  }//清除LINE3上的中断标志位  
}



void EXTIX_Init(void)
{
	
	//PB0使用Exit0号中断线，PG1使用Exit1，PC2使用Exit2，PA3使用Exit3
	NVIC_InitTypeDef   NVIC_InitStructure;
	EXTI_InitTypeDef   EXTI_InitStructure;
	GPIO_InitTypeDef   GPIO_InitStructure;
	

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOG|RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOA, ENABLE);//使能GPIOB,GPIOG,GPIOC,GPIOA时钟
 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//使能SYSCFG时钟（中断需要使用AFIO时钟)
	
	
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0; //PB0对应IO配置
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//普通输入模式
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化GPIOB0
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1; //PG1对应IO配置
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//普通输入模式
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOG, &GPIO_InitStructure);//初始化GPIOG1
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //PC2对应IO配置
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//普通输入模式
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOC, &GPIO_InitStructure);//初始化GPIOC2
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3; //PA3对应IO配置
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//普通输入模式
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA3
	
 
 
 
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource0);//PB0 连接到中断线0
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOG, EXTI_PinSource1);//PG1 连接到中断线1
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource2);//PC2 连接到中断线2
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource3);//PA3 连接到中断线3
	
	//配置中断线
  EXTI_InitStructure.EXTI_Line = EXTI_Line0|EXTI_Line1|EXTI_Line2|EXTI_Line3;//LINE0
	
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//中断事件
	//初始状态为低电平，当检测到物体时，KEY0输出高电平，此时INT0导通，变为低电平，即从上升沿跳变至下降沿时触发
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; 
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;//使能LINE0
  EXTI_Init(&EXTI_InitStructure);//配置
	
	
 
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;//PB0对应的外部中断0
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;//不区分优先级，只需要电平转换时进入中断即可
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//使能外部中断通道
  NVIC_Init(&NVIC_InitStructure);//配置
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;//外部中断2
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;//抢占优先级3
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;//子优先级2
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//使能外部中断通道
  NVIC_Init(&NVIC_InitStructure);//配置
	
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;//外部中断3
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;//抢占优先级2
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;//子优先级2
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//使能外部中断通道
  NVIC_Init(&NVIC_InitStructure);//配置
	
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;//外部中断4
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;//抢占优先级1
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;//子优先级2
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//使能外部中断通道
  NVIC_Init(&NVIC_InitStructure);//配置
	   
}




