#include "sys.h"
#include "delay.h"  
#include "usart.h"  
#include "led.h"
#include "usmart.h"
#include "key.h"
#include "driver.h"
#include "exti.h"




int main(void)
{ 
	
	u8 i = 0;
	u8 j = 10;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
	delay_init(168);      //初始化延时函数
	uart_init(115200);		//初始化串口波特率为115200
	usmart_dev.init(84); 	//初始化USMART			
	EXTIX_Init();

	Driver_InitUD();			//驱动器初始化
	TIM8_OPM_RCR_InitUD(999,168-1); //1MHz计数频率  单脉冲+重复计数模式 
	
	
	while(1)
	{
		
		Locate_RleLR(400,500,CW);
		delay_ms(3000);
		Locate_RleUD(20000,20000,CW);
		delay_ms(5000);
		Locate_RleLR(400,500,CCW);
		delay_ms(3000);
		Locate_RleUD(30000,10000,CCW);
		delay_ms(5000);
	}
	

}
