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

//	Driver_InitUD();			//驱动器初始化
	//TIM8_OPM_RCR_InitUD(999,168-1); //1MHz计数频率  单脉冲+重复计数模式 
	
	//turnUp();
	//turnDown();
	//turnLeft();
	turnRight();
	
	
	
	while(1)
	{
		
		/*负责控制上方传感器信号检测，检测到金属停止上升
				测试成功
		*/
		
//		if(Check_Sensor3_State()==NORMAL){
//		Locate_RleUD(10000,5000,CCW);
//		}
//		else
//		{
//			break;
//		}
		
		

				/*负责控制下方传感器信号检测，检测到金属停止下降
						测试成功
				*/
		
//		if(Check_Sensor2_State()==NORMAL){
//		Locate_RleUD(10000,5000,CW);
//		}
//		else
//		{
//			break;
//		}
		
		
		
		
		/*负责控制右方传感器（逆时针）信号检测，检测到金属停止下降
						测试成功
				*/
		
//	  if(Check_Sensor0_State()==NORMAL)
//			{
//			Locate_RleLR(300,300,CCW);
//			}
//			else
//			{
//				break;
//			}



		/*负责控制右方传感器（逆时针）信号检测，检测到金属停止下降
						测试成功
				*/
		
//	  if(Check_Sensor1_State()==NORMAL)
//			{
//			Locate_RleLR(300,300,CCW);
//			}
//			else
//			{
//				break;
//			}
		
		
		
		/*进行两电机循环运动*/
//		Locate_RleLR(400,500,CW);
//		delay_ms(3000);
//    Locate_RleUD(10000,5000,CCW);
//		delay_ms(5000);
//		Locate_RleLR(400,500,CCW);
//		delay_ms(3000);
//		Locate_RleUD(30000,10000,CW);
//		delay_ms(5000);






	}
	

}
