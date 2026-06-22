#include "sys.h"

//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用
//ALIENTEK STM32F103ZET6开发板
void NVIC_Configuration(void)
{

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	//设置NVIC中断分组2:2位抢占优先级，2位响应优先级

}
