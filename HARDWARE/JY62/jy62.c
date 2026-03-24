#include "jy62.h"

static int8_t RxBuffer[11];
static int RxCounter=0; 

float roll,pitch,yaw;

void MPU_UART1_init(unsigned long baudrate)
{
    //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE); //使能GPIOA时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//使能USART1时钟
 
	//串口1对应引脚复用映射
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_USART1); //GPIOA9复用为USART1
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource7,GPIO_AF_USART1); //GPIOA10复用为USART1
	
	//USART1端口配置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; //GPIOA9与GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOB,&GPIO_InitStructure); //初始化PA9，PA10

   //USART1 初始化设置
	USART_InitStructure.USART_BaudRate = baudrate;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(USART1, &USART_InitStructure); //初始化串口1
	
  USART_Cmd(USART1, ENABLE);  //使能串口1 
	
	//USART_ClearFlag(USART1, USART_FLAG_TC);
	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启相关中断

	//Usart1 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//串口1中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、
}

void UART1_Put_Char(unsigned char DataToSend)
{
	RxBuffer[RxCounter++] = DataToSend;  
  USART_ITConfig(USART1, USART_IT_TXE, ENABLE);  
}

void UART1_Put_String(unsigned char *Str)
{
	while(*Str)
	{
		if(*Str=='\r')UART1_Put_Char(0x0d);
			else if(*Str=='\n')UART1_Put_Char(0x0a);
				else UART1_Put_Char(*Str);
		Str++;
	}
}

//重定义fputc函数 
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
    USART1->DR = (u8) ch;      
	return ch;
}

void USART1_IRQHandler(void)//数据接收
{
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET){

		Data_process((unsigned char)USART1->DR);

		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
  }
	USART_ClearITPendingBit(USART1,USART_IT_ORE);
}

void Data_process(unsigned char data){
	if(0x55 == data){
		RxBuffer[0] = data;
		RxCounter = 1;
	}else if(0x53 == data){
		RxBuffer[1] = data;
		RxCounter = 2;
	}else if(2 == RxCounter){
		RxBuffer[RxCounter++] = data;
	}else{
		return;
	}
	
	if(11 == RxCounter){
		roll = ((RxBuffer[3]<<8)|RxBuffer[2])/32768*180;
		pitch = ((RxBuffer[5]<<8)|RxBuffer[4])/32768*180;
		yaw = ((RxBuffer[7]<<8)|RxBuffer[6])/32768*180;
	}
}

void show_angle(){
	OLED_Show3FNum(80,10,yaw,4,2,8,1);
}