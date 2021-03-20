#include "sys.h"
#include "usart6.h"
#include "judge.h"
#if SYSTEM_SUPPORT_OS
#include "FreeRTOS.h"					//ucos 使用	  
#endif

//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
int _sys_exit(int x) 
{ 
	x = x; 
	return 0;
} 
//__use_no_semihosting was requested, but _ttywrch was referenced, 增加如下函数, 方法2
int _ttywrch(int ch)
{
ch = ch;
		return 0;
}
//重定义fputc函数 
int fputc(int ch, FILE *f)
{ 	
	while((USART6->SR&0X40)==0);//循环发送,直到发送完毕   
	USART6->DR = (u8) ch;      
	return ch;
}
#endif

#if EN_USART1_RX   //如果使能了接收
//串口1中断服务程序
//注意,读取USARTx->SR能避免莫名其妙的错误   	
u8 USART_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
//接收状态
//bit15，	接收完成标志
//bit14，	接收到0x0d
//bit13~0，	接收到的有效字节数目
u16 USART_RX_STA=0;       //接收状态标记	


int Usart6_Clean_IDLE_Flag = 0;  //用来读SR DR寄存器

uint8_t  Judge_Buf[2][ JUDGE_BUF_LEN ];  //裁判系统发过来的数据暂存在这里

//初始化IO 串口1 
//bound:波特率
void usart6_init(u32 bound)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	
	//首先使能时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);//使能GPIOG时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE);//使能USART6时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);//使能DMA时钟

	//串口6对应引脚复用映射
	GPIO_PinAFConfig(GPIOG,GPIO_PinSource9,GPIO_AF_USART6); //GPIOG9复用为USART6
	GPIO_PinAFConfig(GPIOG,GPIO_PinSource14,GPIO_AF_USART6); //GPIOG14复用为USART6
	
	//USART6端口配置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_14; //GPIOG9与GPIOG14
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOG,&GPIO_InitStructure); //初始化PG9，PG14
	
  USART_DeInit(USART6);

	 //USART6 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(USART6, &USART_InitStructure); //初始化串口1
	
  USART_ClearFlag(USART6, USART_FLAG_IDLE);  //清除空闲线路中断标志位
	USART_ITConfig(USART6, USART_IT_IDLE, ENABLE); //使能串口空闲中断
	
  DMA_DeInit(DMA2_Stream1);
	DMA_InitStructure.DMA_Channel = DMA_Channel_5;                              //DMA通道5，在8个通道中选择一个
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (USART6->DR);       //外设地址
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)Judge_Buf[0];             //存储器0地址，双缓存模式还要使用M1AR
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;                     //外设到存储器模式
  DMA_InitStructure.DMA_BufferSize = 100;                                     //数据传输量，以外设数据项为单位
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;            //外设地址保持不变
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                     //存储器地址递增
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;     //外设数据位宽8位
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;             //存储器数据位宽8位
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;                             //循环模式
  DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;                     //高优先级
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;                      //不使能FIFO模式
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;                 //单次传输
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;         //单次传输
  DMA_Init(DMA2_Stream1, &DMA_InitStructure);
  DMA_DoubleBufferModeConfig(DMA2_Stream1, (uint32_t)Judge_Buf[1], DMA_Memory_0);  //双缓冲模式，buffer0先被传输
  DMA_DoubleBufferModeCmd(DMA2_Stream1, ENABLE);                              //使能双缓冲模式
  DMA_Cmd(DMA2_Stream1, DISABLE); //Add a disable
  DMA_Cmd(DMA2_Stream1, ENABLE);

  USART_DMACmd(USART6, USART_DMAReq_Rx, ENABLE);  //DMA串口接收请求中断使能
	USART_DMACmd( USART6, USART_DMAReq_Tx, ENABLE );

	USART_Cmd(USART6, ENABLE);  //使能串口6 
	
	#if EN_USART1_RX	 //如果使能了接收
	//USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);//开启相关中断

	//Usart1 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;//串口1中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、

#endif


  
	
}
void USART6_IRQHandler(void)                	//串口6中断服务程序
{

		if(USART_GetITStatus(USART6, USART_IT_RXNE) != RESET)
    {
        USART_ReceiveData(USART6);
    }
    else if (USART_GetITStatus(USART6, USART_IT_IDLE) != RESET)
    {
//        static uint16_t this_time_rx_len = 0;
        USART_ReceiveData(USART6);
		
        if(DMA_GetCurrentMemoryTarget(DMA2_Stream1) == 0)    //如果DMA正在访问存储区0 ，则CPU可以访问存储区1
        {
            //重新设置DMA
            DMA_Cmd(DMA2_Stream1, DISABLE);
//            this_time_rx_len = JUDGE_BUF_LEN - DMA_GetCurrDataCounter(DMA2_Stream1);  //获得当前收到的字节数，因为DMA中断只有接收长度满时会触发中断
            DMA_SetCurrDataCounter(DMA2_Stream1, JUDGE_BUF_LEN);  //重新设置DMA的读取缓冲区长度
            DMA2_Stream1->CR |= DMA_SxCR_CT;  //将内存1设置为当前内存地址
            //清DMA中断标志 
            DMA_ClearFlag(DMA2_Stream1, DMA_FLAG_TCIF1 | DMA_FLAG_HTIF1);
					
						Judge_Read_Data(Judge_Buf[0]);		//读取裁判系统数据
					
					  memset(Judge_Buf[0], 0, 200);    //对象   内容  长度
					
            DMA_Cmd(DMA2_Stream1, ENABLE);					 
        }
        else
        {
            //重新设置DMA
            DMA_Cmd(DMA2_Stream1, DISABLE);
//            this_time_rx_len = JUDGE_BUF_LEN - DMA_GetCurrDataCounter(DMA2_Stream1);
            DMA_SetCurrDataCounter(DMA2_Stream1, JUDGE_BUF_LEN);
            DMA2_Stream1->CR &= ~(DMA_SxCR_CT);  //将内存0设置为当前内存地址
            //清DMA中断标志
            DMA_ClearFlag(DMA2_Stream1, DMA_FLAG_TCIF1 | DMA_FLAG_HTIF1);
					
					  Judge_Read_Data(Judge_Buf[1]);		//读取裁判系统数据
					
					  memset(Judge_Buf[1], 0, 200);    //对象   内容  长度
					
            DMA_Cmd(DMA2_Stream1, ENABLE);
        }
    }
} 
#endif	

 



