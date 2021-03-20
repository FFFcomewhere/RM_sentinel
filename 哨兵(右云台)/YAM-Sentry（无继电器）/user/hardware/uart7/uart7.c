#include "sys.h"
#include "uart7.h"
#include "vision.h"
#if SYSTEM_SUPPORT_OS
#include "FreeRTOS.h"					//ucos 使用	  
#endif

#if EN_UART7_RX   //如果使能了接收

vision_buffer_t vision_buffer;

//初始化IO 串口1 

//bound:波特率

int Uart7_Clean_IDLE_Flag = 0;  //用来读SR DR寄存器

uint8_t  Vision_Buffer[2][ VISION_BUFFER_LEN ];  //裁判系统发过来的数据暂存在这里

void uart7_init(u32 bound)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	
	//首先使能时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);//使能GPIOE时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART7,ENABLE);//使能USART7时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);//使能DMA时钟
	
	//串口6对应引脚复用映射
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource7,GPIO_AF_UART7); //GPIOE7复用为USART7  RX
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource8,GPIO_AF_UART7); //GPIOE8复用为USART7  TX
	
	//USART6端口配置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7; //GPIOE7 RX     DMA1通道5数据流3
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; //不上拉
	GPIO_Init(GPIOE,&GPIO_InitStructure); //初始化GPIOE7
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8; //GPIOE8 TX   DMA1通道5数据流7
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; //不上拉
	GPIO_Init(GPIOE,&GPIO_InitStructure); //初始化GPIOE8
	

	USART_DeInit(UART7);
	
	 //USART6 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(UART7, &USART_InitStructure); //初始化串口7
	
	USART_ClearFlag(UART7, USART_FLAG_IDLE);  //清除空闲线路中断标志位
	USART_ITConfig(UART7, USART_IT_IDLE, ENABLE); //使能串口空闲中断
	
	DMA_DeInit(DMA1_Stream3);
	DMA_InitStructure.DMA_Channel = DMA_Channel_5;                              //DMA通道5，在8个通道中选择一个
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (UART7->DR);       //外设地址
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)Vision_Buffer[0];             //存储器0地址，双缓存模式还要使用M1AR
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;                     //外设到存储器模式
  DMA_InitStructure.DMA_BufferSize = 100;                                     //数据传输量，以外设数据项为单位
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;            //外设地址保持不变
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                     //存储器地址递增
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;     //外设数据位宽8位
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;             //存储器数据位宽8位
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;                             //循环模式
  DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;                     //高优先级
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;                      //使能FIFO模式
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;                 //单次传输
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;         //单次传输
  DMA_Init(DMA1_Stream3, &DMA_InitStructure);
  DMA_DoubleBufferModeConfig(DMA1_Stream3, (uint32_t)Vision_Buffer[1], DMA_Memory_0);  //双缓冲模式，buffer0先被传输
  DMA_DoubleBufferModeCmd(DMA1_Stream3, ENABLE);                              //使能双缓冲模式
  DMA_Cmd(DMA1_Stream3, DISABLE); //Add a disable
  DMA_Cmd(DMA1_Stream3, ENABLE);
	
	USART_DMACmd(UART7, USART_DMAReq_Rx, ENABLE);  //DMA串口接收请求中断使能
	USART_DMACmd( UART7, USART_DMAReq_Tx, ENABLE );
	
	USART_Cmd(UART7, ENABLE);  //使能串口7 
		
	#if EN_UART7_RX	 //如果使能了接收
	//USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);//开启相关中断

	//Usart1 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = UART7_IRQn;//串口7中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=4;//抢占优先级4
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//子优先级0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、

#endif

	
}

void UART7_IRQHandler(void)                	//串口7中断服务程序
{

 if (USART_GetITStatus(UART7, USART_IT_RXNE) != RESET)
    {
        USART_ReceiveData(UART7);
    }
    else if (USART_GetITStatus(UART7, USART_IT_IDLE) != RESET)
    {
//        static uint16_t this_time_rx_len = 0;
        USART_ReceiveData(UART7);

        if(DMA_GetCurrentMemoryTarget(DMA1_Stream3) == 0)    //如果DMA正在访问存储区0 ，则CPU可以访问存储区1
        {
            //重新设置DMA
            DMA_Cmd(DMA1_Stream3, DISABLE);
//            this_time_rx_len = VISION_BUFFER_LEN - DMA_GetCurrDataCounter(DMA1_Stream3);  //获得当前收到的字节数，因为DMA中断只有接收长度满时会触发中断
            DMA_SetCurrDataCounter(DMA1_Stream3, VISION_BUFFER_LEN);  //重新设置DMA的读取缓冲区长度
            DMA1_Stream3->CR |= DMA_SxCR_CT;  //将内存1设置为当前内存地址
            //清DMA中断标志
            DMA_ClearFlag(DMA1_Stream3, DMA_FLAG_TCIF3 | DMA_FLAG_HTIF3);
					
						Vision_Read_Data(Vision_Buffer[0]);		//读取视觉数据
					  
//					  Vision_Send_Data(0xB1);
//					
					  memset(Vision_Buffer[0], 0, 200);    //对象   内容  长度
					
            DMA_Cmd(DMA1_Stream3, ENABLE);
					 

        }
        else
        {
            //重新设置DMA
            DMA_Cmd(DMA1_Stream3, DISABLE);
//            this_time_rx_len = VISION_BUFFER_LEN - DMA_GetCurrDataCounter(DMA1_Stream3);
            DMA_SetCurrDataCounter(DMA1_Stream3, VISION_BUFFER_LEN);
            DMA1_Stream3->CR &= ~(DMA_SxCR_CT);  //将内存0设置为当前内存地址
            //清DMA中断标志
            DMA_ClearFlag(DMA1_Stream3, DMA_FLAG_TCIF3 | DMA_FLAG_HTIF3);
					
					  Vision_Read_Data(Vision_Buffer[1]);		//读取视觉数据
					
//					  Vision_Send_Data(0xB2);
//					
					  memset(Vision_Buffer[1], 0, 200);    //对象   内容  长度
					
            DMA_Cmd(DMA1_Stream3, ENABLE);
					

        }
    }

} 

/**
  * @brief  串口一次发送一个字节数据
  * @param  数据
  * @retval void
  * @attention  8位
  */
void Uart7_SendChar(uint8_t Data)
{
	while (USART_GetFlagStatus(UART7, USART_FLAG_TC) == RESET);
	
	USART_SendData(UART7, Data);   
}

/*****************  发送一个字符 **********************/
void Usart_SendByte( USART_TypeDef * pUSARTx, uint8_t ch)
{
	/* 发送一个字节数据到USART */
	USART_SendData(pUSARTx,ch);
		
	/* 等待发送数据寄存器为空 */
	while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);	
}

/*****************  发送字符串 **********************/
void Usart_SendString( USART_TypeDef * pUSARTx, char *str)
{
	unsigned int k=0;
  do 
  {
      Usart_SendByte( pUSARTx, *(str + k) );
      k++;
  } while(*(str + k)!='\0');
  
  /* 等待发送完成 */
  while(USART_GetFlagStatus(pUSARTx,USART_FLAG_TC)==RESET)
  {}
}


#endif	
