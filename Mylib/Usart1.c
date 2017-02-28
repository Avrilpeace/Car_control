#include "main.h"

extern void delay_ms(unsigned int t);
extern void SendCan(void);
volatile unsigned char sbus_rx_buffer[25];

RC_Ctl_t RC_Ctl;	

void DMA1_Channel5_IRQHandler(void)
{
if(DMA_GetITStatus(DMA1_IT_TC5))
{
// DMA_ClearFlag(DMA1_Stream5, DMA_FLAG_TCIF5);
// DMA_ClearITPendingBit(DMA1_Stream5, DMA_IT_TCIF5);
DMA_ClearFlag(DMA1_FLAG_TE5);	
DMA_ClearITPendingBit(DMA1_IT_GL5);
RC_Ctl.rc.ch0 = (sbus_rx_buffer[0]| (sbus_rx_buffer[1] << 8)) & 0x07ff; //!< Channel 0
RC_Ctl.rc.ch1 = ((sbus_rx_buffer[1] >> 3) | (sbus_rx_buffer[2] << 5)) & 0x07ff; //!< Channel 1
RC_Ctl.rc.ch2 = ((sbus_rx_buffer[2] >> 6) | (sbus_rx_buffer[3] << 2) | //!< Channel 2
(sbus_rx_buffer[4] << 10)) & 0x07ff;
RC_Ctl.rc.ch3 = ((sbus_rx_buffer[4] >> 1) | (sbus_rx_buffer[5] << 7)) & 0x07ff; //!< Channel 3
RC_Ctl.rc.s1 = ((sbus_rx_buffer[5] >> 4)& 0x000C) >> 2; //!< Switch left
RC_Ctl.rc.s2 = ((sbus_rx_buffer[5] >> 4)& 0x0003); //!< Switch right
RC_Ctl.mouse.x = sbus_rx_buffer[6] | (sbus_rx_buffer[7] << 8); //!< Mouse X axis
RC_Ctl.mouse.y = sbus_rx_buffer[8] | (sbus_rx_buffer[9] << 8); //!< Mouse Y axis
RC_Ctl.mouse.z = sbus_rx_buffer[10] | (sbus_rx_buffer[11] << 8); //!< Mouse Z axis
RC_Ctl.mouse.press_l = sbus_rx_buffer[12]; //!< Mouse Left Is Press ?
RC_Ctl.mouse.press_r = sbus_rx_buffer[13]; //!< Mouse Right Is Press ?
RC_Ctl.key.v = sbus_rx_buffer[14] | (sbus_rx_buffer[15] << 8); //!< KeyBoard value
}

SendCan();
}

/*-----USART1_RX-----PA10----*/ 
//for D-BUS


void USART1_Configuration(void)
{
    USART_InitTypeDef usart1;
	  GPIO_InitTypeDef  gpio;
    NVIC_InitTypeDef  nvic;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |RCC_APB2Periph_AFIO,ENABLE); //gpio和端口复用时钟使能
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1 ,ENABLE);  //DMA直接数据存取时钟使能
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);  //usart通讯时钟使能
	
	gpio.GPIO_Mode=GPIO_Mode_IPU;
	gpio.GPIO_Pin=GPIO_Pin_10;
	GPIO_Init(GPIOA,&gpio);
    
  USART_DeInit(USART1);
	usart1.USART_BaudRate = 100000;   //SBUS 100K baudrate
	usart1.USART_WordLength = USART_WordLength_8b;
	usart1.USART_StopBits = USART_StopBits_1;
	usart1.USART_Parity = USART_Parity_Even;   //偶校验
	usart1.USART_Mode = USART_Mode_Rx;
  usart1.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(USART1,&usart1);
    
	USART_Cmd(USART1,ENABLE);

  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	//USART2 NVIC  接收非空中断
    nvic.NVIC_IRQChannel = USART1_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority=1 ;
    nvic.NVIC_IRQChannelSubPriority = 0;		
    nvic.NVIC_IRQChannelCmd = ENABLE;			
    NVIC_Init(&nvic);	
}

void USART1_IRQHandler(void)                	
{
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  
    {
        DMA_InitTypeDef		dma;
        USART_InitTypeDef	USART_InitStructure;
        NVIC_InitTypeDef	NVIC_InitStructure;

        USART_DeInit(USART1);  

        delay_ms(3);
			
       	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1 ,ENABLE);
			
				USART_InitStructure.USART_BaudRate = 100000;   //SBUS 100K baudrate
				USART_InitStructure.USART_WordLength = USART_WordLength_8b;
				USART_InitStructure.USART_StopBits = USART_StopBits_1;
				USART_InitStructure.USART_Parity = USART_Parity_Even;   //偶校验
				USART_InitStructure.USART_Mode = USART_Mode_Rx;
				USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
				USART_Init(USART1,&USART_InitStructure);
				USART_Cmd(USART1,ENABLE);
			
				USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);  //使能串口接受DMA
				
				DMA_DeInit(DMA1_Channel5);			
// 				dma.DMA_Channel= DMA_Channel_5;
				dma.DMA_PeripheralBaseAddr = (uint32_t)&(USART2->DR);  //源头buf
				dma.DMA_MemoryBaseAddr = (uint32_t)sbus_rx_buffer;    //目标buf
// 				dma.DMA_DIR = DMA_DIR_PeripheralToMemory;
				dma.DMA_DIR = DMA_DIR_PeripheralSRC;      //外设作为源头
				dma.DMA_BufferSize = 18;                  //buf大小
				dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;     //外设地址寄存器不递增
				dma.DMA_MemoryInc = DMA_MemoryInc_Enable;          //内存地址递增
				dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;    //外设字节的单位
				dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;      //内存字节的单位
				dma.DMA_Mode = DMA_Mode_Circular;                  //循环模式
				dma.DMA_Priority = DMA_Priority_VeryHigh;         //4优先级之一（高级）
// 				dma.DMA_FIFOMode = DMA_FIFOMode_Disable;                 
// 				dma.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
// 				dma.DMA_MemoryBurst = DMA_Mode_Normal;
// 				dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
				dma.DMA_M2M=DMA_M2M_Disable;
				DMA_Init(DMA1_Channel5,&dma);

				DMA_ITConfig(DMA1_Channel5,DMA_IT_TC,ENABLE);
				DMA_Cmd(DMA1_Channel5,ENABLE);
				
				NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel5_IRQn;
				NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
				NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
				NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
				NVIC_Init(&NVIC_InitStructure);
		}
}
