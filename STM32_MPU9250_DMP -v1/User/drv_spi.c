/* Standard includes. */

/* hardware includes */
//#include "stm32f10x.h"
#include "drv_spi.h"
#include "stm32f10x_spi.h"

#define MPU6500_CS(X)	  (X==0)?GPIO_ResetBits(GPIOA,GPIO_Pin_4):GPIO_SetBits(GPIOA,GPIO_Pin_4) //MPU6500片选信号

/*
 * 函数名：SPI1_Init
 * 描述  ：SPI1初始化
 * 输入  ：无
 * 输出  ：无
 */ 
void SPI1_Init(void)
{
  SPI_InitTypeDef SPI_InitStructure;
//  EXTI_InitTypeDef   EXTI_InitStructure;
  GPIO_InitTypeDef   GPIO_InitStructure;
//  NVIC_InitTypeDef   NVIC_InitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1 | RCC_APB2Periph_AFIO, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;//CLK,MISO,MOSI
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;//CS
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_SetBits(GPIOA, GPIO_Pin_4);
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  SPI_Cmd(SPI1, DISABLE); //
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //SPI设置为双线双向全双工
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		//SPI主机
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		//发送接收8位帧结构
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;		//时钟悬空高
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;	//数据捕获于第2个时钟沿
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		//NSS信号由硬件控制
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32/*SPI_BaudRatePrescaler_8*/;		//定义波特率预分频的值:波特率预分频值为8=36/8=4.5MHz
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	//数据传输从MSB位开始
  SPI_InitStructure.SPI_CRCPolynomial = 7;	//CRC值计算的多项式
  SPI_Init(SPI1, &SPI_InitStructure);  //根据SPI_InitStruct中指定的参数初始化外设SPIx寄存器

  SPI_Cmd(SPI1, ENABLE); //使能SPI外设

  //#define mpu_irq
  #ifdef mpu_irq
  // setup irq port
  /* Configure PB1 pin as input floating */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* Connect EXTI Line */
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource1);
  /* Configure EXTI6 line */
  EXTI_InitStructure.EXTI_Line = EXTI_Line1;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  /* Enable and set EXTI6 Interrupt to the lowest priority */
  NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x04;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x04;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  #endif

  //printf("MPU6500 SPI1 bus init success...\r\n");
}

/*
 * 函数名：SPI1_Read_Write_Byte
 * 描述  ：读写一个字节
 * 输入  ：TxData:要写入的字节
 * 输出  ：读取到的字节
 */ 
u8 SPI1_Read_Write_Byte(uint8_t TxData)
{		
  u8 retry = 0;				 	
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET) 	//检查指定的SPI标志位设置与否:发送缓存空标志位
  {
	retry++;
	if(retry > 250)	return 0;
  }			  
  SPI_I2S_SendData(SPI1, TxData); 																//通过外设SPIx发送一个数据
  retry = 0;

  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET) //检查指定的SPI标志位设置与否:接受缓存非空标志位
  {
	retry++;
	if(retry > 250) return 0;
  }	  						    
  return SPI_I2S_ReceiveData(SPI1); 															//返回通过SPIx最近接收的数据					    
}

/*************************************************************/
u8 MPU6500_Read_MultiBytes(u8 slavaddr,u8 reg,u8 len,u8 buf[])
{
  uint8_t i;
  uint8_t status;
  MPU6500_CS(0); 						//使能SPI传输
//  SPI1_Read_Write_Byte( reg | 0x80 ); //发送读命令+寄存器号	
//  for(i=0;i<len;i++)					
//  {
//    buf[i] = SPI1_Read_Write_Byte(0xff);//输入0xff,因为slave不识别
//  }
	
  status=SPI1_Read_Write_Byte( reg|0x80 ); //发送读命令+寄存器号	
  for(i=0;i<len;i++)					
  {
    buf[i] = SPI1_Read_Write_Byte(0xff);//输入0xff,因为slave不识别
  }
  MPU6500_CS(1);
  return 0;//0 if successful
}
/*************************************************************/
u8 MPU6500_Write_MultiBytes(u8 slavaddr,u8 reg,u8 len,u8 buf[])
{
  uint8_t status,i;
  MPU6500_CS(0);  					//使能SPI传输
  status = SPI1_Read_Write_Byte(reg); //发送写命令+寄存器号
  for(i=0;i<len;i++)
  {
	SPI1_Read_Write_Byte(buf[i]);	//写入寄存器值
  }
  MPU6500_CS(1);  					//禁止MPU6500
//	return(status);						//返回状态值
  return 0;
}






#ifdef SOLFTSPI

#define SPI1_CLK_PIN  GPIO_Pin_5
#define SPI1_MISO_PIN GPIO_Pin_6
#define SPI1_MOSI_PIN GPIO_Pin_7
#define SPI1_CS_PIN   GPIO_Pin_4
#define SPI1_PORT     GPIOA

#define spi1_clk_high()   GPIO_SetBits(SPI1_PORT, SPI1_CLK_PIN);
#define spi1_clk_low()    GPIO_ResetBits(SPI1_PORT, SPI1_CLK_PIN);
#define spi1_cs_high()    GPIO_SetBits(SPI1_PORT, SPI1_CS_PIN);
#define spi1_cs_low()     GPIO_ResetBits(SPI1_PORT, SPI1_CS_PIN);
#define spi1_mosi_high()  GPIO_SetBits(SPI1_PORT, SPI1_MOSI_PIN);
#define spi1_mosi_low()   GPIO_ResetBits(SPI1_PORT, SPI1_MOSI_PIN);
#define spi1_miso_read()  GPIO_ReadInputDataBit(SPI1_PORT,SPI1_MISO_PIN);

void SolftSpi_Configuration(void)
{
  GPIO_InitTypeDef   GPIO_InitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

  GPIO_InitStructure.GPIO_Pin = SPI1_CLK_PIN | SPI1_MOSI_PIN;//CLK,MOSI
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(SPI1_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = SPI1_CS_PIN;//CS
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_SetBits(SPI1_PORT, SPI1_CS_PIN);
  GPIO_Init(SPI1_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = SPI1_MISO_PIN;//MISO
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(SPI1_PORT, &GPIO_InitStructure); 
}

void SolftSpi_write_one_byte(u8 value)
{
  u8 i;
  for(i=8;i>0;i--)
  {
	if(value&0x80)
	{
	  spi1_mosi_high();
	}
	else
	{
	  spi1_mosi_low();
	}
  }
}
#endif
