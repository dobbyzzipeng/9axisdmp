/* Standard includes. */

/* hardware includes */
//#include "stm32f10x.h"
#include "drv_spi.h"
#include "stm32f10x_spi.h"

#define MPU6500_CS(X)	  (X==0)?GPIO_ResetBits(GPIOA,GPIO_Pin_4):GPIO_SetBits(GPIOA,GPIO_Pin_4) //MPU6500Ƭѡ�ź�

/*
 * ��������SPI1_Init
 * ����  ��SPI1��ʼ��
 * ����  ����
 * ���  ����
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
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //SPI����Ϊ˫��˫��ȫ˫��
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		//SPI����
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		//���ͽ���8λ֡�ṹ
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;		//ʱ�����ո�
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;	//���ݲ����ڵ�2��ʱ����
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		//NSS�ź���Ӳ������
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32/*SPI_BaudRatePrescaler_8*/;		//���岨����Ԥ��Ƶ��ֵ:������Ԥ��ƵֵΪ8=36/8=4.5MHz
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	//���ݴ����MSBλ��ʼ
  SPI_InitStructure.SPI_CRCPolynomial = 7;	//CRCֵ����Ķ���ʽ
  SPI_Init(SPI1, &SPI_InitStructure);  //����SPI_InitStruct��ָ���Ĳ�����ʼ������SPIx�Ĵ���

  SPI_Cmd(SPI1, ENABLE); //ʹ��SPI����

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
 * ��������SPI1_Read_Write_Byte
 * ����  ����дһ���ֽ�
 * ����  ��TxData:Ҫд����ֽ�
 * ���  ����ȡ�����ֽ�
 */ 
u8 SPI1_Read_Write_Byte(uint8_t TxData)
{		
  u8 retry = 0;				 	
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET) 	//���ָ����SPI��־λ�������:���ͻ���ձ�־λ
  {
	retry++;
	if(retry > 250)	return 0;
  }			  
  SPI_I2S_SendData(SPI1, TxData); 																//ͨ������SPIx����һ������
  retry = 0;

  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET) //���ָ����SPI��־λ�������:���ܻ���ǿձ�־λ
  {
	retry++;
	if(retry > 250) return 0;
  }	  						    
  return SPI_I2S_ReceiveData(SPI1); 															//����ͨ��SPIx������յ�����					    
}

/*************************************************************/
u8 MPU6500_Read_MultiBytes(u8 slavaddr,u8 reg,u8 len,u8 buf[])
{
  uint8_t i;
  uint8_t status;
  MPU6500_CS(0); 						//ʹ��SPI����
//  SPI1_Read_Write_Byte( reg | 0x80 ); //���Ͷ�����+�Ĵ�����	
//  for(i=0;i<len;i++)					
//  {
//    buf[i] = SPI1_Read_Write_Byte(0xff);//����0xff,��Ϊslave��ʶ��
//  }
	
  status=SPI1_Read_Write_Byte( reg|0x80 ); //���Ͷ�����+�Ĵ�����	
  for(i=0;i<len;i++)					
  {
    buf[i] = SPI1_Read_Write_Byte(0xff);//����0xff,��Ϊslave��ʶ��
  }
  MPU6500_CS(1);
  return 0;//0 if successful
}
/*************************************************************/
u8 MPU6500_Write_MultiBytes(u8 slavaddr,u8 reg,u8 len,u8 buf[])
{
  uint8_t status,i;
  MPU6500_CS(0);  					//ʹ��SPI����
  status = SPI1_Read_Write_Byte(reg); //����д����+�Ĵ�����
  for(i=0;i<len;i++)
  {
	SPI1_Read_Write_Byte(buf[i]);	//д��Ĵ���ֵ
  }
  MPU6500_CS(1);  					//��ֹMPU6500
//	return(status);						//����״ֵ̬
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
