#include "stm32f10x.h"
#include "delay.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "inv_mpu.h"
#include "math.h"
#include "usart.h"
#include "stm32_iic.h"
#include "drv_spi.h"

/*
*Corporation:	www.Makeblock.com
*Project:		Airblock
*brief:			DMP lib
*Device:		STM32F103RCT6 
*MODULE:		MPU9250(MPU6500+AK8963)
*interface:     SPI
*IO:			SCK->PA5  MISO->PA6 MOSI->PA7 CS->PA4
*				LED0->PC12
*Preference:    DEFAULT_MPU_HZ DMPfifoƵ��200HZ���
                gyro_orientation[9] �����Ƿ�����󣬿����޸��������������
*zzp@2016-11-24
*/
/*
���pitch roll yaw
��ֲʱֻ��Ҫ�޸�ͨ�Žӿ�IIC����SPI��Ȼ��ȡ��inv_mpu.c����� #define i2c_write   MPU6500_Write_MultiBytes//i2cwrite
														    #define i2c_read    MPU6500_Read_MultiBytes//i2cread
option->c/c++������� MPU9250�궨�����֧��ʹ�� MPU9250  MPU6500 DMP
*/

#define  led_on    GPIO_ResetBits(GPIOC, GPIO_Pin_12)
#define  led_off   GPIO_SetBits(GPIOC, GPIO_Pin_12)
    
//ָʾ������
void LED_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
  
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);    

    led_off; 
}

float Pitch=0,Roll=0,Yaw=0;
int main(void)
{  
    u16 count=0;    
    SystemInit();
    USART2_Config();//���ڳ�ʼ��
    LED_Config();
    
//    i2cInit();      //IIC���ߵĳ�ʼ��
    SPI1_Init();
    mpu_dmp_init();
    led_off;
    
    while(1)
    {
        count ++;   //�����������������״̬
        if(count<1000)
        {
            led_on; 
        }
        else if(count<2000)
        {
            led_off;
        }
        else if(count==2000)
        {
            count = 0;
        }
        dmp_get_angle(&Pitch,&Roll ,&Yaw);
    }
}

 


