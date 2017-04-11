#ifndef DRV_SPI_H
#define DRV_SPI_H
#include <stdint.h>
#include <stdio.h>
#include <string.h>

void SPI1_Init(void);
uint8_t SPI1_Read_Write_Byte(uint8_t TxData);
unsigned char MPU6500_Read_MultiBytes(unsigned char slavaddr,unsigned char reg,unsigned char len,unsigned char buf[]);
unsigned char MPU6500_Write_MultiBytes(unsigned char slavaddr,unsigned char reg,unsigned char len,unsigned char buf[]);
#endif
