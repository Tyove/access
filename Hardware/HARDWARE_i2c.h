/**
  ******************************************************************************
  * Copyright (c) 2018,�����пƺƵ�Ƽ����޹�˾
  * All rights reserved.
  * �ļ����ƣ�i2c.h
  * ժ    Ҫ��
  *
  * ��ǰ�汾��V1.0
  * ��    �ߣ������пƺƵ�Ƽ����޹�˾�з��� 
  * ������ڣ�    
  * �޸�˵����
  * 
  *
  * ��ʷ�汾��
  *
  *
  *******************************************************************************/
#ifndef _I2C_H
#define _I2C_H
//�ⲿ�ļ�����
#include "include.h"

//�궨����
#define IIC_SDA_Out P6DIR |= BIT4
#define IIC_SDA_In  P6DIR &= ~BIT4
#define IIC_SCL_Out P6DIR |= BIT5
#define SDA 4
#define SCL 5
#define IIC P6OUT

//��������
uint8_t I2C_Read_Bytes(uint8_t Slaveaddr, uint8_t REG_Address, uint8_t *ptr, uint8_t length);
uint8_t I2C_Read_Byte(uint8_t Slaveaddr, uint8_t REG_Address);
void I2C_Write_Byte(uint8_t Slaveaddr, uint8_t REG_Address, uint8_t REG_data);
void I2C_Init(void);

#endif


/******************* (C) ��Ȩ���� 2018 �����пƺƵ�Ƽ����޹�˾ *******************/
