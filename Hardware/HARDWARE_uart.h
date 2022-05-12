/**
  ******************************************************************************
  * Copyright (c) 2018,�����пƺƵ�Ƽ����޹�˾
  * All rights reserved.
  * �ļ����ƣ�UART.h
  * ժ    Ҫ��
  *
  * ��ǰ�汾��V1.0
  * ��    �ߣ������пƺƵ�Ƽ����޹�˾�з��� 
  * ������ڣ�    
  * �޸�˵����
  * 
  *
  * ��ʷ�汾�����¶�MSP432���첽�����շ�֧��
  *
  *
  *******************************************************************************/
#ifndef _UART_H
#define _UART_H
//�ⲿ�ļ�����
#include "include.h"

#define MAX_RECEIVE_CNT     50
#define MAX_TRANSMIT_CNT    50



typedef struct
{
    /*���ղ������ݱ���*/
    uint32_t moduleInstance;
    bool RxStart;
    void (*RxHandle)(uint8_t *Ptr, uint8_t Length);
    uint8_t RxTimeout;
    uint8_t RxCnt;
    uint8_t RxBuff[MAX_RECEIVE_CNT];
    
    /*���Ͳ������ݱ���*/
    bool TxCommpleate;
    Queue_t qTx;
    uint8_t TxCnt;
    uint8_t TxLength;
    uint8_t TxBuff[MAX_TRANSMIT_CNT];
}Usart_t;

//  * A2����S.BUS���ߣ�������Ϊ100K    
//  * A0����ANO������������Ϊ115200
//  * A3����OpenMV��������115200
//  * A1����������������115200
typedef enum
{
    UART_A0,
    UART_A1,
    UART_A2,
    UART_A3,
    
    Num_USART,
}emUSART_t;


extern Usart_t UsartGroup[Num_USART];

void USART_Init(void);
void USART_TX(Usart_t*, uint8_t* pTx, uint8_t len);
void PollingUSART(void);

#endif

/******************* (C) ��Ȩ���� 2018 �����пƺƵ�Ƽ����޹�˾ *******************/

