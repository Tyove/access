/**
  ******************************************************************************
  * Copyright (c) 2018,�����пƺƵ�Ƽ����޹�˾
  * All rights reserved.
  * �ļ����ƣ�nrf24l01.h
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
#ifndef __NRF24L01_H
#define __NRF24L01_H 
//�ⲿ�ļ�����
#include "include.h"


//�궨����


//���ݽṹ����
typedef enum
{
    NRF_Mode_RX = 0,
    NRF_Mode_TX,
    NRF_Mode_RX2,
    NRF_Mode_TX2,
    
    num_NRF_Mode,
}NRF_Mode_t;

typedef struct
{
    NRF_Mode_t NRF_Mode;
    uint8_t Tx_Addr[5];
    uint8_t Rx_Addr[5];
    uint8_t RC_Frequency;
    
    void (*set_rx_mode)(void);
    void (*set_tx_mode)(void);
    bool (*update_rx_Addr)(uint8_t *);
    bool (*update_tx_Addr)(uint8_t *);
    bool (*update_frequency)(uint8_t );
    uint8_t (*send_buff)(uint8_t *ptr);
    uint8_t (*receive_buff)(uint8_t *ptr);
    bool Check;
}NRF24L01_Manager_t;


//Extern����
extern NRF24L01_Manager_t g_NRFManager;


//��������
void NRF24L01Init(NRF24L01_Manager_t *ptr);
void NRF24L01TxPacketAp(uint8_t * tx_buf, uint8_t len);
void NRF24L01TxPacket(uint8_t * tx_buf, uint8_t len);
#endif

