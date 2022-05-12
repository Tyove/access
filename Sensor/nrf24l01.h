/**
  ******************************************************************************
  * Copyright (c) 2018,北京中科浩电科技有限公司
  * All rights reserved.
  * 文件名称：nrf24l01.h
  * 摘    要：
  *
  * 当前版本：V1.0
  * 作    者：北京中科浩电科技有限公司研发部 
  * 完成日期：    
  * 修改说明：
  * 
  *
  * 历史版本：
  *
  *
  *******************************************************************************/
#ifndef __NRF24L01_H
#define __NRF24L01_H 
//外部文件引用
#include "include.h"


//宏定义区


//数据结构声明
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


//Extern引用
extern NRF24L01_Manager_t g_NRFManager;


//函数声明
void NRF24L01Init(NRF24L01_Manager_t *ptr);
void NRF24L01TxPacketAp(uint8_t * tx_buf, uint8_t len);
void NRF24L01TxPacket(uint8_t * tx_buf, uint8_t len);
#endif

