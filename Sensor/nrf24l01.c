/**
  ******************************************************************************
  * Copyright (c) 2018,北京中科浩电科技有限公司
  * All rights reserved.
  * 文件名称：nrf24l01.c
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

/*==============================================================================
                         ##### How to use this driver #####
==============================================================================
NRF24L01的驱动使用方法如下：
1.实例化一个NRF24L01_Manager_t结构体；
2.调用NRF24L01Init，初始化结构体；
3.根据初始化的模式，选择发送数据的发送函数

*/
//外部文件引用
#include "nrf24l01.h"
#include "SPI.h"
#include <string.h>
#include "nrf24l01.h"
#include "SPI.h"
#include <string.h>
#include "fmuConfig.h"
#include "communication.h"


//宏定义区
#define MAX_TX                      0x10  //达到最大发送次数中断
#define TX_OK                       0x20  //TX发送完成中断
#define RX_OK                       0x40  //接收到数据中断

#define SUCCESS                     0
#define FAILED                      1
#define FLASH_TX_ADDR_OFFSET        7
#define FLASH_RX_ADDR_OFFSET        2
#define FLASH_FREQ_ADDR_OFFSET      0


#define TX_ADR_WIDTH    5                               //5字节的地址宽度
#define RX_ADR_WIDTH    5                               //5字节的地址宽度
#define TX_PLOAD_WIDTH  32                              //20字节的用户数据宽度
#define RX_PLOAD_WIDTH  32                              //20字节的用户数据宽度

#define SPI_READ_REG    0x00  //读配置寄存器,低5位为寄存器地址
#define SPI_WRITE_REG   0x20  //写配置寄存器,低5位为寄存器地址
#define RD_RX_PLOAD     0x61  //读RX有效数据,1~32字节
#define WR_TX_PLOAD     0xA0  //写TX有效数据,1~32字节
#define FLUSH_TX        0xE1  //清除TX FIFO寄存器.发射模式下用
#define FLUSH_RX        0xE2  //清除RX FIFO寄存器.接收模式下用
#define REUSE_TX_PL     0xE3  //重新使用上一包数据,CE为高,数据包被不断发送.
#define NOP             0xFF  //空操作,可以用来读状态寄存器     
#define NCONFIG          0x00  //配置寄存器地址;bit0:1接收模式,0发射模式;bit1:电选择;bit2:CRC模式;bit3:CRC使能;
                              //bit4:中断MAX_RT(达到最大重发次数中断)使能;bit5:中断TX_DS使能;bit6:中断RX_DR使能
#define EN_AA           0x01  //使能自动应答功能  bit0~5,对应通道0~5
#define EN_RXADDR       0x02  //接收地址允许,bit0~5,对应通道0~5
#define SETUP_AW        0x03  //设置地址宽度(所有数据通道):bit1,0:00,3字节;01,4字节;02,5字节;
#define SETUP_RETR      0x04  //建立自动重发;bit3:0,自动重发计数器;bit7:4,自动重发延时 250*x+86us
#define RF_CH           0x05  //RF通道,bit6:0,工作通道频率;
#define RF_SETUP        0x06  //RF寄存器;bit3:传输速率(0:1Mbps,1:2Mbps);bit2:1,发射功率;bit0:低噪声放大器增益
#define STATUS          0x07  //状态寄存器;bit0:TX FIFO满标志;bit3:1,接收数据通道号(最大:6);bit4,达到最多次重发
                              //bit5:数据发送完成中断;bit6:接收数据中断;
#define MAX_TX          0x10  //达到最大发送次数中断
#define TX_OK           0x20  //TX发送完成中断
#define RX_OK           0x40  //接收到数据中断

#define OBSERVE_TX      0x08  //发送检测寄存器,bit7:4,数据包丢失计数器;bit3:0,重发计数器
#define CD              0x09  //载波检测寄存器,bit0,载波检测;
#define RX_ADDR_P0      0x0A  //数据通道0接收地址,最大长度5个字节,低字节在前
#define RX_ADDR_P1      0x0B  //数据通道1接收地址,最大长度5个字节,低字节在前
#define RX_ADDR_P2      0x0C  //数据通道2接收地址,最低字节可设置,高字节,必须同RX_ADDR_P1[39:8]相等;
#define RX_ADDR_P3      0x0D  //数据通道3接收地址,最低字节可设置,高字节,必须同RX_ADDR_P1[39:8]相等;
#define RX_ADDR_P4      0x0E  //数据通道4接收地址,最低字节可设置,高字节,必须同RX_ADDR_P1[39:8]相等;
#define RX_ADDR_P5      0x0F  //数据通道5接收地址,最低字节可设置,高字节,必须同RX_ADDR_P1[39:8]相等;
#define TX_ADDR         0x10  //发送地址(低字节在前),ShockBurstTM模式下,RX_ADDR_P0与此地址相等
#define RX_PW_P0        0x11  //接收数据通道0有效数据宽度(1~32字节),设置为0则非法
#define RX_PW_P1        0x12  //接收数据通道1有效数据宽度(1~32字节),设置为0则非法
#define RX_PW_P2        0x13  //接收数据通道2有效数据宽度(1~32字节),设置为0则非法
#define RX_PW_P3        0x14  //接收数据通道3有效数据宽度(1~32字节),设置为0则非法
#define RX_PW_P4        0x15  //接收数据通道4有效数据宽度(1~32字节),设置为0则非法
#define RX_PW_P5        0x16  //接收数据通道5有效数据宽度(1~32字节),设置为0则非法
#define FIFO_STATUS     0x17  //FIFO状态寄存器;bit0,RX FIFO寄存器空标志;bit1,RX FIFO满标志;bit2,3,保留
                              //bit4,TX FIFO空标志;bit5,TX FIFO满标志;bit6,1,循环发送上一数据包.0,不循环;         



//Extern引用
extern uint8_t NRF_RW(uint8_t data);                                         //写数据区            

//私有函数区
uint8_t WriteBuf(uint8_t regaddr, uint8_t *pBuf, uint8_t datalen);    //写数据区
uint8_t ReadBuf(uint8_t regaddr, uint8_t *pBuf, uint8_t datalen);     //读数据区          
uint8_t ReadReg(uint8_t regaddr);                                     //读寄存器
uint8_t WriteReg(uint8_t regaddr, uint8_t data);                      //写寄存器
uint8_t Check(void);                                                  //检查NRF24L01是否在位
uint8_t TxPacket(uint8_t *txbuf);                                     //发送一个包的数据
uint8_t RxPacket(uint8_t *rxbuf);                                     //接收一个包的数据

//私有变量区
NRF24L01_Manager_t g_NRFManager;

/******************************************************************************
  * 函数名称：WriteReg
  * 函数描述：SPI总线写NRF24L01寄存器
  * 输    入：uint8_t regaddr:寄存器地址
              uint8_t data:需要写入的数据
  * 输    出：void
  * 返    回：是否写入成功，0：写入成功
                           1：写入失败
  * 备    注：null
  *    
  *
******************************************************************************/
uint8_t WriteReg(uint8_t regaddr, uint8_t data)
{
    uint8_t status = 0;   
    
    CLR_NRF_CSN;                    //使能SPI传输
    status = NRF_RW(regaddr); //发送寄存器号 
    NRF_RW(data);            //写入寄存器的值
    SET_NRF_CSN;                    //禁止SPI传输 
    
    return(status);                        //返回状态值
}
//读取SPI寄存器值 ，regaddr:要读的寄存器
/******************************************************************************
  * 函数名称：ReadReg
  * 函数描述：SPI总线读NRF24L01寄存器
  * 输    入：uint8_t regaddr:寄存器地址
  * 输    出：void
  * 返    回：是否读取成功，0：读取成功
                           1：读取失败
  * 备    注：null  
  *    
  *
******************************************************************************/
uint8_t ReadReg(uint8_t regaddr)
{
    uint8_t reg_val = 0; 
    
    CLR_NRF_CSN;                //使能SPI传输        
    NRF_RW(regaddr);                 //发送寄存器号
    reg_val = NRF_RW(0XFF);          //读取寄存器内容
    SET_NRF_CSN;                //禁止SPI传输
    
    return(reg_val);                 //返回状态值
}    

/******************************************************************************
  * 函数名称：ReadBuf
  * 函数描述：批量读取NRF24L01寄存器数据
  * 输    入：uint8_t regaddr:寄存器地址
              uint8_t *pBuf:读取数据的存放地址
              uint8_t datalen:读取数据的长度
  * 输    出：void
  * 返    回：是否读取成功，0：读取成功
                           1：读取失败
  * 备    注：    
  *    
  *
******************************************************************************/
uint8_t ReadBuf(uint8_t regaddr, uint8_t *pBuf, uint8_t datalen)
{
    uint8_t status = 0;
    
    CLR_NRF_CSN;                       //使能SPI传输
    status = NRF_RW(regaddr);                 //发送寄存器值(位置),并读取状态值          
    
    for(uint8_t ctr = 0; ctr < datalen; ctr++)
    {
        pBuf[ctr] = NRF_RW(0XFF);           //读出数据
    }
    
    SET_NRF_CSN;                       //关闭SPI传输
    
    return status;                          //返回读到的状态值
}

/******************************************************************************
  * 函数名称：WriteBuf
  * 函数描述：批量写入NRF24L01寄存器数据
  * 输    入：uint8_t regaddr:要写入数据的寄存器地址
              uint8_t *pBuf:要写入的数据的地址
              uint8_t datalen:要写入的数据的长度
  * 输    出：void
  * 返    回：是否读取成功，0：读取成功
                           1：读取失败
  * 备    注：    
  *    
  *
******************************************************************************/
uint8_t WriteBuf(uint8_t regaddr, uint8_t *pBuf, uint8_t datalen)
{
    uint8_t status = 0;
    
    CLR_NRF_CSN;                                    //使能SPI传输
    status = NRF_RW(regaddr);                            //发送寄存器值(位置),并读取状态值
    
    for(int i = 0; i < datalen; i++)
    {
        NRF_RW(*pBuf++);                                 //写入数据     
    }
    SET_NRF_CSN;                                    //关闭SPI传输

    return status;                                       //返回读到的状态值
}        

/******************************************************************************
  * 函数名称：Check
  * 函数描述：检查NRF24L01硬件是否连接成功
  * 输    入：void
  * 输    出：void
  * 返    回：是否读取成功，0：连接成功
                           1：连接失败
  * 备    注：null
  *    
  *
******************************************************************************/
uint8_t Check(void)
{
    uint8_t buf[5] = {0xA5, 0xA5, 0xA5, 0xA5, 0xA5};
    uint8_t buf1[5] = {0};
    uint8_t i = 0;
    
    WriteBuf(SPI_WRITE_REG + TX_ADDR, buf, 5); //写入5个字节的地址.    
    ReadBuf(TX_ADDR, buf1, 5);                 //读出写入的地址      
    
    for(i = 0; i < 5; i++)
    {
        if(buf1[i] != 0xA5)
        {
            break;                       
        }
    }
    
    if(i != 5)
    {
        return 1;                               //NRF24L01不在位            
    }else
    {
        return 0;                               //NRF24L01在位
    }
}

/******************************************************************************
  * 函数名称：TxPacket
  * 函数描述：发送一个数据包，由于系统对NRF的发送和接收采用轮询方式，此函数只将要
              发送的数据压入通信队列中
  * 输    入：uint8_t *txbuf:发送的数据地址
  * 输    出：void
  * 返    回：0
  * 备    注：由于发送长度固定为32，所以此处也没有发送的数据长度维度可以选择   
  *    
  *
******************************************************************************/
uint8_t TxPacket(uint8_t *txbuf)
{
    enQueue(&NRF_Mannager.qTx, txbuf, TX_PLOAD_WIDTH);
    return 0;
}

/******************************************************************************
  * 函数名称：RxPacket
  * 函数描述：NRF24L01接收函数，采用轮询方式查询当前是否有数据接收到
  * 输    入：uint8_t *rxbuf:接收数据的地址
  * 输    出：void
  * 返    回：是否读取成功，0：接收成功
                           1：接收失败
  * 备    注：    
  *    
  *
******************************************************************************/
uint8_t RxPacket(uint8_t *rxbuf)
{
    uint8_t state = 0;                                              
    
    state = ReadReg(STATUS);                //读取状态寄存器的值         
    WriteReg(SPI_WRITE_REG + STATUS, state); //清除TX_DS或MAX_RT中断标志
    
    if(state & RX_OK)                                 //接收到数据
    {
        ReadBuf(RD_RX_PLOAD, rxbuf, RX_PLOAD_WIDTH);//读取数据
        WriteReg(FLUSH_RX, 0xff);          //清除RX FIFO寄存器
        
        return SUCCESS; 
    }
    return FAILED;                                      //没收到任何数据
}

/******************************************************************************
  * 函数名称：UpdateFreq
  * 函数描述：更新NRF24L01的收发频率
  * 输    入：uint8_t freq:设置收发频率      数据大小为0-64

  * 输    出：void
  * 返    回：是否更改成功：true:更改成功
                            false：更改失败
  * 备    注：    
  *    
  *
******************************************************************************/
bool UpdateFreq(uint8_t freq)
{
    bool status = false;
    
    CLR_NRF24L01_CE;     
    WriteReg(SPI_WRITE_REG + RF_CH, freq); 
    if(ReadReg(SPI_READ_REG + RF_CH) == freq)
    {
        Flash_Write(FLASH_FREQ_ADDR_OFFSET,&freq, 2);
        g_NRFManager.RC_Frequency = freq;
        
        status = true;
    }
    SET_NRF24L01_CE;
    
    return status;
}

/******************************************************************************
  * 函数名称：UpdateRxAddr
  * 函数描述：更新NRF24L01的接收地址
  * 输    入：uint8_t *ptr:接收地址存放的开始地址，长度固定为5
  * 输    出：void
  * 返    回：是否更改成功：true:更改成功
                            flase:更改失败
  * 备    注：    
  *    
  *
******************************************************************************/
bool UpdateRxAddr(uint8_t *ptr)
{
    uint8_t buf1[5];
    bool status = false;
    
    CLR_NRF24L01_CE;     
    WriteBuf(SPI_WRITE_REG + RX_ADDR_P0, ptr, RX_ADR_WIDTH); 
    ReadBuf(RX_ADDR_P0, buf1, 5);
    
    if(memcmp(buf1, ptr, 5) == 0)
    {
        Flash_Write(FLASH_RX_ADDR_OFFSET, ptr, 5);
        memcpy(g_UAVinfo.NRF_MannagerPtr->Rx_Addr, ptr, 5);
        
        status = true;
    }
    SET_NRF24L01_CE;  
    
    return status;
}

/******************************************************************************
  * 函数名称：UpdateTxAddr
  * 函数描述：更新NRF24L01发送地址
  * 输    入：uint8_t *ptr:发送地址存放的开始地址，长度固定为5
  * 输    出：void
  * 返    回：是否更改成功：true:更改成功
                            flase:更改失败 
  * 备    注：    
  *    
  *
******************************************************************************/
bool UpdateTxAddr(uint8_t *ptr)
{
    uint8_t buf1[5];
    bool status = false;
    
    CLR_NRF24L01_CE;     
    WriteBuf(SPI_WRITE_REG + TX_ADDR, ptr, RX_ADR_WIDTH); 
    ReadBuf(TX_ADDR, buf1, 5);
    
    if(memcmp(buf1, ptr, 5) == 0)
    {
        Flash_Write(FLASH_TX_ADDR_OFFSET, ptr, 5);
        memcpy(g_NRFManager.Tx_Addr, ptr, 5);
        
        status = true;
    }
    SET_NRF24L01_CE;
    
    return status;
}

/******************************************************************************
  * 函数名称：NRF24L01Init
  * 函数描述：NRF24L01初始化，并初始化指针指向
  * 输    入：NRF24L01_Manager_t *ptr：NRF24L01管理结构体指针
  * 输    出：void
  * 返    回：void
  * 备    注：null
  *    
  *
******************************************************************************/
void NRF24L01Init(NRF24L01_Manager_t *ptr)
{   
    while(Check() == FAILED)
    {
        static int ChectCnt = 0;
        
        ChectCnt++;
        
        if(ChectCnt == 5)
        {
            g_NRFManager.Check = false;
            return;
        }
    }
    g_NRFManager.Check = true;
    
    CLR_NRF24L01_CE;        

    WriteBuf(SPI_WRITE_REG + TX_ADDR,(uint8_t*)ptr->Tx_Addr, TX_ADR_WIDTH);                            //写TX节点地址 
    WriteBuf(SPI_WRITE_REG + RX_ADDR_P0,(uint8_t*)ptr->Rx_Addr, RX_ADR_WIDTH);                         //写RX节点地址 
    WriteReg(SPI_WRITE_REG + EN_AA, 0x01);                                                      //使能通道0的自动应答  
    WriteReg(SPI_WRITE_REG + EN_RXADDR, 0x01);                                                  //使能通道0的接收地址 
    WriteReg(SPI_WRITE_REG + SETUP_RETR, 0x1a);                                                 //设置自动重发间隔时间:500us + 86us;最大自动重发次数:10次
    WriteReg(SPI_WRITE_REG + RF_CH, ptr->RC_Frequency);                                         //设置RF通道为40    
    WriteReg(SPI_WRITE_REG + RF_SETUP, 0x0f);                                                   //0x27  250K   0x07 1M     

    switch(ptr->NRF_Mode)
    {
        case NRF_Mode_RX:
            WriteReg(SPI_WRITE_REG + RX_PW_P0, RX_PLOAD_WIDTH);                                //选择通道0的有效数据宽度 
            WriteReg(SPI_WRITE_REG + NCONFIG, 0x0f);            
            break;
        case NRF_Mode_TX:
            WriteReg(SPI_WRITE_REG + RX_PW_P0, RX_PLOAD_WIDTH);                                //选择通道0的有效数据宽度 
            WriteReg(SPI_WRITE_REG + NCONFIG, 0x0e);            // IRQ收发完成中断开启,16位CRC,主发送
            break;
        case NRF_Mode_RX2:
            WriteReg(FLUSH_TX, 0xff);
            WriteReg(FLUSH_RX, 0xff);
            WriteReg(SPI_WRITE_REG + NCONFIG, 0x0f);            // IRQ收发完成中断开启,16位CRC,主接收

            NRF_RW(0x50);
            NRF_RW(0x73);
            WriteReg(SPI_WRITE_REG + 0x1c, 0x01);
            WriteReg(SPI_WRITE_REG + 0x1d, 0x06);
            break;
        case NRF_Mode_TX2:
            WriteReg(SPI_WRITE_REG + NCONFIG, 0x0e);            // IRQ收发完成中断开启,16位CRC,主发送
            WriteReg(FLUSH_TX, 0xff);
            WriteReg(FLUSH_RX, 0xff);

            NRF_RW(0x50);
            NRF_RW(0x73);
            WriteReg(SPI_WRITE_REG + 0x1c, 0x01);
            WriteReg(SPI_WRITE_REG + 0x1d, 0x06);
            break;
        default:
            break;
    }
    
    SET_NRF24L01_CE;  
    
    ptr->receive_buff = RxPacket;
    ptr->send_buff = TxPacket;
    ptr->update_rx_Addr = UpdateRxAddr;
    ptr->update_tx_Addr = UpdateTxAddr;
    ptr->update_frequency = UpdateFreq;
}

/******************************************************************************
  * 函数名称：NRF24L01TxPacket
  * 函数描述：发送数据，以发送模式1发送
  * 输    入：uint8_t * tx_buf:发送buff的地址
              uint8_t len:发送buff的长度
  * 输    出：void
  * 返    回：void
  * 备    注：null
  *    
  *
******************************************************************************/
void NRF24L01TxPacket(uint8_t * tx_buf, uint8_t len)
{    
    CLR_NRF24L01_CE;    
    WriteBuf(SPI_WRITE_REG + RX_ADDR_P0, (uint8_t*)g_NRFManager.Tx_Addr, TX_ADR_WIDTH);   // 装载接收端地址
    WriteBuf(WR_TX_PLOAD, tx_buf, len);                                 // 装载数据    
    SET_NRF24L01_CE; 
}

/******************************************************************************
  * 函数名称：NRF24L01TxPacketAp
  * 函数描述：发送数据，以发送模式2发送
  * 输    入：uint8_t * tx_buf:发送buff的地址
              uint8_t len:发送buff的长度
  * 输    出：void
  * 返    回：void
  * 备    注：null 
  *    
  *
******************************************************************************/
void NRF24L01TxPacketAp(uint8_t * tx_buf, uint8_t len)
{    
    CLR_NRF24L01_CE;    
    WriteBuf(0xa8, tx_buf, len);              // 装载数据
    SET_NRF24L01_CE; 
}

/******************* (C) 版权所有 2018 北京中科浩电科技有限公司 *******************/
