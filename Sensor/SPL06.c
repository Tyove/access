/*==============================================================================
                         ##### How to use this driver #####
==============================================================================
SPL06驱动可按如下方式使用：
1.调用 SPL06_Init() 函数，以初始化硬件设备；
2.固定频率调用 UpdateSPL06Info() 函数，以更新气压值和高度值；
*/
#ifndef SPL06_C
#define SPL06_C


//外部文件引用
#include "SPL06.h"
#include "HARDWARE_i2c.h"
#include <math.h>
#include "include.h"
#include <stdio.h>
#include "timer_drv.h"


//宏定义区
#define PRS_CFG                 0x06
#define TMP_CFG                 0x07
#define MEAS_CFG                0x08
#define SPL06_REST_VALUE        0x09
#define PRODUCT_ID              0X0D

//(1 / 5.25588f) Pressure factor
#define CONST_PF                0.1902630958    

// Fixed Temperature. ASL is a function of pressure and temperature,
// but as the temperature changes so much 
//(blow a little towards the flie and watch it drop 5 degrees)
// it corrupts the ASL estimates.
#define FIX_TEMP                25     
#define SPL06_Check             I2C_Read_Byte(HW_ADR, 0x0D)

//Extern引用


//私有函数区
void GetRawTemp(void);
void GetRawPressure(void);
void SetRate(uint8_t u8_Sensor, uint8_t u8_OverSmpl, uint8_t u8_SmplRate);
void SelectMode(uint8_t mode);
void CalcParam(void);


void bsp_SPL06_Get_Calib_Coef(SPL06_t *spl06);
void bsp_SPL06_Pressure_Rate_Set(SPL06_t *spl06, uint8_t measureRate, uint8_t oversampleRate);
void bsp_SPL06_Temperature_Rate_Set(SPL06_t *spl06, uint8_t tmpSensorSelect, uint8_t measureRate, uint8_t oversampleRate);
void bsp_SPL06_Set_And_Start_Measure(SPL06_t *spl06, SPL06_MEAS_MODE measMode, SPL06_MEAS_TYPE measType);
int32_t bsp_SPL06_Get_Temperature_Adc(SPL06_t *spl06);
int32_t bsp_SPL06_Get_Pressure_Adc(SPL06_t *spl06);
float bsp_SPL06_Get_Temperature(SPL06_t *spl06);
float bsp_SPL06_Get_Pressure(SPL06_t *spl06);

//私有变量区
SPL06_t device_SPL06;
SPL06Manager_t g_SPL06Manager;

/******************************************************************************
  * 函数名称：SPL06_Init
  * 函数描述：SPL06-01 初始化函数
  * 输    入：void
  * 输    出：vodd
  * 返    回：void
  * 备    注：null   
  *    
  *
******************************************************************************/
void SPL06_Init(void)
{
    if(SPL06_Check == 0x10)
    {
        g_SPL06Manager.Check = true;
    }else
    {
        g_SPL06Manager.Check = false;
    }
    
    g_SPL06Manager.i32RawPressure = 0;
    g_SPL06Manager.i32RawTemperature = 0;
    g_SPL06Manager.u8Chip_id = 0x34;

    CalcParam();

    SetRate(PRESSURE_SENSOR, 128, 32);   
    SetRate(TEMPERATURE_SENSOR, 32, 8);
    SelectMode(CONTINUOUS_P_AND_T);
    Delay_ms(3000);
    
    UpdateSPL06Info();
    g_SPL06Manager.fGround_Alt = g_SPL06Manager.fALT;
}

/******************************************************************************
  * 函数名称:SetRate
  * 函数描述:设置温度传感器的每秒采样次数以及过采样率
  * 输    入:
  * uint8_t u8_OverSmpl:过采样率,最大值为128
  * uint8_t u8_SmplRate:每秒采样次数(Hz),最大值为128
  * uint8_t u8_Sensor  :传感器选择
  *                     0:气压计
  *                     1:温度计
  * 输    出:void
  * 返    回:void
  * 备    注:null
  *
  *
******************************************************************************/
void SetRate(uint8_t u8_Sensor, uint8_t u8_SmplRate, uint8_t u8_OverSmpl)
{
    uint8_t u8Reg = 0;
    int32_t i32KPkT = 0;
    switch(u8_SmplRate)
    {
        case 2:
            u8Reg |= (1 << 5);
            break;
        case 4:
            u8Reg |= (2 << 5);
            break;
        case 8:
            u8Reg |= (3 << 5);
            break;
        case 16:
            u8Reg |= (4 << 5);
            break;
        case 32:
            u8Reg |= (5 << 5);
            break;
        case 64:
            u8Reg |= (6 << 5);
            break;
        case 128:
            u8Reg |= (7 << 5);
            break;
        case 1:
        default:
            break;
    }
    
    switch(u8_OverSmpl)
    {
        case 2:
            u8Reg |= 1;
            i32KPkT = 1572864;
            break;
        case 4:
            u8Reg |= 2;
            i32KPkT = 3670016;
            break;
        case 8:
            u8Reg |= 3;
            i32KPkT = 7864320;
            break;
        case 16:
            i32KPkT = 253952;
            u8Reg |= 4;
            break;
        case 32:
            i32KPkT = 516096;
            u8Reg |= 5;
            break;
        case 64:
            i32KPkT = 1040384;
            u8Reg |= 6;
            break;
        case 128:
            i32KPkT = 2088960;
            u8Reg |= 7;
            break;
        case 1:
        default:
            i32KPkT = 524288;
            break;
    }

    if(u8_Sensor == 0)
    {
        g_SPL06Manager.i32KP = i32KPkT;
        I2C_Write_Byte(HW_ADR, 0x06, u8Reg);
        if(u8_OverSmpl > 8)
        {
            u8Reg = I2C_Read_Byte(HW_ADR, 0x09);
            I2C_Write_Byte(HW_ADR, 0x09, u8Reg | 0x04);
        }
    }
    
    if(u8_Sensor == 1)
    {
        g_SPL06Manager.i32KT = i32KPkT;
        
        //Using mems temperature
        I2C_Write_Byte(HW_ADR, 0x07, u8Reg|0x80);  
        
        if(u8_OverSmpl > 8)
        {
            u8Reg = I2C_Read_Byte(HW_ADR, 0x09);
            I2C_Write_Byte(HW_ADR, 0x09, u8Reg | 0x08);
        }
    }
}

/******************************************************************************
  * 函数名称：CalcParam
  * 函数描述：获取校准参数
  * 输    入：void
  * 输    出：void
  * 返    回：void 
  * 备    注：null   
  *    
  *
******************************************************************************/
void CalcParam(void)
{
    g_SPL06Manager.Param.i16C0 = 204;
    g_SPL06Manager.Param.i16C1 = -261;
    g_SPL06Manager.Param.i32C00 = 80469;
    g_SPL06Manager.Param.i32C10 = -54769;
    g_SPL06Manager.Param.i16C01 = -2803;
    g_SPL06Manager.Param.i16C11 = 1226;
    g_SPL06Manager.Param.i16C20 = -10787;
    g_SPL06Manager.Param.i16C21 = 183;
    g_SPL06Manager.Param.i16C30 = -1603; 
}

/******************************************************************************
  * 函数名称：SelectMode
  * 函数描述：Select node for the continuously measurement
  * 输    入：
  * uint8_t mode:模式选择
  *              1:气压模式;
  *              2:温度模式; 
  *              3:气压和温度模式;
  * 输    出：
  * 返    回：
  * 备    注：
  *    
  *
******************************************************************************/

void SelectMode(uint8_t mode)
{
    I2C_Write_Byte(HW_ADR, 0x08, mode + 4);
}

/******************************************************************************
  * 函数名称：GetRawTemp
  * 函数描述：获取温度的原始值，并转换成32Bits整数
  * 输    入：void
  * 输    出：void
  * 返    回：void
  * 备    注：null    
  *    
  *
******************************************************************************/
void GetRawTemp(void)
{
    uint8_t u8Data[3] = {0};
    
    u8Data[0] = I2C_Read_Byte(HW_ADR, 0x03);
    u8Data[1] = I2C_Read_Byte(HW_ADR, 0x04);
    u8Data[2] = I2C_Read_Byte(HW_ADR, 0x05);

    g_SPL06Manager.i32RawTemperature = (int32_t)u8Data[0] << 16 | \
                                          (int32_t)u8Data[1] << 8  | \
                                          (int32_t)u8Data[2];
    
    g_SPL06Manager.i32RawTemperature = (g_SPL06Manager.i32RawTemperature & 0x800000)   ? \
                                          (0xFF000000 | g_SPL06Manager.i32RawTemperature) : \
                                          (g_SPL06Manager.i32RawTemperature);
}

/******************************************************************************
  * 函数名称：GetRawPressure
  * 函数描述：获取压力原始值，并转换成32bits整数
  * 输    入：void
  * 输    出：void
  * 返    回：void
  * 备    注：null      
  *    
  *
******************************************************************************/
void GetRawPressure(void)
{
    uint8_t u8Data[3] = {0};
    
    u8Data[0] = I2C_Read_Byte(HW_ADR, 0x00);
    u8Data[1] = I2C_Read_Byte(HW_ADR, 0x01);
    u8Data[2] = I2C_Read_Byte(HW_ADR, 0x02);
    
    g_SPL06Manager.i32RawPressure = (int32_t)u8Data[0] << 16 | \
                                       (int32_t)u8Data[1] << 8  | \
                                       (int32_t)u8Data[2];
    g_SPL06Manager.i32RawPressure = (g_SPL06Manager.i32RawPressure & 0x800000)   ? \
                                       (0xFF000000 | g_SPL06Manager.i32RawPressure) : \
                                       (g_SPL06Manager.i32RawPressure);
}

/******************************************************************************
  * 函数名称：GetTemp
  * 函数描述：在获取原始值的基础上，返回浮点校准后的温度值
  * 输    入：void
  * 输    出：void
  * 返    回：void
  * 备    注：null   
  *    
  *
******************************************************************************/
float GetTemp(void)
{
    float fTCompensate = 0;
    float fTsc = 0;

    fTsc = g_SPL06Manager.i32RawTemperature / (float)g_SPL06Manager.i32KT;
    fTCompensate =  g_SPL06Manager.Param.i16C0 * 0.5 + \
                    g_SPL06Manager.Param.i16C1 * fTsc;
    
    return fTCompensate;
}

/******************************************************************************
  * 函数名称：GetSPL06Press
  * 函数描述：在获取原始值的基础上，返回浮点校准后的压力值
  * 输    入：void
  * 输    出：void
  * 返    回：void
  * 备    注：null   
  *    
  *
******************************************************************************/
float GetSPL06Press(void)
{
    float fTsc = 0;
    float fPsc = 0;
    float fqua2 = 0;
    float fqua3 = 0;
    float fPCompensate = 0;

    fTsc = g_SPL06Manager.i32RawTemperature / (float)g_SPL06Manager.i32KT;
    fPsc = g_SPL06Manager.i32RawPressure / (float)g_SPL06Manager.i32KP;
    
    fqua2 = g_SPL06Manager.Param.i32C10 \
           + fPsc * (g_SPL06Manager.Param.i16C20 \
           + fPsc* g_SPL06Manager.Param.i16C30);
    fqua3 = fTsc * fPsc * (g_SPL06Manager.Param.i16C11 \
           + fPsc * g_SPL06Manager.Param.i16C21);
    
    fPCompensate = g_SPL06Manager.Param.i32C00 \
                   + fPsc * fqua2 + fTsc * g_SPL06Manager.Param.i16C01\
                   + fqua3;
    
    return fPCompensate;
}

/******************************************************************************
  * 函数名称：UpdateSPL06Info
  * 函数描述：更新气压计高度信息
  * 输    入：void
  * 输    出：void
  * 返    回：void
  * 备    注：null     
  *    
  *
******************************************************************************/
void UpdateSPL06Info()
{
    /* tropospheric properties (0-11km) for standard atmosphere */
    /* temperature at base height in Kelvin, [K] = [°C] + 273.15 */
    const double T1 = 15.0 + 273.15;

    /* temperature gradient in degrees per metre */    
    const double a = -6.5 / 1000;    
    
    /* gravity constant in m / s/s */
    const double g = 9.80665;    
    
    /* ideal gas constant in J/kg/K */
    const double R = 287.05;    
    
    /* current pressure at MSL in kPa */
    double p1 = 101325.0 / 1000.0;

    /* measured pressure in kPa */
    double p = bsp_SPL06_Get_Pressure(&device_SPL06) / 1000.0f;

    //Altitude = (((exp((-(a * R) / g) * log((p / p1)))) * T1) - T1) / a;
    g_SPL06Manager.fALT = (((pow((p / p1), (-(a * R) / g))) * T1) - T1) / a;
    g_SPL06Manager.fRelative_Alt = g_SPL06Manager.fALT - g_SPL06Manager.fGround_Alt;
}

void ResetAlt()
{
    g_SPL06Manager.fGround_Alt = g_SPL06Manager.fALT;
}

#define SPL06I2C    I2C0
#define HAL_SPL06_I2C_READ(addr, regisaddr,length,data) \
        I2C_Read_Bytes(addr, regisaddr,data,length)

#define HAL_SPL06_I2C_WRITE(SPL06_SLAVEADDR, regisaddr, data)\
        I2C_Write_Byte(SPL06_SLAVEADDR, regisaddr, data)

#define SPL06_SLAVEADDR						(0x77)		/*SDO Low: 0x77 / SDO High or NC: 0x77*/
#ifdef SPL06_SLAVEADDR
#undef SPL06_SLAVEADDR
#define HW_ADR_L                    0x76//SDO LOW
#define HW_ADR                      HW_ADR_L<<1
#define SPL06_SLAVEADDR             HW_ADR

#endif

/* SPL06?ú2???′??÷*/			
/*data register*/			
#define SPL06_PSR_B2						(0x00)		/*Prresure data[23:16](R)*/
#define SPL06_PSR_B1						(0x01)		/*Prresure data[15:8](R)*/
#define SPL06_PSR_B0						(0x02)		/*Prresure data[7:0](R)*/
#define SPL06_TMP_B2						(0x03)		/*temperature data[23:16](R)*/
#define SPL06_TMP_B1						(0x04)		/*temperature data[15:8](R)*/
#define SPL06_TMP_B0						(0x05)		/*temperature data[7:0](R)*/
				
/*Configuration register*/				
#define SPL06_PRS_CFG						(0x06)		/*Config pressure measurement rate(PM_RATE) and resolution(PM_PRC)(R/W)*/
#define SPL06_TMP_CFG						(0x07)		/*Config temperature measurement rate (TMP_RATE) and resolution (TMP_PRC)*/
#define SPL06_MEAS_CFG						(0x08)		/*Setup measurement mode(COEF_RDY / SENSOR_RDY / TMP_RDY / PRS_RDY / MEAS_CTRL)*/
#define SPL06_CFG_REG						(0x09)		/*Configuration of interrupts, measurement data shift, and FIFO enable*/
				
/*status register*/				
#define SPL06_INT_STS						(0x0A)		/*Interrupt status register. The register is cleared on read*/
#define SPL06_FIFO_STS						(0x0B)		/*FIFO status register(FIFO_FULL / FIFO_EMPTY)*/
				
/*system*/				
#define SPL06_RESET							(0x0C)		/*Flush FIFO or generate soft reset(FIFO_FLUSH / FIFO_RST)*/
#define SPL06_ID							(0x0D)		/*Product and Revision ID(PROD_ID / REV_ID)*/

/*Calibration Coefficients register*/
#define SPL06_COEF_C0_H						(0x10)		/*c0[11:4]				  */
#define SPL06_COEF_C0_L_C1_H				(0x11)		/*c0[3:0]     + c1[11:8]  */
#define SPL06_COEF_C1_L						(0x12)		/*c1[7:0]				  */
#define SPL06_COEF_C00_H					(0x13)		/*c00[19:12]			  */
#define SPL06_COEF_C00_M					(0x14)		/*c00[11:4]				  */
#define SPL06_COEF_C00_L_C10_H				(0x15)		/*c00[3:0]    + c10[19:16]*/
#define SPL06_COEF_C10_M					(0x16)		/*c10[15:8]				  */
#define SPL06_COEF_C10_L					(0x17)		/*c10[7:0]				  */
#define SPL06_COEF_C01_H					(0x18)		/*c01[15:8]               */
#define SPL06_COEF_C01_L					(0x19)		/*c01[7:0]                */
#define SPL06_COEF_C11_H					(0x1A)		/*c11[15:8]				  */
#define SPL06_COEF_C11_L					(0x1B)		/*c11[7:0]                */
#define SPL06_COEF_C20_H					(0x1C)		/*c20[15:8]               */
#define SPL06_COEF_C20_L					(0x1D)		/*c20[7:0]                */
#define SPL06_COEF_C21_H					(0x1E)		/*c21[15:8]   			  */
#define SPL06_COEF_C21_L					(0x1F)		/*c21[7:0]				  */
#define SPL06_COEF_C30_H					(0x20)		/*c30[15:8]				  */
#define SPL06_COEF_C30_L					(0x21)		/*c30[7:0]				  */
			
//#define SPL06_RESERVED					(0x22~0x27)
#define SPL06_COEF_SRCE						(0x28)		/*Internal / External temprature sensor(TMP_COEF_SRCE(r))	*/


/*SPL06??′??÷ì??÷?μ?°??ó|??′??÷*/
/*SPL06_PRS_CFG*/
/*PM_RATE(Pressure measurement rate)*/
#define SPL06_PRS_1_MEAS_RATE_W		    	(0x00 << 4)	/*1 measurements*/
#define SPL06_PRS_2_MEAS_RATE_W		    	(0x01 << 4)	/*2 measurements*/
#define SPL06_PRS_4_MEAS_RATE_W		    	(0x02 << 4)	/*4 measurements*/
#define SPL06_PRS_8_MEAS_RATE_W		  	    (0x03 << 4)	/*8 measurements*/
#define SPL06_PRS_16_MEAS_RATE_W		    (0x04 << 4)	/*16 measurements*/
#define SPL06_PRS_32_MEAS_RATE_W		    (0x05 << 4)	/*32 measurements*/
#define SPL06_PRS_64_MEAS_RATE_W		    (0x06 << 4)	/*64 measurements*/
#define SPL06_PRS_128_MEAS_RATE_W		    (0x07 << 4)	/*128 measurements*/
/*PM_PRC(Pressure oversampling rate)*/
#define SPL06_PRS_1_OVERSAMP_RATE_W			(0x00)	/*single times  		   kP=524288,  3.6ms*/
#define SPL06_PRS_2_OVERSAMP_RATE_W			(0x01)	/*2 times_LOW_Power 	   kP=1572864, 5.2ms*/
#define SPL06_PRS_4_OVERSAMP_RATE_W			(0x02)	/*4 times 				   kP=3670016, 8.4ms*/
#define SPL06_PRS_8_OVERSAMP_RATE_W			(0x03)	/*8 times 				   kP=7864320, 14.8ms*/
#define SPL06_PRS_16_OVERSAMP_RATE_W		(0x04)	/*16 times_Standard 	   kP=253952,  27.6ms*/
#define SPL06_PRS_32_OVERSAMP_RATE_W		(0x05)	/*32 times 				   kP=516096,  53.2ms*/
#define SPL06_PRS_64_OVERSAMP_RATE_W		(0x06)	/*64 times_High Precision  kP=1040384, 104.4ms*/
#define SPL06_PRS_128_OVERSAMP_RATE_W		(0x07)	/*128 times 			   kP=2088960, 206.8ms*/

/*SPL06_TMP_CFG*/
/*TMP_EXT(Temperature measurement sensor select)*/
#define SPL06_TMP_INT_SENSOR_W				(0x00 << 7) /*Internal sensor (in ASIC)*/
#define SPL06_TMP_EXT_SENSOR_W				(0x01 << 7) /*External sensor (in pressure sensor MEMS element)*/
/*TMP_RATE(Temperature measurement rate)*/
#define SPL06_TMP_1_MEAS_RATE_W		    	(0x00 << 4)	/*1 measurements*/
#define SPL06_TMP_2_MEAS_RATE_W		    	(0x01 << 4)	/*2 measurements*/
#define SPL06_TMP_4_MEAS_RATE_W		    	(0x02 << 4)	/*4 measurements*/
#define SPL06_TMP_8_MEAS_RATE_W		    	(0x03 << 4)	/*8 measurements*/
#define SPL06_TMP_16_MEAS_RATE_W		    (0x04 << 4)	/*16 measurements*/
#define SPL06_TMP_32_MEAS_RATE_W		    (0x05 << 4)	/*32 measurements*/
#define SPL06_TMP_64_MEAS_RATE_W		    (0x06 << 4)	/*64 measurements*/
#define SPL06_TMP_128_MEAS_RATE_W		    (0x07 << 4)	/*128 measurements*/
/*TMP_PRC(Temperature measurement rate)*/
#define SPL06_TMP_1_OVERSAMP_RATE_W			(0x00)	/*single times              kP=524288,  3.6ms*/
#define SPL06_TMP_2_OVERSAMP_RATE_W			(0x01)	/*2 times_LOW_Power         kP=1572864, 5.2ms*/
#define SPL06_TMP_4_OVERSAMP_RATE_W			(0x02)	/*4 times                   kP=3670016, 8.4ms*/
#define SPL06_TMP_8_OVERSAMP_RATE_W			(0x03)	/*8 times                   kP=7864320, 14.8ms*/
#define SPL06_TMP_16_OVERSAMP_RATE_W		(0x04)	/*16 times_Standard         kP=253952,  27.6ms*/
#define SPL06_TMP_32_OVERSAMP_RATE_W		(0x05)	/*32 times                  kP=516096,  53.2ms*/
#define SPL06_TMP_64_OVERSAMP_RATE_W		(0x06)	/*64 times_High Precision   kP=1040384, 104.4ms*/
#define SPL06_TMP_128_OVERSAMP_RATE_W		(0x07)	/*128 times                 kP=2088960, 206.8ms*/

/*SPL06_MEAS_CFG*/
/*COEF_RDY(Coefficients will be read to the Coefficients Registers after start- up)*/
#define SPL06_COEF_DATA_NOT_READY_R			(0x00) /*Coefficients are not available*/
#define SPL06_COEF_DATA_READY_R				(0x80) /*Coefficients are available*/
/*SENSOR_RDY(The pressure sensor is running through self initialization after start-up)*/
#define SPL06_PRS_SENSOR_INIT_NOT_CPLT_R	(0x00) /*Sensor initialization not complete*/
#define SPL06_PRS_SENSOR_INIT_CPLT_R	    (0x40) /*Sensor initialization complete*/
/*TMP_RDY(Temperature measurement ready)*/
#define SPL06_TMP_NEW_DATA_READY_R			(0x20)	/*new tmp data read, cleared when read*/
#define SPL06_PRS_NEW_DATA_READY_R			(0x10) /*new prs data read, cleared when read*/
/*MEAS_CTRL(Set measurement mode and type)*/
#define SPL06_MEAS_STANDBY_MODE_WR			(0x00) /*(000) Idle / Stop background measurement*/
#define SPL06_MEAS_PRS_COMMAND_MODE_WR		(0x01) /*(001) Pressure Command Mode measurement*/
#define SPL06_MEAS_TMP_COMMAND_MODE_WR		(0x02) /*(010) temperature Command Mode measurement*/
#define SPL06_MEAS_PRS_CONTINU_MODE_WR		(0x05) /*(101) Pressure Background Mode measurement*/
#define SPL06_MEAS_TMP_CONTINU_MODE_WR		(0x06) /*(110) temperature Background Mode measurement*/
#define SPL06_MEAS_PRS_TMP_CONTINU_MODE_WR  (0x07) /*(111) Pressure and temperature Background Mode measurement*/

/*SPL06_CFG_REG*/
/*T_SHIFT(Temperature result bit-shift)*/
#define SPL06_CFG_T_SHIFT_WR				(0x08) /*shift result right in data register,  Must be set to '1' when the oversampling rate is >8 times.*/
/*P_SHIFT(Pressure result bit-shif)*/
#define SPL06_CFG_P_SHIFT_WR				(0x04) /*shift result right in data register,  Must be set to '1' when the oversampling rate is >8 times.*/

/*SPL06_RESET*/
/*SOFT_RST(soft reset)*/
#define SPL06_SOFT_RESET_W					(0x89)	/*(1001) Write '1001' to generate a soft reset. A soft reset will run though the
												  same sequences as in power-on reset, 0x80 fifo clear*/

/*SPL06_ID*/												  
#define SPL06_PROD_REV_ID_R					(0x10)	/*Product and revision ID*/




bool SPL06Check(void);

bool SPL06Init(void)
{
    device_SPL06.Device.bEnable = false;
    device_SPL06.Device.bCheck = false;
    device_SPL06.Device.FInit = SPL06Init;
    device_SPL06.Device.FUpdate = SPL06Update;
    sprintf(device_SPL06.Device.DeviceName, "SPL06");
    
    device_SPL06.Device.bCheck = SPL06Check();
    if(device_SPL06.Device.bCheck)
    {
//        xQueueCreate(1, sizeof(device_SPL06.RawAltitude));
        device_SPL06.Device.bEnable = true;
		device_SPL06.ChipId = SPL06_PROD_REV_ID_R;

        bsp_SPL06_Get_Calib_Coef(&device_SPL06);
		bsp_SPL06_Pressure_Rate_Set(&device_SPL06, 128, 64);
		bsp_SPL06_Temperature_Rate_Set(&device_SPL06, SPL06_TMP_EXT_SENSOR_W, 32, 8);
		bsp_SPL06_Set_And_Start_Measure(&device_SPL06, SPL06_BACKGROUND_MODE, SPL06_CONTINUOUS_P_AND_T_TYPE);
    }

    UpdateSPL06Info();
    g_SPL06Manager.fGround_Alt = g_SPL06Manager.fALT;
    return device_SPL06.Device.bEnable;
}

void SPL06Update(void)
{

}

bool SPL06Check()
{
    bool Check = false;
    uint8_t u8Check = 0;

    HAL_SPL06_I2C_READ(SPL06_SLAVEADDR, SPL06_ID, 1, &u8Check);

    if(u8Check == SPL06_PROD_REV_ID_R)
    {
        Check = true;
    }

    return Check;
}

void bsp_SPL06_Get_Calib_Coef(SPL06_t *spl06)
{
	uint8_t dataHigh, dataMid, dataLow;

	HAL_SPL06_I2C_READ(SPL06_SLAVEADDR, SPL06_COEF_C0_H, 1, &dataHigh);
	HAL_SPL06_I2C_READ(SPL06_SLAVEADDR, SPL06_COEF_C0_L_C1_H, 1, &dataMid);
	HAL_SPL06_I2C_READ(SPL06_SLAVEADDR, SPL06_COEF_C1_L, 1, &dataLow);	
	
	spl06->SPL06Param.c0 = ((int16_t)dataHigh << 4) | (dataMid >> 4);		/*c0 : 12bit*/
	spl06->SPL06Param.c0 = (spl06->SPL06Param.c0 & 0x0800) ? (0xF000 | spl06->SPL06Param.c0) : spl06->SPL06Param.c0;
	spl06->SPL06Param.c1 = ((int16_t)(dataMid & 0x0F) << 8) | dataLow;	/*c1 : 12bit*/
	spl06->SPL06Param.c1 = (spl06->SPL06Param.c1 & 0x0800) ? (0xF000 | spl06->SPL06Param.c1) : spl06->SPL06Param.c1;
	
	HAL_SPL06_I2C_READ(SPL06_SLAVEADDR, SPL06_COEF_C00_H, 1, &dataHigh);
	HAL_SPL06_I2C_READ(SPL06_SLAVEADDR, SPL06_COEF_C00_M, 1, &dataMid);
	HAL_SPL06_I2C_READ(SPL06_SLAVEADDR, SPL06_COEF_C00_L_C10_H, 1, &dataLow); 
	
	spl06->SPL06Param.c00 = ((int32_t)dataHigh << 12) | ((int16_t)dataMid << 4) | (dataLow >> 4);	/*c00 : 20bit*/
	spl06->SPL06Param.c00 = (spl06->SPL06Param.c00 & 0x080000) ? (0xFFF00000 | spl06->SPL06Param.c00) : spl06->SPL06Param.c00;
	
	dataHigh = dataLow;

	HAL_SPL06_I2C_READ(SPL06_SLAVEADDR, SPL06_COEF_C10_M, 1, &dataMid);		
	HAL_SPL06_I2C_READ(SPL06_SLAVEADDR, SPL06_COEF_C10_L, 1, &dataLow);
	
	spl06->SPL06Param.c10 = (((int32_t)dataHigh & 0x0f) << 16) | ((int16_t)dataMid << 8) | dataLow; /*c10 : 20bit*/
	spl06->SPL06Param.c10 = (spl06->SPL06Param.c10 & 0x080000) ? (0xFFF00000 | spl06->SPL06Param.c10) : spl06->SPL06Param.c10;
	
	HAL_SPL06_I2C_READ(SPL06_SLAVEADDR, SPL06_COEF_C01_H, 1, &dataHigh);		
	HAL_SPL06_I2C_READ(SPL06_SLAVEADDR, SPL06_COEF_C01_L, 1, &dataLow);

	spl06->SPL06Param.c01 = ((int16_t)dataHigh << 8) | dataLow;	/*c01 : 16bit*/
	
	HAL_SPL06_I2C_READ(SPL06_SLAVEADDR, SPL06_COEF_C11_H, 1, &dataHigh);		
	HAL_SPL06_I2C_READ(SPL06_SLAVEADDR, SPL06_COEF_C11_L, 1, &dataLow);

	spl06->SPL06Param.c11 = ((int16_t)dataHigh << 8) | dataLow;	/*c11 : 16bit*/

	HAL_SPL06_I2C_READ(SPL06_SLAVEADDR, SPL06_COEF_C20_H, 1, &dataHigh);		
	HAL_SPL06_I2C_READ(SPL06_SLAVEADDR, SPL06_COEF_C20_L, 1, &dataLow);

	spl06->SPL06Param.c20 = ((int16_t)dataHigh << 8) | dataLow;	/*c20 : 16bit*/
	
	HAL_SPL06_I2C_READ(SPL06_SLAVEADDR, SPL06_COEF_C21_H, 1, &dataHigh);		
	HAL_SPL06_I2C_READ(SPL06_SLAVEADDR, SPL06_COEF_C21_L, 1, &dataLow);

	spl06->SPL06Param.c21 = ((int16_t)dataHigh << 8) | dataLow;	/*c21 : 16bit*/
	
	HAL_SPL06_I2C_READ(SPL06_SLAVEADDR, SPL06_COEF_C30_H, 1, &dataHigh);		
	HAL_SPL06_I2C_READ(SPL06_SLAVEADDR, SPL06_COEF_C30_L, 1, &dataLow);

	spl06->SPL06Param.c30 = ((int16_t)dataHigh << 8) | dataLow;	/*c30 : 16bit*/
}

void bsp_SPL06_Pressure_Rate_Set(SPL06_t *spl06, uint8_t measureRate, uint8_t oversampleRate)
{
	uint8_t prsCfgReg = 0;
	uint32_t kp = 0;
	
	/*测量频率设置*/
	switch(measureRate)
	{
		case 1:
		{
			prsCfgReg |= SPL06_PRS_1_MEAS_RATE_W;	/*000*/
		}break;	
		
		case 2:
		{
			prsCfgReg |= SPL06_PRS_2_MEAS_RATE_W;	/*001*/		
		}break;
		
		case 4:
		{
			prsCfgReg |= SPL06_PRS_4_MEAS_RATE_W;	/*010*/					
		}break;

		case 8:
		{
			prsCfgReg |= SPL06_PRS_8_MEAS_RATE_W;	/*011*/					
		}break;

		case 16:
		{
			prsCfgReg |= SPL06_PRS_16_MEAS_RATE_W;	/*100*/			
		}break;

		case 32:
		{
			prsCfgReg |= SPL06_PRS_32_MEAS_RATE_W;	/*101*/			
		}break;

		case 64:
		{
			prsCfgReg |= SPL06_PRS_64_MEAS_RATE_W;	/*110*/			
		}break;

		case 128:
		{
			prsCfgReg |= SPL06_PRS_128_MEAS_RATE_W;	/*111*/			
		}break;	

		default:break;
	}
	
	/*细分采样设置(精度precision)*/	
	switch(oversampleRate)
	{
		/*single(单次)*/
		case 1:
		{
			prsCfgReg |= SPL06_PRS_1_OVERSAMP_RATE_W; /*0000*/
			kp = 524288;
		}break;
		
		/*low power(低功耗)*/
		case 2:
		{
			prsCfgReg |= SPL06_PRS_2_OVERSAMP_RATE_W; /*0001*/
			kp = 1572864;		
		}break;
		
		case 4:
		{
			prsCfgReg |= SPL06_PRS_4_OVERSAMP_RATE_W; /*0010*/			
			kp = 3670016;
		}break;

		case 8:
		{
			prsCfgReg |= SPL06_PRS_8_OVERSAMP_RATE_W; /*0011*/	
			kp = 7864320;
		}break;

		/*standard(标准)*/
		case 16:
		{
			prsCfgReg |= SPL06_PRS_16_OVERSAMP_RATE_W; /*0100*/			
			kp = 253952;			
		}break;

		case 32:
		{
			prsCfgReg |= SPL06_PRS_32_OVERSAMP_RATE_W; /*0101*/			
			kp = 516096;			
		}break;

		/*high precision(高精度)*/
		case 64:
		{
			prsCfgReg |= SPL06_PRS_64_OVERSAMP_RATE_W; /*0110*/			
			kp = 1040384;			
		}break;

		case 128:
		{
			prsCfgReg |= SPL06_PRS_128_OVERSAMP_RATE_W; /*0111*/			
			kp = 2088960;			
		}break;	
		
		default:break;
	}
	
	/*气压采样设置*/
	spl06->Kp = kp;
	HAL_SPL06_I2C_WRITE(SPL06_SLAVEADDR, SPL06_PRS_CFG, prsCfgReg);
		
	if (oversampleRate > 8) /*> 8 times*/
	{
		HAL_SPL06_I2C_READ(SPL06_SLAVEADDR, SPL06_CFG_REG, 1, &prsCfgReg);
		HAL_SPL06_I2C_WRITE(SPL06_SLAVEADDR, SPL06_CFG_REG, prsCfgReg | SPL06_CFG_P_SHIFT_WR);
	}
}

/*temperature  rate set*/
void bsp_SPL06_Temperature_Rate_Set(SPL06_t *spl06, uint8_t tmpSensorSelect, uint8_t measureRate, uint8_t oversampleRate)
{
	uint8_t tmpCfgReg = 0;
	uint32_t kt = 0;
	
	/*测量频率设置*/
	switch(measureRate)
	{
		case 1:
		{
			tmpCfgReg |= SPL06_TMP_1_MEAS_RATE_W;	/*000*/
		}break;	
		
		case 2:
		{
			tmpCfgReg |= SPL06_TMP_2_MEAS_RATE_W;	/*001*/		
		}break;
		
		case 4:
		{
			tmpCfgReg |= SPL06_TMP_4_MEAS_RATE_W;	/*010*/					
		}break;

		case 8:
		{
			tmpCfgReg |= SPL06_TMP_8_MEAS_RATE_W;	/*011*/					
		}break;

		case 16:
		{
			tmpCfgReg |= SPL06_TMP_16_MEAS_RATE_W;	/*100*/			
		}break;

		case 32:
		{
			tmpCfgReg |= SPL06_TMP_32_MEAS_RATE_W;	/*101*/			
		}break;

		case 64:
		{
			tmpCfgReg |= SPL06_TMP_64_MEAS_RATE_W;	/*110*/			
		}break;

		case 128:
		{
			tmpCfgReg |= SPL06_TMP_128_MEAS_RATE_W;	/*111*/			
		}break;	

		default:break;
	}
	
	/*细分采样设置(精度precision)*/	
	switch(oversampleRate)
	{
		/*single(单次)*/
		case 1:
		{
			tmpCfgReg |= SPL06_TMP_1_OVERSAMP_RATE_W; /*0000*/
			kt = 524288;
		}break;
		
		/*low power(低功耗)*/
		case 2:
		{
			tmpCfgReg |= SPL06_TMP_2_OVERSAMP_RATE_W; /*0001*/
			kt = 1572864;		
		}break;
		
		case 4:
		{
			tmpCfgReg |= SPL06_TMP_4_OVERSAMP_RATE_W; /*0010*/			
			kt = 3670016;
		}break;

		case 8:
		{
			tmpCfgReg |= SPL06_TMP_8_OVERSAMP_RATE_W; /*0011*/	
			kt = 7864320;
		}break;

		/*standard(标准)*/
		case 16:
		{
			tmpCfgReg |= SPL06_TMP_16_OVERSAMP_RATE_W; /*0100*/			
			kt = 253952;			
		}break;

		case 32:
		{
			tmpCfgReg |= SPL06_TMP_32_OVERSAMP_RATE_W; /*0101*/			
			kt = 516096;			
		}break;

		/*high precision(高精度)*/
		case 64:
		{
			tmpCfgReg |= SPL06_TMP_64_OVERSAMP_RATE_W; /*0110*/			
			kt = 1040384;			
		}break;

		case 128:
		{
			tmpCfgReg |= SPL06_TMP_128_OVERSAMP_RATE_W; /*0111*/			
			kt = 2088960;			
		}break;	
		
		default:break;
	}
	
	/*温度传感器选择内部/外部*/
	tmpCfgReg |= tmpSensorSelect;
	
	/*温度采样设置*/
	spl06->Kt = kt;
	HAL_SPL06_I2C_WRITE(SPL06_SLAVEADDR, SPL06_TMP_CFG, tmpCfgReg);
		
	if (oversampleRate > 8)  /*> 8 times*/
	{
		HAL_SPL06_I2C_READ(SPL06_SLAVEADDR, SPL06_CFG_REG, 1, &tmpCfgReg);
		HAL_SPL06_I2C_WRITE(SPL06_SLAVEADDR, SPL06_CFG_REG, tmpCfgReg | SPL06_CFG_T_SHIFT_WR);
	}
}

/*set measurement mode and type then start measure*/
void bsp_SPL06_Set_And_Start_Measure(SPL06_t *spl06, SPL06_MEAS_MODE measMode, SPL06_MEAS_TYPE measType)
{
	HAL_SPL06_I2C_WRITE(SPL06_SLAVEADDR, SPL06_MEAS_CFG, measType);
}

/*spl06 get raw temperature(adc)*/
int32_t bsp_SPL06_Get_Temperature_Adc(SPL06_t *spl06)
{
	uint8_t dataBuff[3] = {0};
	int32_t tmpAdc;
	
	HAL_SPL06_I2C_READ(SPL06_SLAVEADDR, SPL06_TMP_B2, 3, dataBuff);		

	tmpAdc = ((int32_t)dataBuff[0] << 16) | ((int32_t)dataBuff[1] << 8) | (int32_t)dataBuff[2];
	tmpAdc = (tmpAdc & 0x800000) ? (0xFF000000 | tmpAdc) : tmpAdc;
	
	spl06->RawTemperature = tmpAdc;	

	return (spl06->RawTemperature);	
}

/*spl06 get raw pressure(adc)*/
int32_t bsp_SPL06_Get_Pressure_Adc(SPL06_t *spl06)
{
	uint8_t dataBuff[3] = {0};
	int32_t prsAdc;
	
	HAL_SPL06_I2C_READ(SPL06_SLAVEADDR, SPL06_PSR_B2, 3, dataBuff);		
	
	prsAdc = ((int32_t)dataBuff[0] << 16) | ((int32_t)dataBuff[1] << 8) | (int32_t)dataBuff[2];
	prsAdc = (prsAdc & 0x800000) ? (0xFF000000 | prsAdc) : prsAdc;
	
	spl06->RawPressure = prsAdc;

	return (spl06->RawPressure);
}

/*spl06 get temperature*/
float bsp_SPL06_Get_Temperature(SPL06_t *spl06)
{
	float Tcomp, Traw_sc;
	int32_t rawTemperature;
	
	/*获取气压计温度原始值*/
	rawTemperature = bsp_SPL06_Get_Temperature_Adc(spl06);
	
	/*Calculate scaled measurement results*/
	Traw_sc = (float)rawTemperature / (float)spl06->Kt;
	
	/*Calculate compensated measurement results*/
	Tcomp   = (spl06->SPL06Param.c0 * 0.5f) + (spl06->SPL06Param.c1 * Traw_sc);

	spl06->SPL06Temperature = Tcomp;
	
	return (spl06->SPL06Temperature);
}

/*spl06 get pressure(Pa)*/
float bsp_SPL06_Get_Pressure(SPL06_t *spl06)
{
	int32_t rawPressure;
	float Traw_sc, Praw_sc, Pcomp;
	float qua2, qua3;	
	
	/*Calculate scaled measurement results*/
	Traw_sc = (float)spl06->RawPressure / (float)spl06->Kt;
	
	/*get raw pressure*/	
	rawPressure = bsp_SPL06_Get_Pressure_Adc(spl06);
	
	/*Calculate scaled measurement results*/
	Praw_sc = (float)rawPressure / (float)spl06->Kp;
	
	/*Calculate compensated measurement results*/
	qua2  = spl06->SPL06Param.c10 + Praw_sc * (spl06->SPL06Param.c20 + Praw_sc * spl06->SPL06Param.c30);
	qua3  = Traw_sc * Praw_sc * (spl06->SPL06Param.c11 + Praw_sc * spl06->SPL06Param.c21);
	Pcomp = spl06->SPL06Param.c00 + Praw_sc * qua2 + Traw_sc * spl06->SPL06Param.c01 + qua3;
	
	spl06->SPL06Press = Pcomp;
	
	return (spl06->SPL06Press);
}

#endif

/******************* (C) 版权所有 2018 北京中科浩电科技有限公司 *******************/
