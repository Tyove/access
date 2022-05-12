#include <stdint.h>
#include <stdbool.h>

void MCU_Init(void);

//I2C

//SPI
#ifdef ZKHD_HAWK_SPI_ENABLE
uint8_t Hardware_SPI1_RW(uint8_t data);
uint8_t Hardware_SPI1_MEM_Write(uint8_t Reg, uint8_t data);
uint8_t Hardware_SPI1_MEM_Writes(uint8_t Reg, uint8_t *Buff, uint8_t Length);
uint8_t Hardware_SPI1_MEM_Read(uint8_t Reg);
uint8_t Hardware_SPI1_MEM_Reads(uint8_t Reg, uint8_t *Buff, uint8_t Length);
uint8_t Hardware_SPI1_Device1_CSN_H(void);
uint8_t Hardware_SPI1_Device1_CSN_L(void);
#endif

//串口
#ifdef ZKHD_HAWK_USART1_ENABLE
uint8_t Hardware_USART1_Tx(uint8_t *ptr, uint8_t Length);
uint8_t Hardware_USART1_Rx(void);
#endif

#ifdef ZKHD_HAWK_USART2_ENABLE
uint8_t Hardware_USART2_Tx(uint8_t *ptr, uint8_t Length);
uint8_t Hardware_USART2_Rx(void);
#endif

#ifdef ZKHD_HAWK_USART3_ENABLE
uint8_t Hardware_USART3_Tx(uint8_t *ptr, uint8_t Length);
uint8_t Hardware_ USART3_Rx(void);
#endif

#ifdef ZKHD_HAWK_USART4_ENABLE
uint8_t Hardware_USART4_Tx(uint8_t *ptr, uint8_t Length);
uint8_t Hardware_USART4_Rx(void);
#endif

//Flash区
#ifdef ZKHD_HAWK_FLASH_ENABLE
uint8_t Hardware_Flash_Write(uint8_t *Addr, uint8_t Length);
uint8_t *Hardware_Flash_Read(uint8_t Length);
#endif

//ADC区
#ifdef ZKHD_HAWK_ADC_ENABLE
uint16_t Hardware_ADC_GetChannel1(void);
uint16_t Hardware_ADC_GetChannel2(void);
uint16_t Hardware_ADC_GetChannel3(void);
uint16_t Hardware_ADC_GetChannel4(void);
uint16_t Hardware_ADC_GetChannel5(void);
uint16_t Hardware_ADC_GetChannel6(void);
uint16_t Hardware_ADC_GetChannel7(void);
#endif
//Time

//GPIO
#ifdef ZKHD_HAWK_GPIO_ENABLE
//void GPIO_Pin1_Init(uint8_t);
//void GPIO_Pin1_Set(uint8_t);
//uint8_t GPIO_Pin1_Read();

//void GPIO_Pin2_Init(uint8_t);
//void GPIO_Pin2_Set(uint8_t);
//uint8_t GPIO_Pin2_Read();
#endif

//LED区
#ifdef ZKHD_HAWK_RGB_LED_ENABLE
void Hardware_LED_Red_ON(uint8_t);
void Hardware_LED_Red_OFF(void);
void Hardware_LED_Red_TOGGLE(void);
void Hardware_LED_Green_ON(uint8_t);
void Hardware_LED_Green_OFF(void);
void Hardware_LED_Green_TOGGLE(void);
void Hardware_LED_Blue_ON(uint8_t);
void Hardware_LED_Blue_OFF(void);
void Hardware_LED_Blue_TOGGLE(void);
#endif

#ifdef ZKHD_HAWK_MOTOR_LED_ENABLE
void Hardware_LED_MOTOR1_ON(void);
void Hardware_LED_MOTOR1_OFF(void);
void Hardware_LED_MOTOR1_TOGGLE(void);
void Hardware_LED_MOTOR2_ON(void);
void Hardware_LED_MOTOR2_OFF(void);
void Hardware_LED_MOTOR2_TOGGLE(void);
void Hardware_LED_MOTOR3_ON(void);
void Hardware_LED_MOTOR3_OFF(void);
void Hardware_LED_MOTOR3_TOGGLE(void);
void Hardware_LED_MOTOR4_ON(void);
void Hardware_LED_MOTOR4_OFF(void);
void Hardware_LED_MOTOR4_TOGGLE(void);
#endif

//Beep区
#ifdef ZKHD_HAWK_BEEP_ENABLE
void BEEP_ON(void);
void BEEP_OFF(void);
void BEEP_TOGGLE(void);
#endif

#ifdef ZKHD_HAWK_SYSTICK_ENABLE
void SysTickHandle(void);
#endif


//void Delay_ms(uint16_t ms);
void MotorUpdate(uint16_t M1, uint16_t M2, uint16_t M3, uint16_t M4);
