#include "eeprom.h"
#include "string.h"
#include "include.h"

bool drv_eeprom_write(uint8_t *ptr,uint8_t shift, uint16_t length);
uint8_t* drv_eeprom_read(uint8_t shift);


#define FLASH_LENGTH    300

//4kb = 4*1024 = 4096 = 0xFFF
//属于MAIN FLASH BANK1 SELECT31

//main flash 共有2个bank :bank0 bank1
//每个bank 32个扇区(sectors)
//每个sectors 4KB 共0xFFF空间
//main flash起始地址：0x0000 - 0x3F FFF
#define CALIBRATION_START   0x3F000

void HAL_EEPROM_Init(void)
{
    
}

void HAL_EEPROM_Read(uint8_t shift, uint8_t length, uint8_t *Target)
{
    memcpy(Target, drv_eeprom_read(shift), length);
}

bool HAL_EEPROM_Write(uint8_t shift, uint8_t *ptr, uint16_t length)
{
    return drv_eeprom_write(ptr, shift, length);
}

bool drv_eeprom_write(uint8_t *ptr,uint8_t shift, uint16_t length)
{
    bool Check = true;
    uint8_t Temp[300];
    
    MAP_FlashCtl_unprotectSector(FLASH_MAIN_MEMORY_SPACE_BANK1, FLASH_SECTOR31);
    memcpy(Temp, (uint8_t*)CALIBRATION_START, FLASH_LENGTH);
    memcpy(Temp+shift, ptr, length);
    
    if(!MAP_FlashCtl_eraseSector(CALIBRATION_START))
        while(1);
    
    if(!MAP_FlashCtl_programMemory(Temp, (void*) CALIBRATION_START, 300))
    {
        Check = false;
    }
            
    MAP_FlashCtl_protectSector(FLASH_MAIN_MEMORY_SPACE_BANK1,FLASH_SECTOR31);
    
    return Check;
}

uint8_t* drv_eeprom_read(uint8_t shift)
{
    return (uint8_t*)(CALIBRATION_START + shift);
}
