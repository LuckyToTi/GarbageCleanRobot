#include "eeprom.h"
#include "iic3.h"
#include "rtthread.h"

#define MIN(a, b) ((a) < (b) ? (a) : (b))

#define EEPROM_OPERATE_DELAY_TIME (300)
#define EEPROM_DEVADD   (0x57 << 1)     //I2C地址
#define PAGE_NUM        (512)           // 总页数
#define PAGE_SIZE       (64)            // 每页大小

int EepromRead_NoRtos(unsigned short eeprom_addr, void* p_data, unsigned short size)
{
    // 检查地址是否超出范围
    if ((eeprom_addr + size) > (PAGE_NUM * PAGE_SIZE)) {
      return -1;//错误处理
    }
    unsigned short remaining_bytes = size;
    unsigned short current_address = eeprom_addr;
    while (remaining_bytes > 0) {
      unsigned short read_size = MIN(PAGE_SIZE - (current_address % PAGE_SIZE), remaining_bytes);
      if(I2c3MemRead_NoRtos(EEPROM_DEVADD, current_address, I2C_MEMADD_SIZE_16BIT, p_data, read_size) != 0)
      {
          return -2;//错误处理
      }
      p_data += read_size;
      remaining_bytes -= read_size;
      current_address += read_size;
      HAL_Delay(EEPROM_OPERATE_DELAY_TIME);
    }
    return 0u;
}

int EepromRead(unsigned short eeprom_addr, void* p_data, unsigned short size)
{
    // 检查地址是否超出范围
    if ((eeprom_addr + size) > (PAGE_NUM * PAGE_SIZE)) {
      return -1;//错误处理
    }
    unsigned short remaining_bytes = size;
    unsigned short current_address = eeprom_addr;
    while (remaining_bytes > 0) {
      unsigned short read_size = MIN(PAGE_SIZE - (current_address % PAGE_SIZE), remaining_bytes);
      if(I2c3MemRead(EEPROM_DEVADD, current_address, I2C_MEMADD_SIZE_16BIT, p_data, read_size) != 0)
      {
          return -2;//错误处理
      }
      p_data += read_size;
      remaining_bytes -= read_size;
      current_address += read_size;
      rt_thread_mdelay(EEPROM_OPERATE_DELAY_TIME);
    }
    return 0u;
}

int EepromWrite(unsigned short eeprom_addr, void* p_data, unsigned short size)
{
    // 检查地址是否超出范围
    if ((eeprom_addr + size) > (PAGE_NUM * PAGE_SIZE)) {
      return -1;//错误处理
    }
    unsigned short remaining_bytes = size;
    unsigned short current_address = eeprom_addr;
    while (remaining_bytes > 0) {
      unsigned short write_size = MIN(PAGE_SIZE - (current_address % PAGE_SIZE), remaining_bytes);
      if(I2c3MemWrite(EEPROM_DEVADD, current_address, I2C_MEMADD_SIZE_16BIT, p_data, write_size) != 0)
      {
          return -2;//错误处理
      }
      p_data += write_size;
      remaining_bytes -= write_size;
      current_address += write_size;
      rt_thread_mdelay(EEPROM_OPERATE_DELAY_TIME);
    }
    return 0u;
}
