#ifndef __EEPROM_H_
#define __EEPROM_H_

#include <stm32f4xx.h>

int EepromRead_NoRtos(unsigned short eeprom_addr, void* p_data, unsigned short size);
int EepromRead(unsigned short eeprom_addr, void* p_data, unsigned short size);
int EepromWrite(unsigned short eeprom_addr, void* p_data, unsigned short size);

#endif  // __EEPROM_H_
