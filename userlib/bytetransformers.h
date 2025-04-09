#ifndef __BYTETRANSFORMERS_H_
#define __BYTETRANSFORMERS_H_

#include "stm32f4xx.h"

enum Significant_Bit {
        LSB,  // 低位先行
        MSB   // 高位先行
    };

uint16_t get_u16(const void *arr, enum Significant_Bit sb);
int16_t get_i16(const void *arr, enum Significant_Bit sb);
uint32_t get_u32(const void *arr, enum Significant_Bit sb);
int32_t get_i32(const void *arr, enum Significant_Bit sb);
float get_f(const void *arr, enum Significant_Bit sb);
void split_number(const void* src, size_t size, uint8_t* dst, enum Significant_Bit sb);

#endif  // __BYTETRANSFORMERS_H_
