#include "bytetransformers.h"

uint16_t get_u16(const void *arr, enum Significant_Bit sb)
{
    if (arr == NULL) return 0;
    if (sb == LSB)
        return ((((uint8_t*)arr)[1] << 8) | (((uint8_t*)arr)[0]));
    else if (sb == MSB)
        return ((((uint8_t*)arr)[0] << 8) | (((uint8_t*)arr)[1]));
    return 0;
}


int16_t get_i16(const void *arr, enum Significant_Bit sb) {
    uint16_t v = get_u16(arr, sb);
    return *((int16_t*)&v);
}

uint32_t get_u32(const void *arr, enum Significant_Bit sb)
{
    if(arr == NULL) return 0;
    if (sb == LSB)
        return ((((uint8_t*)arr)[3] << 24) | (((uint8_t*)arr)[2] << 16) | (((uint8_t*)arr)[1] << 8) | (((uint8_t*)arr)[0]));
    else if (sb == MSB)
        return ((((uint8_t*)arr)[0] << 24) | (((uint8_t*)arr)[1] << 16) | (((uint8_t*)arr)[2] << 8) | (((uint8_t*)arr)[3]));
    return 0;
}


int32_t get_i32(const void *arr, enum Significant_Bit sb) {
    uint32_t v = get_u32(arr, sb);
    return *((int32_t*)&v);
}

inline static float get_f_from_u32(const uint32_t data) {
    uint32_t data_tmp = data;
    return *((float *)&data_tmp);
}

float get_f(const void *arr, enum Significant_Bit sb) {
    return get_f_from_u32(get_u32(arr, sb));
}

void split_number(const void* src, size_t size, uint8_t* dst, enum Significant_Bit sb) {
    for(int i=0; i<size; i++) {
        dst[i] = ((uint8_t*)src)[(sb == LSB) ? i : (size-i-1)];
    }
}

