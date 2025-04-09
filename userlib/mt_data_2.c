#include "mt_data_2.h"

int XBusMTData2HeaderCheck(const uint8_t* data) {
    static const uint8_t header_check[] = {0xFA, 0xFF, 0x36};
    for(uint8_t i = 0; i < 3; i++) {
        if(data[i] != header_check[i]) {
            return 1;
        }
    }
    return 0;
}

int XBusCheckGetLen(const uint8_t* data, int data_len) {
    if((data_len-5) == data[3]) return data[3];
    return -1;
}

int XBusCheckChecksum(const uint8_t* data, int data_len) {
    return 0;
}

//数组解析出float
static float Hex4_To_Float1(const unsigned char *array)
{
    unsigned char array_copy[4];
    float f;

    for(unsigned int i=0; i<4; i++)
    {
        array_copy[3-i]=array[i];
    }
    f = *((float *)((array_copy)));
    return f;
}

int OrientationEulerAnglesDataParse(const uint8_t* data, struct MTData2* mtdata2) {
    if(!(data[0] == 0x20 && data[1] == 0x30 && data[2] == 0x0C)) {
        return 1;
    }
    mtdata2->euler_angle.roll_x = Hex4_To_Float1(data + 3);
    mtdata2->euler_angle.pitch_y = Hex4_To_Float1(data + 7);
    mtdata2->euler_angle.yaw_z = Hex4_To_Float1(data + 11);
    return 0;
}
