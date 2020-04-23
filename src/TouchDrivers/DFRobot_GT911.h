#ifndef __DFROBOT_GT911_H
#define __DFROBOT_GT911_H

//GT911配置数组表
//x坐标输出最大值0x0140=320
//y坐标输出最大值0x01E0=480
//0x8047~0x80FE  168个寄存器
//0x80FF配置信息校验，0x8047~0x80FE之字节和的补码
//0x8100  配置更新标记
//数组组织方式
//1.寄存器起始地址0x8047
//2.数据个数 0x00BA
//0寄存器位数 1每个寄存器存的数据的位数 16bit-reg 8bit-data
//前5个数据为控制数据：寄存器位数，寄存器值位数、起始寄存器、配置数据个数

// GT911 configuration array table
// x coordinate output maximum value 0x0140 = 320
// y coordinate output maximum value 0x01E0 = 480
// 0x8047 ~ 0x80FE 168 registers
// 0x80FF configuration information check, the complement of the byte sum of 0x8047 ~ 0x80FE
// 0x8100 configuration update flag
// Array organization
// 1. Register start address 0x8047
// 2. Number of data 0x00BA
// 0 register digits 1 digits of data stored in each register 16bit-reg 8bit-data
// The first 5 data are control data: register digits, register value digits, start register, configuration data number
static const uint8_t PROGMEM touchGT911ConfigTable[] = {
2,
1,
0x80, 0x47, 
0x00, 0xBA,//(寄存器起始地址0x8047,数据个数0x00BA)  (Register start address 0x8047, data number 0x00BA)
0x41, 0x40, 0x01, 0xE0, 0x01, 0x05, 0x05, 0x00, 0x01, 0x08,  
0x28, 0x05, 0x50, 0x32, 0x03, 0x05, 0x00, 0x00, 0x00, 0x00,  
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x87, 0x28, 0x0A,  
0x17, 0x15, 0x31, 0x0D, 0x00, 0x00, 0x00, 0x9A, 0x04, 0x25,  
0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x64, 0x32, 0x00, 0x00, 
0x00, 0x0F, 0x94, 0x94, 0xC5, 0x02, 0x07, 0x00, 0x00, 0x04, 
0x8D, 0x13, 0x00, 0x5C, 0x1E, 0x00, 0x3B, 0x30, 0x00, 0x28, 
0x4C, 0x00, 0x1C, 0x78, 0x00, 0x1C, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x14, 0x12, 0x10, 0x0E, 0x0C, 0x0A, 0x08, 0x06, 
0x04, 0x02, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x02, 0x04, 0x06, 0x08, 0x0A, 0x0C, 0x26, 
0x24, 0x22, 0x21, 0x20, 0x1F, 0x1E, 0x1D, 0xFF, 0xFF, 0xFF, 
0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x12, 0x00
};


#endif