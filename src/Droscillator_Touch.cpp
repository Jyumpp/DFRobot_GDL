#include "Droscillator_Touch.h"
#include <DFRobot_Type.h>
#include "Wire.h"
#define ADDR 0x5D

GDL_IF_PB_DEV(gdl_Dev_GTXXX_TOUCH_HW_IIC, NULL, DEV_TYPE_TOUCH, IF_COM_HW_IIC)
GDL_IF_PB_DEV(gdl_Dev_XPT2046_TOUCH_HW_SPI, NULL, DEV_TYPE_TOUCH, IF_COM_HW_SPI)

Drosc_Touch::Drosc_Touch(sGdlIFDev_t *dev, uint8_t addr, uint8_t rst, uint8_t irq)
  :DFRobot_IF(dev, addr, rst, irq){}
Drosc_Touch::Drosc_Touch(sGdlIFDev_t *dev, uint8_t cs, uint8_t rst, uint8_t irq, uint8_t dc)
  :DFRobot_IF(dev, dc, cs, rst, irq){}

Drosc_Touch::~Drosc_Touch(){}
void Drosc_Touch::initTouch(){
  _points.numTouch = 0;
  _pNum = 0;
  memset(&_size, 0, sizeof(_size));
  memset(&_point, 0, sizeof(sPoints_t));
  initInterface();
}

void Drosc_Touch::touchConfig(uint8_t *addr){
  uint8_t regByte = pgm_read_byte(addr++);
  uint8_t regValByte = pgm_read_byte(addr++);
  uint8_t regBuf[regByte];
  uint32_t reg = 0;
  for(uint8_t i = 0; i < regByte; i++){
      regBuf[i] = pgm_read_byte(addr++);
  }
  for(uint8_t j = 0; j < regByte; j++){
      reg |= (uint32_t)(regBuf[regByte -1 -j] << 8*j);
  }
  uint16_t length =  (pgm_read_byte(addr++)<<8) | (pgm_read_byte(addr++));
  //int8_t flag = regByte - 1;
  uint8_t buf[regByte+1+regValByte];
  buf[0] = regByte;
  for(uint8_t i = 0; i < length; i++){
      memcpy(buf+1, regBuf, regByte);
      for(uint8_t j = 0; j < regValByte; j++){
          buf[regByte+1+j] = pgm_read_byte(addr++);
      }
      //Serial.println(buf[regByte+1]);
      _if.dev->talk(&_if, IF_COM_WRITE_RAM_INC, buf, regValByte);
      reg += 1;
      for(uint8_t i = 0; i < regByte; i++){
          regBuf[regByte - 1- i] = (uint8_t)(reg >> 8*i);
      }
  }
}

void Drosc_Touch::setRotation(uint8_t rotate)
{
   direction = rotate;
}


String Drosc_Touch::pointRemap(uint16_t &x,uint16_t &y,uint16_t _width,uint16_t _height){

   uint16_t tempX = x;
   uint16_t tempY = y;
   switch(direction){
     case 0:{

     break;
     }
     case 1:{
       x = tempY;
       y = _width -tempX;
     break;
    }
     case 2:{
       x = _width -tempX;
       y = _height -tempY;
     break;
     }
     case 3:{
       x = _height -tempY;
       y = tempX;
     break;
    }
     
   }
  
   return "ok";

}

Drosc_Touch_GT911::Drosc_Touch_GT911(uint8_t addr, uint8_t rst, uint8_t irq)
  :Drosc_Touch(&gdl_Dev_GTXXX_TOUCH_HW_IIC, addr, rst, irq){
  id = "";
  memset(_p, 0, sizeof(_p));
}
Drosc_Touch_GT911::~Drosc_Touch_GT911(){
  
}
void Drosc_Touch_GT911::begin(uint32_t freq){
  freq = freq;
  initTouch();
  char temp[4]={0};//Get chip id
  uint16_t sizeReg = 0;
  readReg(0x8140,temp,4);
  id += temp;
  //Serial.println(id);
  IC = GT911;
  _if.dev->addr = (uint8_t *)touchGT911ConfigTable;
  sizeReg = 0x8048;
  uint8_t *addr = _if.dev->addr;
  touchConfig(addr);
  readReg(sizeReg,temp,4);

  _size.xw = ((uint8_t)temp[1] << 8) | (uint8_t)temp[0];
  _size.yh = ((uint8_t)temp[3] << 8) | (uint8_t)temp[2];
  
  //Serial.println("_size.xw = ");Serial.println(_size.xw);
  //Serial.println("_size.yh = ");Serial.println(_size.yh);
}

Drosc_Touch::sPointList_t Drosc_Touch_GT911::scan()
{
  uint8_t flag = 0;
  uint8_t val = 0x00;
  memset(_p, 0, sizeof(_p));
  readReg(0x814E, &flag, 1);
  if((flag & 0x80) ||((flag&0x0F)<6)){
      writeBuf(0x814E, &val, 1);
  }
  sPointList_t pList;
  if((flag & 0x80) &&((flag&0x0F)<6)){
      readReg(0x814F, &_p, sizeof(_p));
      _pNum = flag&0x0F;
      
      pList.numTouch = _pNum;
      for(uint8_t i = 0; i < _pNum; i++){
          _point.id = _p[i].id;
          if(id == "5688")
              _point.id &= 0x0F;
          _point.x =  _p[i].xl + (_p[i].xh << 8);
          _point.y =  _p[i].yl + (_p[i].yh << 8);
          _point.wSize = _p[i].wSize;
          _point.hSize = _p[i].hSize;
          if((_point.x <= _size.xw) && (_point.y <= _size.yh)){
              pointRemap(_point.x,_point.y,319,479); 
              pList.points[i] = _point;
          }
      }
  }
  delay(10);
  _points = pList;
  return pList;
}
