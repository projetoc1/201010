#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>    
#include <stdlib.h>    
#include <inttypes.h>  
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include "ads1115.h"

ads1115::ads1115()
{
 
}

void ads1115::begin(uint8_t address)
{
  fd_ads = open("/dev/i2c-1", O_RDWR);
  ioctl(fd_ads, I2C_SLAVE, address);
  //setup default
  writeBuf[0] = 1;
  writeBuf[1] = 0b11000000;
  writeBuf[2] = 0b10000101;
  write(fd_ads, writeBuf, 3);
  sleep(1);
}
void ads1115::set_single_channel(int ch)
{
  usleep(25);
  if(ch==0)
  {
    writeBuf[1] = 0b11000000;
  }
  if(ch==1)
  {
    writeBuf[1] = 0b11010000;
  }
  if(ch==2)
  {
    writeBuf[1] = 0b11100000;
  }
  if(ch==3)
  {
    writeBuf[1] = 0b11110000;
  }
  if(ch==4)//Capacidade de ler diferencial e escala 0.256V
  {
    writeBuf[1] = 0b10001110;
  }
  if(ch==5)//Capacidade de ler diferencial e escala 0.512V
  {
    writeBuf[1] = 0b10001000;
  }
  if(ch==6)//Capacidade de ler diferencial e escala 1.024V
  {
    writeBuf[1] = 0b10000110;
  }
  if(ch==7)//Capacidade de ler diferencial e escala 2.048V
  {
    writeBuf[1] = 0b10000100;
  }
  if(ch==8)//Capacidade de ler diferencial e escala 4.096V
  {
    writeBuf[1] = 0b10000010;
  }
  if(ch==9)//Capacidade de ler diferencial e escala 6.144V
  {
    writeBuf[1] = 0b10000000;
  }

  writeBuf[0] = 1;
  writeBuf[2] = 0b10000101;
  write(fd_ads, writeBuf, 3);
  usleep(25000);
}

int16_t ads1115::ads_read()
{
  readBuf[0] = 0;
  write(fd_ads, readBuf, 1);

  read(fd_ads, readBuf, 2);
  val = readBuf[0] << 8 | readBuf[1];
  return val;
}
