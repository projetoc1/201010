#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>    // read/write usleep
#include <stdlib.h>    // exit function
#include <inttypes.h>  // uint8_t, etc
#include <linux/i2c-dev.h> // I2C bus definitions
#include <sys/ioctl.h>
#include "ads1115.h"
#include <pigpio.h>

ads1115 ads;

int main()
{
  ads.begin(0x48);
  const float VPS = 6.144 / 32768.0;
  float leitura1=0,leitura2=0,leitura3=0,leitura4=0;
  int adc1 = 0, adc2 = 0 , adc3 = 0, adc4 = 0;
  while(1)
  {
    ads.set_single_channel(0);
    adc1 = ads.ads_read();
    leitura1 = adc1*VPS;
    ads.set_single_channel(1);
    adc2 = ads.ads_read();
    leitura2 = adc2*VPS;
    float temperatura = leitura1/0.01;
    // ads.set_single_channel(2);
    // adc3 = ads.ads_read();
    // leitura3 = adc3*VPS;
    // ads.set_single_channel(3);
    // adc4 = ads.ads_read();
    // leitura4 = adc4*VPS;

    printf("adc1= %6d V1= %7.4f temp = %6.2f °C\n",adc1,leitura1,temperatura);
    //printf("adc1=%6d V1=%7.4f adc2 =%6d V2=%7.4f adc3=%6d V3=%7.4f adc4 =%6d V4=%7.4f\n",adc1,leitura1,adc2,leitura2,adc3,leitura3,adc4,leitura4);
    
    //usleep(33332);
  }
  

  return 0;
}