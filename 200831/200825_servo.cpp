#include <stdlib.h>
#include <cmath>
#include <math.h>
#include <MS5611.h>
#include <Util.h>
#include <unistd.h>
#include <stdio.h>
#include "Nextion.h"
#include "RCOutput_Navio2.h"
#include <RCInput_Navio2.h>
#include "PWM.h"
#include <memory>
#include <string>
#include <sys/time.h>
#include "MPU9250.h"
#include "LSM9DS1.h"
#include <pthread.h>
#include <ADC_Navio2.h>
#include <MS4525DO.h>
#include <filtro.h>
#include <Ublox.h>
#include <wiringSerial.h>
#include "ads1115.h"

#define PWM_OUTPUT_1 0
#define PWM_OUTPUT_2 1
#define PWM_OUTPUT_3 2
#define PWM_OUTPUT_4 3

ads1115 ads1;
const float VPS02 = 0.256 / 32768.0;
const float VPS05 = 0.512 / 32768.0;
const float VPS1  = 1.024 / 32768.0;
const float VPS2  = 2.048 / 32768.0;
const float VPS4  = 4.096 / 32768.0;
const float VPS6  = 6.144 / 32768.0;
float VPS_;
float leitura1=0;
float leitura2=0;
float leitura3=0;
float leitura4=0;
int adc1 = 0;
int adc2 = 0;
int adc3 = 0;
int adc4 = 0;

//PWM 
RCOutput_Navio2 servo1;
int contador = 1000;
int sinal = 1;
int comando = 1000;
int main()
{
    ads1.begin(0x49);
    ads1.set_single_channel(3);
	//CONFIGURA 4 PORTAS DE PWM - freq 50Hz
    servo1.initialize(PWM_OUTPUT_1);
    servo1.set_frequency(PWM_OUTPUT_1, 50);
    servo1.enable(PWM_OUTPUT_1);

    servo1.initialize(PWM_OUTPUT_2);
    servo1.set_frequency(PWM_OUTPUT_2, 50);
    servo1.enable(PWM_OUTPUT_2);

    servo1.initialize(PWM_OUTPUT_3);
    servo1.set_frequency(PWM_OUTPUT_3, 50);
    servo1.enable(PWM_OUTPUT_3);

    servo1.initialize(PWM_OUTPUT_4);
    servo1.set_frequency(PWM_OUTPUT_4, 50);
    servo1.enable(PWM_OUTPUT_4);

    while(1)
    {
    	if(contador>2000)
    	{
    		contador = 2000;
    		sinal = -1;
    	}
    	else if (contador<1000)
    	{
    		contador = 1000;
    		sinal = 1;
    	}
    	else
    	{
    		contador = contador + sinal;
    	}
    	// contador = 2000;
    	// servo1.set_duty_cycle(PWM_OUTPUT_1, contador);
    	// usleep(2000000);

        // VPS_ = VPS6;
        // adc1 = ads1.ads_read();
        // leitura1 = adc1*VPS_;
        // printf("leitura: %8.6f\n",leitura1);

    	// contador = 1000;
    	// servo1.set_duty_cycle(PWM_OUTPUT_1, contador);
    	// usleep(2000000);
    	

        VPS_ = VPS6;
        adc1 = ads1.ads_read();
        leitura1 = adc1*VPS_;
        printf("leitura: %8.6f\n",leitura1);
    	servo1.set_duty_cycle(PWM_OUTPUT_1, contador);
    	servo1.set_duty_cycle(PWM_OUTPUT_2, 1000);
    	servo1.set_duty_cycle(PWM_OUTPUT_3, 1000);
    	servo1.set_duty_cycle(PWM_OUTPUT_4, 1000);
    	usleep(100000);
    }

	return 0;
}