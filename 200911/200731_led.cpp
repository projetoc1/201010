float p0=102000;
//BIBLIOTECAS
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

//TEMPO
struct timeval tv;
float dt,segundos=0,contador=0;
float segundos_ant = 0;
static unsigned long previoustime, currenttime;

int main()
{
    //CHECA NAVIO2
    if (check_apm()) 
    {
        return 1;
    }
    //CONFIGURA PINOS 17 E 18 DO GPIO PARA SAIDA
    system("gpio -g mode 17 out");
    system("gpio -g mode 18 out");

    //DESLIGA INICIALMENTE OS LEDS
    system("gpio -g write 18 0");
    system("gpio -g write 17 0");
    
    int alternador_led = 0;
    while(1)
    {
        //CONTANDO TEMPO
        //Contagem de tempo. Tempo total : variável segundos
        //                   Delta       : varíavel dt     
        gettimeofday(&tv,NULL);
        previoustime = currenttime;
        currenttime = 1000000 * tv.tv_sec + tv.tv_usec;
        dt = (currenttime - previoustime) / 1000000.0;
        //IGNORA LEITURA INICIAL ALTA.
        if (dt>100)
        {
            dt = 0;
        }
        segundos = segundos + dt;

        if(alternador_led == 0)
        {
        	//0 DESLIGA E 1 LIGA OS LEDS
        	system("gpio -g write 18 1");
        	system("gpio -g write 17 0");
            alternador_led = 1;
                   
        }
        else
        {
        	system("gpio -g write 18 0");
        	system("gpio -g write 17 1");
            alternador_led = 0;
                      
        }
              
        printf(" dt = %16.6f",dt);
        printf("\n");
        usleep(100000); //10Hz
    }


    return 0;
}
