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
#include "Adafruit_ADS1015.h"
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/joystick.h>
#include "PID.h"

//TELA NEXTION 
NexVariable setp_lat = NexVariable(0,37,"setp_lat");
NexVariable setp_lon = NexVariable(0,38,"setp_lon");

NexVariable flag_ganho = NexVariable(0,59,"flag_ganho");

NexVariable lat_p = NexVariable(0,52,"lat_p");
NexVariable lat_i = NexVariable(0,53,"lat_i");
NexVariable lat_d = NexVariable(0,54,"lat_d");

NexVariable lon_p = NexVariable(0,55,"lon_p");
NexVariable lon_i = NexVariable(0,56,"lon_i");
NexVariable lon_d = NexVariable(0,57,"lon_d");

NexVariable error_lat = NexVariable(0,58,"error_lat");
NexVariable error_lon = NexVariable(0,59,"error_lon");

NexVariable flag = NexVariable(0,60,"flag");
NexVariable flag_piloto = NexVariable(0,63,"flag_piloto");

NexVariable lat_navio2 = NexVariable(0,67,"lat_navio2");
NexVariable lon_navio2 = NexVariable(0,68,"lon_navio2");

NexVariable adc1 = NexVariable(0,69,"adc1");
NexVariable adc2 = NexVariable(0,70,"adc2");

uint32_t setpoint_lat = 0;
uint32_t setpoint_lon = 0;
uint32_t        latp  = 0;
uint32_t        lati  = 0;
uint32_t        latd  = 0;
uint32_t        lonp  = 0;
uint32_t        loni  = 0;
uint32_t        lond  = 0;
float        errorlat = 3;
float        errorlon = 4;
uint32_t        flag0 = 0;
uint32_t   flagpiloto = 0;
uint32_t    flagganho = 0;
float       latnavio2 = 1;
float       lonnavio2 = 1;
float     leituraadc1 = 6;
float     leituraadc2 = 7;

float flag_piloto_nextion = 0;


void* f_tela(void* data)
{
     //NEXTION
    int cont = 1;
    while(1)
    {
        if(cont>10)
        {
            cont=1;
        }
        else
        {
            cont++;
        }
        //ENVIA PARA TELA NEXTION
        if (cont==1)
        {
            lat_navio2.setValue(latnavio2);
        }
        if (cont==2)
        {
            lon_navio2.setValue(lonnavio2);
        }
        if (cont==3)
        {
            adc1.setValue(leituraadc1);
        }
        if (cont==4)
        {
           adc2.setValue(leituraadc2);
        } 
        if (cont==5)
        {
            error_lat.setValue(errorlat);
        }
        if (cont==6)
        {
            error_lon.setValue(errorlon);
        }
        //LEITURA DA TELA
        if (cont==7)
        {
           setp_lat.getValue(&setpoint_lat);
        }
        if (cont==8)
        {
            setp_lon.getValue(&setpoint_lon);
        }
        if (cont==9)
        {
            flag_ganho.getValue(&flagganho);
            if (flagganho==0)
            {
                lat_p.getValue(&latp);
            }
            if (flagganho==1)
            {
                lat_i.getValue(&lati);
            }
            if (flagganho==2)
            {
                lat_d.getValue(&latd);
            }
            if (flagganho==3)
            {
                lon_p.getValue(&lonp);
            }
            if (flagganho==4)
            {
                lon_i.getValue(&loni);
            }
            if (flagganho==5)
            {
                lon_d.getValue(&lond);
            }
        }
        if (cont==10)
        {
            flag_piloto.getValue(&flagpiloto);
        }
    }
}

int main()
{
    printf("Nextion_in\n");
    nexInit();
    printf("Nextion_out\n");

    pthread_t thread3;
    pthread_create(&thread3,NULL,f_tela,NULL);
    while(1)
    {
        //flag_piloto_nextion = (float)flagpiloto;
        //printf("flag piloto = %.0f\n",flag_piloto_nextion);
        usleep(20000);
    }
    return 0;
}