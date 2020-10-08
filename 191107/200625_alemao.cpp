float p0=102000;
//BIBLIOTECAS
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
#include "imuheader.h"

//TEMPO
struct timeval tv;
float dt,segundos=0,contador=0;
float segundos_ant = 0;
static unsigned long previoustime, currenttime;
float dt_ant = 0;

//MPU9250
MPU9250 mpu1;
float axmpu,aympu,azmpu;
float gxmpu,gympu,gzmpu;
float mxmpu,mympu,mzmpu;

//LSM9DS1
LSM9DS1 lsm1;
float axlsm,aylsm,azlsm;
float gxlsm,gylsm,gzlsm;
float mxlsm,mylsm,mzlsm;

//atitude alemao
// IMU Data
//imdata_t IMDATA;
// Attitude Data
imuheader imu;

imdata_t IMDATA;

int main()
{
    //CHECA NAVIO2
    if (check_apm()) 
    {
        return 1;
    }
    //INICIALIZAÇÕES

    //MPU9250
    mpu1.initialize();

    //LSM9DS1
    lsm1.initialize();

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

        //MPU9250
        //LEITURA DE DADOS DE ACELERAÇÃO, GIRO E MAGNÉTICOS
        mpu1.update();
        mpu1.read_accelerometer(&axmpu, &aympu, &azmpu);
        mpu1.read_gyroscope(&gxmpu, &gympu, &gzmpu);
        mpu1.read_magnetometer(&mxmpu, &mympu, &mzmpu);

        //LSM9DS1
        //LEITURA DE DADOS DE ACELERAÇÃO, GIRO E MAGNÉTICOS
        lsm1.update();
        lsm1.read_accelerometer(&axlsm, &aylsm, &azlsm);
        lsm1.read_gyroscope(&gxlsm, &gylsm, &gzlsm);
        lsm1.read_magnetometer(&mxlsm, &mylsm, &mzlsm);


    }


    return 0;
}
