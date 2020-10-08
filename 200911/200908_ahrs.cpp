// Header
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "MPU9250.h"
#include <sys/time.h>
#include <unistd.h>
#include "filtro.h"
#include "LSM9DS1.h"


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

//AHRS
AHRS euler,euler1; // euler LSM e euler1 MPU
float roll, pitch, yaw; //LSM
float roll1, pitch1, yaw1; //MPU

int main()
{
    double DT = 0.01;

    mpu1.initialize();
    euler1.sensorinit(0);
    euler1.setGyroOffset();


     while(1)
     {
        //TEMPO
        gettimeofday(&tv,NULL);
        previoustime = currenttime;
        currenttime = 1000000 * tv.tv_sec + tv.tv_usec;
        dt = (currenttime - previoustime) / 1000000.0;
        //IGNORA LEITURA INICIAL ALTA.
        if (dt>100)
        {
            dt = 0.0;
        }
        segundos = segundos + dt;

        mpu1.update();
        mpu1.read_accelerometer(&axmpu, &aympu, &azmpu);
        mpu1.read_gyroscope(&gxmpu, &gympu, &gzmpu);
        mpu1.read_magnetometer(&mxmpu, &mympu, &mzmpu);

        euler.updateIMU(dt);
        euler.getEuler(&roll, &pitch, &yaw);
        euler1.updateIMU(dt);
        euler1.getEuler(&roll1, &pitch1, &yaw1);
        printf("MPU: roll =%6.2f pitch =%6.2f yaw =%6.2f\n",roll1,pitch1,yaw1);
        

     }

    
    return 0;
}

