


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

//TEMPO
struct timeval tv;
float dt,segundos=0,contador=0;
static unsigned long previoustime, currenttime;


//MPU9250
MPU9250 mpu1;
float axmpu,aympu,azmpu;
float axmpu_antes=0,aympu_antes=0,azmpu_antes=0;
float axmpu_f,aympu_f,azmpu_f;
float axmpu_f_antes=0,aympu_f_antes=0,azmpu_f_antes=0;
float gxmpu,gympu,gzmpu;
float mxmpu,mympu,mzmpu;

//LSM9DS1
LSM9DS1 lsm1;
float axlsm,aylsm,azlsm;
float axlsm_antes=0,aylsm_antes=0,azlsm_antes=0;
float axlsm_f,aylsm_f,azlsm_f;
float axlsm_f_antes=0,aylsm_f_antes=0,azlsm_f_antes=0;
float gxlsm,gylsm,gzlsm;
float mxlsm,mylsm,mzlsm;

//AHRS
AHRS euler;
float roll, pitch, yaw;

void* f_thread1(void* data)
{
    //Telemetria pela serial USB0
    int uart = serialOpen("/dev/ttyUSB0",115200);
    while(1)
    {
        int rolamento = ((roll)+500)*100;
        int arfagem   = ((pitch)+500)*100;
        int guinada   = ((yaw)+500)*100;
        int tempo     = segundos*100;
        int accelx    = (axmpu+500)*100;
        int accely    = (aympu+500)*100;
        int accelz    = (azmpu+500)*100;

        String telemetria;

        telemetria+=String("A");
        telemetria+=String(((tempo)/100000)%10);//25
        telemetria+=String(((tempo)/10000)%10);//25
        telemetria+=String(((tempo)/1000)%10);//25
        telemetria+=String(((tempo)/100)%10);//25
        telemetria+=String(((tempo)/10)%10);//26
        telemetria+=String(((tempo)/1)%10);//27
        telemetria+=",";
        telemetria+=String(((rolamento)/10000)%10);//25
        telemetria+=String(((rolamento)/1000)%10);//25
        telemetria+=String(((rolamento)/100)%10);//25
        telemetria+=String(((rolamento)/10)%10);//26
        telemetria+=String(((rolamento)/1)%10);//27
        telemetria+=",";
        telemetria+=String(((arfagem)/10000)%10);//28
        telemetria+=String(((arfagem)/1000)%10);//28
        telemetria+=String(((arfagem)/100)%10);//28
        telemetria+=String(((arfagem)/10)%10);//29
        telemetria+=String(((arfagem)/1)%10);//30
        telemetria+=",";
        telemetria+=String(((guinada)/10000)%10);//31
        telemetria+=String(((guinada)/1000)%10);//31
        telemetria+=String(((guinada)/100)%10);//31
        telemetria+=String(((guinada)/10)%10);//32
        telemetria+=String(((guinada)/1)%10);//33
        telemetria+=",";
        telemetria+=String(((rolamento)/10000)%10);//25
        telemetria+=String(((rolamento)/1000)%10);//25
        telemetria+=String(((rolamento)/100)%10);//25
        telemetria+=String(((rolamento)/10)%10);//26
        telemetria+=String(((rolamento)/1)%10);//27
        telemetria+=",";
        telemetria+=String(((arfagem)/10000)%10);//28
        telemetria+=String(((arfagem)/1000)%10);//28
        telemetria+=String(((arfagem)/100)%10);//28
        telemetria+=String(((arfagem)/10)%10);//29
        telemetria+=String(((arfagem)/1)%10);//30
        telemetria+=",";
        telemetria+=String(((guinada)/10000)%10);//31
        telemetria+=String(((guinada)/1000)%10);//31
        telemetria+=String(((guinada)/100)%10);//31
        telemetria+=String(((guinada)/10)%10);//32
        telemetria+=String(((guinada)/1)%10);//33
        telemetria+=",";
        telemetria+=String(((accelx)/10000)%10);//31
        telemetria+=String(((accelx)/1000)%10);//31
        telemetria+=String(((accelx)/100)%10);//31
        telemetria+=String(((accelx)/10)%10);//32
        telemetria+=String(((accelx)/1)%10);//33
        telemetria+=",";
        telemetria+=String(((accely)/10000)%10);//31
        telemetria+=String(((accely)/1000)%10);//31
        telemetria+=String(((accely)/100)%10);//31
        telemetria+=String(((accely)/10)%10);//32
        telemetria+=String(((accely)/1)%10);//33
        telemetria+=",";
        telemetria+=String(((accelz)/10000)%10);//31
        telemetria+=String(((accelz)/1000)%10);//31
        telemetria+=String(((accelz)/100)%10);//31
        telemetria+=String(((accelz)/10)%10);//32
        telemetria+=String(((accelz)/1)%10);//33
        telemetria+=String("Z");
        telemetria+="\n";

        const char* conv = telemetria.c_str();
        serialPrintf(uart, conv);
        serialFlush(uart);

        const char* conv2;
        const char* aux;
        int byte = serialGetchar(uart); //primeiro byte lido
        int arm[100] = {10};
        String msg;
        if(byte == 65) //identifica header A = 65 em ascii
        {
        	msg="";
        	
        	while(byte!=90) //enquanto diferente de Z = 90
        	{
        		byte = serialGetchar(uart);
                switch(byte)
                {
                    case 32:
                    {
                        msg+=" ";
                        break;
                    }
                    case 90:
                    {
                        msg+="";
                        break;
                    }
                    case 45:
                    {
                        msg+="-";
                        break;
                    }
                    default:
                    {
                        msg+=String(byte-48);
                        break;
                    }
                }

        	}
        	conv2 = msg.c_str();
        	//printf("msg completa : %s\n",conv2);
            printf("msg completa : %s\n",conv);
        	sscanf(conv2,"%d %d %d %d %d",&arm[0],&arm[1],&arm[2],&arm[3],&arm[4]);
        }
        printf("a = %6d b = %6d c = %6d d = %6d e = %6d\n",arm[0],arm[1],arm[2],arm[3],arm[4]);
        //usleep(33332); //30Hz
        usleep(10000); //100Hz



    }
}

int main()
{
    //CHECA NAVIO2
    if (check_apm()) 
    {
        return 1;
    }

    //MPU9250
    mpu1.initialize();

    //LSM9DS1
    lsm1.initialize();

    //AHRS
    euler.sensorinit();
    euler.setGyroOffset();

    //Threads
    pthread_t thread1;
    pthread_create(&thread1,NULL,f_thread1,NULL);

    //THREAD0
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

        //AHRS rolagem, arfagem e guinada : roll, pitch e yaw
        //de preferência o mais próximo possível da contagem de tempo
        euler.updateIMU(dt);
        euler.getEuler(&roll, &pitch, &yaw);

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
        usleep(33332); //30Hz

    }


    return 0;
}