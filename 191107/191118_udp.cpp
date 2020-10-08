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

#define size 512
#define port 7878

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

int comando1 = 500000000;
int comando2 = 500000000;
//int uart = serialOpen("/dev/ttyUSB0",115200);
int uart = serialOpen("/dev/ttyUSB0",3000000);


void* f_thread1(void* data)
{
    int abecd;
}
void diep(char *s)
{
	perror(s);
	exit(1);
}
void* f_thread2(void* data)
{
	struct sockaddr_in si_other, si_me;
	int s;
	socklen_t slen=sizeof(si_other);
	int recv_len;
	char buf[size];
	s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	memset((char *) &si_me, 0, sizeof(si_me));
	si_me.sin_family = AF_INET;
	si_me.sin_port = htons(port);
	si_me.sin_addr.s_addr = htonl(INADDR_ANY);
	bind(s, (struct sockaddr*)&si_me, sizeof(si_me));
	int alfa=500000,beta=50;
	printf("%d       %d\n",alfa,beta);
	while(1)
	{
		printf("%d       %d\n",alfa,beta);
		int recv = recvfrom(s, buf, size, 0, (struct sockaddr*)&si_other, &slen);
		printf("%d       %f\n",alfa,segundos);
		sscanf(buf,"A%d,%dZ",&alfa,&beta);
		String dados;
		dados+="A";
		dados+=String(alfa);
		dados+=",";
		dados+=String(beta);
		dados+="Z";
		dados+="\n";
		const char* udp = dados.c_str();
		printf("%s\n",udp);
		serialPrintf(uart, udp);
		//send(s,udp,strlen(udp),0);
		//sendto ( s, udp, strlen(udp), 0, (struct sockaddr*)&si_other, slen);
		//usleep(10000);
	}


}
void* f_thread3(void* data)
{
    while(1)
    {
       // usleep(10000); //100Hz

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
    pthread_t thread1,thread2,thread3;
    pthread_create(&thread1,NULL,f_thread1,NULL);
    pthread_create(&thread2,NULL,f_thread2,NULL);
    pthread_create(&thread3,NULL,f_thread3,NULL);

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
        //printf("roll = %5.2f pitch = %5.2f yaw = %5.2f\n",pitch,roll,yaw);

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
        //usleep(33332); //30Hz

    }


    return 0;
}
// void* f_thread1(void* data)
// {
//     //Telemetria pela serial USB0
//     //FILE *texto;
//     //texto = fopen("data.txt","a");
//     printf("começou\n");
//     while(1)
//     {
//         //RECEBENDO VALORES
//         const char* conv2;
//         const char* aux;
//         int byte = serialGetchar(uart); //primeiro byte lido
//         int arm[10]          = {500000000,500000000,500000000,500000000,500000000,500000000,500000000,500000000,500000000,500000000};
//         int arm_anterior[10] = {10};
//      int arm_novo[10]     = {10};
//         String msg;
//         if(byte == 65) //identifica header A = 65 em ascii
//         {
//             msg="";
            
//             while(byte!=90) //enquanto diferente de Z = 90
//             {
//                 byte = serialGetchar(uart);
//                 switch(byte)
//                 {
//                     case 44:
//                     {
//                         msg+=",";
//                         break;
//                     }
//                     case 90:
//                     {
//                         msg+="";
//                         break;
//                     }
//                     case 45:
//                     {
//                         msg+="-";
//                         break;
//                     }
//                     default:
//                     {
//                         msg+=String(byte-48);
//                         break;
//                     }
//                 }

//             }
//             conv2 = msg.c_str();
//             //printf("msg completa : %s\n",conv2);
//             //printf("msg completa : %s\n",conv);
//             //sscanf(conv2,"%d %d %d %d %d",&arm[0],&arm[1],&arm[2],&arm[3],&arm[4]);
//             sscanf(conv2,"%d,%d",&arm_novo[4],&arm_novo[5]);
//             if(arm_novo[4]==0)
//             {
//              arm[4]=arm_anterior[4];
//              arm[5]=arm_anterior[5];

//             }
//             else
//             {
//              arm[4]=arm_novo[4];
//              arm[5]=arm_novo[5];
//              arm_anterior[4]=arm_novo[4];
//              arm_anterior[5]=arm_novo[5];
//             }
//         }
        
//         float constante1 = -0.839;
//         float constante2 = 10;
//         //float constante = -0.418;
//         float recebido1  = ((float)arm[4]/1000000) - 500;
//         float recebido12; 
//         float recebido2  = ((float)arm[5]/1000000) - 500;
//         float recebido22; 
//         //recebido = 5+2*sin(3.1415*segundos);
//         recebido12 = (recebido1*constante1+500)*1000000;
//         recebido22 = (recebido2*constante2+500)*1000000;
//         //ENVIANDO VALORES
//         int rolamento = ((roll)+500)*100;
//         int arfagem   = ((pitch)+500)*100;
//         int guinada   = ((yaw)+500)*100;
//         //int comando   = (recebido+500)*1000;
//         comando1   = (int)recebido12;
//         comando2   = (int)recebido22;
//         //int comando   = arm[4];
//         int accelx    = (axmpu+500)*100;
//         int accely    = (aympu+500)*100;
//         int accelz    = (azmpu+500)*100;
//         //printf("armazenado = %9d recebido = %15.8f recebido2 = %15f e comando = %9d tempo = %6.4fs\n",arm[4],recebido1,recebido12,comando1,segundos);
//         printf("armazenado = %9d recebido = %15.8f recebido2 = %15.0f e comando = %9d tempo = %6.4fs\n",arm[5],recebido2,recebido22,comando2,segundos);
        
//         //printf("Recebido1 = %15.8f e Recebido 2 = %15.8f",recebido1,recebido2);
//         //printf(" comando1 = %15d e comando2 = %15d\n",comando1,arm[5]);
//         //fprintf(texto,"%16.6f\n",recebido*constante);
//         if(segundos>30)
//         {
//          //fclose(texto);
//         }
//         String telemetria;

//         telemetria+=String("A");
//         telemetria+=String(((comando1)/100000000)%10);//25
//         telemetria+=String(((comando1)/10000000)%10);//25
//         telemetria+=String(((comando1)/1000000)%10);//25
//         telemetria+=String(((comando1)/100000)%10);//25
//         telemetria+=String(((comando1)/10000)%10);//25
//         telemetria+=String(((comando1)/1000)%10);//25
//         telemetria+=String(((comando1)/100)%10);//25
//         telemetria+=String(((comando1)/10)%10);//26
//         telemetria+=String(((comando1)/1)%10);//27
//         telemetria+=",";
//         telemetria+=String(((comando2)/1000000000)%10);//25
//         telemetria+=String(((comando2)/100000000)%10);//25
//         telemetria+=String(((comando2)/10000000)%10);//25
//         telemetria+=String(((comando2)/1000000)%10);//25
//         telemetria+=String(((comando2)/100000)%10);//25
//         telemetria+=String(((comando2)/10000)%10);//25
//         telemetria+=String(((comando2)/1000)%10);//25
//         telemetria+=String(((comando2)/100)%10);//25
//         telemetria+=String(((comando2)/10)%10);//26
//         telemetria+=String(((comando2)/1)%10);//27
//         telemetria+=String("Z");
//         telemetria+="\n";

//         const char* conv = telemetria.c_str();
//         serialPrintf(uart, conv);
//         serialFlush(uart);
//         //usleep(100000); //10Hz
//         //usleep(33332); //30Hz
//         //usleep(10000); //100Hz
//         //usleep(1000000); //100Hz
//     }
// }
