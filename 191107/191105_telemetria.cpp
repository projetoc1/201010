/*Este código tem como objetivo testar todas as funções disponíveis da Navio2
com todas as bibliotecas criadas para tal.
LISTA DE ITENS INCORPORADOS:

    MPU9250
    LSM9DS1
    MS4525DO
    MS5611
    AHRS

    ADC
    PWM
    TEMPO

*/
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
//#include "Adafruit_ADS1015.h"
//ADS1115
//Adafruit_ADS1115 ads;
//uint16_t adc0;
//TELA NEXTION
//caixas de texto
NexText t9  = NexText( 0 , 20 , "t9"  ); //TEMPO
NexText t10 = NexText( 0 , 21 , "t10" ); //ROLL
NexText t11 = NexText( 0 , 22 , "t11" ); //PITCH
NexText t12 = NexText( 0 , 23 , "t12" ); //YAW
NexText t13 = NexText( 0 , 24 , "t13" ); //Ax
NexText t14 = NexText( 0 , 25 , "t14" ); //Ay
NexText t15 = NexText( 0 , 26 , "t15" ); //Az
NexText t16 = NexText( 0 , 27 , "t16" ); //Alt baro
NexText t17 = NexText( 0 , 28 , "t17" ); //Alt gps
//variaveis em segundo plano
NexVariable tempo_tela = NexVariable(0,10,"tempo");
NexVariable roll_tela  = NexVariable(0,3,"roll");
NexVariable pitch_tela = NexVariable(0,4,"pitch");
NexVariable yaw_tela   = NexVariable(0,5,"yaw");
NexVariable ax_tela    = NexVariable(0,7,"ax");
NexVariable ay_tela    = NexVariable(0,8,"ay");
NexVariable az_tela    = NexVariable(0,9,"az");
NexVariable alt_baro_tela = NexVariable(0,29,"alt_baro");
NexVariable alt_gps_tela  = NexVariable(0,6,"alt_gps");
NexVariable vel_gps_tela  = NexVariable(0,2,"vel_gps");

//DEFINIÇÕES PARA PORTAS DE PWM
#define SERVO_MIN 1000 /*uS*/
#define SERVO_MAX 2000 /*uS*/
#define PWM_OUTPUT_1 0
#define PWM_OUTPUT_2 1

//TEMPO
struct timeval tv;
float dt,segundos=0,contador=0;
static unsigned long previoustime, currenttime;

//LEITURA ADC
ADC_Navio2 ADC;

float ch0 = 0; //TENSÃO DA PLACA
float ch1 = 0; //TENSÃO DA LINHA DE SERVOS
float ch2 = 0; //TENSÃO   DO POWER MODULE
float ch3 = 0; //CORRENTE DO POWER MODULE
float ch4 = 0; //TENSÃO DA PORTA ADC2
float ch5 = 0; //TENSÃO DA PORTA ADC3 

//PPM
RCInput_Navio2 ppm;

//PWM 
RCOutput_Navio2 servo;
float ppm_ch1;
float ppm_ch2;
float ppm_ch3;
float ppm_ch4;
float ppm_ch5;
float ppm_ch6;
float ppm_ch7;
float ppm_ch8;

//MS4525DO
//MS4525DO pitot;
float pressao_pitot=0;
float temperatura_pitot=0;


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

//MS5611
MS5611 barometer;
float pressao_baro,temperatura_baro;

//AHRS
AHRS euler;
float roll, pitch, yaw;

//GPS
using namespace std;
Ublox gps;
float time_of_week,Longitude,Latitude,height_e,height_msl;
float Horizontal_acc,Vertial_acc;

float time_of_week2;
float vel_norte;
float vel_leste;
float vel_descida;
float vel_3d;
float vel_solo;
float heading_gps;
int v_aero = 20;

//GPS SUBSECTION ECEF
float f = 0.0033528128981; //achatamento do elipsoide
float e = 0.0818191827452; //excentricidade
float a = 6378137; // raio da terra no equador
float N = 0;
//coordenadas retangulares no padrão ECEF seguindo o modelo wgs84
float ecef_x = 0;
float ecef_y = 0;
float ecef_z = 0;

//GPS SUBSECTION NED
//coordenadas retangulares no padrão NED usará matrizes de rotação.
float ned_x = 0;
float ned_y = 0;
float ned_z = 0;
//PONTO DE REFERENCIA
float lat0  = 0;
float lon0  = 0;
float height_e0 = 0;
float ecef_x0 = 0;
float ecef_y0 = 0;
float ecef_z0 = 0;
float N0 = 0;
//delta ECEF ATUAL COM A REFERENCIA
float delta_x = 0;
float delta_y = 0;
float delta_z = 0;

//FLAG PARA TEMPO DE INCIALIZAÇÃO (1)
int  flag_ned0 = 0;
float sen_lat0 = 0;
float cos_lat0 = 0;
float sen_lon0 = 0;
float cos_lon0 = 0;

void calc_ned(void)
{
	float pi=atan(1)*4;
	if(segundos>50 && flag_ned0==0 /*&& Horizontal_acc<=2 && Vertial_acc<=7*/) //(1) nao serao usados no ensaio de voo
	{
		lat0 = Latitude;
		lon0 = Longitude;
		height_e0 = height_e;
		sen_lat0 = sin(lat0*pi/180);
		cos_lat0 = cos(lat0*pi/180);
		sen_lon0 = sin(lon0*pi/180);
		cos_lon0 = cos(lon0*pi/180);
		N0 = a/(sqrt(1-(e*e*sen_lat0*sen_lat0)));
		ecef_x0 = (N0+height_e0)*cos_lat0*cos_lon0;
		ecef_y0 = (N0+height_e0)*cos_lat0*sen_lon0;
		ecef_z0 = (N0*(1-e*e)+height_e0)*sen_lat0;
		flag_ned0 = 1;
	}
	if(flag_ned0==1)
	{
		delta_x = ecef_x - ecef_x0;
		delta_y = ecef_y - ecef_y0;
		delta_z = ecef_z - ecef_z0;
		ned_x = -sen_lat0*cos_lon0*delta_x -sen_lat0*sen_lon0*delta_y + cos_lat0*delta_z;
		ned_y = -sen_lon0*delta_x + cos_lon0*delta_y;
		ned_z = -cos_lon0*cos_lat0*delta_x -sen_lon0*cos_lat0*delta_y -sen_lat0*delta_z;
	}
}

//CALCULA AS COORDENADAS RETANGULARES NO ECEF
void calc_ecef(void)
{
	float pi=atan(1)*4;
	N = a/(sqrt(1-(e*e*sin(Latitude*pi/180)*sin(Latitude*pi/180))));
	ecef_x = (N+height_e)*cos(Latitude*pi/180)*cos(Longitude*pi/180);
	ecef_y = (N+height_e)*cos(Latitude*pi/180)*sin(Longitude*pi/180);
	ecef_z = (N*(1-e*e)+height_e)*sin(Latitude*pi/180);

}


void set_tela(uint32_t number,const char *nome)
{
    char buf[10] = {0};
    String cmd;
    
    sprintf(buf,"%u",number);
    cmd += nome;
    cmd += ".val=";
    cmd += buf;

    sendCommand(cmd.c_str());
}

float f_passa_baixa(float x_antes,float delta_t, float leitura)
{
    float freq_corte = 5;//hz
    float w = freq_corte*2*3.1415;
    return (x_antes+(delta_t*w*(leitura-x_antes)));
}
//Telemetria pela serial USB0
//int uart = serialOpen("/dev/ttyUSB0",57600);
//const char* conv;
void* f_thread1(void* data)
{
    //Telemetria pela serial USB0
    int uart = serialOpen("/dev/ttyUSB0",115200);
    while(1)
    {
        // BAROMETRO
        barometer.refreshPressure();
        usleep(10000); // Waiting for pressure data ready
        barometer.readPressure();

        barometer.refreshTemperature();
        usleep(10000); // Waiting for temperature data ready
        barometer.readTemperature();

        barometer.calculatePressureAndTemperature();

        pressao_baro     = -8430.125*log((barometer.getPressure()*100)/p0); //milibar
        temperatura_baro = barometer.getTemperature(); //ºC
        //TELEMETRIA PELA SERIAL USB0
        int rolamento = ((roll)+500)*100;
        int arfagem   = (pitch)+500;
        int guinada   = (yaw)+500;
        int temporal  = segundos*1000;
        int lat       = -1*Latitude*10000000;
        int lon       = -1*Longitude*10000000;
        int g_speed   = 3.6*vel_solo;
        v_aero    = 3.6*sqrt((abs(pressao_pitot*6894.76)/(0.5*1.22501)));
        //v_aero = 20;
        int alt       = height_msl;
        int rpm       = 0;
        int v_bat     = ch2*10/1000;
        int a_bat     = ch3*10/1000;
        if(alt<0)
        {
            alt=0;
        }
        String telemetria;
        telemetria+=String("A");
        telemetria+=String(((v_aero)/100)%10);//0
        telemetria+=String(((v_aero)/10)%10);//1
        telemetria+=String(((v_aero)/1)%10);//2
        telemetria+=",";
        telemetria+=String(((lat)/100000000)%10);//3
        telemetria+=String(((lat)/10000000)%10);//4
        telemetria+=String(((lat)/1000000)%10);//5
        telemetria+=String(((lat)/100000)%10);//6
        telemetria+=String(((lat)/10000)%10);//7
        telemetria+=String(((lat)/1000)%10);//8
        telemetria+=String(((lat)/100)%10);//9
        telemetria+=String(((lat)/10)%10);//10
        telemetria+=String(((lat)/1)%10);//11
        telemetria+=",";
        telemetria+=String(((lon)/100000000)%10);//12
        telemetria+=String(((lon)/10000000)%10);//13
        telemetria+=String(((lon)/1000000)%10);//14
        telemetria+=String(((lon)/100000)%10);//15
        telemetria+=String(((lon)/10000)%10);//16
        telemetria+=String(((lon)/1000)%10);//17
        telemetria+=String(((lon)/100)%10);//18
        telemetria+=String(((lon)/10)%10);//19
        telemetria+=String(((lon)/1)%10);//20
        telemetria+=",";
        telemetria+=String(((alt)/1000)%10);//21
        telemetria+=String(((alt)/100)%10);//22
        telemetria+=String(((alt)/10)%10);//23
        telemetria+=String(((alt)/1)%10);//24
        telemetria+=",";
        telemetria+=String(((rolamento)/10000)%10);//25
        telemetria+=String(((rolamento)/1000)%10);//25
        telemetria+=String(((rolamento)/100)%10);//25
        telemetria+=String(((rolamento)/10)%10);//26
        telemetria+=String(((rolamento)/1)%10);//27
        telemetria+=",";
        telemetria+=String(((arfagem)/100)%10);//28
        telemetria+=String(((arfagem)/10)%10);//29
        telemetria+=String(((arfagem)/1)%10);//30
        telemetria+=",";
        telemetria+=String(((guinada)/100)%10);//31
        telemetria+=String(((guinada)/10)%10);//32
        telemetria+=String(((guinada)/1)%10);//33
        telemetria+=",";
        telemetria+=String(((rpm)/100)%10);//34
        telemetria+=String(((rpm)/10)%10);//35
        telemetria+=String(((rpm)/1)%10);//36
        telemetria+=",";
        telemetria+=String(((v_bat)/100)%10);//37
        telemetria+=String(((v_bat)/10)%10);//38
        telemetria+=String(((v_bat)/1)%10);//39
        telemetria+=",";
        telemetria+=String(((a_bat)/100)%10);//40
        telemetria+=String(((a_bat)/10)%10);//41
        telemetria+=String(((a_bat)/1)%10);//42
        telemetria+="\n";
        const char* conv = telemetria.c_str();
        const char* conv2;
        const char* aux;
        serialFlush(uart);
        serialPrintf(uart, conv);
        int byte = serialGetchar(uart); //primeiro byte lido
        int arm[100] = {-10};
        int conta_virgulas = 0;
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
        	sscanf(conv2,"%d %d %d %d %d",&arm[0],&arm[1],&arm[2],&arm[3],&arm[4]);
        }
        //printf("a = %d b = %d c = %d d = %d e = %d\n",arm[0],arm[1],arm[2],arm[3],arm[4]);
        usleep(20000);
        //sleep(1);



    }
}

int file_exist(char *filename)
{
    struct stat buffer;
    return (stat(filename,&buffer)==0);
}

void* f_thread2(void* data)
{
    int txt = 1;
    int cont = 1;
    FILE *texto;
    texto = fopen("data.txt","a");
    //Telemetria pela serial USB0
    //int uart = serialOpen("/dev/ttyUSB0",57600);
    
    if(txt==1)
    {

    fprintf(texto,"\n\n\n\n\n\n\n");
    //cabeçalho
    // fprintf(texto,"tempo,axmpu,aympu,azmpu,gxmpu,gympu,gzmpu,mxmpu,mympu,mzmpu,axlsm,aylsm,azlsm,");
    // fprintf(texto,"gxlsm,gylsm,gzlsm,mxlsm,mylsm,mzlsm,roll,pitch,yaw,temperatura_baro,pressao_baro,temperatura_pitot,pressao_pitot");
    // fprintf(texto,",RPI3_V,SERVO_V,PWR_V,PWR_A,ADC2,ADC3,Longitude,Latitude,alt_elipsoide");
    // fprintf(texto, ",ch1,ch2,ch3,ch4,ch5,ch6,ch7,ch8");
    // fprintf(texto, ",vel_3d,vel_solo,heading_gps");
    // fprintf(texto, ",axmpu_filtrado,aympu_filtrado,azmpu_filtrado");
    // fprintf(texto, ",axlsm_filtrado,aylsm_filtrado,azlsm_filtrado\n");

    fprintf(texto,"           tempo");
    fprintf(texto,"           axmpu");
    fprintf(texto,"           aympu");
    fprintf(texto,"           azmpu");
    fprintf(texto,"           gxmpu");
    fprintf(texto,"           gympu");
    fprintf(texto,"           gzmpu");
    fprintf(texto,"           mxmpu");
    fprintf(texto,"           mympu");
    fprintf(texto,"           mzmpu");
    fprintf(texto,"           axlsm");
    fprintf(texto,"           aylsm");
    fprintf(texto,"           azlsm");
    fprintf(texto,"           gxlsm");
    fprintf(texto,"           gylsm");
    fprintf(texto,"           gzlsm");
    fprintf(texto,"      gps_2d_acc");
    fprintf(texto,"      gps_3d_acc");
    fprintf(texto,"          v_aero");
    fprintf(texto,"           pitch");
    fprintf(texto,"            roll");
    fprintf(texto,"             yaw");
    fprintf(texto,"          t_baro");
    fprintf(texto,"          p_baro");
    fprintf(texto,"         t_pitot");
    fprintf(texto,"         p_pitot");
    fprintf(texto,"          RPI3_V");
    fprintf(texto,"         SERVO_V");
    fprintf(texto,"           PWR_V");
    fprintf(texto,"           PWR_A");
    fprintf(texto,"            ADC2");
    fprintf(texto,"            ADC3");
    fprintf(texto,"             Lon");
    fprintf(texto,"             Lat");
    fprintf(texto,"           alt_e");
    fprintf(texto,"             ch1");
    fprintf(texto,"             ch2");
    fprintf(texto,"             ch3");
    fprintf(texto,"             ch4");
    fprintf(texto,"             ch5");
    fprintf(texto,"             ch6");
    fprintf(texto,"             ch7");
    fprintf(texto,"             ch8");
    fprintf(texto,"          vel_3d");
    fprintf(texto,"        vel_solo");
    fprintf(texto,"        head_gps");
    fprintf(texto,"         axmpu_f");
    fprintf(texto,"         aympu_f");
    fprintf(texto,"         azmpu_f");
    fprintf(texto,"         axlsm_f");
    fprintf(texto,"         aylsm_f");
    fprintf(texto,"         azlsm_f");
    fprintf(texto,"              yI");
    fprintf(texto,"              xI");
    fprintf(texto,"              zI");
    fprintf(texto,"\n");








    while(1)
    {
        if (segundos>=7200) //setar depois com uma hora = 3600segundos
        {
            fclose(texto);
            return 0;
        }
        else
        {


            fprintf(texto,"%16.6f",segundos);
            fprintf(texto,"%16.2f%16.2f%16.2f",axmpu,aympu,azmpu);
            fprintf(texto,"%16.2f%16.2f%16.2f",gxmpu,gympu,gzmpu);
            fprintf(texto,"%16.2f%16.2f%16.2f",mxmpu,mympu,mzmpu);
            fprintf(texto,"%16.2f%16.2f%16.2f",axlsm,aylsm,azlsm);
            fprintf(texto,"%16.2f%16.2f%16.2f",gxlsm,gylsm,gzlsm);
            fprintf(texto,"%16.2f%16.2f%16.2f",Horizontal_acc,Vertial_acc,v_aero); //gps - precisão horizontal 2d e 3d
            fprintf(texto,"%16.2f%16.2f%16.2f",roll,pitch,yaw);
            fprintf(texto,"%16.2f%16.2f",temperatura_baro,pressao_baro);
            fprintf(texto,"%16.2f%16.2f",temperatura_pitot,pressao_pitot);
            fprintf(texto,"%16.2f%16.2f%16.2f",ch0,ch1,ch2);
            fprintf(texto,"%16.2f%16.2f%16.2f",ch3,ch4,ch5);
            fprintf(texto,"%16.8f%16.8f%16.2f",Longitude,Latitude,height_e);
            fprintf(texto,"%16.2f%16.2f%16.2f%16.2f",ppm_ch1,ppm_ch2,ppm_ch3,ppm_ch4);
            fprintf(texto,"%16.2f%16.2f%16.2f%16.2f",ppm_ch5,ppm_ch6,ppm_ch7,ppm_ch8);
            fprintf(texto,"%16.2f%16.2f%16.2f",vel_3d,vel_solo,heading_gps);
            fprintf(texto,"%16.2f%16.2f%16.2f",axmpu_f,aympu_f,azmpu_f);
            fprintf(texto,"%16.2f%16.2f%16.2f",axlsm_f,aylsm_f,azlsm_f);
            fprintf(texto,"%16.2f%16.2f%16.2f",ned_x,ned_y,ned_z); // inerciais Ix Iy Iz
            fprintf(texto,"\n");
            //serialPrintf(uart, conv);
            usleep(33332); //30Hz
        }
    }
    }
}


void* f_thread3(void* data)
{
    //GPS
    std::vector<double> pos_data;
    std::vector<double> vel_data;
    if(gps.testConnection())
    {
        printf("Ublox test OK\n");
    }
    if (!gps.configureSolutionRate(200)) //taxa de atualização em millisegundos
    {
        printf("Setting new rate: FAILED\n");
    }

    while(1)
    {
        //GPS -DADOS DE POSIÇÃO POSLLH = posição latitude longitude altitude
        if(gps.decodeSingleMessage(Ublox::NAV_POSLLH, pos_data)==1)
        {

            time_of_week   = pos_data[0]/1000;
            Longitude      = pos_data[1]/10000000;
            Latitude       = pos_data[2]/10000000;
            height_e       = pos_data[3]/1000;
            height_msl     = pos_data[4]/1000;
            Horizontal_acc = pos_data[5]/1000;
            Vertial_acc    = pos_data[6]/1000;
        }
        //usleep(100000);
        if(gps.decodeSingleMessage(Ublox::NAV_VELNED, vel_data)==1)
        {
            time_of_week2 = vel_data[0]/1000;
            vel_norte     = vel_data[1]/100;
            vel_leste     = vel_data[2]/100;
            vel_descida   = vel_data[3]/100;
            vel_3d        = vel_data[4]/100;
            vel_solo      = vel_data[5]/100;
            heading_gps   = vel_data[6]/100000;
            //printf("ok");
        }
        //printf("alt gps = %.2f\n",height_msl);


        //printf(" v_3d = %  .2f v_solo = %  .2f v_n = %  .2f v_l = %  .2f v_d = %  .2f heading = %  .2f\n",vel_3d,vel_solo,vel_norte,vel_leste,vel_descida,heading_gps);
        //usleep(100000);
    }
}

int main()
{
    //CHECA NAVIO2
    if (check_apm()) 
    {
        return 1;
    }
    //INICIALIZAÇÕES
    //ADS1115
    //ads.setGain(GAIN_EIGHT);
    //ads.begin();
    //PITOT
    //pitot.initialize();

    //HABILITA LEITURA DE PORTAS ADC's
    ADC.initialize();

    //MPU9250
    mpu1.initialize();

    //LSM9DS1
    lsm1.initialize();

    //AHRS
    euler.sensorinit();
    euler.setGyroOffset();

    //BARÔMETRO MS5611
    //rodará no thread3
    barometer.initialize();

    //ARQUIVO TEXTO COM DADOS
    //rodará no thread2

    //GPS
    //rodará no thread3

    //PPM
    ppm.initialize();

    //CONFIGURA 2 PORTAS DE PWM - freq 50Hz
    servo.initialize(PWM_OUTPUT_1);
    servo.set_frequency(PWM_OUTPUT_1, 50);
    servo.enable(PWM_OUTPUT_1);

    servo.initialize(PWM_OUTPUT_2);
    servo.set_frequency(PWM_OUTPUT_2, 50);
    servo.enable(PWM_OUTPUT_2);

    //MULTITHREADING
    //RASPBERRY PI 3B+ 4 NÚCLEOS = RODA 4 THREADS
    //SENDO O MAIN O THREAD0
    //THREAD0 - main()
    //THREAD1 - F_THREAD1
    //THREAD2 - F_THREAD2
    //THREAD3 - F_THREAD3
    pthread_t thread1,thread2,thread3;
    pthread_create(&thread1,NULL,f_thread1,NULL);
    pthread_create(&thread2,NULL,f_thread2,NULL);
    pthread_create(&thread3,NULL,f_thread3,NULL);

    //LOOP PRINCIPAL CONTA COMO THREAD
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


        //Leitura do canal ADC
        ch0 = ADC.read(0); //TENSÃO DA PLACA
        ch1 = ADC.read(1); //TENSÃO DA LINHA DE SERVOS
        ch2 = ADC.read(2); //TENSÃO   DO POWER MODULE
        ch3 = ADC.read(3); //CORRENTE DO POWER MODULE
        ch4 = ADC.read(4); //TENSÃO DA PORTA ADC2
        ch5 = ADC.read(5); //TENSÃO DA PORTA ADC3

        //COMANDO SINAIS DE PWM. 1000 = 1000us
        servo.set_duty_cycle(PWM_OUTPUT_1, 1000);
        //servo.set_duty_cycle(PWM_OUTPUT_2, 1000);

        //PPM
        ppm_ch1 = ppm.read(0);
        ppm_ch2 = ppm.read(1);
        ppm_ch3 = ppm.read(2);
        ppm_ch4 = ppm.read(3);
        ppm_ch5 = ppm.read(4);
        ppm_ch6 = ppm.read(5);
        ppm_ch7 = ppm.read(6);
        ppm_ch8 = ppm.read(7);

        //MPU9250
        //LEITURA DE DADOS DE ACELERAÇÃO, GIRO E MAGNÉTICOS
        mpu1.update();
        mpu1.read_accelerometer(&axmpu, &aympu, &azmpu);
        mpu1.read_gyroscope(&gxmpu, &gympu, &gzmpu);
        mpu1.read_magnetometer(&mxmpu, &mympu, &mzmpu);
        //VALOR FILTRADO
        axmpu_f = f_passa_baixa(axmpu_f_antes,dt,axmpu_antes);
        axmpu_f_antes=axmpu_f;
        axmpu_antes=axmpu;
        aympu_f = f_passa_baixa(aympu_f_antes,dt,aympu_antes);
        aympu_f_antes=aympu_f;
        aympu_antes=aympu;
        azmpu_f = f_passa_baixa(azmpu_f_antes,dt,azmpu_antes);
        azmpu_f_antes=azmpu_f;
        azmpu_antes=azmpu;


        //LSM9DS1
        //LEITURA DE DADOS DE ACELERAÇÃO, GIRO E MAGNÉTICOS
        lsm1.update();
        lsm1.read_accelerometer(&axlsm, &aylsm, &azlsm);
        lsm1.read_gyroscope(&gxlsm, &gylsm, &gzlsm);
        lsm1.read_magnetometer(&mxlsm, &mylsm, &mzlsm);
        //VALOR FILTRADO
        axlsm_f = f_passa_baixa(axlsm_f_antes,dt,axlsm_antes);
        axlsm_f_antes=axlsm_f;
        axlsm_antes=axlsm;
        aylsm_f = f_passa_baixa(aylsm_f_antes,dt,aylsm_antes);
        aylsm_f_antes=aylsm_f;
        aylsm_antes=aylsm;
        azlsm_f = f_passa_baixa(azlsm_f_antes,dt,azlsm_antes);
        azlsm_f_antes=azlsm_f;
        azlsm_antes=azlsm;

        //MS4525DO LEITURA DE PRESSÃO DIFERENCIAL E TEMPERATURA
        //temperatura_pitot = pitot.get_Temperature();
        //pressao_pitot     = pitot.get_Pressure();

        servo.set_duty_cycle(PWM_OUTPUT_2, ppm_ch3);

        //função de calculo das coordenadas ECEF
        calc_ecef();
        //função de calculo das coordenadas no NED
        calc_ned();

        //leitura ads1115
        //adc0 = ads.readADC_SingleEnded(0);
        //printf("tensao = % .2f\t corrente = % .2f\n",ch2*0.0110344827,ch3);
        //printf("ch1 = % .0f ch2 = % .0f ",ppm_ch1,ppm_ch2);
        //printf("ch3 = % .0f ch4 = % .0f ",ppm_ch3,ppm_ch4);
        //printf("ch5 = % .0f ch6 = % .0f ",ppm_ch5,ppm_ch6);
        //printf("ch7 = % .0f ch8 = % .0f\n",ppm_ch7,ppm_ch8);
        //printf(" pressao pitot = %14.2f\n",pressao_pitot);
        //printf("%14.6f ",segundos);
        // printf("%13.2f%13.2f%13.2f",axmpu,aympu,azmpu);
        // printf("%13.2f%13.2f%13.2f",gxmpu,gympu,gzmpu);
        // printf("%13.2f%13.2f%13.2f",mxmpu,mympu,mzmpu);
        // printf("%13.2f%13.2f%13.2f",axlsm,aylsm,azlsm);
        // printf("%13.2f%13.2f%13.2f",gxlsm,gylsm,gzlsm);
        // printf("%13.2f%13.2f%13.2f",mxlsm,mylsm,mzlsm);
        // printf("%13.2f%13.2f%13.2f",roll,pitch,yaw);
        // printf("%13.2f%13.2f",temperatura_baro,pressao_baro);
        // printf("%13.2f%13.2f",temperatura_pitot,pressao_pitot);
        printf("%13.2f%13.2f%13.2f",ch0,ch1,ch2);
        printf("%13.2f%13.2f%13.2f",ch3,ch4,ch5);
        //printf(" %12.8f %12.8f %12.2f",Longitude,Latitude,height_e);
        // printf("%13.2f%13.2f%13.2f%13.2f",ppm_ch1,ppm_ch2,ppm_ch3,ppm_ch4);
        // printf("%13.2f%13.2f%13.2f%13.2f",ppm_ch5,ppm_ch6,ppm_ch7,ppm_ch8);
        // printf("%13.2f%13.2f%13.2f",vel_3d,vel_solo,heading_gps);
        // printf("%13.2f%13.2f%13.2f",axmpu_f,aympu_f,azmpu_f);
        // printf("%13.2f%13.2f%13.2f",axlsm_f,aylsm_f,azlsm_f);
        //printf(" %16.6f %16.6f %16.6f",ecef_x/1000,ecef_y/1000,ecef_z/1000);
        //printf(" %8.2f %8.2f %8.2f",ned_x,ned_y,ned_z);
        //printf(" %6.2f %6.2f",Horizontal_acc,Vertial_acc);
        //printf("adc = %d",adc0);
        printf("\n");
        usleep(33332); //30Hz

    }


    return 0;
}