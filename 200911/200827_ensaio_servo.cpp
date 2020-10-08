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

#define PWM_OUTPUT_1 0
#define PWM_OUTPUT_2 1
#define PWM_OUTPUT_3 2
#define PWM_OUTPUT_4 3

//TEMPO
struct timeval tv;
float dt,segundos=0,contador=0;
static unsigned long previoustime, currenttime;
float time_hold_h = 0, intervalo = 0;
float time_hold_v = 0;

//PWM 
RCOutput_Navio2 servo1;
int sinal = 1;
float comando = 1000;

//texto de interpolação
int   n_lines = 19446;
float prof[19446]={};
float t[19446]={};
float angulo_prof = 0;


//Lê arquivo de texto com comando de profundor e tempo e coloca em dois arrays
void le_texto(void)
{
    FILE *dados;
    int i = 0;
    dados = fopen("200826_t_pert_prof_seno.txt","r");
    while(i<n_lines)
    {
        fscanf(dados,"%f,%f",&t[i],&prof[i]);
        i = i + 1;
    }
    fclose(dados);

}

float interpolacao(float tempo) // recebe como referencia o tempo para localizar no arquivo texto
{

    int i       = 0;
    float t1    = 0;
    float t2    = 0;
    float p1    = 0;
    float p2    = 0;
    float p_out = 0;

    float t_max = t[n_lines-1];

    if(tempo > t_max) // caso o tempo seja maior que o último valor do array de tempo retorne zero
    {
        
        return 0;
    }
    else
    {
        while(t[i] < tempo) // indica o valor de iésimo termo anterior ao tempo pedido externamente.
        {
            i = i + 1;
        }

        t1 = t[i-1];
        t2 = t[i];

        p1 = prof[i-1];
        p2 = prof[i];
        p_out = p1+(p2-p1)*(tempo-t1)/(t2-t1);
        return p_out;

    }
}

int main()
{
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

    //lendo arquivo texto
    le_texto();

    while(1)
    {
    	//pegando tempo de execução
    	gettimeofday(&tv,NULL);
        previoustime = currenttime;
        currenttime = 1000000 * tv.tv_sec + tv.tv_usec;
        dt = (currenttime - previoustime) / 1000000.0;
        if (dt>100)
        {
            dt = 0;
        }
        segundos = segundos + dt;

        if(segundos >= 10 )
        {
        	angulo_prof = interpolacao(segundos - 10);
        	comando = 1000 + ((angulo_prof+45)*(1000/90));

        }


    	servo1.set_duty_cycle(PWM_OUTPUT_1, comando);
    	servo1.set_duty_cycle(PWM_OUTPUT_2, 1000);
    	servo1.set_duty_cycle(PWM_OUTPUT_3, 1000);
    	servo1.set_duty_cycle(PWM_OUTPUT_4, 1000);
    	printf("tempo = %6.2f com_prof = %10.4f angulo_prof = %5.2f",segundos - 10,comando, angulo_prof);
    	printf("\n");
    	usleep(5000);
    }

	return 0;
}