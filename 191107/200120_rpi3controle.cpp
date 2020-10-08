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

//joystick
#define device "/dev/input/js0"

//TEMPO
struct timeval tv;
float dt,segundos=0,contador=0;
static unsigned long previoustime, currenttime;

//comandos de saída da serial 
int comando1 = 500000000;
int comando2 = 500000000;
int comando3 = 500000000;

//inicialização da comunicação serial
int uart = serialOpen("/dev/ttyUSB0",3000000);

void* f_thread1(void* data)
{
int n_axis = 0;
    int x;
    int n_bots = 0;
    int axis[10];
    char nome_joystick[80];
    char button[20];
    struct js_event joy;
    float leme = 50;
    float leme_raw;

    int js = open(device,O_RDONLY);

    ioctl( js, JSIOCGAXES, &n_axis );
    ioctl( js, JSIOCGBUTTONS, &n_bots );
    ioctl( js, JSIOCGNAME(80), &nome_joystick );

    //printf("%s %d %d\n",nome_joystick,n_bots,n_axis);

    fcntl( js, F_SETFL, O_NONBLOCK );

    while(1)
    {
        read(js, &joy, sizeof(struct js_event));

        switch (joy.type & ~JS_EVENT_INIT)
        {
            case JS_EVENT_AXIS:
                axis   [ joy.number ] = joy.value;
                break;
            case JS_EVENT_BUTTON:
                button [ joy.number ] = joy.value;
                break;
        }

            //printf("R: %6d  ", axis[3] );
        leme_raw = (float)axis[1];
        leme = ((0.0015258789)*leme_raw) + 50;
        printf("%6d %6f",axis[1],leme);
        //for( x=0 ; x<n_bots ; ++x )
            //printf("B%d: %d  ", x, button[x] );

        printf("  \n");
        //fflush(stdout);
        usleep(20000);

    }


    close( js );
}

int main()
{
    pthread_t thread1;
    pthread_create(&thread1,NULL,f_thread1,NULL);
    //armazenadores dos dados recebidos via serial
    int arm[10]          = {500000000,500000000,500000000,500000000,500000000,500000000,500000000,500000000,500000000,500000000};
    int arm_anterior[10] = {500000000,500000000,500000000,500000000,500000000,500000000,500000000,500000000,500000000,500000000};
    int arm_novo[10]     = {500000000,500000000,500000000,500000000,500000000,500000000,500000000,500000000,500000000,500000000};
    printf("começou\n");
    float pi = 3.14159;
    //INTEGRADORES
    float h_i=0;
    float theta_i=0;
    float v_i=0;
    float interconexao=0;
    //THREAD0
    while(1)
    {

        const char* conv2;
        int byte = serialGetchar(uart); //primeiro byte lido
        //em caso de timout da serial resetar armazenadores
        if(byte==-1)
        {
            arm[0] = 500000000;
            arm[1] = 500000000;
            arm[2] = 500000000;
            arm[3] = 500000000;
            arm[4] = 500000000;
            arm[5] = 500000000;
            arm[6] = 500000000;
            arm[7] = 500000000;
            arm[8] = 500000000;
            arm[9] = 500000000;
        }
        else
        {
            String msg;
            if(byte == 65) //identifica header A = 65 em ascii
            {
                msg="";
            
                while(byte!=90) //enquanto diferente de Z = 90
                {
                    byte = serialGetchar(uart);
                    switch(byte)
                    {
                        case 44:
                        {
                            msg+=",";
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
                sscanf(conv2,"%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",&arm_novo[0],&arm_novo[1],&arm_novo[2],&arm_novo[3],&arm_novo[4],&arm_novo[5],&arm_novo[6],&arm_novo[7],&arm_novo[8],&arm_novo[9]);
                if(arm_novo[4]==0)
                {
                    arm[1]=arm_anterior[1];
                    arm[2]=arm_anterior[2];
                    arm[3]=arm_anterior[3];
                    arm[4]=arm_anterior[4];
                    arm[5]=arm_anterior[5];
                    arm[6]=arm_anterior[6];
                    arm[7]=arm_anterior[7];
                    arm[8]=arm_anterior[8];
                    arm[9]=arm_anterior[9];
                    arm[0]=arm_anterior[0];

                }
                else
                {
                    arm[1]=arm_novo[1];
                    arm[2]=arm_novo[2];
                    arm[3]=arm_novo[3];
                    arm[4]=arm_novo[4];
                    arm[5]=arm_novo[5];
                    arm[6]=arm_novo[6];
                    arm[7]=arm_novo[7];
                    arm[8]=arm_novo[8];
                    arm[9]=arm_novo[9];
                    arm[0]=arm_novo[0];

                    arm_anterior[1]=arm_novo[1];
                    arm_anterior[2]=arm_novo[2];
                    arm_anterior[3]=arm_novo[3];
                    arm_anterior[4]=arm_novo[4];
                    arm_anterior[5]=arm_novo[5];
                    arm_anterior[6]=arm_novo[6];
                    arm_anterior[7]=arm_novo[7];
                    arm_anterior[8]=arm_novo[8];
                    arm_anterior[9]=arm_novo[9];
                    arm_anterior[0]=arm_novo[0];
                }
            }//if do header identificado
        }//else da leitura da serial

        //Leitura do tempo da raspberry pi
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

        ////////////////CONTROLE///////////////
        //CONSTANTES
        float ganho_p        = -0.0045;
        float ganho_r        = -0.0783;
        float ganho_erro_phi = 0.4854;

        float ganho_h_p     = -2.1665;
        float ganho_h_i     = -0.4325;
        float ganho_theta_p = 10;
        float ganho_theta_i = 15;

        float ganho_q_p     = -0.839;

        float ganho_v_p     = -1.098;
        float ganho_v_i     = -0.9707;

        float ganho_interconexao = -11.25;

        //VARIAVEIS
        float p       = ((float)arm[0]/1000000) - 500;
        float r       = ((float)arm[1]/1000000) - 500;
        float phi_ref = ((float)arm[2]/1000000) - 500;
        float phi     = ((float)arm[3]/1000000) - 500;

        float h_ref   = ((float)arm[4]/1000000) - 500;
        float h       = ((float)arm[5]/1000000) - 500;
        float theta   = ((float)arm[6]/1000000) - 500;
        float v_ref   = ((float)arm[7]/1000000) - 500;
        float v       = ((float)arm[8]/1000000) - 500;
        float q       = ((float)arm[9]/1000000) - 500;

        //CALCULOS PARTE SUPERIOR
        float erro_phi        = phi_ref - phi;
        float comando_aileron = -(p*ganho_p + r*ganho_r +erro_phi*ganho_erro_phi);

        //CALCULOS DA PARTE INFERIOR
        //altitude e theta
        float erro_h = h_ref - h;
        float h_p    = erro_h*ganho_h_p;
        h_i          = (h_i + erro_h*dt);
        float h_pi   = h_p+h_i*ganho_h_i;


        float theta_ref = -h_pi + 0.020173*180/pi;
        if(theta_ref>20)
        {
            theta_ref=20;
        }
        if(theta_ref<-20)
        {
            theta_ref=-20;
        }
        if(byte==-1)
        {
            theta_ref = 0;
        }
        float erro_theta = theta_ref - theta;
        float theta_p    = erro_theta*ganho_theta_p;
        theta_i          = (theta_i + erro_theta*dt);
        float theta_pi   = theta_p+theta_i*ganho_theta_i;

        float q_p  = q*ganho_q_p;

        float theta_e_q = -theta_pi -q_p;

        //velocidade
        float erro_v = v_ref - v;
        float v_p    =  erro_v*ganho_v_p;
        v_i          = (v_i + erro_v*dt);
        float v_pi   = v_p + v_i*ganho_v_i;

        interconexao = (-v_pi*ganho_interconexao) + theta_e_q;
        ////////////FIM DO CONTROLE////////////

        //PADRÃO DE SAÍDA
        float saida1 = (comando_aileron+500)*1000000;
        comando1     = (int)saida1;
        float saida2 = (interconexao+500)*1000000;
        comando2     = (int)saida2;
        float saida3 = ((-1*v_pi)+500)*1000000;
        comando3     = (int)saida3;

        //print de auxilio
        
        //printf("aileron = %14.8f theta = %14.8f v = %14.8f\n",comando_aileron,interconexao,-1*v_pi);
        //printf("erro_h=%.2f h_ref=%.2f h=%.2f h_i=%.2f h_pi=%.2f\n",erro_h,h_ref,h,h_i,h_pi);
        //printf("erro_theta = %.2f theta_p = %.2f theta_i = %.2f h_pi = %.2f erro_h = %.2f\n",erro_theta,theta_p,theta_i,h_pi,erro_h);
        //printf("theta_i = %.2f erro_theta = %.2f ganho_theta_i = %.2f dt = %.8f\n",theta_i,erro_theta,ganho_theta_i,dt);

        //Escrita na serial.
        String telemetria;

        telemetria+=String("A");
        telemetria+=String(((comando1)/100000000)%10);//25
        telemetria+=String(((comando1)/10000000)%10);//25
        telemetria+=String(((comando1)/1000000)%10);//25
        telemetria+=String(((comando1)/100000)%10);//25
        telemetria+=String(((comando1)/10000)%10);//25
        telemetria+=String(((comando1)/1000)%10);//25
        telemetria+=String(((comando1)/100)%10);//25
        telemetria+=String(((comando1)/10)%10);//26
        telemetria+=String(((comando1)/1)%10);//27
        telemetria+=",";
        telemetria+=String(((comando2)/100000000)%10);//25
        telemetria+=String(((comando2)/10000000)%10);//25
        telemetria+=String(((comando2)/1000000)%10);//25
        telemetria+=String(((comando2)/100000)%10);//25
        telemetria+=String(((comando2)/10000)%10);//25
        telemetria+=String(((comando2)/1000)%10);//25
        telemetria+=String(((comando2)/100)%10);//25
        telemetria+=String(((comando2)/10)%10);//26
        telemetria+=String(((comando2)/1)%10);//27
        telemetria+=",";
        telemetria+=String(((comando3)/100000000)%10);//25
        telemetria+=String(((comando3)/10000000)%10);//25
        telemetria+=String(((comando3)/1000000)%10);//25
        telemetria+=String(((comando3)/100000)%10);//25
        telemetria+=String(((comando3)/10000)%10);//25
        telemetria+=String(((comando3)/1000)%10);//25
        telemetria+=String(((comando3)/100)%10);//25
        telemetria+=String(((comando3)/10)%10);//26
        telemetria+=String(((comando3)/1)%10);//27
        telemetria+=String("Z");
        telemetria+="\n";

        //envio pela serial
        const char* conv = telemetria.c_str();
        serialPrintf(uart, conv);
        serialFlush(uart);

    }//while thread0
    return 0;
}