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
float leme = 50;
float leme_raw;
int n_axis1 = 0;
int x;
int n_bots = 0;
int axis[10];
char nome_joystick[80];
char button[20];
struct js_event joy;
int js = open(device,O_RDONLY);

#define device2 "/dev/input/js1"
float profundor = 50;
float profundor_raw;
float aileron = 50;
float aileron_raw;
float manete = 50;
float manete_raw;
int n_axis2 = 0;
int x2;
int n_bots2 = 0;
int axis2[10];
char nome_joystick2[80];
char button2[20];
int modo_pm_pa = 1; // inicializar o modo de pilotagem 0 -> PM 1-> PA
struct js_event joy2;
int js2 = open(device2,O_RDONLY);
//flag de botoes
int botao_h_up   = 0;
int botao_h_down = 0;
int botao_v_up   = 0;
int botao_v_down = 0;
float flag_sinal_h = 1;
float h_inicial=1000;
float delta_h_ref;
float h_ref=1000;;
float delta_v_ref;
float v_ref=1000;;

//TEMPO
struct timeval tv;
float dt,segundos=0,contador=0;
static unsigned long previoustime, currenttime;
float time_hold_h = 0, intervalo = 0;
float time_hold_v = 0;

//comandos de saída da serial 
int comando1 = 500000000;
int comando2 = 500000000;
int comando3 = 500000000;
int comando4 = 500000000;

//altitude referencia para incrementos ++ --


//inicialização da comunicação serial
int uart = serialOpen("/dev/ttyUSB0",3000000);

void* f_thread1(void* data)
{
    ioctl( js, JSIOCGAXES, &n_axis1 );
    ioctl( js, JSIOCGBUTTONS, &n_bots );
    ioctl( js, JSIOCGNAME(80), &nome_joystick );

    //printf("%s %d %d\n",nome_joystick,n_bots,n_axis1);

    fcntl( js, F_SETFL, O_NONBLOCK );

    while(1)
    {
        read(js, &joy, sizeof(joy));

        switch (joy.type & ~JS_EVENT_INIT)
        {
            case JS_EVENT_AXIS:
                axis   [ joy.number ] = joy.value;
                break;
            case JS_EVENT_BUTTON:
                button [ joy.number ] = joy.value;
                break;
        }
        leme_raw = (float)axis[1];
        leme = ((0.0015258789)*leme_raw) + 50;
        //usleep(20000);

    }


    close( js );
}

void* f_thread2(void* data)
{
    int botao_h_up_2   = 0;
    int botao_h_down_2 = 0;
    int botao_v_up_2   = 0;
    int botao_v_down_2 = 0;

    ioctl( js2, JSIOCGAXES, &n_axis2 );
    ioctl( js2, JSIOCGBUTTONS, &n_bots2 );
    ioctl( js2, JSIOCGNAME(80), &nome_joystick2 );
    fcntl( js2, F_SETFL, O_NONBLOCK );

    while(1)
    {
        read(js2, &joy2, sizeof(joy2));

        switch (joy2.type & ~JS_EVENT_INIT)
        {
            case JS_EVENT_AXIS:
                axis2   [ joy2.number ] = joy2.value;
                break;
            case JS_EVENT_BUTTON:
                button2 [ joy2.number ] = joy2.value;
                break;
        }
        profundor_raw = (float)axis2[1];
        profundor = ((0.0015258789)*profundor_raw) + 50;
        aileron_raw = (float)axis2[0];
        aileron = ((0.0015258789)*aileron_raw) + 50;
        manete_raw = -1*((float)axis2[3]);
        manete = ((0.0015258789)*manete_raw) + 50;
        if(button2[3]==1)
        {
            modo_pm_pa =0;
        }
        if(button2[5]==1)
        {
            modo_pm_pa =1;
        }
        if(button2[4] == 0)
        {
            botao_h_up   = 0;
            botao_h_up_2 = 0;
        }
        if(button2[6] == 0)
        {
            botao_h_down   = 0;
            botao_h_down_2 = 0;
        }
        if(button2[7] == 0)
        {
            botao_v_down   = 0;
            botao_v_down_2 = 0;
        }
        if(button2[9] == 0)
        {
            botao_v_up   = 0;
            botao_v_up_2 = 0;
        }
        if(modo_pm_pa == 0)
        {
            if(button2[4] == 1)
            {
                if(botao_h_up_2 == 0)
                {
                    delta_h_ref = 50;
                    h_ref = h_ref + delta_h_ref;
                    botao_h_up_2 = 1;
                }
            }
            if(button2[6] == 1)
            {
                if(botao_h_down_2 == 0)
                {
                    delta_h_ref = -50;
                    h_ref = h_ref + delta_h_ref;
                    botao_h_down_2 = 1;
                }
            }
            if(button2[9] == 1)
            {
                if(botao_v_up_2 == 0)
                {
                    delta_v_ref = 1.388888;
                    v_ref = v_ref + delta_v_ref;
                    botao_v_up_2 = 1;
                }
            }
            if(button2[7] == 1)
            {
                if(botao_v_down_2 == 0)
                {
                    delta_v_ref = -1.388888;
                    v_ref = v_ref + delta_v_ref;
                    botao_v_down_2 = 1;
                }
            }
        }
        //printf("aileron = %6.2f profundor = %6.2f leme = %6.2f manete = %6.2f modo_pm_pa = %1d comando = %1d",aileron,profundor,leme,manete,modo_pm_pa,button2[2]);
        //printf("  \n");
        //usleep(20000);

    }


    close( js2 );
}

int main()
{
    pthread_t thread1,thread2;
    pthread_create(&thread1,NULL,f_thread1,NULL);
    pthread_create(&thread2,NULL,f_thread2,NULL);
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

    //flags para alternancia entre pm e pa na seleção de h_ref
    int flag_pm = 0;
    int flag_h  = 1; 

    //THREAD0
    float h_ref_exp=1000;
    float v_ref_exp=120;
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

        //VARIAVEIS RECEBIDAS VIA SERIAL
        float p       = ((float)arm[0]/1000000) - 500;
        float r       = ((float)arm[1]/1000000) - 500;
        float phi_ref = ((float)arm[2]/1000000) - 500;
        float phi     = ((float)arm[3]/1000000) - 500;

        

        float h       = ((float)arm[5]/1000000) - 500;
        float theta   = ((float)arm[6]/1000000) - 500;
        float v       = ((float)arm[8]/1000000) - 500;
        float q       = ((float)arm[9]/1000000) - 500;
        //VARIAVEIS DE CALCULO
        float comando_profundor;
        float comando_leme = 0;
        float comando_manete;
        float comando_aileron;
        float v_pi;

        //alternar entre PM E PA
        if(modo_pm_pa == 0) //PM
        {
            if(flag_pm==0)
            {
                flag_pm = 1;
            }
            if(aileron > 50)
            {
                comando_aileron   = (-3*aileron + 150)/5;
            }
            if(aileron == 50)
            {
                comando_aileron = 0;
            }
            if(aileron < 50)
            {
                comando_aileron   = (15*aileron + 750)/50;
            }
            if(profundor > 50)
            {
                comando_profundor   = (-3*profundor + 150)/5;
            }
            if(profundor == 50)
            {
                comando_profundor = 0;
            }
            if(profundor < 50)
            {
                comando_profundor   = (-2*profundor + 100)/5;
            }



            //comando_aileron   = (-45*aileron + 1500)/100;
            //comando_profundor = (-profundor + 40)/2;
            interconexao = comando_profundor;
            comando_leme      = (7*leme - 350)/10;
            comando_manete = -manete/100;
            v_pi = comando_manete;

            flag_h = 0;


        }
        if(modo_pm_pa == 1) //PA
        {
            //delta de variação de h
            float delta_h_ref_valor = 50;
            //valor da taxa de variação de h
            float h_ratio = 5;
            float v_ratio = 0.5;
            if(flag_pm==0)
            {
                h_ref   = ((float)arm[4]/1000000) - 500;
                v_ref   = ((float)arm[7]/1000000) - 500;
            }

            if(flag_h==0)
            {
                h_ref = h;
                h_i=0;
                theta_i=0;
                v_i=0;
                interconexao=0;
                flag_h = 1;
            }
            if(button2[4]==1)
            {
                if(botao_h_up == 0)
                {
                    delta_h_ref = 50;
                    h_ref = h_ref + delta_h_ref;
                    flag_sinal_h = 1;
                    time_hold_h = segundos;
                    botao_h_up = 1;
                }
            }
            if(button2[6]==1)
            {  
                if(botao_h_down == 0)
                {
                    delta_h_ref = -50;
                    h_ref = h_ref + delta_h_ref;
                    flag_sinal_h = -1;
                    time_hold_h = segundos;
                    botao_h_down = 1;
                }
            }
            if(button2[9]==1)
            {
                if(botao_v_up==0)
                {
                    delta_v_ref = 1.388888;
                    v_ref = v_ref + delta_v_ref;
                    time_hold_v = segundos;
                    botao_v_up = 1;

                }
            }
            if(button2[7]==1)
            {
                if(botao_v_down == 0)
                {
                    delta_v_ref = -1.388888;
                    v_ref = v_ref + delta_v_ref;
                    time_hold_v = segundos;
                    botao_v_down = 1;
                }
            }
            
            //CALCULOS PARTE SUPERIOR
            float erro_phi        = phi_ref - phi;
            comando_aileron = -(p*ganho_p + r*ganho_r +erro_phi*ganho_erro_phi);

            //CALCULOS DA PARTE INFERIOR
            //altitude e theta
            //VARIAÇÃO EXPONENCIAL DE H_REF PARA SUAVIZAÇÃO DA MALHA DE ALTITUDE
            h_ref_exp = (h_ref - delta_h_ref) + (delta_h_ref)*(1 - exp(-(segundos - time_hold_h)/(abs(delta_h_ref)/h_ratio)));
            if(h_ref_exp>5000)
            {
                h_ref_exp = 5000;
            }
            if(h_ref_exp<-500)
            {
                h_ref_exp = -500;
            }
            float erro_h = h_ref_exp - h;
            //erro_h = (h_ref-h)*(1-exp(-segundos/200));
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
            v_ref_exp = (v_ref - delta_v_ref) + (delta_v_ref)*(1 - exp(-(segundos - time_hold_v)/(abs(delta_v_ref)/v_ratio)));
            float erro_v = v_ref_exp - v;
            //float erro_v = v_ref - v;
            float v_p    =  erro_v*ganho_v_p;
            v_i          = (v_i + erro_v*dt);
            v_pi   = v_p + v_i*ganho_v_i;

            interconexao = (-v_pi*ganho_interconexao) + theta_e_q;
            if(comando_aileron>15)
            {
                comando_aileron=15;
            }
            if(comando_aileron<-30)
            {
                comando_aileron=-30;
            }
            if(interconexao>20)
            {
                interconexao=20;
            }
            if(interconexao<-30)
            {
                interconexao=-30;
            }
            if(comando_leme>35)
            {
                comando_leme=35;
            }
            if(comando_leme<-35)
            {
                comando_leme=-35;
            }
            if(v_pi>1)
            {
                v_pi = 1;
            }
            if(v_pi<-1)
            {
                v_pi = -1;
            }
        ////////////FIM DO CONTROLE////////////
        }
        //BATENTES DE COMANDO
        // if(comando_aileron>15)
        // {
        //     comando_aileron = 15;
        // }
        // if(comando_aileron<-15)
        // {
        //     comando_aileron;
        // }
        //PADRÃO DE SAÍDA
        float saida1 = (comando_aileron+500)*1000000; // comando aileron
        comando1     = (int)saida1;
        float saida2 = (interconexao+500)*1000000; // comando profundor
        comando2     = (int)saida2;
        float saida3 = ((-1*v_pi)+500)*1000000; // comando de manete
        comando3     = (int)saida3;
        float saida4 = (comando_leme+500)*1000000; // comando de leme
        comando4     = (int)saida4;

        //print de auxilio
        
        printf("aileron = %6.2f profundor = %6.2f leme = %6.2f manete = %6.2f h = %6.2f h_ref = %6.2f h_ref_exp = %6.2f v = %6.2f v_ref = %6.2f\n",comando_aileron,interconexao,comando_leme,v_pi,h,h_ref,h_ref_exp,v*3.6,v_ref*3.6);
        //printf("erro_h=%.2f h_ref=%.2f h=%.2f h_i=%.2f h_pi=%.2f\n",erro_h,h_ref,h,h_i,h_pi);
        //printf("erro_theta = %.2f theta_p = %.2f theta_i = %.2f h_pi = %.2f erro_h = %.2f\n",erro_theta,theta_p,theta_i,h_pi,erro_h);
        //printf("theta_i = %.2f erro_theta = %.2f ganho_theta_i = %.2f dt = %.8f\n",theta_i,erro_theta,ganho_theta_i,dt);

        //Escrita na serial.
        String telemetria;

        telemetria+=String("A");
        telemetria+=String(((comando4)/100000000)%10);//25
        telemetria+=String(((comando4)/10000000)%10);//25
        telemetria+=String(((comando4)/1000000)%10);//25
        telemetria+=String(((comando4)/100000)%10);//25
        telemetria+=String(((comando4)/10000)%10);//25
        telemetria+=String(((comando4)/1000)%10);//25
        telemetria+=String(((comando4)/100)%10);//25
        telemetria+=String(((comando4)/10)%10);//26
        telemetria+=String(((comando4)/1)%10);//27
        telemetria+=",";
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