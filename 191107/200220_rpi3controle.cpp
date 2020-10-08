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
NexVariable flag_piloto = NexVariable(0,4,"fp");
float flag_piloto_out = 1;
uint32_t flag_piloto_in = 1;

NexVariable h_ref_nextion = NexVariable(0,13,"hr");
float    h_ref_out,h_ref_out_anterior;
uint32_t h_ref_in;

NexVariable h_nextion = NexVariable(0,14,"h");
float    h_out;
uint32_t h_in;
NexVariable v_ref_nextion = NexVariable(0,15,"vr");
float    v_ref_out, v_ref_out_anterior;
uint32_t v_ref_in;

NexVariable v_nextion = NexVariable(0,16,"v");
float    v_out;
uint32_t v_in;

NexVariable flag_ganho = NexVariable(0,32,"fg");
uint32_t flag_ganho_in;
int    flag_ganho_out;

NexVariable atualiza_ganho = NexVariable(0,35,"ag");
uint32_t atualiza_ganho_in;
float    atualiza_ganho_out, atualiza_ganho_out_anterior = 10000;

NexVariable delta_nextion = NexVariable(0,33,"d");
uint32_t delta_nextion_in;
float    delta_nextion_out;

NexVariable reset = NexVariable(0,37,"r");
uint32_t reset_in;
int reset_out;

NexVariable psi_ref_nextion = NexVariable(0,38,"pr");
uint32_t psi_ref_in;
float psi_ref_out =0;

NexVariable psi_nextion = NexVariable(0,45,"p");
uint32_t psi_int;
float psi_out;

NexText valor = NexText(0,26,"t1");

//GANHOS PARA O PID
float ganho_h_p     = -2.1665;  //0
float ganho_h_i     = -0.4325;  //1
float ganho_h_d     = 0;

float ganho_p        = -0.0045; //2
float ganho_r        = -0.0783; //3
float ganho_erro_phi = 0.4854;  //4

float ganho_theta_p = 10;       //5
float ganho_theta_i = 15;       //6

float ganho_q_p     = -0.839;   //7

float ganho_v_p     = -1.098;   //8
float ganho_v_i     = -0.9707;  //9

float ganho_interconexao = -11.25; //10

float ganho_psi = 0.484;         //11

//Variáveis de controle
float h = 750;
float v = 33.33333333;
float psi = 0; 

// ROTINA PID
PID pid_h;
PID pid_theta;
PID pid_v;



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
int modo_pm_pa = 0; // inicializar o modo de pilotagem 0 -> PM 1-> PA
struct js_event joy2;
int js2 = open(device2,O_RDONLY);
//flag de botoes
int botao_h_up   = 0;
int flag_botao_h_up_uma_vez   = 0;
int flag_botao_h_down_uma_vez = 0;
int flag_botao_v_down_uma_vez = 0;
int flag_botao_v_up_uma_vez   = 0;
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
int uart = serialOpen("/dev/ttyUSB1",3000000);

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
        manete_raw = -1*((float)axis2[2]);
        manete = ((0.0015258789)*manete_raw) + 50;
        //printf("a = %6.2f p = %6.2f m = %6.2f\n",aileron,profundor,manete);

    }
    close( js2 );
}
//TELA NEXTION
void* f_thread3(void* data)
{
	char buffer[20] = {0};
	String texto;

    while(1)
    {
        sendCommand("");
        sendCommand("bkcmd=1");
        bool ret1 = recvRetCommandFinished();
        flag_piloto.getValue(&flag_piloto_in);
        flag_piloto_out = (float)flag_piloto_in;
        if(flag_piloto_out == 0)
        {
            modo_pm_pa = 0;
        }
        if(flag_piloto_out == 1)
        {
            modo_pm_pa = 1;
        }

        h_ref_nextion.getValue(&h_ref_in);
        h_ref_out = (float)h_ref_in;

        v_ref_nextion.getValue(&v_ref_in);
        v_ref_out = (float)v_ref_in/3.6;

        psi_ref_nextion.getValue(&psi_ref_in);
        psi_ref_out = ((float)psi_ref_in)-180;

        h_nextion.setValue((int)h);
        float v_aux = v*3.6;
        v_nextion.setValue((int)v_aux);
        psi_nextion.setValue((int)psi);


        flag_ganho.getValue(&flag_ganho_in);
        flag_ganho_out = (int)flag_ganho_in;

        delta_nextion.getValue(&delta_nextion_in);
        delta_nextion_out = (float)delta_nextion_in;

        atualiza_ganho.getValue(&atualiza_ganho_in);
        atualiza_ganho_out = (float)atualiza_ganho_in;

        reset.getValue(&reset_in);
        reset_out = (int)reset_in;

        //printf("fg = %d delta = %4.1f ag = %7.2f\n",flag_ganho_out,delta_nextion_out,atualiza_ganho_out);
        switch(flag_ganho_out)
        {
        	case 0:
        		sprintf(buffer,"%.4f",ganho_h_p);
        	    valor.setText(buffer);
        		if(atualiza_ganho_out > atualiza_ganho_out_anterior)
        		{
        			atualiza_ganho_out_anterior = atualiza_ganho_out;
        			ganho_h_p = ganho_h_p + ganho_h_p*(delta_nextion_out/100);
        			sprintf(buffer,"%.4f",ganho_h_p);
        			valor.setText(buffer);


        		}
        		if(atualiza_ganho_out < atualiza_ganho_out_anterior)
        		{
        			atualiza_ganho_out_anterior = atualiza_ganho_out;
        			ganho_h_p = ganho_h_p - ganho_h_p*(delta_nextion_out/100);
        			sprintf(buffer,"%.4f",ganho_h_p);
        			valor.setText(buffer);
        		}
        		pid_h.ganhos(ganho_h_p, ganho_h_i, ganho_h_d);
        		if(reset_out!=0)
        		{
        			ganho_h_p = -2.1665;
        		}
        		break;
        		case 1:
        		sprintf(buffer,"%.4f",ganho_h_i);
        		valor.setText(buffer);
        		if(atualiza_ganho_out > atualiza_ganho_out_anterior)
        		{
        			atualiza_ganho_out_anterior = atualiza_ganho_out;
        			ganho_h_i = ganho_h_i + ganho_h_i*(delta_nextion_out/100);
        			sprintf(buffer,"%.4f",ganho_h_i);
        			valor.setText(buffer);


        		}
        		if(atualiza_ganho_out < atualiza_ganho_out_anterior)
        		{
        			atualiza_ganho_out_anterior = atualiza_ganho_out;
        			ganho_h_i = ganho_h_i - ganho_h_i*(delta_nextion_out/100);
        			sprintf(buffer,"%.4f",ganho_h_i);
        			valor.setText(buffer);
        		}
        		pid_h.ganhos(ganho_h_p, ganho_h_i, ganho_h_d);
        		if(reset_out!=0)
        		{
        			ganho_h_i = -0.4325;
        		}
        		break;
        		case 2:
        		sprintf(buffer,"%.4f",ganho_p);
        		valor.setText(buffer);
        		if(atualiza_ganho_out > atualiza_ganho_out_anterior)
        		{
        			atualiza_ganho_out_anterior = atualiza_ganho_out;
        			ganho_p = ganho_p + ganho_p*(delta_nextion_out/100);
        			sprintf(buffer,"%.4f",ganho_p);
        			valor.setText(buffer);


        		}
        		if(atualiza_ganho_out < atualiza_ganho_out_anterior)
        		{
        			atualiza_ganho_out_anterior = atualiza_ganho_out;
        			ganho_p = ganho_p - ganho_p*(delta_nextion_out/100);
        			sprintf(buffer,"%.4f",ganho_p);
        			valor.setText(buffer);
        		}
        		if(reset_out!=0)
        		{
        			ganho_p = -0.0045;
        		}
        		break;
        		case 3:
        		sprintf(buffer,"%.4f",ganho_r);
        		valor.setText(buffer);
        		if(atualiza_ganho_out > atualiza_ganho_out_anterior)
        		{
        			atualiza_ganho_out_anterior = atualiza_ganho_out;
        			ganho_r = ganho_r + ganho_r*(delta_nextion_out/100);
        			sprintf(buffer,"%.4f",ganho_r);
        			valor.setText(buffer);


        		}
        		if(atualiza_ganho_out < atualiza_ganho_out_anterior)
        		{
        			atualiza_ganho_out_anterior = atualiza_ganho_out;
        			ganho_r = ganho_r - ganho_r*(delta_nextion_out/100);
        			sprintf(buffer,"%.4f",ganho_r);
        			valor.setText(buffer);
        		}
        		if(reset_out!=0)
        		{
        			ganho_r = -0.0783;
        		}
        		break;
        		case 4:
        		sprintf(buffer,"%.4f",ganho_erro_phi);
        		valor.setText(buffer);
        		if(atualiza_ganho_out > atualiza_ganho_out_anterior)
        		{
        			atualiza_ganho_out_anterior = atualiza_ganho_out;
        			ganho_erro_phi = ganho_erro_phi + ganho_erro_phi*(delta_nextion_out/100);
        			sprintf(buffer,"%.4f",ganho_erro_phi);
        			valor.setText(buffer);


        		}
        		if(atualiza_ganho_out < atualiza_ganho_out_anterior)
        		{
        			atualiza_ganho_out_anterior = atualiza_ganho_out;
        			ganho_erro_phi = ganho_erro_phi - ganho_erro_phi*(delta_nextion_out/100);
        			sprintf(buffer,"%.4f",ganho_erro_phi);
        			valor.setText(buffer);
        		}
        		if(reset_out!=0)
        		{
        			ganho_erro_phi = 0.4854;
        		}
        		break;
        		case 5:
        		sprintf(buffer,"%.4f",ganho_theta_p);
        		valor.setText(buffer);
        		if(atualiza_ganho_out > atualiza_ganho_out_anterior)
        		{
        			atualiza_ganho_out_anterior = atualiza_ganho_out;
        			ganho_theta_p = ganho_theta_p + ganho_theta_p*(delta_nextion_out/100);
        			sprintf(buffer,"%.4f",ganho_theta_p);
        			valor.setText(buffer);


        		}
        		if(atualiza_ganho_out < atualiza_ganho_out_anterior)
        		{
        			atualiza_ganho_out_anterior = atualiza_ganho_out;
        			ganho_theta_p = ganho_theta_p - ganho_theta_p*(delta_nextion_out/100);
        			sprintf(buffer,"%.4f",ganho_theta_p);
        			valor.setText(buffer);
        		}
        		pid_theta.ganhos(ganho_theta_p,ganho_theta_i,0);
        		if(reset_out!=0)
        		{
        			ganho_theta_p = 10;
        		}
        		break;
        		case 6:
        		sprintf(buffer,"%.4f",ganho_theta_i);
        		valor.setText(buffer);
        		if(atualiza_ganho_out > atualiza_ganho_out_anterior)
        		{
        			atualiza_ganho_out_anterior = atualiza_ganho_out;
        			ganho_theta_i = ganho_theta_i + ganho_theta_i*(delta_nextion_out/100);
        			sprintf(buffer,"%.4f",ganho_theta_i);
        			valor.setText(buffer);


        		}
        		if(atualiza_ganho_out < atualiza_ganho_out_anterior)
        		{
        			atualiza_ganho_out_anterior = atualiza_ganho_out;
        			ganho_theta_i = ganho_theta_i - ganho_theta_i*(delta_nextion_out/100);
        			sprintf(buffer,"%.4f",ganho_theta_i);
        			valor.setText(buffer);
        		}
        		pid_theta.ganhos(ganho_theta_p,ganho_theta_i,0);
        		if(reset_out!=0)
        		{
        			ganho_theta_i = 15;
        		}
        		break;
        		case 7:
        		sprintf(buffer,"%.4f",ganho_q_p);
        		valor.setText(buffer);
        		if(atualiza_ganho_out > atualiza_ganho_out_anterior)
        		{
        			atualiza_ganho_out_anterior = atualiza_ganho_out;
        			ganho_q_p = ganho_q_p + ganho_q_p*(delta_nextion_out/100);
        			sprintf(buffer,"%.4f",ganho_q_p);
        			valor.setText(buffer);


        		}
        		if(atualiza_ganho_out < atualiza_ganho_out_anterior)
        		{
        			atualiza_ganho_out_anterior = atualiza_ganho_out;
        			ganho_q_p = ganho_q_p - ganho_q_p*(delta_nextion_out/100);
        			sprintf(buffer,"%.4f",ganho_q_p);
        			valor.setText(buffer);
        		}
        		if(reset_out!=0)
        		{
        			ganho_q_p = -0.839;
        		}
        		break;
        		case 8:
        		sprintf(buffer,"%.4f",ganho_v_p);
        		valor.setText(buffer);
        		if(atualiza_ganho_out > atualiza_ganho_out_anterior)
        		{
        			atualiza_ganho_out_anterior = atualiza_ganho_out;
        			ganho_v_p = ganho_v_p + ganho_v_p*(delta_nextion_out/100);
        			sprintf(buffer,"%.4f",ganho_v_p);
        			valor.setText(buffer);


        		}
        		if(atualiza_ganho_out < atualiza_ganho_out_anterior)
        		{
        			atualiza_ganho_out_anterior = atualiza_ganho_out;
        			ganho_v_p = ganho_v_p - ganho_v_p*(delta_nextion_out/100);
        			sprintf(buffer,"%.4f",ganho_v_p);
        			valor.setText(buffer);
        		}
        		pid_v.ganhos(ganho_v_p,ganho_v_i,0);
        		if(reset_out!=0)
        		{
        			ganho_v_p = -1.098;
        		}
        		break;
        		case 9:
        		sprintf(buffer,"%.4f",ganho_v_i);
        		valor.setText(buffer);
        		if(atualiza_ganho_out > atualiza_ganho_out_anterior)
        		{
        			atualiza_ganho_out_anterior = atualiza_ganho_out;
        			ganho_v_i = ganho_v_i + ganho_v_i*(delta_nextion_out/100);
        			sprintf(buffer,"%.4f",ganho_v_i);
        			valor.setText(buffer);


        		}
        		if(atualiza_ganho_out < atualiza_ganho_out_anterior)
        		{
        			atualiza_ganho_out_anterior = atualiza_ganho_out;
        			ganho_v_i = ganho_v_i - ganho_v_i*(delta_nextion_out/100);
        			sprintf(buffer,"%.4f",ganho_v_i);
        			valor.setText(buffer);
        		}
        		pid_v.ganhos(ganho_v_p,ganho_v_i,0);
        		if(reset_out!=0)
        		{
        			ganho_v_i = -0.9707;
        		}
        		break;
        		case 10:
        		sprintf(buffer,"%.4f",ganho_interconexao);
        		valor.setText(buffer);
        		if(atualiza_ganho_out > atualiza_ganho_out_anterior)
        		{
        			atualiza_ganho_out_anterior = atualiza_ganho_out;
        			ganho_interconexao = ganho_interconexao + ganho_interconexao*(delta_nextion_out/100);
        			sprintf(buffer,"%.4f",ganho_interconexao);
        			valor.setText(buffer);


        		}
        		if(atualiza_ganho_out < atualiza_ganho_out_anterior)
        		{
        			atualiza_ganho_out_anterior = atualiza_ganho_out;
        			ganho_interconexao = ganho_interconexao - ganho_interconexao*(delta_nextion_out/100);
        			sprintf(buffer,"%.4f",ganho_interconexao);
        			valor.setText(buffer);
        		}
        		if(reset_out!=0)
        		{
        			ganho_interconexao = -11.25;
        		}
        		break;


        }

        printf("psi_ref = %6.2f\n",psi_ref_out);
		//usleep(200000);
    }
}

int main()
{
    //NEXTION
    printf("Nextion_in\n");
    nexInit();
    printf("Nextion_out\n");
    //PID's
    pid_h.init();
    pid_h.ganhos(ganho_h_p, ganho_h_i, ganho_h_d);

    pid_theta.init();
    pid_theta.ganhos(ganho_theta_p,ganho_theta_i,0);

    pid_v.init();
    pid_v.ganhos(ganho_v_p,ganho_v_i,0);

    pthread_t thread1,thread2,thread3;
    pthread_create(&thread1,NULL,f_thread1,NULL);
    pthread_create(&thread2,NULL,f_thread2,NULL);
    pthread_create(&thread3,NULL,f_thread3,NULL);
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

        

        //VARIAVEIS RECEBIDAS VIA SERIAL
        float p       = ((float)arm[0]/1000000) - 500;
        float r       = ((float)arm[1]/1000000) - 500;
        psi           = ((float)arm[2]/1000000) - 500;
        float phi     = ((float)arm[3]/1000000) - 500;

        

        h             = ((float)arm[5]/1000000) - 500;
        float theta   = ((float)arm[6]/1000000) - 500;
        v             = ((float)arm[8]/1000000) - 500;
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
                h_ref = h_ref_out;
                //ROTINA PID's
                pid_h.zera_integrador();
                pid_theta.zera_integrador();
                pid_v.zera_integrador();

                interconexao=0;
                flag_h = 1;
            }            
            //CALCULOS PARTE SUPERIOR no matlab
            float erro_psi = psi_ref_out - psi;

            float erro_phi        = erro_psi*ganho_psi - phi;
            comando_aileron = -(p*ganho_p + r*ganho_r +erro_phi*ganho_erro_phi);

            //CALCULOS DA PARTE INFERIOR
            //altitude e theta
            //VARIAÇÃO EXPONENCIAL DE H_REF PARA SUAVIZAÇÃO DA MALHA DE ALTITUDE
            //tela nextion pega alt ref e ajusta delta h ref
            if(h_ref_out!=h_ref_out_anterior)
            {
                time_hold_h = segundos;
                if(h_ref_out - h_ref_out_anterior > 0)
                {
                	delta_h_ref = 50;
                }
                if(h_ref_out - h_ref_out_anterior < 0)
                {
                	delta_h_ref = -50;
                }
                h_ref_out_anterior = h_ref_out;

            }

            h_ref = h_ref_out;
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
            
            //ROTINA PID's
            float h_pi = pid_h.calc_pid(h_ref_exp, dt, h);


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

            //ROTINA PID's
            float theta_pi = pid_theta.calc_pid(theta_ref, dt, theta);

            float q_p  = q*ganho_q_p;

            float theta_e_q = -theta_pi -q_p;

            //velocidade
            //nextion
            if(v_ref_out!=v_ref_out_anterior)
            {
                time_hold_v = segundos;
                if(v_ref_out - v_ref_out_anterior > 0)
                {
                	delta_v_ref = 1.388888;
                }
                if(v_ref_out - v_ref_out_anterior < 0)
                {
                	delta_v_ref = -1.388888;
                }
                v_ref_out_anterior = v_ref_out;

            }
            v_ref = v_ref_out;
            v_ref_exp = (v_ref - delta_v_ref) + (delta_v_ref)*(1 - exp(-(segundos - time_hold_v)/(abs(delta_v_ref)/v_ratio)));
            //ROTINA PID's
            v_pi = pid_v.calc_pid(v_ref_exp, dt, v);

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

        //PADRÃO DE SAÍDA
        float saida1 = (comando_aileron+500)*1000000; // comando aileron
        comando1     = (int)saida1;
        float saida2 = (interconexao+500)*1000000; // comando profundor
        comando2     = (int)saida2;
        float saida3 = ((-1*v_pi)+500)*1000000; // comando de manete
        comando3     = (int)saida3;
        float saida4 = (comando_leme+500)*1000000; // comando de leme
        comando4     = (int)saida4;

        //JOGANDO VALORES DE COMANDO NA SERIAL
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