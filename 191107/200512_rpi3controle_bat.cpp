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

//BATERIA
float v_bat = 0;
float i_bat = 0;

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

NexVariable load = NexVariable(0,51,"l");
uint32_t load_in;
int load_out;

NexVariable save = NexVariable(0,50,"s");
uint32_t save_in;
int save_out;

NexVariable psi_ref_nextion = NexVariable(0,38,"pr");
uint32_t psi_ref_in;
float psi_ref_out = 0;

NexVariable psi_nextion = NexVariable(0,45,"p");
uint32_t psi_int;
float psi_out;

NexText valor = NexText(0,26,"t1");

//GANHOS PARA O PID set 0
// float ganho_h_p      = -2.1665;  //0
// float ganho_h_i      = -0.4325;  //1
// float ganho_h_d      = 0;
// float ganho_p        = -0.0045; //2
// float ganho_r        = -0.0783; //3
// float ganho_erro_phi = 0.4854;  //4
// float ganho_theta_p  = 5;       //5
// float ganho_theta_i  = 7.5;       //6
// float ganho_q_p     = -0.22;    //7
// float ganho_v_p     = -0.1012;   //8
// float ganho_v_i     = -0.9707;  //9
// float ganho_interconexao = -11.25; //10
// float ganho_psi = 0.484;         //11

// GANHO PARA O PID SET 1
// float ganho_h_p      = -1.5460; //0
// float ganho_h_i      = -0.1799; //1
// float ganho_h_d      = 0;
// float ganho_p        = -0.0045; //2
// float ganho_r        = -0.0783; //3
// float ganho_erro_phi = 0.4854;  //4
// float ganho_theta_p = 1;        //5
// float ganho_theta_i = 0.6;      //6
// float ganho_q_p     = -0.22;    //7
// float ganho_v_p     = -0.2;     //8
// float ganho_v_i     = -0.025;   //9
// float ganho_interconexao = -11.2566; //10
// float ganho_psi = 0.484;         //11

//GANHOS PARA O PID SET 2
float ganho_h_p      = -1.8462;  //0
float ganho_h_i      = -0.4224;  //1
float ganho_h_d      = 0;
float ganho_p        = -0.0045; //2
float ganho_r        = -0.0783; //3
float ganho_erro_phi = 0.4854;  //4
float ganho_theta_p  = 1.4886;       //5
float ganho_theta_i  = 3.5494 ;       //6
float ganho_q_p      = -0.1224;    //7
float ganho_v_p      = -0.4109;   //8
float ganho_v_i      = -0.0347;  //9
float ganho_interconexao = -11.2566; //10
float ganho_psi = 0.484;         //11

 float flag_h_tela = 0;

// ROTINA PID
PID pid_h;
PID pid_theta;
PID pid_v;

//VARIAVEIS RECEBIDAS VIA SERIAL
float p;
float q;
float r;
float phi = 0;
float psi = 0; 
float theta = 0;
float h = 750;
float v = 27.78;
float v_ant = 27.78;
//THREAD0
float h_ref_exp    = 1000;
float v_ref_exp    = 27.78;
float theta_pi     = 0;
float theta_ref    = 0;
float erro_phi     = 0;
float erro_psi     = 0;
float interconexao = 0;
float erro_h = 0;
float erro_v = 0;

//VARIAVEIS DE CALCULO
float comando_profundor;
float comando_leme = 50;
float comando_manete =-0.5;
float comando_aileron = 0;
float v_pi = 0;

//joystick
#define device "/dev/input/js1"
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

#define device2 "/dev/input/js0"
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
int flag_h_tela2 = 0;
float h_init;
float h_ref0 = 1000;

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

//inicialização da comunicação serial
int uart = serialOpen("/dev/ttyUSB1",3000000);
//int nextion_display = serialOpen("/dev/ttyUSB0",115200);

//MANOBRAS PRE - PROGRAMADAS
int flag_comandado = 0;
float time_hold_comandado = 0;
float comando_inicial = 0;
float perturbacao = 0;
float freq_vario = 0;
float contador_freq = 0;
float tempo_pa_ativo = 0;
float contador_aero = 1;
float soma_aero = 0;
float v_filtrado = 27.78;
float v_filtrado_antes = 27.78;
float v_table = 0;

//RUIDO
int n_lines = 983;
float v_noise[983]={};
float t_noise[983]={};


//lê ruído de arquivo texto com ruído no formato t,v
void noise_read(void)
{
    FILE *noise;
    int i = 0;
    noise = fopen("noise.txt","r");
    while(i<n_lines)
    {
        fscanf(noise,"%f,%f",&t_noise[i],&v_noise[i]);
        i = i + 1;
    }
    fclose(noise);

}

//intercepta por interpolação o ruído
float noise_intercept(float tempo)
{

    int i       = 0;
    float t1    = 0;
    float t2    = 0;
    float v1    = 0;
    float v2    = 0;
    float v_out = 1;

    float t_max = t_noise[n_lines-1];

    if(tempo > t_max)
    {
        
        return 0;
    }
    else
    {
        while(t_noise[i] < tempo)
        {
            i = i + 1;
        }

        t1 = t_noise[i-1];
        t2 = t_noise[i];

        v1 = v_noise[i-1];
        v2 = v_noise[i];
        v_out = v1+(v2-v1)*(tempo-t1)/(t2-t1);
        return v_out;

    }
}

//FILTRO PASSA BAIXA 
float f_passa_baixa(float x_antes,float delta_t, float leitura, float freq_corte)
{
    //float freq_corte = 0.25;//hz
    float w = freq_corte*2*3.1415;
    return (x_antes+(delta_t*w*(leitura-x_antes)));
}

//FUNÇÃO DA MANOBRA PRE PROGRAMADA
float manobras_seno (float inicio, float tempo, float comandado, float detla_tempo_comandado, float frequencia)
{
	float w = frequencia*2*3.1415;
	if(tempo < inicio)
	{
		return 0;
	}
	if(tempo > inicio && tempo < inicio + detla_tempo_comandado)
	{
		return comandado*sin(w*tempo);
	}
	return 0;
}
float manobras (float inicio, float tempo, float comandado, float detla_tempo_comandado)
{
	if(tempo < inicio)
	{
		return 0;
	}
	if(tempo > inicio && tempo < inicio + detla_tempo_comandado)
	{
		return comandado;
	}
	if (tempo > inicio + detla_tempo_comandado && tempo < inicio + 2*detla_tempo_comandado)
	{
		return -1*comandado;
	}
	return 0;
}
int random(int limite)
{
	float temp_rand = segundos*10;
	srand((unsigned)temp_rand);
	int randomico = (rand() % limite + 1);
	return randomico;

}


void* f_thread1(void* data)
{
    ioctl( js, JSIOCGAXES, &n_axis1 );
    ioctl( js, JSIOCGBUTTONS, &n_bots );
    ioctl( js, JSIOCGNAME(80), &nome_joystick );

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
    }
    close( js2 );
}
//TELA NEXTION
void* f_thread3(void* data)
{
	char buffer[20] = {0};
	//String texto;
	FILE *ganhos;
	ganhos = fopen("ganhos.txt","a");
	fclose(ganhos);

    while(1)
    {
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
        if(flag_h_tela2 == 0)
        {
        	flag_h_tela2 = 1;
        	h_ref_out_anterior = h_ref_out;
        }

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

        load.getValue(&load_in);
        load_out = (int)load_in;

        save.getValue(&save_in);
        save_out = (int)save_in;
        if(save_out!=0)
        {
        	ganhos = fopen("ganhos.txt","w");
			fprintf(ganhos,"%.4f,%.4f,%.4f,%.4f,",ganho_h_p,ganho_h_i,ganho_p,ganho_r);
			fprintf(ganhos,"%.4f,%.4f,%.4f,%.4f,",ganho_erro_phi,ganho_theta_p,ganho_theta_i,ganho_q_p);
			fprintf(ganhos,"%.4f,%.4f,%.4f\n",ganho_v_p,ganho_v_i,ganho_interconexao);
			fclose(ganhos);
        }
        if(load_out!=0)
        {
        	float g0,g1,g2,g3,g4,g5,g6,g7,g8,g9,g10;
        	ganhos = fopen("ganhos.txt","r");
        	fscanf(ganhos, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f",&g0,&g1,&g2,&g3,&g4,&g5,&g6,&g7,&g8,&g9,&g10);
        	fclose(ganhos);
        	printf("%.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f\n",g0,g1,g2,g3,g4,g5,g6,g7,g8,g9,g10);

        	ganho_h_p          = g0;
        	ganho_h_i          = g1;
        	ganho_p            = g2;
        	ganho_r            = g3;
        	ganho_erro_phi     = g4;
        	ganho_theta_p      = g5;
        	ganho_theta_i      = g6;
        	ganho_q_p          = g7;
        	ganho_v_p          = g8;
        	ganho_v_i          = g9;
        	ganho_interconexao = g10;
        }

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
    }
}

void* f_thread4(void* data)
{
    int txt = 1;
    int cont = 1;
    FILE *texto;
    FILE *refs;
    texto = fopen("data.txt","a");
    refs  = fopen("data_ref.txt","a");
      

    if(txt==1)
    {
    	fprintf(texto,"\n\n\n\n\n\n\n");
    	fprintf(texto,"               t");
    	fprintf(texto,"               p");
    	fprintf(texto,"               q");
    	fprintf(texto,"               r");
    	fprintf(texto,"         theta_r");
    	fprintf(texto,"           theta");
    	fprintf(texto,"           h_ref");
    	fprintf(texto,"               h");
    	fprintf(texto,"         psi_ref");
    	fprintf(texto,"             psi");
    	fprintf(texto,"         phi_ref");
    	fprintf(texto,"             phi");
    	fprintf(texto,"           v_ref");
    	fprintf(texto,"               v");
    	fprintf(texto,"         pic_cic");
    	fprintf(texto,"         cmd_ail");
    	fprintf(texto,"        cmd_prof");
    	fprintf(texto,"         cmd_man");
    	fprintf(texto,"         md_leme");
    	fprintf(texto,"          pert_a");
    	fprintf(texto,"          pert_e");
    	fprintf(texto,"          pert_m");
    	fprintf(texto,"          pert_l");
        fprintf(texto,"          v_f1hz");
        fprintf(texto,"          h_erro");
        fprintf(texto,"          v_erro");
        fprintf(texto,"             h_i");
        fprintf(texto,"         theta_i");
        fprintf(texto,"             v_i");
        fprintf(texto,"           v_bat");
        fprintf(texto,"           i_bat");
    	fprintf(texto,"\n");

    	fprintf(refs,"\n\n\n\n\n\n\n");
    	fprintf(refs,"               t");
    	fprintf(refs,"           theta");
    	fprintf(refs,"               h");
    	fprintf(refs,"             psi");
    	fprintf(refs,"             phi");
    	fprintf(refs,"               v");
    	fprintf(refs,"         cmd_ail");
    	fprintf(refs,"        cmd_prof");
    	fprintf(refs,"         cmd_man");
    	fprintf(refs,"         md_leme");
    	fprintf(refs,"\n");

    	while(1)
    	{
    		if(segundos>7200)
    		{
    			fclose(texto);
    			fclose(refs);
    			return 0;
    		}
    		else
    		{
    			fprintf(texto,"%16.6f",segundos);
    			fprintf(texto,"%16.6f",p);
    			fprintf(texto,"%16.6f",q);
    			fprintf(texto,"%16.6f",r);
    			fprintf(texto,"%16.6f",theta_ref);
    			fprintf(texto,"%16.6f",theta);
    			fprintf(texto,"%16.6f",h_ref_exp);
    			fprintf(texto,"%16.6f",h);
    			fprintf(texto,"%16.6f",psi_ref_out);
    			fprintf(texto,"%16.6f",psi);
    			fprintf(texto,"%16.6f",erro_psi*ganho_psi);
    			fprintf(texto,"%16.6f",phi);
    			fprintf(texto,"%16.6f",v_ref_exp);
    			fprintf(texto,"%16.6f",v);
    			fprintf(texto,"%16.6d",modo_pm_pa);
    			fprintf(texto,"%16.6f",comando_aileron);
    			fprintf(texto,"%16.6f",interconexao);
    			fprintf(texto,"%16.6f",v_pi);
    			fprintf(texto,"%16.6f",comando_leme);
    			fprintf(texto,"%16.6f",perturbacao);
    			fprintf(texto,"%16.6f",perturbacao);
    			fprintf(texto,"%16.6f",perturbacao);
    			fprintf(texto,"%16.6f",perturbacao);
                fprintf(texto,"%16.6f",v_filtrado);
                fprintf(texto,"%16.6f",erro_h);
                fprintf(texto,"%16.6f",erro_v);
                fprintf(texto,"%16.6f",pid_h.I);
                fprintf(texto,"%16.6f",pid_theta.I);
                fprintf(texto,"%16.6f",pid_v.I);
                fprintf(texto,"%16.6f",v_bat);
                fprintf(texto,"%16.6f",i_bat);
    			fprintf(texto,"\n");

    			fprintf(refs,"%16.6f",segundos);
    			fprintf(refs,"%16.6f",theta_ref);
    			fprintf(refs,"%16.6f",h_ref_exp);
    			fprintf(refs,"%16.6f",psi_ref_out);
    			fprintf(refs,"%16.6f",erro_psi*ganho_psi);
    			fprintf(refs,"%16.6f",v_ref_exp);
    			fprintf(refs,"%16.6f",perturbacao);
    			fprintf(refs,"%16.6f",perturbacao);
    			fprintf(refs,"%16.6f",perturbacao);
    			fprintf(refs,"%16.6f",perturbacao);
    			fprintf(refs,"\n");


    			usleep(20000); //50Hz

    		}
    	}

    }
}

int main()
{
    //NEXTION
    printf("Nextion_in\n");
    nexInit();
    printf("Nextion_out\n");
    FILE *v_aero_table;
    v_aero_table = fopen("v_aero_table.txt","r");

    //LENDO RUÍDO DO ARQUIVO noise.txt
    noise_read();
 
    //PID's
    pid_h.init();
    pid_h.ganhos(ganho_h_p, ganho_h_i, ganho_h_d);

    pid_theta.init();
    pid_theta.ganhos(ganho_theta_p,ganho_theta_i,0);

    pid_v.init();
    pid_v.ganhos(ganho_v_p,ganho_v_i,0);

    pthread_t thread1,thread2,thread3,thread4;
    pthread_create(&thread1,NULL,f_thread1,NULL);
    pthread_create(&thread2,NULL,f_thread2,NULL);
    pthread_create(&thread3,NULL,f_thread3,NULL);
    pthread_create(&thread4,NULL,f_thread4,NULL);
    //armazenadores dos dados recebidos via serial
    int arm[11]          = {500000000,500000000,500000000,500000000,500000000,500000000,500000000,500000000,500000000,500000000,500000000};
    int arm_anterior[11] = {500000000,500000000,500000000,500000000,500000000,500000000,500000000,500000000,500000000,500000000,500000000};
    int arm_novo[11]     = {500000000,500000000,500000000,500000000,500000000,500000000,500000000,500000000,500000000,500000000,500000000};
    printf("começou\n");
    float pi = 3.14159;
    //INTEGRADORES
    float h_i=0;
    float theta_i=0;
    float v_i=0;

    //flags para alternancia entre pm e pa na seleção de h_ref
    int flag_pm = 0;
    int flag_h  = 1;
    int flag_h2 = 0; 

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
            arm[10]= 500000000;
            arm[11]= 500000000;
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
                sscanf(conv2,"%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",&arm_novo[0],&arm_novo[1],&arm_novo[2],&arm_novo[3],&arm_novo[4],&arm_novo[5],&arm_novo[6],&arm_novo[7],&arm_novo[8],&arm_novo[9],&arm_novo[10],&arm_novo[11]);
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
                    arm[10]=arm_anterior[10];
                    arm[11]=arm_anterior[11];

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
                    arm[10]=arm_novo[10];
                    arm[11]=arm_novo[11];

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
                    arm_anterior[10]=arm_novo[10];
                    arm_anterior[11]=arm_novo[11];
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

        //VARIAVEIS RECEBIDAS VIA SERIAL
        p       = ((float)arm[0]/1000000) - 500;
        r       = ((float)arm[1]/1000000) - 500;
        psi     = ((float)arm[2]/1000000) - 500;
        phi     = ((float)arm[3]/1000000) - 500;
        h       = ((float)arm[5]/100000)  - 500;
        theta   = ((float)arm[6]/1000000) - 500;
        v       = ((float)arm[8]/1000000) - 500;
        q       = ((float)arm[9]/1000000) - 500;
        v_bat   = ((float)arm[10]/1000000) - 500;
        i_bat   = ((float)arm[11]/1000000) - 500;

        //alternar entre PM E PA
        if(modo_pm_pa == 0) //PM
        {
            if(flag_pm==0)
            {
                flag_pm = 1;
            }
            if(aileron > 50)
            {
                //comando_aileron   = (-3*aileron + 150)/5;
                comando_aileron = -0.6*(aileron - 50);
            }
            if(aileron == 50)
            {
                comando_aileron = 0;
            }
            if(aileron < 50)
            {
                // comando_aileron   = (15*aileron + 750)/50;
                comando_aileron   = -0.3*(aileron-50);
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
            flag_h_tela = 0;
            flag_h2 = 0;
            flag_h_tela2 = 0;
            flag_comandado = 0;
            time_hold_comandado = 0;
            perturbacao = 0;
            contador = 0;
            contador_freq = 0;

            theta_ref = theta;



        }

        if(modo_pm_pa == 1) //PA
        {
        	flag_h2 = 1;
            //delta de variação de h
            float delta_h_ref_valor = 50;
            float delta_t_ref_valor = 10;
            //valor da taxa de variação de h
            float h_ratio = 2.5;
            float v_ratio = 0.5;

            if(flag_h==0)
            {
            	tempo_pa_ativo = segundos;
                h_ref = h;
                h_ref_exp = h;
                h_init = h;
                h_ref0 = h;
                //ROTINA PID's
                pid_h.zera_integrador();
                pid_theta.zera_integrador();
                pid_v.zera_integrador();

                interconexao = 0;
                flag_h = 1;
                flag_h_tela = 0;

                interconexao = comando_profundor;
            	comando_manete = -manete/100;
            	v_pi = comando_manete;
            }

            //CALCULOS PARTE SUPERIOR no matlab
            erro_psi = psi_ref_out - psi;
            //batente de psi
            if(erro_psi>30)
            {
            	erro_psi = 30;
            }
            if(erro_psi<-30)
            {
            	erro_psi = -30;
            }

            erro_phi        = erro_psi*ganho_psi - phi;

            comando_aileron = -(p*ganho_p + r*ganho_r +erro_phi*ganho_erro_phi);

            //CALCULOS DA PARTE INFERIOR
            //altitude e theta
            //VARIAÇÃO EXPONENCIAL DE H_REF PARA SUAVIZAÇÃO DA MALHA DE ALTITUDE
            //tela nextion pega alt ref e ajusta delta h ref
            
            if(h_ref_out!=h_ref_out_anterior)
            {
            	flag_h_tela = 1;
                time_hold_h = segundos;
                if(h_ref_out - h_ref_out_anterior > 0)
                {
                	delta_h_ref = 50;
                }
                if(h_ref_out - h_ref_out_anterior < 0)
                {
                	delta_h_ref = -50;
                }
                h_init = h;
                h_ref_out_anterior = h_ref_out;
            }
            if(flag_h_tela2 == 0)
            {
            	delta_h_ref = 0;
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
            erro_h = h_ref_exp - h;
            
            //ROTINA PID's
            float batente_h = 100;
            float h_pi = pid_h.calc_pid(h_ref_exp, dt, h,batente_h,0);

            theta_ref = -h_pi + 0.020173*180/pi;
            if(theta_ref>10)
            {
                theta_ref=10;
            }
            if(theta_ref<-10)
            {
                theta_ref=-10;
            }
            if(byte==-1)
            {
                theta_ref = 0;
            }

            //ROTINA PID's
            float batente_theta = 10;
            //1 ativa o batententeador 
            theta_pi = pid_theta.calc_pid(theta_ref, dt, theta,batente_theta,2);

            float q_p  = q*ganho_q_p;

            float theta_e_q = -theta_pi -q_p;

            //velocidade
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

            float v_noisy = 0;
        	if(segundos - tempo_pa_ativo > 60 && segundos - tempo_pa_ativo < 80)
        	{
            	float tempo_atual = segundos - tempo_pa_ativo - 60;
            	v_noisy = noise_intercept(tempo_atual);
            	v = v + v_noisy;
                printf("v_bat = %8.2f i_bat = %8.2f\n",v_bat,i_bat);
            	//printf("v = %.2f v_ref = %.2f h = %.2f h_ref = %.2f\n",v,v_ref_exp,h,h_ref_exp);
            	//printf("tempo = %.2f ruído = %.2f\n",segundos - tempo_pa_ativo-60,v_noisy);
        	}
        	else
        	{
                printf("v_bat = %8.2f i_bat = %8.2f\n",v_bat,i_bat);
            	//printf("t = %.2f h_i = %.4f theta_i = %.4f v_i = %.4f \n",segundos- tempo_pa_ativo,pid_h.I,pid_theta.I,pid_v.I);

        	}
            //COLOCAR IF DE RETORNO DA VELOCIDADE.

            v_filtrado = f_passa_baixa(v_filtrado_antes,dt, v_ant,1);
            v_filtrado_antes = v_filtrado;
            v_ant = v;
            erro_v = v_ref_exp - v_filtrado;
            float batente_v = 30;
            v_pi = pid_v.calc_pid(v_ref_exp, dt, v_filtrado,batente_v,2); // 2 significa anti windup do integrador será ativado


            interconexao = (-v_pi*ganho_interconexao) + theta_e_q;
        	
       		//SUAVIZAÇÃO DO COMANDO DE PROFUNDOR
        	// if(segundos - tempo_pa_ativo<30)
        	// {
        	// 	//interconexao = interconexao*exp(-5/(1*(segundos- tempo_pa_ativo)));
        	// 	interconexao = interconexao*(1-exp(-(segundos - tempo_pa_ativo)/10));
        	// }

            //LIMITES DE COMANDOS (BATENTES)
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
            if(v_pi>0)
            {
                v_pi = 0;
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
        float integrativo = pid_v.ki*pid_v.I;
        
        //printf("tempo = %.2f ruído = %.2f\n",segundos - tempo_pa_ativo,v_noisy);
        //printf("tempo_pa_ativo =%8.2f h =%8.2f h_ref =%8.2f v =%8.2f\n",segundos - tempo_pa_ativo,h,h_ref,v);
        //usleep(20000); //50Hz
    }//while thread0
    return 0;
}
