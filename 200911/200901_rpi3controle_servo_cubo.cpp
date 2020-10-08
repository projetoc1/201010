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

//SERVOS MOTORES
#define PWM_OUTPUT_1 0
#define PWM_OUTPUT_2 1
#define PWM_OUTPUT_3 2
#define PWM_OUTPUT_4 3

//CONSUMO DE BATERIA
float P_TOTAL = 0; // kwh
float bateria = 10360; //W
float energia = 1;
float P = 0;
float arr[2]; // arr[0] = h e arr[1] = v
float flag_trajetoria = 0;
float tempo_atualiza_trajetoria = 0;
float flag_atualiza_trajetoria = 0;
char nome_arquivo_traj[20] = "solmaxalc-";

RCOutput_Navio2 servo;
float pwm_ail  = 1500;
float pwm_prof = 1500;
float pwm_man  = 1500;
float pwm_leme = 1500;

//BATERIA
float v_bat = 0;
float i_bat = 0;

//TELA NEXTION
NexVariable flag_piloto = NexVariable(0,4,"fp");
float    flag_piloto_out   = 1;
uint32_t flag_piloto_in = 1;

NexVariable h_ref_nextion = NexVariable(0,13,"hr");
float    h_ref_out, h_ref_out_anterior;
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
int      flag_ganho_out;

NexVariable atualiza_ganho = NexVariable(0,35,"ag");
uint32_t atualiza_ganho_in;
float    atualiza_ganho_out, atualiza_ganho_out_anterior = 10000;

NexVariable delta_nextion = NexVariable(0,33,"d");
uint32_t delta_nextion_in;
float    delta_nextion_out;

NexVariable reset = NexVariable(0,37,"r");
uint32_t reset_in;
int      reset_out;

NexVariable load = NexVariable(0,51,"l");
uint32_t load_in;
int      load_out;

NexVariable save = NexVariable(0,50,"s");
uint32_t save_in;
int      save_out;

NexVariable psi_ref_nextion = NexVariable(0,38,"pr");
uint32_t psi_ref_in;
float    psi_ref_out = 0;

NexVariable psi_nextion = NexVariable(0,45,"p");
uint32_t psi_int;
float    psi_out;

NexText valor = NexText(0,26,"t1");

//GANHOS PARA O PID set 0
// float ganho_h_p          = -2.1665; //0
// float ganho_h_i          = -0.4325; //1
// float ganho_h_d          = 0;       // não necessário
// float ganho_p            = -0.0045; //2
// float ganho_r            = -0.0783; //3
// float ganho_erro_phi     = 0.4854;  //4
// float ganho_theta_p      = 5;       //5
// float ganho_theta_i      = 7.5;     //6
// float ganho_q_p          = -0.22;   //7
// float ganho_v_p          = -0.1012; //8
// float ganho_v_i          = -0.9707; //9
// float ganho_interconexao = -11.25;  //10
// float ganho_psi          = 0.484;   //11

// GANHO PARA O PID SET 1
// float ganho_h_p          = -1.5460;  //0
// float ganho_h_i          = -0.1799;  //1
// float ganho_h_d          = 0;        // não necessário
// float ganho_p            = -0.0045;  //2
// float ganho_r            = -0.0783;  //3
// float ganho_erro_phi     = 0.4854;   //4
// float ganho_theta_p      = 1;        //5
// float ganho_theta_i      = 0.6;      //6
// float ganho_q_p          = -0.22;    //7
// float ganho_v_p          = -0.2;     //8
// float ganho_v_i          = -0.025;   //9
// float ganho_interconexao = -11.2566; //10
// float ganho_psi          = 0.484;    //11

//GANHOS PARA O PID SET 2
float ganho_h_p          = -1.8462;  //0
float ganho_h_i          = -0.4224;  //1
float ganho_h_d          = 0;        // não necessário
float ganho_p            = -0.0045;  //2
float ganho_r            = -0.0783;  //3
float ganho_erro_phi     = 0.4854;   //4
float ganho_theta_p      = 1.4886;   //5
float ganho_theta_i      = 3.5494 ;  //6
float ganho_q_p          = -0.1224;  //7
float ganho_v_p          = -0.4109;  //8
float ganho_v_i          = -0.0347;  //9
float ganho_interconexao = -11.2566; //10
float ganho_psi          = 0.484;    //11


float flag_h_tela = 0;
float freq_sen    = 0;

// ROTINA PID
PID pid_h;
PID pid_theta;
PID pid_v;

//VARIAVEIS RECEBIDAS VIA SERIAL
float p;
float q;
float r;
float phi   = 0;
float psi   = 0; 
float theta = 0;
float h     = 750;
float v     = 27.78;
float v_ant = 27.78;
//THREAD0
float h_ref_exp    = 1000;
float v_ref_exp    = 27.78;
float theta_pi     = 0;
float theta_ref    = 0;
float erro_phi     = 0;
float erro_psi     = 0;
float interconexao = 0;
float erro_h       = 0;
float erro_v       = 0;

float batente_v = 100;

//VARIAVEIS DE CALCULO
float comando_profundor;
float comando_leme    = 50;
float comando_manete  = -0.5;
float comando_aileron = 0;
float v_pi            = 0;
float h_traj          = 0;
float tempo_traj_up   = 0;
float tempo_traj_down = 0;
float h_plato         = 0;
float delta_prof      = 0;
float delta_ail       = 0;
float delta_man       = 0;
float h_ref_ant       = 0;
float h_ref_novo      = 0;

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
float aileron   = 50;
float aileron_raw;
float manete    = 50;
float manete_raw;
int n_axis2     = 0;
int x2;
int n_bots2     = 0;
int axis2[10];
char nome_joystick2[80];
char button2[20];
int modo_pm_pa  = 0; // inicializar o modo de pilotagem 0 -> PM 1-> PA
struct js_event joy2;
int js2 = open(device2,O_RDONLY);
//flag de botoes
int botao_h_up                = 0;
int flag_botao_h_up_uma_vez   = 0;
int flag_botao_h_down_uma_vez = 0;
int flag_botao_v_down_uma_vez = 0;
int flag_botao_v_up_uma_vez   = 0;
int botao_h_down              = 0;
int botao_v_up                = 0;
int botao_v_down              = 0;
float flag_sinal_h            = 1;
float h_inicial               = 1000;
float delta_h_ref;
float h_ref                   = 1000;;
float delta_v_ref;
float v_ref                   = 1000;;
int flag_h_tela2              = 0;
float h_init;
float h_ref0                  = 1000;

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
int flag_comandado        = 0;
float time_hold_comandado = 0;
float comando_inicial     = 0;
float perturbacao         = 0;
float perturbacao_comando = 0;
float freq_vario          = 0;
float contador_freq       = 0;
float tempo_pa_ativo      = 0;
float contador_aero       = 1;
float soma_aero           = 0;
float v_filtrado          = 27.78;
float v_filtrado_antes    = 27.78;
float v_table             = 0;

//RUIDO
int n_lines        = 983;
float v_noise[983] = {};
float t_noise[983] = {};

//TRAJETORIA MORALES
int n_lines_traj   = 400; // numero de linhas das trajetorias otimas salvas
float traj_t[400]  = {}; // vetor de valores de tempo da trajetoria otima, em segundos
float traj_h[400]  = {}; // vetor de valores de altitude da trajetoria otima, em metros
float traj_V[400]  = {}; // vetor de valores de velocidade da trajetoria otima, em METROS POR SEGUNDO
float traj_E[400]  = {}; // vetor de valores de Energia da trajetoria otima, ADIMENSIONAL (de 0 a 1) 
float traj_xt[400] = {}; // vetor de valores de desolcamento horizontal da trajetoria otima, em metros

int n_lines_aresta_E = 10; // numero de elementos da aresta de Energia do cubo
float aresta_E[10]   = {}; // vetor onde ficarao guardados os valores da aresta de Energia do cubo

int n_lines_aresta_h_i = 25; // numero de elementos da aresta de altitude inicial do cubo
float aresta_h_i[25]   = {}; // vetor onde ficarao guardados os valores da aresta de altitude inicial do cubo

int n_lines_aresta_h_f = 25; // numero de elementos da aresta de altitude final do cubo
float aresta_h_f[25]   = {}; // vetor onde ficarao guardados os valores da aresta de altitude final do cubo

float E; // Energia atual
float h_i; // altitude (inicial) atual
float h_f; // altitude final desejada pelo piloto

int iE; // elemento da aresta de Energia mais proximo da Energia atual
int ihi; // elemento da aresta de altitude incial mais proximo da altitude (inicial) atual
int ihf; // elemento da aresta de altitude final mais proximo da altitude final (desejada)

int ntraj; // elemento do cubo desdobrado que contem as informacoes da trajetoria otima que sera utilizada
int iotim; // numero da trajetoria otima que sera utilizada
int iinterc; // elemento a partir do qual a trajetoria otima sera utilizada
int n_lines_util; // numero de linhas que serao utilizadas da trajetoria otima

float tini; // valor do tempo no instante a partir do qual a trajetoria otima sera utilizada
float hini; // valor de altitude no instante a partir do qual a trajetoria otima sera utilizada

float h_f_estimada;  // altitude final estimada, em METROS

void arestaE_read(void) // le os pontos da aresta de Energia e guarda no vetor "aresta_E"
{
  FILE *arquivo_arestaE;
  int i = 0;
  arquivo_arestaE = fopen("arestaE.txt","r");
  while (i < n_lines_aresta_E)
  {
    fscanf(arquivo_arestaE, "%f", &aresta_E[i]);
    i = i + 1;
  }
  fclose(arquivo_arestaE);
}


void arestahi_read(void) // le os pontos da aresta de altitude (inicial) e guarda no vetor "aresta_h_i"
{
  FILE *arquivo_arestahi;
  int i = 0;
  arquivo_arestahi = fopen("arestahi.txt","r");
  while (i < n_lines_aresta_h_i)
  {
    fscanf(arquivo_arestahi, "%f", &aresta_h_i[i]);
    i = i + 1;
  }
  fclose(arquivo_arestahi);
}


void arestahf_read(void) // le os pontos da aresta de altitude final e guarda no vetor "aresta_h_f"
{
  FILE *arquivo_arestahf;
  int i = 0;
  arquivo_arestahf = fopen("arestahf.txt","r");
  while (i < n_lines_aresta_h_f)
  {
    fscanf(arquivo_arestahf, "%f", &aresta_h_f[i]);
    i = i + 1;
  }
  fclose(arquivo_arestahf);
}


int coordcubo_find(float valor, float vec[], int n_lines) // a partir de um valor dado de "E", "h_i" ou "h_f" encontra o ponto mais proximo na aresta correspondente;
{
  int i = 0;
  int condi = 1;
  while (condi) {
    if (vec[i] <= valor) {
      i = i + 1;
    } else {
      condi = 0;
      if (i == 0) {
        i = 0;
      } else {
      i = i - 1;
      }
    }
    if (i > n_lines - 1) {
      condi = 0;
      i = n_lines - 1;
    }
  }
  return i;
}


void cuborotulos_read(void) // le a linha "ntraj" do cubo desdobrado de rotulos e guarda em "iotim", que eh o numero da trajetoria otima a ser utilizada
{  
  FILE *arquivo_rotulos;
  arquivo_rotulos = fopen("cuborotulos.txt","r");
  int i = 0;
  while (i < ntraj)
  {
    fscanf(arquivo_rotulos, "%i", &iotim);
    i = i + 1;
  }
  fclose(arquivo_rotulos);
}


void cuboiinterc_read(void) // le a linha "ntraj" do cubo desdobrado de instantes de interceptacao e guarda em "iinterc", que eh o elemento a partir do qual a trajetoria otima deve ser considerada
{  
  FILE *arquivo_iinterc;
  arquivo_iinterc = fopen("cuboiinterc.txt","r");
  int i = 0;
  while (i < ntraj)
  {
    fscanf(arquivo_iinterc, "%i", &iinterc);
    iinterc = iinterc - 1;
    i = i + 1;
  }
  fclose(arquivo_iinterc);
}


void traj_read(char nome_arquivo[]) // le a trajetoria de numero "iotim" e guarda os valores a partir do elemento "iinterc" em "traj_t", "traj_h", "traj_V"...
{
  FILE *arquivo_traj;
  arquivo_traj = fopen(nome_arquivo,"r");
  int i = 0;
  int j = 0;
  while (i < n_lines_traj)
  {
    if (i < iinterc) {
      fscanf(arquivo_traj, "%f,%f,%f,%f,%f", &traj_t[0], &traj_h[0], &traj_V[0], &traj_E[0], &traj_xt[0]);
    } else {
      fscanf(arquivo_traj, "%f,%f,%f,%f,%f", &traj_t[j], &traj_h[j], &traj_V[j], &traj_E[j], &traj_xt[j]);
      j = j + 1;
    }
    i = i + 1;
  }
  fclose(arquivo_traj);
}


void t_shift(void) // subtrai o instante inicial "tini" em todos os elementos de "traj_t", para começar em zero
{
  int i = 0;
  while (i < n_lines_util)
  {
    traj_t[i] = traj_t[i] - tini;
    i = i + 1;
  }
}


void h_shift(void) // subtrai a altitude inicial "hini" em todos os elementos de "traj_h", para começar em zero
{
  int i = 0;
  while (i < n_lines_util)
  {
    traj_h[i] = traj_h[i] - hini;
    i = i + 1;
  }
}


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

//intercepta por interpolação da trajetoria
// struct trajetoria //struct para enviar mais de uma valor
// {
//   float t_h;
//   float t_v;
// };
// typedef struct trajetoria Struct;

void interpol(float tempo, float array[])
{
    // Struct s;
    int i       = 0;
    float t1    = 0;
    float t2    = 0;
    float v1    = 0;
    float v2    = 0;
    float h1    = 0;
    float h2    = 0;
    float v_out = 1;
    float h_out = 2;

    float t_max = traj_t[400-2];

    if(tempo > t_max)
    {
      
        
        array[0] = 3;
        array[1] = 4;
    }
    else
    {
        while(traj_t[i] < tempo)
        {
            i = i + 1;
        }

        t1 = traj_t[i-1];
        t2 = traj_t[i];

        v1 = traj_V[i-1];
        v2 = traj_V[i];

        h1 = traj_h[i-1];
        h2 = traj_h[i];

        v_out = v1+(v2-v1)*(tempo-t1)/(t2-t1);
        h_out = h1+(h2-h1)*(tempo-t1)/(t2-t1);

        array[1] = v_out;
        array[0] = h_out;

        

    }
    //printf("h_ref =%6.2f h_traj =%6.2f v_ref =%6.2f v_traj =%6.2f\n",h_plato, arr[0],v_ref,arr[1]);
    //printf("t1 = %6.2f t2 = %6.2f v1 = %6.2f v2 = %6.2f v_out = %6.2f h1 = %6.2f h2 = %6.2f h_out = %6.2f arr0 = %6.2f arr1 = %6.2f\n",t1,t2,v1,v2,v_out,h1,h2,h_out,arr[0],arr[1]);
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

    //TRAJETORIA CUBO
    arestaE_read();  // le os pontos da aresta de Energia e guarda no vetor "aresta_E"
    arestahi_read(); // le os pontos da aresta de altitude (inicial) e guarda no vetor "aresta_hi"
    arestahf_read(); // le os pontos da aresta de altitude final e guarda no vetor "aresta_hf"

    //Valores para teste:
    E   = 0.15;
    h_i = 1000;
    h_f = 300;
    arr[0] = 0;
    arr[1] = 0;
 	
    


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
        leme     = ((0.0015258789)*leme_raw) + 50;

        //TRAJETORIAS DO CUBO
        if(modo_pm_pa == 1 && flag_trajetoria == 0)
        {
        	tempo_atualiza_trajetoria = segundos;
        	flag_trajetoria = 1;
        	E = energia;
        	h_i = h;
    		iE  = coordcubo_find(E, aresta_E, n_lines_aresta_E); // elemento da aresta de Energia mais proximo da Energia atual
    		ihi = coordcubo_find(h_i, aresta_h_i, n_lines_aresta_h_i); // elemento da aresta de altitude incial mais proximo da altitude (inicial) atual
    		ihf = coordcubo_find(h_f, aresta_h_f, n_lines_aresta_h_f); // elemento da aresta de altitude final mais proximo da altitude final (desejada)
        
    		ntraj = iE*n_lines_aresta_h_i*n_lines_aresta_h_f + ihi*n_lines_aresta_h_f + ihf + 1; // elemento do cubo desdobrado que contem as informacoes da trajetoria otima que sera utilizada

    		cuborotulos_read(); // le a linha "ntraj" do cubo desdobrado de rotulos e guarda em "iotim", que eh o numero da trajetoria otima a ser utilizada
    		cuboiinterc_read();  // le a linha "ntraj" do cubo desdobrado de instantes de interceptacao e guarda em "iinterc", que eh o elemento a partir do qual a trajetoria otima deve ser considerada
        
    		//Montando nome do arquivo texto da trajetoria
    		char nome_arquivo_traj[20] = "solmaxalc-";
    		char numero[5];
    		sprintf(numero, "%d", iotim);
    		strcat(nome_arquivo_traj,numero);
    		char sufixo[5] = ".txt";
    		strcat(nome_arquivo_traj,sufixo);
    		printf("%s \n", nome_arquivo_traj);

    		traj_read(nome_arquivo_traj);  // le a trajetoria de numero "iotim" e guarda os valores em "traj_t", "traj_h", "traj_V"... a partir do elemento "iinterc"

    		n_lines_util = n_lines_traj - iinterc; // numero de linhas que serao utilizadas da trajetoria otima
    		tini = traj_t[0]; // valor do tempo no instante a partir do qual a trajetoria otima sera utilizada
    		hini = traj_h[0]; // valor de altitude no instante a partir do qual a trajetoria otima sera utilizada
    		t_shift(); // subtrai o instante inicial "tini" em todos os elementos de "traj_t", para começar em zero
    		h_shift(); // subtrai a altitude inicial "hini" em todos os elementos de "traj_h", para começar em zero

    		// int j_traj = 0;
    		// while(j_traj<400)
    		// {
      // 			printf("t = %6.2f h = %6.2f v = %6.2f\n",traj_t[j_traj],traj_h[j_traj],traj_V[j_traj]);
      // 			j_traj ++;

    		// }


        }
        if(modo_pm_pa == 1 && segundos - tempo_atualiza_trajetoria > 5)
        {
        	flag_atualiza_trajetoria = 1;
        	tempo_atualiza_trajetoria = segundos;
        	pid_h.zera_integrador();
            pid_theta.zera_integrador();
            pid_v.zera_integrador();
        	E = energia;
        	h_i = h;
    		iE  = coordcubo_find(E, aresta_E, n_lines_aresta_E); // elemento da aresta de Energia mais proximo da Energia atual
    		ihi = coordcubo_find(h_i, aresta_h_i, n_lines_aresta_h_i); // elemento da aresta de altitude incial mais proximo da altitude (inicial) atual
    		ihf = coordcubo_find(h_f, aresta_h_f, n_lines_aresta_h_f); // elemento da aresta de altitude final mais proximo da altitude final (desejada)
        
    		ntraj = iE*n_lines_aresta_h_i*n_lines_aresta_h_f + ihi*n_lines_aresta_h_f + ihf + 1; // elemento do cubo desdobrado que contem as informacoes da trajetoria otima que sera utilizada

    		cuborotulos_read(); // le a linha "ntraj" do cubo desdobrado de rotulos e guarda em "iotim", que eh o numero da trajetoria otima a ser utilizada
    		cuboiinterc_read();  // le a linha "ntraj" do cubo desdobrado de instantes de interceptacao e guarda em "iinterc", que eh o elemento a partir do qual a trajetoria otima deve ser considerada
        
    		//Montando nome do arquivo texto da trajetoria
    		char nome_arquivo_traj[20] = "solmaxalc-";
    		char numero[5];
    		sprintf(numero, "%d", iotim);
    		strcat(nome_arquivo_traj,numero);
    		char sufixo[5] = ".txt";
    		strcat(nome_arquivo_traj,sufixo);
    		printf("%s \n", nome_arquivo_traj);

    		traj_read(nome_arquivo_traj);  // le a trajetoria de numero "iotim" e guarda os valores em "traj_t", "traj_h", "traj_V"... a partir do elemento "iinterc"

    		n_lines_util = n_lines_traj - iinterc; // numero de linhas que serao utilizadas da trajetoria otima
    		tini = traj_t[0]; // valor do tempo no instante a partir do qual a trajetoria otima sera utilizada
    		hini = traj_h[0]; // valor de altitude no instante a partir do qual a trajetoria otima sera utilizada
    		t_shift(); // subtrai o instante inicial "tini" em todos os elementos de "traj_t", para começar em zero
    		h_shift(); // subtrai a altitude inicial "hini" em todos os elementos de "traj_h", para começar em zero
    		
    		printf("h %10.2f h_ref = %10.2f  trajetoria %s \n",h, h_plato, nome_arquivo_traj);
        }



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
        profundor     = ((0.0015258789)*profundor_raw) + 50;
        aileron_raw   = (float)axis2[0];
        aileron       = ((0.0015258789)*aileron_raw) + 50;
        manete_raw    = -1*((float)axis2[2]);
        manete        = ((0.0015258789)*manete_raw) + 50;
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
    			fprintf(texto,"%16.6f",h_plato);
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
    //criando struct de trajetoria
    // Struct traj_ref;
    // traj_ref.t_v = 0;
    // traj_ref.t_h = 0;
    //TRAJETORIAS DO CUBO
    

    //SERVOS MOTORES
    servo.initialize(PWM_OUTPUT_1);
    servo.set_frequency(PWM_OUTPUT_1, 50);
    servo.enable(PWM_OUTPUT_1);

    servo.initialize(PWM_OUTPUT_2);
    servo.set_frequency(PWM_OUTPUT_2, 50);
    servo.enable(PWM_OUTPUT_2);

    servo.initialize(PWM_OUTPUT_3);
    servo.set_frequency(PWM_OUTPUT_3, 50);
    servo.enable(PWM_OUTPUT_3);

    servo.initialize(PWM_OUTPUT_4);
    servo.set_frequency(PWM_OUTPUT_4, 50);
    servo.enable(PWM_OUTPUT_4);
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
            arm[0]  = 500000000;
            arm[1]  = 500000000;
            arm[2]  = 500000000;
            arm[3]  = 500000000;
            arm[4]  = 500000000;
            arm[5]  = 500000000;
            arm[6]  = 500000000;
            arm[7]  = 500000000;
            arm[8]  = 500000000;
            arm[9]  = 500000000;
            arm[10] = 500000000;
            arm[11] = 500000000;
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
                    arm[1]  = arm_anterior[1];
                    arm[2]  = arm_anterior[2];
                    arm[3]  = arm_anterior[3];
                    arm[4]  = arm_anterior[4];
                    arm[5]  = arm_anterior[5];
                    arm[6]  = arm_anterior[6];
                    arm[7]  = arm_anterior[7];
                    arm[8]  = arm_anterior[8];
                    arm[9]  = arm_anterior[9];
                    arm[0]  = arm_anterior[0];
                    arm[10] = arm_anterior[10];
                    arm[11] = arm_anterior[11];

                }
                else
                {
                    arm[1]  = arm_novo[1];
                    arm[2]  = arm_novo[2];
                    arm[3]  = arm_novo[3];
                    arm[4]  = arm_novo[4];
                    arm[5]  = arm_novo[5];
                    arm[6]  = arm_novo[6];
                    arm[7]  = arm_novo[7];
                    arm[8]  = arm_novo[8];
                    arm[9]  = arm_novo[9];
                    arm[0]  = arm_novo[0];
                    arm[10] = arm_novo[10];
                    arm[11] = arm_novo[11];

                    arm_anterior[1]  = arm_novo[1];
                    arm_anterior[2]  = arm_novo[2];
                    arm_anterior[3]  = arm_novo[3];
                    arm_anterior[4]  = arm_novo[4];
                    arm_anterior[5]  = arm_novo[5];
                    arm_anterior[6]  = arm_novo[6];
                    arm_anterior[7]  = arm_novo[7];
                    arm_anterior[8]  = arm_novo[8];
                    arm_anterior[9]  = arm_novo[9];
                    arm_anterior[0]  = arm_novo[0];
                    arm_anterior[10] = arm_novo[10];
                    arm_anterior[11] = arm_novo[11];
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
        p       = ((float)arm[0]/1000000)  - 500;
        r       = ((float)arm[1]/1000000)  - 500;
        psi     = ((float)arm[2]/1000000)  - 500;
        phi     = ((float)arm[3]/1000000)  - 500;
        h       = ((float)arm[5]/100000)   - 500;
        theta   = ((float)arm[6]/1000000)  - 500;
        v       = ((float)arm[8]/1000000)  - 500;
        q       = ((float)arm[9]/1000000)  - 500;
        v_bat   = ((float)arm[10]/1000000) - 500;
        i_bat   = ((float)arm[11]/1000000) - 500;

        //CONSUMO DE BATERIA
        if(i_bat <=0)
        {
          i_bat = 0;
        }

        P = v_bat*i_bat*dt/3600; //consumo de bateria.
        P_TOTAL = P_TOTAL + P;
        
        energia = 1 - (P_TOTAL/bateria);
        //printf("V =%6.2f Volts I =%6.2f A P =%10.6f W P_TOTAL =%12.4f Wh Bateria =%5.3f\n",v_bat,i_bat,P,P_TOTAL,energia);

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

            flag_h              = 0;
            flag_h_tela         = 0;
            flag_h2             = 0;
            flag_h_tela2        = 0;
            flag_comandado      = 0;
            time_hold_comandado = 0;
            perturbacao         = 0;
            contador            = 0;
            contador_freq       = 0;

            //delta de comandos!
            delta_prof = interconexao;
            delta_man  = v_pi;
            delta_ail  = comando_aileron;
            flag_trajetoria = 0;
            //printf("ail = %6.2f prof = %6.2f man = %6.2f\n",comando_aileron,interconexao,v_pi);



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
                h_ref          = h;
                h_ref_exp      = h;
                h_init         = h;
                h_ref0         = h;
                //ROTINA PID's
                pid_h.zera_integrador();
                pid_theta.zera_integrador();
                pid_v.zera_integrador();

                interconexao = 0;
                flag_h       = 1;
                flag_h_tela  = 0;

                h_ref_novo = h_ref;
                h_ref_ant  = h_ref;

            }

            //TRAJETORIA PLATO
            //if(segundos - tempo_pa_ativo > 60 && segundos - tempo_pa_ativo < 80)
            
            if(segundos - tempo_pa_ativo > 10)
            {
               float tempo_atual = segundos - tempo_pa_ativo - 10;
               interpol(tempo_atual, arr);
               //printf("h = %8.2f v = %8.2f\n",arr[0], arr[1]);
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
            //PLATO DE SUBIDA E DESCIDA
            h_plato = h_ref + arr[0];

       

            //LÓGICA DO ANTIWINDUP VARIÁVEL
            h_ref_novo = h_plato;
            if(h_ref_novo < h_ref_ant)
            {
                batente_v = 5;
            }
            else
            {
                batente_v = 100;
            }
            h_ref_ant = h_ref_novo;

            // if(h_plato < h_ref)
            // {
            //     h_plato = h_ref;
            // }

            float h_pi = pid_h.calc_pid(h_plato, dt, h,batente_h,0);

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
            if(segundos - tempo_pa_ativo > 50)
            {
               v_ref = arr[1];
            }
            v_ref_exp = (v_ref - delta_v_ref) + (delta_v_ref)*(1 - exp(-(segundos - time_hold_v)/(abs(delta_v_ref)/v_ratio)));
            //ROTINA PID's
            // printf("h = %8.2f v = %8.2f\n",h_plato, v_ref);

            v_filtrado       = f_passa_baixa(v_filtrado_antes,dt, v_ant,1);
            v_filtrado_antes = v_filtrado;
            v_ant            = v;
            erro_v = v_ref_exp - v_filtrado;
            
            v_pi = pid_v.calc_pid(v_ref_exp, dt, v_filtrado,batente_v,2); // 2 significa anti windup do integrador será ativado
            //printf("v_pi antes = %8.2f v_i antes = %8.2f ",v_pi,pid_v.I);
            //v_pi = v_pi - 0.56;
            v_pi = v_pi - delta_man;
            if(v_pi>0)
            {
                v_pi = 0;
            }

            if(v_pi<-1)
            {
                v_pi = -1;
            }


            interconexao = (-v_pi*ganho_interconexao) + theta_e_q;
        	
       		//SUAVIZAÇÃO DO COMANDO DE PROFUNDOR
        	if(segundos - tempo_pa_ativo<30)
        	{
        		//interconexao = interconexao*exp(-5/(1*(segundos- tempo_pa_ativo)));
        		interconexao = interconexao*(1-exp(-(segundos - tempo_pa_ativo)/5));
        	}

        	// if(flag_atualiza_trajetoria == 1)
        	// {
        	// 	if(segundos - tempo_atualiza_trajetoria < )
        	// 	{
        	// 		flag_atualiza_trajetoria == 0;
        	// 	}
        	// 	//interconexao = interconexao*exp(-5/(1*(segundos- tempo_pa_ativo)));
        	// 	interconexao = interconexao*(1-exp(-(segundos - tempo_pa_ativo)/5));
        	// }



        	//
        	perturbacao_comando = 0;
        	//DEGRAU DE COMANDO
        	// if(segundos - tempo_pa_ativo > 80 && segundos - tempo_pa_ativo < 95)
        	// {
        	// 	perturbacao_comando = manobras(80, segundos - tempo_pa_ativo, 10, 2);
        	// }
        	// if(segundos - tempo_pa_ativo > 130 && segundos - tempo_pa_ativo < 145)
        	// {
        	// 	perturbacao_comando = manobras(130, segundos - tempo_pa_ativo, 10, 2);
        	// }
        	// if(segundos - tempo_pa_ativo > 180 && segundos - tempo_pa_ativo < 195)
        	// {
        	// 	perturbacao_comando = manobras(180, segundos - tempo_pa_ativo, 10, 2);
        	// }



            // v_pi            = v_pi + delta_man;
            interconexao    = interconexao + delta_prof + perturbacao_comando;
            comando_aileron = comando_aileron + delta_ail ;
            perturbacao     = perturbacao_comando;

            //LIMITES DE COMANDOS (BATENTES)
            if(comando_aileron  > 15)
            {
                comando_aileron = 15;
            }
            if(comando_aileron  < -30)
            {
                comando_aileron = -30;
            }
            if(interconexao     > 20)
            {
                interconexao    = 20;
            }
            if(interconexao     < -30)
            {
                interconexao    = -30;
            }
            if(comando_leme     > 35)
            {
                comando_leme    = 35;
            }
            if(comando_leme     < -35)
            {
                comando_leme    = -35;
            }
            
            //printf(" v_pi depois = %8.2f v_i depois %8.2f \n",v_pi,pid_v.I);
        ////////////FIM DO CONTROLE////////////
            


        }

        //SERVOS MOTORES
        //conv de angulos para comando pwm :
        //-45º - 45º -> 1000us - 2000us
        pwm_ail  = 1000 + ((comando_aileron+45)*(1000/90));
        pwm_prof = 1000 + ((interconexao+45)*(1000/90));
        pwm_man  = 1000 + ((-1*v_pi)*(1000/1));
        pwm_leme = 1000 + ((comando_leme+45)*(1000/90));

        servo.set_duty_cycle(PWM_OUTPUT_1, pwm_ail);
        servo.set_duty_cycle(PWM_OUTPUT_2, pwm_prof);
        servo.set_duty_cycle(PWM_OUTPUT_3, pwm_man);
        servo.set_duty_cycle(PWM_OUTPUT_4, pwm_leme);
        

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
        //Montando a string de envio                      Bytes
        telemetria+=String("A");                          //01
        telemetria+=String(((comando4)/100000000)%10);    //02
        telemetria+=String(((comando4)/10000000)%10);     //03
        telemetria+=String(((comando4)/1000000)%10);      //04
        telemetria+=String(((comando4)/100000)%10);       //05
        telemetria+=String(((comando4)/10000)%10);        //06
        telemetria+=String(((comando4)/1000)%10);         //07
        telemetria+=String(((comando4)/100)%10);          //08
        telemetria+=String(((comando4)/10)%10);           //09
        telemetria+=String(((comando4)/1)%10);            //10
        telemetria+=",";                                  //11
        telemetria+=String(((comando1)/100000000)%10);    //12
        telemetria+=String(((comando1)/10000000)%10);     //13
        telemetria+=String(((comando1)/1000000)%10);      //14
        telemetria+=String(((comando1)/100000)%10);       //15
        telemetria+=String(((comando1)/10000)%10);        //16
        telemetria+=String(((comando1)/1000)%10);         //17
        telemetria+=String(((comando1)/100)%10);          //18
        telemetria+=String(((comando1)/10)%10);           //19
        telemetria+=String(((comando1)/1)%10);            //20
        telemetria+=",";                                  //21
        telemetria+=String(((comando2)/100000000)%10);    //22
        telemetria+=String(((comando2)/10000000)%10);     //23
        telemetria+=String(((comando2)/1000000)%10);      //24
        telemetria+=String(((comando2)/100000)%10);       //25
        telemetria+=String(((comando2)/10000)%10);        //26
        telemetria+=String(((comando2)/1000)%10);         //27
        telemetria+=String(((comando2)/100)%10);          //28
        telemetria+=String(((comando2)/10)%10);           //29
        telemetria+=String(((comando2)/1)%10);            //30
        telemetria+=",";                                  //31
        telemetria+=String(((comando3)/100000000)%10);    //32
        telemetria+=String(((comando3)/10000000)%10);     //33
        telemetria+=String(((comando3)/1000000)%10);      //34
        telemetria+=String(((comando3)/100000)%10);       //35
        telemetria+=String(((comando3)/10000)%10);        //36
        telemetria+=String(((comando3)/1000)%10);         //37
        telemetria+=String(((comando3)/100)%10);          //38
        telemetria+=String(((comando3)/10)%10);           //39
        telemetria+=String(((comando3)/1)%10);            //40
        telemetria+=String("Z");                          //41
        telemetria+="\n";                                 //42
                                                          //TOTAL 42 BYTES
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
