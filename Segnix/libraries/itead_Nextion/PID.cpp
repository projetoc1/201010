#include "PID.h"

// PID::PID()
// {
// }
void PID::init()
{
	P  = 0;
	I  = 0;
	D  = 0;
	kp = 0;
	ki = 0;
	kd = 0;
	erro_anterior = 0;
}
void PID::zera_integrador()
{
	I = 0;
}

void PID::ganhos(float _kp, float _ki, float _kd)
{
	kp = _kp;
	ki = _ki;
	kd = _kd;
}

float PID::calc_pid(float _setpoint,float _delta_t, float _leitura, float batente_p, float ativa_limitador)
{
	setpoint = _setpoint;
	delta_t  = _delta_t;
	leitura  = _leitura;

	erro = setpoint - leitura;

	P = erro*kp;
	//rotina de limitador de erro proporcional
	if(ativa_limitador == 1)
	{
		if(P > batente_p)
		{
			P = batente_p;
		}
		if (P < -1*batente_p)
		{
			P = -1*batente_p;
		}
	}

	I = I + erro*delta_t;
	if(ativa_limitador == 2)
	{
		if(I > batente_p)
		{
			I = batente_p;
		}
		if(I < -1*batente_p)
		{
			I = -1*batente_p;
		}
	}

	D = (erro - erro_anterior)/delta_t;

	pid = P + ki*I + kd*D;
	erro_anterior = erro;
	return pid;


}