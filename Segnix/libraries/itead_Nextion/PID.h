#ifndef PID_HPP
#define PID_HPP

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

class PID 
{
	public:
		PID()
		{
		}
		void init();
		void zera_integrador();
		void ganhos(float _kp, float _ki, float kd);
		float calc_pid(float _setpoint,float _delta_t, float _leitura, float batente_p, float ativa_limitador);
		float kp,ki,kd;
		float P,I,D;
		float pid;
		float setpoint;
		float leitura;
		float erro;
		float erro_anterior;
		float delta_t;
	private:
};
#endif