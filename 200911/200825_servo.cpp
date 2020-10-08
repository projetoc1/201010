#include "RCOutput_Navio2.h"
#include "PWM.h"
#include <unistd.h>

#define PWM_OUTPUT_1 0
#define PWM_OUTPUT_2 1
#define PWM_OUTPUT_3 2
#define PWM_OUTPUT_4 3

//PWM 
RCOutput_Navio2 servo1;
int contador = 1000;
int sinal = 1;
int comando = 1000;
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

    while(1)
    {
    	if(contador>2000)
    	{
    		contador = 2000;
    		sinal = -1;
    	}
    	else if (contador<1000)
    	{
    		contador = 1000;
    		sinal = 1;
    	}
    	else
    	{
    		contador = contador + sinal;
    	}
    	// contador = 2000;
    	// servo1.set_duty_cycle(PWM_OUTPUT_1, contador);
    	// usleep(2000000);

    	// contador = 1000;
    	// servo1.set_duty_cycle(PWM_OUTPUT_1, contador);
    	// usleep(2000000);
    	



    	servo1.set_duty_cycle(PWM_OUTPUT_1, contador);
    	servo1.set_duty_cycle(PWM_OUTPUT_2, 1000);
    	servo1.set_duty_cycle(PWM_OUTPUT_3, 1000);
    	servo1.set_duty_cycle(PWM_OUTPUT_4, 1000);
    	usleep(100);
    }

	return 0;
}