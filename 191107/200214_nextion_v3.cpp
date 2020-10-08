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
NexVariable flag_piloto = NexVariable(0,63,"flag_piloto");
//NexNumber  n0 = NexNumber(0, 5, "n0");


uint32_t   aa;
float aaa = 560;




int main()
{
    printf("Nextion_in\n");
    nexInit();
    printf("Nextion_out\n");

    while(1)
    {
        flag_piloto.getValue(&aa);
        aaa = (float)aa;
        printf("flag piloto %.0f\n",aaa);
        //n0.getValue(&aa);
        //usleep(20000);
    }
    return 0;
}