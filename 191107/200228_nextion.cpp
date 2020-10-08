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

int nextion_display = serialOpen("/dev/ttyUSB0",115200);

NexVariable v_ref_nextion = NexVariable(0,15,"vr");
float    v_ref_out, v_ref_out_anterior;
uint32_t v_ref_in;

void fim_comando(void)
{
    nexSerial.write(0xFF);
    nexSerial.write(0xFF);
    nexSerial.write(0xFF);

}

int main()
{
    nexInit();
    while(1)
    {
        String enviar,enviar2;
        enviar = "page0.n0.val=99";
        enviar2 = "page1.n0.val=77";
        const char* conv = enviar.c_str();
        const char* conv2 = enviar2.c_str();
        serialPrintf(nextion_display,conv);
        fim_comando();
        usleep(10);
        serialPrintf(nextion_display,conv2);
        fim_comando();
        serialFlush(nextion_display);
        usleep(200000);
    }
    return 0;
}