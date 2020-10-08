#ifndef rtc_HPP
#define rtc_HPP

#include "I2Cdev.h"
#include<stdio.h>
#include<stdlib.h>
#include<unistd.h>


#define RTC_ADDRESS     0x68
#define REG_SEG         0x00
#define REG_MIN         0x01
#define REG_HOR         0x02
#define REG_DOW         0x03
#define REG_DIA         0x04
#define REG_MES         0x05
#define REG_ANO         0x06

class rtc {
    public:    

        void set_tempo(uint8_t segundo, uint8_t minuto, uint8_t hora1);
        void set_data(uint8_t dia1, uint8_t mes1, uint8_t ano1);
	    int  update_rtc();
	    int  get_rtc(int *buffer, int count);
	    uint8_t seg;
	    uint8_t min;
	    uint8_t hora;
	    uint8_t dia_da_semana;
	    uint8_t dia;
	    uint8_t mes;
	    uint8_t ano;
	    uint8_t saida[7];


    private:
	    
	    
};




#endif //rtc ds1307 fim do if