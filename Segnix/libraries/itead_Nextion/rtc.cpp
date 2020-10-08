/* Esta biblioteca tem como objetivo implementar  o sensor MS4525DO.
Ete sensor mede pressão diferencial e temperatura.*/

#include "rtc.h"

//Inicilialização do Sensor e calibração de pressão zero



int rtc::update_rtc() 
{
	uint8_t buff[8];
	uint8_t aux_u=0,aux_d=0;
	//uint8_t buffer[1];
	//I2Cdev::readBytes(RTC_ADDRESS,REG_SEG,8,buff,0);
	I2Cdev::writeByte(RTC_ADDRESS,RTC_ADDRESS,0x00);
	I2Cdev::readBytes(RTC_ADDRESS , 0x00, 8, buff);
	dia_da_semana = (int)(buff[3]&0x07);
	aux_d = (buff[0]>>4 & 0x07)*10;
	aux_u =  buff[0]&0x0f;
	seg  = aux_d+aux_u;
	aux_d = (buff[1]>>4 & 0x07)*10;
	aux_u =  buff[1]&0x0f;
	min  = aux_d+aux_u;
	aux_d = (buff[2]>>4 & 0x07)*10;
	aux_u =  buff[2]&0x0f;
	hora = aux_d+aux_u;
	aux_d = (buff[4]>>4 & 0x07)*10;
	aux_u =  buff[4]&0x0f;
	dia =  aux_d+aux_u;
	aux_d = (buff[5]>>4 & 0x07)*10;
	aux_u =  buff[5]&0x0f;
	mes =  aux_d+aux_u;
	aux_d = (buff[6]>>4 & 0x07)*10;
	aux_u =  buff[6]&0x0f;
	ano =  aux_d+aux_u;
	saida[0]=seg;
	saida[1]=min;
	saida[2]=hora;
	saida[3]=dia_da_semana;
	saida[4]=dia;
	saida[5]=mes;
	saida[6]=ano;
}

/** Write single byte to an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register address to write to
 * @param data New byte value to write
 * @return Status of operation (true = success)
 */
void rtc::set_tempo(uint8_t segundo, uint8_t minuto, uint8_t hora1)
{
	// uint8_t aux_d = 0,aux_u = 0;
	// uint8_t ss;
	// uint8_t hh;
	// uint8_t mm;
	// aux_d = (segundo>>4 & 0x07)*10;
	// aux_u =  segundo&0x0f;
	// ss  = aux_d+aux_u;
	I2Cdev::writeByte(RTC_ADDRESS,0x00,segundo);
	I2Cdev::writeByte(RTC_ADDRESS,0x01,minuto);
	I2Cdev::writeByte(RTC_ADDRESS,0x02,hora1);
}

void rtc::set_data(uint8_t dia1, uint8_t mes1, uint8_t ano1)
{
	I2Cdev::writeByte(RTC_ADDRESS,0x04,dia1);
	I2Cdev::writeByte(RTC_ADDRESS,0x05,mes1);
	I2Cdev::writeByte(RTC_ADDRESS,0x06,ano1);
}

int rtc::get_rtc(int *buffer, int count)
{
	for (int i = 0; i < count; ++i)
	{
		buffer[i] = saida[i];
	}
}