//#ifndef ads1115_H
//#define ads1115_H

class ads1115
{
public:
  ads1115();
  int fd_ads;
  uint8_t writeBuf[3];
  uint8_t canal;
  uint8_t ganho;
  uint8_t readBuf[2];
  int16_t val;
  void begin(uint8_t address);
  void set_single_channel(int ch);
  int16_t ads_read(void);
};
//#endif