#ifndef SNAPChannelHardwareSerial_h
#define SNAPChannelHardwareSerial_h

#include "SNAPChannel.h"

class SNAPChannelHardwareSerial : public SNAPChannel {
public:
  SNAPChannelHardwareSerial(HardwareSerial * serial);
  void begin(uint32_t speed);
  int available();
  int read();
  size_t printByte(byte c);
  void printFlush();

private:
  HardwareSerial * serial;
};

#endif // ifndef SNAPChannelHardwareSerial_h
