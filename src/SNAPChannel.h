#ifndef SNAPChannel_h
#define SNAPChannel_h

#include "Arduino.h"

class SNAPChannel {
public:
  virtual void begin(uint32_t speed) = 0;
  virtual int available() = 0;
  virtual int read()      = 0;
  virtual size_t printByte(byte c) = 0;
  virtual void printFlush()        = 0;
};

#endif // ifndef SNAPChannel_h
