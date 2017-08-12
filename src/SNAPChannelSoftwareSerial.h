/*
 * make use of SoftwareSerial sp read limitations on https://www.arduino.cc/en/Reference/SoftwareSerial
 */


#ifndef SNAPChannelSoftwareSerial_h
#define SNAPChannelSoftwareSerial_h

#include "SNAPChannel.h"
#include "SoftwareSerial.h"

class SNAPChannelSoftwareSerial : public SNAPChannel {
public:
  SNAPChannelSoftwareSerial(byte rxPin, byte txPin, long speed);
  int available();
  int read();
  size_t printByte(byte c);
  void printFlush();

private:
  SoftwareSerial * serial;
};

#endif // ifndef SNAPChannelSoftwareSerial_h
