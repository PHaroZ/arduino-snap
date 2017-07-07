#include "SNAPChannelHardwareSerial.h"

SNAPChannelHardwareSerial::SNAPChannelHardwareSerial(HardwareSerial * serial) {
  this->serial = serial;
}

int SNAPChannelHardwareSerial::available() {
  return this->serial->available();
}

int SNAPChannelHardwareSerial::read() {
  return this->serial->read();
}

size_t SNAPChannelHardwareSerial::printByte(byte c) {
  return this->serial->write(c);
}

void SNAPChannelHardwareSerial::printFlush() {
  this->serial->flush();
}
