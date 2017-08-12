#include "SNAPChannelHardwareSerial.h"
#include "DebugUtils.h"

SNAPChannelHardwareSerial::SNAPChannelHardwareSerial(HardwareSerial * serial, long speed) {
  this->serial = serial;
  this->serial->begin(speed);
}

int SNAPChannelHardwareSerial::available() {
  return this->serial->available();
}

int SNAPChannelHardwareSerial::read() {
  return this->serial->read();
}

size_t SNAPChannelHardwareSerial::printByte(byte c) {
  // DEBUG_PRINT(String(c, BIN));
  return this->serial->write(c);
}

void SNAPChannelHardwareSerial::printFlush() {
  this->serial->flush();
}
