#include "SNAPChannelSoftwareSerial.h"
#include "DebugUtils.h"

SNAPChannelSoftwareSerial::SNAPChannelSoftwareSerial(byte rxPin, byte txPin) {
  this->serial = new SoftwareSerial(rxPin, txPin);
}

void SNAPChannelSoftwareSerial::begin(uint32_t speed) {
  this->serial->begin(speed);
}

int SNAPChannelSoftwareSerial::available() {
  return this->serial->available();
}

int SNAPChannelSoftwareSerial::read() {
  return this->serial->read();
}

size_t SNAPChannelSoftwareSerial::printByte(byte c) {
  return this->serial->write(c);
}

void SNAPChannelSoftwareSerial::printFlush() {
  // nothing to do
}
