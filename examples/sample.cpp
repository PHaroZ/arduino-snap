// #define DEBUG

#include "Arduino.h"
#include "SNAP.h"
#include "SNAPChannel.h"
#include "SNAPChannelHardwareSerial.h"
#include "SNAPChannelSoftwareSerial.h"

#include "DebugUtils.h"

const byte snapAddressMaster = 1;
const byte snapAddressSlave  = 2;

SNAPChannelHardwareSerial snapChannelMaster = SNAPChannelHardwareSerial(&Serial3);
SNAP<16> snapMaster = SNAP<16>(&snapChannelMaster, snapAddressMaster, 24);

SNAPChannelSoftwareSerial snapChannelSlave = SNAPChannelSoftwareSerial(10, 11);
// SNAPChannelHardwareSerial snapChannelSlave = SNAPChannelHardwareSerial(&Serial2);
SNAP<16> snapSlave = SNAP<16>(&snapChannelSlave, snapAddressSlave, 26);

void setup() {
  Serial.begin(250000); // debug

  snapChannelMaster.begin(57600);
  snapChannelSlave.begin(57600);
}

void loop() {
  if (Serial.available() > 0) {
    long startAt = millis();
    snapMaster.sendStart(snapAddressSlave, 100);
    byte toSend;
    while (Serial.available() > 0 || (millis() - startAt) < 1000) {
      if (Serial.available() > 0) {
        toSend = Serial.read();
        snapMaster.sendDataByte(toSend);
      }
    }
    snapMaster.sendMessage();
  }

  if (snapMaster.isWaitingForAck()) {
    // do nothing ?
  } else {
    if (snapMaster.receivePacket()) {
      Serial.print("master getBytes :");
      for (byte i = 0; i < 16; i++) {
        Serial.print(' ');
        Serial.print(snapMaster.getByte(i), BIN);
      }
      Serial.println('.');
      snapMaster.releaseReceive();
    }
  }

  if (snapSlave.receivePacket()) {
    Serial.print(millis());
    Serial.print(" slave getBytes :");
    for (byte i = 0; i < 16; i++) {
      Serial.print(' ');
      Serial.print(snapSlave.getByte(i), BIN);
    }
    Serial.println('.');
    snapSlave.releaseReceive();
  }
} // loop
