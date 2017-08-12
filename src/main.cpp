#include "Arduino.h"
#include "SNAP.cpp" // TODO we should include .h not .cpp
#include "SNAPChannel.h"
#include "SNAPChannelHardwareSerial.h"

#include "DebugUtils.h"

const byte snapAddressMaster = 1;
const byte snapAddressSlave  = 2;

SNAPChannelHardwareSerial snapChannelMaster = SNAPChannelHardwareSerial(&Serial1);
SNAP<16> snapMaster = SNAP<16>(&snapChannelMaster, snapAddressMaster, 13u);

SNAPChannelHardwareSerial snapChannelSlave = SNAPChannelHardwareSerial(&Serial2);
SNAP<16> snapSlave = SNAP<16>(&snapChannelSlave, snapAddressSlave, -1);

void setup() {
  Serial.begin(57600); // debug

  Serial1.begin(57600);

  Serial2.begin(57600);
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

  if (snapMaster.waitForAck()) {
    // do nothing ?
  } else {
    if (snapMaster.receivePacket()) {
      Serial.print("master getBytes :");
      for (byte i = 0; i < 16; i++) {
        Serial.print(' ');
        Serial.print(snapMaster.getByte(i), BIN);
      }
      Serial.println('.');
      snapMaster.releaseLock();
    }
  }

  if (snapSlave.receivePacket()) {
    Serial.print("slave getBytes :");
    for (byte i = 0; i < 16; i++) {
      Serial.print(' ');
      Serial.print(snapSlave.getByte(i), BIN);
    }
    Serial.println('.');
    snapSlave.releaseLock();
  }
} // loop
