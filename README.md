Arduino implementation of SNAP protocol ( http://www.hth.com/snap/ ).

Based on alx work ( https://github.com/alx/reprap-arduino-firmware/tree/master/library/SNAP )


## Description
This implemention of SNAP protocol is compatible for :
 * master or slave node
 * HardwareSerial and SoftwareSerial
 * hardware that need a pin for controlling txMode (such as MAX485 chip). In example above set txPin to -1 if you don't need this feature.

## HardwareSerial example
```cpp
// create a channel based on HardwareSerial, use Serial3
SNAPChannelHardwareSerial snapChannelMaster = SNAPChannelHardwareSerial(&Serial3);
// create a SNAP object with a buffer of 16 bytes, SNAP address of this node is 1, use pin 24 as txMode pin (this pin is set to HIGH when transmitting)
SNAP<16> snapMaster = SNAP<16>(&snapChannelMaster, 1, 24);

void setup() {
  // init channel at 57600 bauds
  snapChannelMaster.begin(57600);
}
```

## SoftwareSerial example
First of all, read SoftwareSerial limitations on https://www.arduino.cc/en/Reference/SoftwareSerial
```cpp
// create a channel based on SoftwareSerial, use pin 10 as rxPin, pin 11 as txPin
SNAPChannelSoftwareSerial snapChannelSlave = SNAPChannelSoftwareSerial(10, 11);
// create a SNAP object with a buffer of 16 bytes, SNAP address of this node is 2, use pin 26 as txMode pin
SNAP<16> snapSlave = SNAP<16>(&snapChannelSlave, 2, 26);

void setup() {
  // init channel at 57600 bauds
  snapChannelSlave.begin(57600);
}
```

## Send data
```cpp
// declare a new message to destination node 2, this node have 20ms to send back an ACK
snapMaster.sendStart(2, 20);
// first byte to send
snapMaster.sendDataByte(B01010000);
// second byte to send
snapMaster.sendDataByte(B00000010);
// send as much as you want in the limit of 16 bytes (the buffer size defined earlier)
// send message now
snapMaster.sendMessage();

// wait for ACK
snapMaster.waitForAck()
// if ACK is not received in the limit on 20ms (defined earlier) or if NACK is received, message will be resend up to 2 times
```

## Receive message
```cpp
// test if new message is available
if (snapSlave.checkForPacket()) {
	// loop other all buffer size
	for (byte i = 0; i < 16; i++) {
    	// read a byte
        snapSlave.getByte(i);
    }
    // clear buffered message and signal to snapSlave that it can receive another message
    snapSlave.releaseReceive();
}
```

## Complete example
For a complete example see examples/allInOne/main.cpp : a single arduino MEGA wich communicate with itself over its differents Serial.
