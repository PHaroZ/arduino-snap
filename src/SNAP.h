/*
 * SNAP.h - SNAP Communications library for Arduino
 *
 * This library implements easy SNAP based communication
 *
 * History:
 * (0.1) Ported from PIC library by Zach Smith.
 * (0.2) Updated and fixed by the guys from Metalab in Austra (kintel and wizard23)
 * (0.3) Rewrote and refactored all code. Added separate buffers and variables for Rx/Tx by Zach Smith.
 * (1.0) Rewrote and refactored all code for better support of specifications (PHaroZ)
 *
 * License: GPL v2.0
 */

#ifndef SNAP_h
#define SNAP_h

// include types & constants of Wiring core API
// u#include "WConstants.h"
#include "Arduino.h"
#include "SNAPChannel.h"

// broadcast reserved address
#define BROADCAST_ADDRESS 0

// our sync packet value.
#define SNAP_SYNC 0x54 // B01010100

// The defines below are for error checking and such.
// Bit0 is for serialError-flag for checking if an serial error has occured,
//  if set, we will reset the communication
// Bit1 is set if we are currently transmitting a message, that means bytes of
//  a message have been put in the transmitBuffer, but the message is not
//  finished.
// Bit2 is set if we are currently building a send-message
// Bit3 is set if we are busy with the last command and have to abort the message
// Bit4 is set when we have a wrong uartState
// Bit5 is set when we receive a wrong byte
// Bit6 is set if we have to acknowledge a received message
// Bit7 is set if we have received a message for local processing
#define serialErrorBit     B00000001
#define inTransmitMsgBit   B00000010
#define inSendQueueMsgBit  B00000100
#define msgAbortedBit      B00001000
#define wrongStateErrorBit B00010000
#define wrongByteErrorBit  B00100000
#define ackRequestedBit    B01000000
#define processingLockBit  B10000000

// these are the states for processing a packet.
enum SNAP_states {
  SNAP_idle = 0x30,
  SNAP_haveSync,
  SNAP_haveHDB2,
  SNAP_haveHDB1,
  SNAP_haveDAB,
  SNAP_readingData,
  SNAP_dataComplete,
  SNAP_waitForAck
};

template <byte BUFFER_SIZE = 16>
class SNAP {
public:
  SNAP(SNAPChannel * channel, byte address, uint8_t pinTxMode);
  void begin(uint32_t speed);
  void setPinRxDebug(uint8_t pin);

  bool waitForAck();
  bool checkForPacket();
  bool isWaitingForAck();
  bool packetReady();

  byte getDestination();
  byte getSource();
  byte getLength();
  size_t readBytes(byte * arr, size_t length);
  byte getByte(byte index);
  int getInt(byte index); // get 16 bits

  void sendStart(byte to, unsigned long ackWaitTime);
  void sendDataByte(byte c);
  void sendDataInt(int data);
  void sendDataLong(long data);
  void sendMessage();

  void releaseReceive();

private:
  bool receivePacket();
  void receiveByte(byte b);
  void receiveError();
  void transmit(byte c);
  void transmitMessage();
  void transmitStart();
  void transmitEnd();

  // our crc functions.
  byte computeCRC(byte b, byte crc);
  byte computeRxCRC(byte c, bool forceDestForMe = false);
  byte computeTxCRC(byte c);

  // our communication Channel (typically a proxy to HardwareSerial or SoftwareSerial)
  SNAPChannel * channel;
  byte address;
  uint8_t pinTxMode;
  uint8_t pinRxDebug;

  // these are variables for the packet we're currently receiving.
  byte rxState;               // Current SNAP packet state
  byte rxFlags;               // flags for checking status of the serial-communication
  byte rxHDB1;                // 1st header byte
  byte rxHDB2;                // 2nd header byte
  byte rxLength;              // Length of packet being received
  byte rxDestAddress;         // Destination of packet being received (us)
  bool rxDestForMe;           // Is current receive packet for me ? (broadcast or juste for me)
  byte rxSourceAddress;       // Source of packet being received
  byte rxCRC;                 // Incrementally calculated CRC value
  byte rxBufferIndex;         // Current receive buffer index
  byte rxBuffer[BUFFER_SIZE]; // Receive buffer

  // these are the variables for the packet we're currently transmitting.
  byte txHDB1;                           // 1st header byte
  byte txHDB2;                           // 2nd header byte
  byte txLength;                         // transmit packet length
  byte txDestAddress;                    // transmit packet destination
  byte txSourceAddress;                  // transmit packet source (us)
  byte txCRC;                            // incrementally calculated CRC value
  byte txBuffer[BUFFER_SIZE];            // Last packet data, for auto resending on a NAK
  unsigned long txMillis;                // number of ms since last transmission
  unsigned long txAckWaitTime;           // number of ms we are allow to wait for an ACK (0 means no ACK)
  unsigned short txRetryCount;           // number of retries when no ACK received
  const unsigned short txMaxNoRetry = 2; // maximum number of retries when no ACK received
};

#endif // ifndef SNAP_h
