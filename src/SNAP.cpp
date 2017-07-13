#include "Arduino.h"
#include "SNAP.h"
#include "DebugUtils.h"

template <byte BUFFER_SIZE> SNAP<BUFFER_SIZE>::SNAP(SNAPChannel * channel, byte address) {
  this->channel = channel;
  this->address = address;

  // init our rx valuesSNAPChannel
  this->rxState         = SNAP_idle;
  this->rxFlags         = 0;
  this->rxHDB1          = 0;
  this->rxHDB2          = 0;
  this->rxLength        = 0;
  this->rxDestAddress   = 0;
  this->rxDestForMe     = false;
  this->rxSourceAddress = 0;
  this->rxCRC         = 0;
  this->rxBufferIndex = 0;

  // clear our rx buffer.
  for (byte i = 0; i < BUFFER_SIZE; i++)
    this->rxBuffer[i] = 0;

  // init our tx values
  this->txDestAddress   = 0;
  this->txSourceAddress = 0;
  this->txLength        = 0;
  this->txHDB2        = 0;
  this->txCRC         = 0;
  this->txAckWaitTime = 0;
  this->txRetryCount  = 0;

  // clear our tx buffer.
  for (byte i = 0; i < BUFFER_SIZE; i++)
    this->txBuffer[i] = 0;
}

template <byte BUFFER_SIZE> bool SNAP<BUFFER_SIZE>::waitForAck() {
  if (this->txAckWaitTime > 0) {
    DEBUG_PRINT("waint ack");
    // we are waiting for an ACK
    if (this->receivePacket()) {
      // ACK just received ?
      if (this->rxHDB2 & B00000010) {
        DEBUG_PRINT("#" + String(this->address) + " ACK received in " + String(millis() - this->txMillis) + "ms");
        // YEAH !! it's an ACK response
        this->txAckWaitTime = 0;
        this->releaseLock();
        return false;
      } else if (this->rxHDB2 & B00000011) {
        DEBUG_PRINT("#" + String(this->address) + " received NACK in " + String(
            millis() - this->txMillis) + "ms, resending");
        // it's a NACK response, so re-transmit the message
        transmitMessage();
        return true;
      } else {
        DEBUG_PRINT("#" + String(this->address) + " received something that is not an ACK nor a NACK");
        // it's something else, this should not occurs, ignore it
        this->rxFlags |= msgAbortedBit;
        this->receiveError();
        return true;
      }
    } else {
      // no ACK available
      if (millis() - this->txMillis > this->txAckWaitTime) {
        // fail to get an ACK within the time limit, retry to send it if possible
        if (this->txRetryCount < this->txMaxNoRetry) {
          DEBUG_PRINT("#" + String(this->address) + " wait too much time (" + String(
              millis() - this->txMillis) + "ms) for ACK/NACK, resending");
          this->txRetryCount++;
          transmitMessage();
          return true;
        } else {
          DEBUG_PRINT("#" + String(this->address) + " give up the transmission");
          // we have done all what can be done, give up the transmission and the ACK
          this->txAckWaitTime = 0;
          return false;
        }
      } else {
        // continue to wait for an ACK
        return true;
      }
    }
  } else {
    // NO ACK waiting
    return false;
  }
} // waitForAck

template <byte BUFFER_SIZE> bool SNAP<BUFFER_SIZE>::receivePacket() {
  byte cmd;

  while (this->channel->available() > 0 && !this->packetReady()) {
    cmd = this->channel->read();
    // DEBUG_PRINT("#" + String(this->address) + " received something " + String(cmd, BIN));
    this->receiveByte(cmd);
  }

  return this->packetReady();
}

template <byte BUFFER_SIZE> void SNAP<BUFFER_SIZE>::receiveByte(byte c) {
  // DEBUG_PRINT("#" + String(this->address) + " {{");
  if (this->rxFlags & serialErrorBit) {
    this->receiveError();
    // DEBUG_PRINT("#" + String(this->address) + " receiveError");
    return;
  }


  // DEBUG_PRINT("#" + String(this->address) + " rxState=" + String(this->rxState, HEX));
  switch (this->rxState) {
      case SNAP_idle:
        // In the idle state, we wait for a sync byte.  If none is
        // received, we remain in this state.
        if (c == SNAP_SYNC) {
          this->rxState  = SNAP_haveSync;
          this->rxFlags &= ~msgAbortedBit; // clear
        }
        break;

      case SNAP_haveSync:
        // In this state we are waiting for header definition bytes. First
        // HDB2.  We currently insist that all packets meet our expected
        // format which is 1 byte destination address, 1 byte source
        // address, and no protocol specific bytes.  The ACK/NAK bits may
        // be anything.
        this->rxHDB2 = c;
        if ((c & B11111100) != B01010000) {
          // Unsupported header.  Drop it an reset
          this->rxFlags |= serialErrorBit; // set serialError
          this->rxFlags |= wrongByteErrorBit;
          this->receiveError();
        }
        // All is well
        else {
          // do we want ack?
          if ((c & B00000011) == B00000001)
            this->rxFlags |= ackRequestedBit;  // set ackRequested-Bit
          else
            this->rxFlags &= ~ackRequestedBit;  // clear
          this->rxCRC = 0;

          this->computeRxCRC(c, true);

          this->rxState = SNAP_haveHDB2;
        }
        break;

      case SNAP_haveHDB2:
        // For HDB1, we insist on high bits are 0011 and low bits are the length
        // of the payload.
        this->rxHDB1 = c;
        if ((c & B11110000) != B00110000) {
          this->rxFlags |= serialErrorBit; // set serialError
          this->rxFlags |= wrongByteErrorBit;
          this->receiveError();
        } else {
          this->rxLength = c & 0x0f;

          if (this->rxLength > BUFFER_SIZE) {
            this->rxLength = BUFFER_SIZE;
          }

          this->computeRxCRC(c, true);

          this->rxState = SNAP_haveHDB1;
        }
        break;

      case SNAP_haveHDB1:
        // save our address, as we may have multiple addresses on one arduino.
        this->rxDestAddress = c;
        this->rxDestForMe   = (c == BROADCAST_ADDRESS) || (c == this->address);

        this->computeRxCRC(c);
        this->rxState = SNAP_haveDAB;
        break;

      case SNAP_haveDAB:

        /*
         * // this may not be required.... we check this flag before accepting new packets...
         * if (this->rxFlags & processingLockBit)
         * {
         * this->rxCRC = 0;
         *
         * //we have not finished the last order, reject (send a NAK)
         * this->transmit(SNAP_SYNC);
         * this->transmit(computeRxCRC(B01010011));        //HDB2: NAK
         * this->transmit(computeRxCRC(B00110000));        // HDB1: 0 bytes, with 8 bit CRC
         * this->transmit(computeRxCRC(this->rxSourceAddress));        // Return to sender
         * this->transmit(computeRxCRC(this->rxDestAddress));        // From us
         * this->transmit(this->rxCRC);  // CRC
         *
         * this->rxFlags &= ~ackRequestedBit; //clear
         * this->rxFlags |= msgAbortedBit; //set
         *
         * this->rxState = SNAP_idle;
         * }
         */

        this->rxSourceAddress = c;
        this->rxBufferIndex   = 0;
        this->computeRxCRC(c);

        this->rxState = 0 == this->rxLength ? SNAP_dataComplete : SNAP_readingData;
        break;

      case SNAP_readingData:
        if (this->rxDestForMe) {
          rxBuffer[this->rxBufferIndex] = c;
        }

        this->computeRxCRC(c);

        this->rxBufferIndex++;
        if (this->rxBufferIndex == this->rxLength)
          this->rxState = SNAP_dataComplete;
        break;

      case SNAP_dataComplete:
        // check CRC & send ACK if packet is for me
        if (this->rxDestForMe) {
          // DEBUG_PRINT("#" + String(this->address) + " it's for me");
          byte hdb2 = B01010000; // 1 byte addresses

          // We should be receiving a CRC after data, and it
          // should match what we have already computed
          if (c == this->rxCRC) {
            // DEBUG_PRINT("#" + String(this->address) + " CRC ok");
            // All is good, so process the command.  Rather than calling the
            // appropriate function directly, we just set a flag to say
            // something is ready for processing.  Then in the main loop we
            // detect this and process the command.  This allows further
            // comms processing (such as passing other tokens around the
            // ring) while we're actioning the command.

            hdb2 |= B00000010;
            this->rxFlags |= processingLockBit; // set processingLockBit
          }
          // CRC mismatch, so we will NAK the packet
          else {
            // DEBUG_PRINT("#" + String(this->address) + " CRC fail : " + String(c, BIN) + "!=" + String(this->rxCRC, BIN));
            hdb2 |= B00000011;
          }

          // Send ACK or NAK back to source
          // TODO Send nack if (hdb2 & B00000011)
          if (this->rxFlags & ackRequestedBit) {
            // DEBUG_PRINT("#" + String(this->address) + " send ack {{");
            this->transmitStart();
            this->transmit(SNAP_SYNC);
            this->txCRC = 0;
            this->transmit(this->computeTxCRC(hdb2));
            this->transmit(this->computeTxCRC(B00110000));             // HDB1: 0 bytes, with 8 bit CRC
            this->transmit(this->computeTxCRC(this->rxSourceAddress)); // Return to sender
            this->transmit(this->computeTxCRC(this->rxDestAddress));   // From us
            this->transmit(this->txCRC);                               // CRC
            this->transmitEnd();                                       // transmission end
            // DEBUG_PRINT("#" + String(this->address) + " }} send ack");
            this->rxFlags &= ~ackRequestedBit; // clear
          }
        }

        this->rxState = SNAP_idle;
        break;

      default:
        this->rxFlags |= serialErrorBit; // set serialError
        this->rxFlags |= wrongStateErrorBit;
        this->receiveError();
  }
  // DEBUG_PRINT("#" + String(this->address) + " }}");
} // receiveByte

template <byte BUFFER_SIZE> void SNAP<BUFFER_SIZE>::receiveError() {
  // init our rx values
  this->rxState         = SNAP_idle;
  this->rxFlags         = 0;
  this->rxHDB1          = 0;
  this->rxHDB2          = 0;
  this->rxLength        = 0;
  this->rxDestAddress   = 0;
  this->rxSourceAddress = 0;
  this->rxCRC         = 0;
  this->rxBufferIndex = 0;

  // clear our rx buffer.
  for (byte i = 0; i < BUFFER_SIZE; i++)
    this->rxBuffer[i] = 0;
}

template <byte BUFFER_SIZE> void SNAP<BUFFER_SIZE>::sendStart(byte to, unsigned long ackWaitTime) {
  // DEBUG_PRINT("#" + String(this->address) + " {{");

  // initialize our addresses.
  this->txDestAddress   = to;
  this->txSourceAddress = this->address;

  // set ack wait time
  this->txAckWaitTime = ackWaitTime;

  // initalize our variables.
  this->txLength = 0;
  this->txHDB2   = B01010000;
  if (ackWaitTime > 0) {
    this->txHDB2 |= B00000001;
  }
  this->txCRC = 0;

  // clear our buffer.
  for (byte i = 0; i < BUFFER_SIZE; i++)
    this->txBuffer[i] = 0;

  // DEBUG_PRINT("#" + String(this->address) + " }}");
}

/*!
 * High level routine that queues a byte during construction of a packet.
 */
template <byte BUFFER_SIZE> void SNAP<BUFFER_SIZE>::sendDataByte(byte c) {
  // Put byte into packet sending buffer.  Don't calculated CRCs
  // yet as we don't have complete information.

  // Drop if trying to send too much
  if (this->txLength >= BUFFER_SIZE)
    return;

  this->txBuffer[this->txLength] = c;
  this->txLength++;
}

template <byte BUFFER_SIZE> void SNAP<BUFFER_SIZE>::sendDataInt(int i) {
  this->sendDataByte(i & 0xff);
  this->sendDataByte(i >> 8);
}

template <byte BUFFER_SIZE> void SNAP<BUFFER_SIZE>::sendDataLong(long i) {
  this->sendDataByte(i & 0xff);
  this->sendDataByte(i >> 8);
  this->sendDataByte(i >> 16);
  this->sendDataByte(i >> 24);
}

/*!
 * Create headers and synchronously transmit the message.
 */
template <byte BUFFER_SIZE> void SNAP<BUFFER_SIZE>::sendMessage() {
  // DEBUG_PRINT("#" + String(this->address) + " {{");

  byte txNDB;

  { // adjust size to SNAP spec
    byte realLength;
    if (this->txLength <= 8) {
      realLength = this->txLength;
      txNDB      = txLength;
    } else if (this->txLength <= 16) {
      realLength = 16;
      txNDB      = B00001001;
    } else if (this->txLength <= 32) {
      realLength = 32;
      txNDB      = B00001010;
    } else if (this->txLength <= 64) {
      realLength = 64;
      txNDB      = B00001011;
    } else if (this->txLength <= 128) {
      realLength = 128;
      txNDB      = B00001100;
    } else if (this->txLength <= 256) {
      realLength = 256;
      txNDB      = B00001101;
    } else if (this->txLength <= 512) {
      realLength = 512;
      txNDB      = B00001110;
    } else {
      realLength = this->txLength;
      txNDB      = B00001111;
    }
    for (byte i = this->txLength; i < realLength; i++) {
      this->sendDataByte(B00000000);
    }
  }

  this->txHDB1 = B00110000 | txNDB;

  this->txRetryCount = 0;

  { // compute CRC
    this->txCRC = 0;
    this->computeTxCRC(this->txHDB2);
    this->computeTxCRC(this->txHDB1);
    this->computeTxCRC(this->txDestAddress);
    this->computeTxCRC(this->txSourceAddress);
    for (byte i = 0; i < this->txLength; i++) {
      this->computeTxCRC(this->txBuffer[i]);
    }
  }

  this->transmitMessage();

  // DEBUG_PRINT("#" + String(this->address) + " }}")
} // sendMessage

template <byte BUFFER_SIZE> bool SNAP<BUFFER_SIZE>::packetReady() {
  return (this->rxFlags & processingLockBit);
}

/*!
 * Must be manually called by the main loop when the message payload
 * has been consumed.
 */
template <byte BUFFER_SIZE> void SNAP<BUFFER_SIZE>::releaseLock() {
  // init our rx values
  this->rxState         = SNAP_idle;
  this->rxFlags         = 0;
  this->rxHDB1          = 0;
  this->rxHDB2          = 0;
  this->rxLength        = 0;
  this->rxDestAddress   = 0;
  this->rxSourceAddress = 0;
  this->rxCRC         = 0;
  this->rxBufferIndex = 0;

  // clear our rx buffer.
  for (byte i = 0; i < BUFFER_SIZE; i++)
    this->rxBuffer[i] = 0;
}

template <byte BUFFER_SIZE> void SNAP<BUFFER_SIZE>::transmitMessage() {
  this->transmitStart();

  // here is our header.
  this->transmit(SNAP_SYNC);
  this->transmit(this->txHDB2);          // HDB2
  this->transmit(this->txHDB1);          // HDB1
  this->transmit(this->txDestAddress);   // Destination
  this->transmit(this->txSourceAddress); // Source (us)

  // payload.
  for (byte i = 0; i < this->txLength; i++) {
    this->transmit(this->txBuffer[i]);
  }

  // CRC
  this->transmit(this->txCRC);

  this->transmitEnd();

  this->txMillis = millis();
}

template <byte BUFFER_SIZE> void SNAP<BUFFER_SIZE>::transmit(byte c) {
  this->channel->printByte(c);
}

template <byte BUFFER_SIZE> void SNAP<BUFFER_SIZE>::transmitStart() { }

template <byte BUFFER_SIZE> void SNAP<BUFFER_SIZE>::transmitEnd() {
  this->channel->printFlush();
}

/*!
 * Incrementally adds b to crc computation and updates crc.
 * returns \c.
 */
template <byte BUFFER_SIZE> byte SNAP<BUFFER_SIZE>::computeCRC(byte b, byte crc) {
  byte i = b ^ crc;

  crc = 0;

  if (i & 1) crc ^= 0x5e;
  if (i & 2) crc ^= 0xbc;
  if (i & 4) crc ^= 0x61;
  if (i & 8) crc ^= 0xc2;
  if (i & 0x10) crc ^= 0x9d;
  if (i & 0x20) crc ^= 0x23;
  if (i & 0x40) crc ^= 0x46;
  if (i & 0x80) crc ^= 0x8c;

  return crc;
}

template <byte BUFFER_SIZE> byte SNAP<BUFFER_SIZE>::computeRxCRC(byte b, bool forceDestForMe) {
  if (forceDestForMe || this->rxDestForMe) {
    this->rxCRC = this->computeCRC(b, this->rxCRC);
  }
  return b;
}

template <byte BUFFER_SIZE> byte SNAP<BUFFER_SIZE>::computeTxCRC(byte b) {
  this->txCRC = this->computeCRC(b, this->txCRC);

  return b;
}

template <byte BUFFER_SIZE> byte SNAP<BUFFER_SIZE>::getDestination() {
  return this->rxDestAddress;
}

template <byte BUFFER_SIZE> byte SNAP<BUFFER_SIZE>::getByte(byte index) {
  return this->rxBuffer[index];
}

template <byte BUFFER_SIZE> int SNAP<BUFFER_SIZE>::getInt(byte index) {
  return (this->rxBuffer[index + 1] << 8) + this->rxBuffer[index];
}
