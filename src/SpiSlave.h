#pragma once

#include <cstdlib>
#include <cstdint>

struct SpiIrqCount {
  uint16_t ERR = 0;
  uint16_t SSL = 0;
  uint16_t RXC = 0;
  uint16_t TXC = 0;
  uint16_t DRE = 0;
};

#define SPIDEBUG
#ifdef SPIDEBUG
extern volatile SpiIrqCount irqcount;
#endif

class SpiSlave
{
public:
  typedef uint32_t Buffer;

  void begin(Buffer* _ptrToSendBuf, unsigned int chipSelectDelay);
  void begin(Buffer* _ptrToSendBuf);

  Buffer getReceivedData() const;
  unsigned long getLastReceiveTime() const;
  unsigned long getLastSelectTime() const;

  void onTransmissionStart();
  void onTransmissionDone();
  void onReadyForNewData();


  const uint8_t dataLen = sizeof(Buffer);
  volatile Buffer mRecvBuffer = {};
  volatile size_t mRecvIdx = 0;
  volatile Buffer mSendBuffer = {};
  volatile size_t mSendIdx = 0;
  volatile Buffer mDoneRecvBuffer = {};
  Buffer* mPtrToSendBuf = NULL;
  unsigned long mLastReceive = 0;
  unsigned long mLastChipSelect = 0;
  unsigned int mChipSelectCount = 0;
  unsigned int mChipSelectDelay = 0;
};

extern SpiSlave spi;