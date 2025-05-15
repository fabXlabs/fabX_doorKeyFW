#include "SpiSlave.h"
#include "pinconfig.h"
#include <Arduino.h>

SpiSlave spi;

static const uint8_t SPI_SS   = PIN_PA13; 
static const uint8_t SPI_MOSI = PIN_PB10;
static const uint8_t SPI_MISO = PIN_PA12;
static const uint8_t SPI_SCK  = PIN_PB11;

#ifdef SPIDEBUG
volatile SpiIrqCount irqcount;
#endif

// this is attached to an interrupt on the chip select signal
// but only before the actual SPI is configured
void onChipSelectInPrePhase(void) {
  spi.mChipSelectCount++;
#ifdef SPIDEBUG
  irqcount.SSL++;
#endif
  if (spi.mChipSelectCount > spi.mChipSelectDelay) {
    EIC->INTENCLR.reg = EIC_INTENSET_EXTINT13;
    spi.begin(spi.mPtrToSendBuf);
  }
}

/**
 * Setup for SPI Slave mode with given send buffer.
 * If chipSelectDelay is > 0 it defines the number of falling edges on the chip select signal
 * are waited before the SPI slave is actually configured.
 * This allows ensuring the SPI, especially the CS line is actually setup on the master side,
 * otherwise we might end up answering to transmission for another chip.
 */
void SpiSlave::begin(Buffer* _ptrToSendBuf, unsigned int chipSelectDelay)
{
  if (chipSelectDelay == 0) return begin(_ptrToSendBuf);

  mPtrToSendBuf = _ptrToSendBuf;
  mChipSelectDelay = chipSelectDelay;
  // PIN_SS (38 maps to PA13 in variant.cpp (why?))
  pinMode(38, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(38), onChipSelectInPrePhase, FALLING);
}

void SpiSlave::begin(Buffer* _ptrToSendBuf)
{
  mPtrToSendBuf = _ptrToSendBuf;
  mChipSelectDelay = 0;
  // Set pin modes
  // Set PB10 as input   (MOSI)     0    PB10 corresponds to: PORTB, PMUX[5], Even
  PORT->Group[PORTB].PINCFG[10].bit.PMUXEN = 0x1; // Enable Peripheral Multiplexing for SERCOM4 SPI PB12
  PORT->Group[PORTB].PMUX[5].bit.PMUXE = 0x3; // SERCOM4 is selected for peripheral use of this pad (0x3 selects peripheral function D: SERCOM-ALT)  
  // Set PB11 as input   (SCK)      1    PB11 corresponds to: PORTB, PMUX[5], Odd
  PORT->Group[PORTB].PINCFG[11].bit.PMUXEN = 0x1; // Enable Peripheral Multiplexing for SERCOM4 SPI PB13
  PORT->Group[PORTB].PMUX[5].bit.PMUXO = 0x3; // SERCOM4 is selected for peripheral use of this pad (0x3 selects peripheral function D: SERCOM-ALT)
  // Set PA13 as input   (SS)       2    PA13 corresponds to: PORTA, PMUX[6], Odd
  PORT->Group[PORTA].PINCFG[13].bit.PMUXEN = 0x1; // Enable Peripheral Multiplexing for SERCOM4 SPI PB14
  PORT->Group[PORTA].PMUX[6].bit.PMUXO = 0x3; // SERCOM4 is selected for peripheral use of this pad (0x3 selects peripheral function D: SERCOM-ALT)
  // Set PA12 as output  (MISO)     3    PA12 corresponds to: PORTA, PMUX[6], Even
  PORT->Group[PORTA].PINCFG[12].bit.PMUXEN = 0x1; // Enable Peripheral Multiplexing for SERCOM4 SPI PB15
  PORT->Group[PORTA].PMUX[6].bit.PMUXE = 0x3; // SERCOM4 is selected for peripheral use of this pad (0x3 selects peripheral function D: SERCOM-ALT)

	// Disable SPI 1
  SERCOM4->SPI.CTRLA.bit.ENABLE = 0; // page 481
  while (SERCOM4->SPI.SYNCBUSY.bit.ENABLE); // Wait until bit is enabled.
  
  // Reset SPI 1
  SERCOM4->SPI.CTRLA.bit.SWRST = 1; // page 481
  while (SERCOM4->SPI.CTRLA.bit.SWRST || SERCOM4->SPI.SYNCBUSY.bit.SWRST); // Wait until software reset is complete.
  
  // Setting up Nested Vectored Interrupt Controller (NVIC) and Generic Clock Controller
  NVIC_EnableIRQ(SERCOM4_IRQn);
	//NVIC_SetPriority(SERCOM4_IRQn, (1 << __NVIC_PRIO_BITS) - 1);
  NVIC_SetPriority(SERCOM4_IRQn, 0);

  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(GCM_SERCOM4_CORE) | // Generic Clock 0
                      GCLK_CLKCTRL_GEN_GCLK0 | // Generic Clock Generator 0 is the source
                      GCLK_CLKCTRL_CLKEN; // Enable Generic Clock Generator

  while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY); // Wait for synchronisation

  // Setting the CTRLA register
  //  - MISO: PAD[0], PA12, D21
  //  - MOSI: PAD[2], PB10, D19
  //  - SCK:  PAD[3], PB11, D20
  //  - SS:   PAD[1], PA13, D22

	// For slave mode, information for DIPO/DOPO page 493-494 translates to
	// DOPO   DIPO     MOSI/DI    MISO/DO    SCK     SS
	// 0x0    0x3      3          0          1       2
	// 0x1    0x0      0          2          3       1
	// 0x2    0x0      0          3          1       2
	// 0x3    0x2      2          0          3       1
	//
	// the library lenvm/SercomSPISlave only implements DOPO mode 0x02, but we use 0x03

  // Valid PAD > PIN mappings for SERCOM4, according to Table 6-1., page 21-23, are
	//                  SERCOM-ALT      SERCOM
	//                  (Function D)    (Function C)
  // SERCOM4/PAD[0]   PB8,  PA12      PB12
	// SERCOM4/PAD[1]   PB9,  PA13      PB13
	// SERCOM4/PAD[2]   PB10, PA14      PB14
	// SERCOM4/PAD[3]   PB11, PA15      PB15

  // Set up SPI control A register
  SERCOM4->SPI.CTRLA.bit.DORD = 0; // MSB is transferred first. // page 492
  SERCOM4->SPI.CTRLA.bit.CPOL = 0; // SCK is low when idle. The leading edge of a clock cycle is a rising edge, while the trailing edge is a falling edge. // page 492
  SERCOM4->SPI.CTRLA.bit.CPHA = 0; // Data is sampled on a leading SCK edge and changed on a trailing SCK edge. // page 492
  SERCOM4->SPI.CTRLA.bit.FORM = 0x0; // SPI frame // page 493
  SERCOM4->SPI.CTRLA.bit.DIPO = 0x2; // DATA PAD0 is used as slave input: MOSI // (slave mode) page 493
  SERCOM4->SPI.CTRLA.bit.DOPO = 0x3; // DATA PAD2 is used as slave output: MISO // (slave mode) page 493
  SERCOM4->SPI.CTRLA.bit.MODE = 0x2; // SPI slave operation. // page 494
  SERCOM4->SPI.CTRLA.bit.IBON = 0x1; // Immediate Buffer Overflow Notification. STATUS.BUFOVF is asserted immediately upon buffer overflow. // page 494
  SERCOM4->SPI.CTRLA.bit.RUNSTDBY = 1; // Wake on Receive Complete interrupt. // page 494

  // Set up SPI control B register
  //SERCOM4->SPI.CTRLB.bit.RXEN = 0x1; // Enable Receiver // page 496
  SERCOM4->SPI.CTRLB.bit.SSDE = 0x1; // Enable Slave Select Low Detect // page 497
  SERCOM4->SPI.CTRLB.bit.CHSIZE = 0; // Character Size 8 bits // page 497
  //SERCOM4->SPI.CTRLB.bit.PLOADEN = 0x1; // Enable Slave Data Preload // page 497
  //while (SERCOM4->SPI.SYNCBUSY.bit.CTRLB); // Wait until receiver is enabled

  // Set up SPI interrupts
  SERCOM4->SPI.INTENSET.bit.SSL = 0x1; // Enable Slave Select Low interrupt. // page 501
  SERCOM4->SPI.INTENSET.bit.RXC = 0x1; // Enable Receive Complete interrupt. // page 501
  SERCOM4->SPI.INTENSET.bit.TXC = 0x1; // Enable Transmit Complete interrupt. // page 501
  SERCOM4->SPI.INTENSET.bit.ERROR = 0x1; // Enable Error interrupt. // page 501
  SERCOM4->SPI.INTENSET.bit.DRE = 0x1; // Enable Data Register Empty interrupt. // page 501

  // Init SPI CLK // not used in SPI slave operation // page 481
  //SERCOM4->SPI.BAUD.reg = SERCOM_FREQ_REF / (2*4000000u)-1;

  // Enable SPI
  SERCOM4->SPI.CTRLA.bit.ENABLE = 1; // page 481
  while (SERCOM4->SPI.SYNCBUSY.bit.ENABLE); // Wait until bit is enabled.
  SERCOM4->SPI.CTRLB.bit.RXEN = 0x1; // Enable Receiver // page 496. This is done here rather than in section "Set up SPI control B register" due to an errate issue, e.g. page 1026.
  while (SERCOM4->SPI.SYNCBUSY.bit.CTRLB); // Wait until receiver is enabled.
}

void SpiSlave::onTransmissionStart()
{
  mRecvIdx = 0;
  mSendIdx = 2; // offset of 2 bytes, shift-register + TxDATA, see below
  mRecvBuffer = 0;
  mLastChipSelect = millis();
}

void SpiSlave::onTransmissionDone()
{
  mDoneRecvBuffer = mRecvBuffer;
  mLastReceive = millis();
}

void SpiSlave::onReadyForNewData()
{
  mSendBuffer = *mPtrToSendBuf;
}

SpiSlave::Buffer SpiSlave::getReceivedData() const
{
  return mDoneRecvBuffer;
}

unsigned long SpiSlave::getLastReceiveTime() const
{
  return mLastReceive;
}
unsigned long SpiSlave::getLastSelectTime() const
{
  return mLastChipSelect;
}

void SERCOM4_Handler(void)
{
  /**
   * Explanation of SSL, RXC and DRE
   * We send 4 bytes
   *   SSL        |
   *   RXC                       |       |       |       |
   *   DRE                         |       |       |       |
   *                     _______ _______ _______ _______
   * Data Bytes         |_______|_______|_______|_______|
   * 
   * SSL is trigged shorty after the CS line goes low.
   * However, at this time it is already to late to load data into the TxDATA register,
   * as the shift register is already loaded with the previous value of TxDATA.
   * This value is shifted out as the first byte. After this is completed the completion of the byte is notified by the RXC interrupt.
   * During this time the current value of the TxDATA register is being transfered to the shift register (takes up to 3 SCK cycles).
   * Once this is complete the DRE interrupt is triggered and requests the next data byte.
   * The consequence of this is that the data is always late by 2 bytes. Assume shift register and TxDATA is 0x00 when SSL occurs.
   * First the shift-register's 0x00 is shifted out, after it is done the 0x00 value of TxDATA is transfered to the shift register and shifted out.
   * So the next byte shifted out is again a 0x00. At the beginning of the 2. byte the DRE is triggered for the first time in this transaction.
   * The data value set here is therefore send out as the third byte.
   * So with a data length of N the correct moment to load in new data is in the N-2 DRE, after writing the last data byte.
   * 
   * TXC is usually never triggered, as we never run out of data
   */
	
   //SpiSlave& spi = SpiSlave::getInstance();

  /*
  *  1. ERROR
  *    Occurs when the SPI receiver has one or more errors.
  */
  if (SERCOM4->SPI.INTFLAG.bit.ERROR)
  {
    #ifdef SPIDEBUG
    irqcount.ERR++;
    #endif
    SERCOM4->SPI.INTFLAG.bit.ERROR = 1;
  }
  
  
  /*
  *  2. SSL: Slave Select Low
  *    Occurs when SS goes low
  */
  if (SERCOM4->SPI.INTFLAG.bit.SSL)
  {
    #ifdef SPIDEBUG
    irqcount.SSL++;
    #endif
    spi.onTransmissionStart();
    SERCOM4->SPI.INTFLAG.bit.SSL = 1;
  }
  

  /*
  *  3. TXC: Transmission Complete
  *    Occurs when SS goes high AND no new data in DATA. The transmission is complete.
  */
  if (SERCOM4->SPI.INTFLAG.bit.TXC)
  {
    #ifdef SPIDEBUG
    irqcount.TXC++;
    #endif
    SERCOM4->SPI.INTFLAG.bit.TXC = 1;
  }
  
  
  /*
  *  4. RXC: Receive Complete
  *    Occurs after a character has been full stored in the data buffer. It can now be retrieved from DATA.
  */
  if (SERCOM4->SPI.INTFLAG.bit.RXC)
  {
    #ifdef SPIDEBUG
    irqcount.RXC++;
    #endif
    uint8_t data = SERCOM4->SPI.DATA.reg;
		if (spi.mRecvIdx < spi.dataLen) {
			reinterpret_cast<volatile uint8_t*>(&(spi.mRecvBuffer))[spi.mRecvIdx++] = data;
		}

    if (spi.mRecvIdx >= spi.dataLen) { // we just received the last expected byte
      spi.onTransmissionDone();
    }
    SERCOM4->SPI.INTFLAG.bit.RXC = 1;
  }
  
  
  /*
  *  5. DRE: Data Register Empty
  *    Occurs when data register is empty
  */
  if (SERCOM4->SPI.INTFLAG.bit.DRE)
  {
    #ifdef SPIDEBUG
    irqcount.DRE++;
    #endif
    // IMPORTANT!!!! The DATA register ALWAYS needs to be written here
    // if not this interrupt is re-triggered immediately and the main loop is completely blocked
    // not setting the DATA register just re-transmits the previously received byte
    if (spi.mSendIdx < spi.dataLen) {
		  
      //SERCOM4->SPI.DATA.reg = reinterpret_cast<volatile uint8_t*>(&(spi.sendBuffer))[spi.dataLen-1-(spi.sendIdx)];
      SERCOM4->SPI.DATA.reg = reinterpret_cast<volatile uint8_t*>(&(spi.mSendBuffer))[spi.mSendIdx];
      spi.mSendIdx++;
      if (spi.mSendIdx >= spi.dataLen) spi.mSendIdx = 0;
    }
    else {
      SERCOM4->SPI.DATA.reg = 0x99; // should never happen
    }

    // sendIdx zero means we just wrote the last byte of this transmission to TxDATA
    // now we are ready to read new data in
    if (spi.mSendIdx == 0) {
      spi.onReadyForNewData();
    }

		SERCOM4->SPI.INTFLAG.bit.DRE = 1;
  }
}