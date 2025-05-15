#include <Arduino.h>
#include "SpiSlave.h"
#include "keypad.h"
#include "display.h"
#include "pinconfig.h"

#include <Adafruit_NeoPixel_ZeroDMA.h>

#define PIN        6
#define NUM_PIXELS 12

Adafruit_NeoPixel_ZeroDMA strip(NUM_PIXELS, PIN, NEO_GRB);

KeyPad keypad;

// timeout of 0 disables the timeout
const unsigned long typingTimeout = 1000; // timeout TYPING > ENTER_CODE
const unsigned long codeEntryTimeout = 10000; // timeout ENTER_CODE > IDLE
const unsigned long idleTimeout = 0; // timeout other states > IDLE
const unsigned long spiTimeout = 1000; // no-SPI => indicator timeout
const unsigned long spiRebootTimeout = 10000; // no-SPI => reboot timeout

enum class KeypadCommand : uint8_t {
  NO_COMMAND,     /** < Do nothing, can be used to just poll the state */
  IDLE,           /** < Go to into idle state */
  CODE_REQUIRED,  /** < Card scanned, second factor required => ENTER_CODE */
  ACT_BUSY,       /** < Backend is checking the code => BUSY  */
  AUTH_OK,        /** < Authentication successful */
  AUTH_FAIL_CARD, /** < Authentication failed, card unknown */
  AUTH_FAIL_CODE, /** < Authentication failed, code wrong */
  TOOL_UNLOCKED,  /** < Tool unlocked */
  TOOL_LOCKED,    /** < Tool locked */
  REBOOT,         /** < Reboot microcontroller */
};

/**
 * Request struct, send from FabXDevice > Keypad
 */
struct CommandBuffer {
  uint32_t Spare : 24; /** < Not yet used */
  KeypadCommand Command : 8; /** < Controls the keypad's target state */
};

enum class KeypadState : uint8_t {
  IDLE,         /** < Idle, show logo screen, wait for FabX to scan card */
  ENTER_CODE,   /** < Card scanned, code as second factor required */
  TYPING,       /** < Informative state, user is typing (in ENTER_CODE) */
  CODE_READY,   /** < Code entered and ready for FabX to verify */
  BUSY,         /** < Display progress indicator */
  SUCCESS,      /** < Authentication successful (code ok or not required) */
  UNLOCKED,     /** < Tool unlocked */
  LOCKED,       /** < Tool locked */
  UNKNOWN_CARD, /** < Authentication failed, card unknown */
  WRONG_CODE,   /** < Authentication failed, code wrong */
};

/**
 * Response struct, send from Keypad > FabXDevice
 */
struct StateBuffer {
  uint32_t Code : 20; /** < Entered code as 20-bit unsigned integer,
                            which is sufficient for up to 6 digit numbers. */
  uint8_t CodeLen : 4; /** < Length of entered code,
                             allows handling codes with leading 0 */
  KeypadState State : 8; /** < The keypad's actual state */
};

U8G2Display display;
StateBuffer state;
bool spiDebug = false;

void resetState()
{
  state.State = KeypadState::IDLE;
  state.Code = 0;
  state.CodeLen = 0;
}

void setup(void)
{
  // LED configurations, all LEDs off
  PORT->Group[PORTA].DIRSET.reg = 1<<TXLED;
  PORT->Group[PORTA].OUTSET.reg = 1<<TXLED;
  PORT->Group[PORTA].DIRSET.reg = 1<<RXLED;
  PORT->Group[PORTA].OUTSET.reg = 1<<RXLED;
  PORT->Group[PORTA].DIRSET.reg = 1<<LED;
  PORT->Group[PORTA].OUTCLR.reg = 1<<LED;

  // Arduino Zero Pin 6 / PA20 / 29 is not supported by the Library directly
  strip.begin(&sercom5, SERCOM5, SERCOM5_DMAC_ID_TX,
                6, SPI_PAD_2_SCK_3, PIO_SERCOM);

  keypad.begin();

  // Keypad ROW pins are outputs
  PORT->Group[PORTA].DIRSET.reg = 1<<PIN_ROW3 | 1<<PIN_ROW2 |
                                  1<<PIN_ROW1 | 1<<PIN_ROW0;
  resetState();

  spi.begin(reinterpret_cast<SpiSlave::Buffer*>(&state), 3);

  __enable_irq();
  display.begin();
}

void HardFault_Handler(void)
{
  while (true) {
    PORT->Group[PORTA].OUTSET.reg = 1<<LED;
    delay(100);
    PORT->Group[PORTA].OUTCLR.reg = 1<<LED;
    delay(100);
  }
}

void neoRainbow()
{
  // Rainbow cycle
  static uint32_t startTime = micros();
  uint32_t t       = micros();
  uint32_t elapsed = t - startTime;
  uint32_t firstPixelHue = elapsed / 32;
  for(uint16_t i=0; i<strip.numPixels(); i++) {
    uint32_t pixelHue = firstPixelHue + (i * 65536L / strip.numPixels());
    strip.setPixelColor(i, strip.gamma32(strip.ColorHSV(pixelHue)));
  }
  strip.show();
}

void handleCommand(CommandBuffer* cmd)
{
  switch(cmd->Command) {
    case KeypadCommand::NO_COMMAND:
      break;
    case KeypadCommand::IDLE:
      if (state.State != KeypadState::IDLE) {
        resetState();
      }
      break;
    case KeypadCommand::REBOOT:
      NVIC_SystemReset();
      break;
    case KeypadCommand::CODE_REQUIRED:
      if (state.State != KeypadState::TYPING
          && state.State != KeypadState::ENTER_CODE) {
        state.State = KeypadState::ENTER_CODE;
        state.Code = 0;
        state.CodeLen = 0;
      }
      break;
    case KeypadCommand::ACT_BUSY:
      state.State = KeypadState::BUSY;
      break;
    case KeypadCommand::AUTH_OK:
      state.State = KeypadState::SUCCESS;
      break;
    case KeypadCommand::AUTH_FAIL_CARD:
      state.State = KeypadState::UNKNOWN_CARD;
      break;
    case KeypadCommand::AUTH_FAIL_CODE:
      state.State = KeypadState::WRONG_CODE;
      break;
    case KeypadCommand::TOOL_UNLOCKED:
      state.State = KeypadState::UNLOCKED;
      break;
    case KeypadCommand::TOOL_LOCKED:
      state.State = KeypadState::LOCKED;
      break;
  }
}

void handleCodeInput(uint8_t key, uint8_t maxCodeLen)
{
  if (key == KEY_BS) {
    state.Code /= 10;
    if (state.CodeLen>0) state.CodeLen--;
  }
  else if (key >= 0 && key <= 9 && state.Code <= 99999
      && state.CodeLen < 6 && state.CodeLen < maxCodeLen) {
    state.Code = 10*state.Code + key;
    state.CodeLen++;
    if (state.CodeLen == maxCodeLen) {
      state.State = KeypadState::CODE_READY;
    }
  }
}

void handleInput()
{
  static unsigned long lastInteraction = 0;
  const int maxCodeLen = 4;

  uint8_t key = keypad.readNewKey();

  if (key != KEY_NOKEY) lastInteraction = millis();

  if (state.State == KeypadState::IDLE) {
    handleCodeInput(key, 7 /* never reached */);
    #ifdef SPIDEBUG
    if (state.Code == 126874 && state.CodeLen == 6) {
      spiDebug = !spiDebug;
      resetState();
    }
    if (spiDebug && key == 0) {
      irqcount.ERR = 0;
      irqcount.SSL = 0;
      irqcount.RXC = 0;
      irqcount.TXC = 0;
      irqcount.DRE = 0;
    }
    if (spiDebug && (millis() - lastInteraction) > 60000) {
      spiDebug = 0;
    }
    #endif
    if (millis() - lastInteraction > 2000) {
      resetState();
    }
  }

  if (state.State == KeypadState::ENTER_CODE
      || state.State == KeypadState::TYPING) {

    if (key != KEY_NOKEY) {
      state.State = KeypadState::TYPING;

      handleCodeInput(key, maxCodeLen);

      if (key == KEY_ENTER) {
        state.State = KeypadState::CODE_READY;
      }
    }
    else {
      if (state.State == KeypadState::TYPING
          && millis() - lastInteraction > typingTimeout) {
        state.State = KeypadState::ENTER_CODE;
      }
    }
  }
}

void updateDisplay(const CommandBuffer* cmd)
{
  display.clear();
  switch(state.State) {
    case KeypadState::IDLE:
      display.drawIdleScreen();
      neoRainbow();
      break;
    case KeypadState::ENTER_CODE:
      // fall through
    case KeypadState::TYPING:
      display.drawCodeScreen(state.Code, state.CodeLen);
      neoRainbow();
      break;
    case KeypadState::UNKNOWN_CARD:
      display.drawCodeVerifiedScreen(false);
      strip.fill(0xff0000);
      strip.show();
      break;
    case KeypadState::WRONG_CODE:
      display.drawCodeVerifiedScreen(false);
      strip.fill(0xff0000);
      strip.show();
      break;
    case KeypadState::SUCCESS:
      display.drawCodeVerifiedScreen(true);
      strip.fill(0x00ff00);
      strip.show();
      break;
    case KeypadState::UNLOCKED:
      display.drawLockedScreen(false);
      strip.fill(0x00ff00);
      strip.show();
      break;
    case KeypadState::LOCKED:
      display.drawLockedScreen(true);
      strip.fill(0x00ff00);
      strip.show();
      break;
    case KeypadState::CODE_READY:
      // fall through
    case KeypadState::BUSY:
      display.drawProgressScreen();
      strip.fill(0xffff00);
      strip.show();
      break;
    default:
      break;
  }

  // when the FabXDevice is not yet connected to the backend and thus not yet
  // configured, it does not know that it has a keypad connected and the chip
  // select signal is not configured therefore we receive all transmissions
  // which are meant for the display
  bool spiError = (spiTimeout && millis() - spi.getLastReceiveTime() > spiTimeout);
  display.drawSpiStatusIndicator(!spiError);

  // reboot after spiRebootTimeout ms no SPI command
  if (spiRebootTimeout && millis() - spi.getLastReceiveTime() > spiRebootTimeout) {
    NVIC_SystemReset();
  }

  #ifdef SPIDEBUG
  if (spiDebug) {
    display.drawSpiDebugScreen(irqcount, static_cast<int>(cmd->Command),
                                         static_cast<int>(state.State));
  }
  #endif

  display.pushCanvas();
}

void handleTimeouts(unsigned long lastStateChange)
{
  if (state.State == KeypadState::IDLE) {
    // already in idle, no timeout here
  }
  else if (state.State == KeypadState::TYPING) {
    // typing timeout handled in handleInput
  }
  else if (state.State == KeypadState::CODE_READY
    && codeEntryTimeout > 0
    && millis() - lastStateChange > codeEntryTimeout) {
    // handleInput transitions to TYPING on input, so we won't timeout while typing
    resetState();
  }
  else if (idleTimeout > 0 && millis() - lastStateChange > idleTimeout) {
    resetState();
  }
}

void loop(void)
{
  static unsigned long lastRefresh = 0;
  static unsigned long lastStateChange = 0;

  CommandBuffer cmd;
  uint32_t recv = spi.getReceivedData();
  memcpy(&cmd, &recv, sizeof(recv));

  KeypadState prevState = state.State;

  // update state from command
  handleCommand(&cmd);

  // update state from input
  handleInput();

  if (prevState != state.State) {
    lastStateChange = millis();
  }

  // timeout to IDLE, after(!) updating lastStateChange
  handleTimeouts(lastStateChange);

  if (millis() - lastRefresh > 50) {
    lastRefresh = millis();
    updateDisplay(&cmd);
  }
}