#pragma once
#include <U8g2lib.h>
#include "resources.h"
#include "SpiSlave.h"

class U8G2Display
{
public:
    U8G2Display();
    void begin();
    void clear();
    void pushCanvas();
    void drawIdleScreen();
    void drawSpiStatusIndicator(bool spiOk);
    void drawCodeScreen(uint32_t code, uint8_t len);
    void drawProgressScreen();
    void drawCodeVerifiedScreen(bool codeOk);
    void drawLockedScreen(bool locked);
    void drawSpiDebugScreen(const volatile SpiIrqCount& irqc, int cmd, int state);
    void drawString(const char* str);

private:
  //void drawBitmap(int x, int y, int sx, int sy, unsigned int *data);
    U8G2_SH1106_128X64_NONAME_F_HW_I2C mU8g2;
};

U8G2Display::U8G2Display()
 : mU8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE)
{

}

void U8G2Display::begin()
{
  mU8g2.begin();
	mU8g2.setFont(u8g2_font_6x10_tr);
  mU8g2.setFontRefHeightExtendedText();
  mU8g2.setDrawColor(1);
  mU8g2.setFontPosTop();
  mU8g2.setFontDirection(0);
  drawIdleScreen();
  pushCanvas();
}

void U8G2Display::clear()
{
  mU8g2.clearBuffer();
}

void U8G2Display::pushCanvas()
{
  mU8g2.sendBuffer();
}

void U8G2Display::drawIdleScreen()
{
  uint16_t logoXOffset = (mU8g2.getWidth() - fablab_width)/2;
  uint16_t logoYOffset = (mU8g2.getHeight() - fablab_height)/2;
  mU8g2.setColorIndex(0);
  mU8g2.drawXBMP(logoXOffset, logoYOffset+7, fablab_width, fablab_height, fablab_bits);
  mU8g2.setFont(u8g2_font_6x10_tr);
  mU8g2.setColorIndex(1);
  mU8g2.drawStr(2, 2, "Willkommen im FabLab");
}

void U8G2Display::drawSpiStatusIndicator(bool spiOk)
{
  if (!spiOk) {
      const uint8_t x = mU8g2.getWidth() - cloud_width;
      const uint8_t y = mU8g2.getHeight() - cloud_height;
      mU8g2.setColorIndex(1);
      if (millis() % 500 < 250) {
        mU8g2.drawXBMP(x, y, nocloud_width, nocloud_height, nocloud_bits);
      }
      else {
        mU8g2.drawXBMP(x, y, cloud_width, cloud_height, cloud_bits);
      }
  }
}

void U8G2Display::drawSpiDebugScreen(const volatile SpiIrqCount& irqc, int cmd, int state)
{
  const uint8_t x = 2;
  const uint8_t y = 45;
  const uint8_t h = 10;
  const uint8_t w = 40;
  static char buf[128];
  mU8g2.setDrawColor(0);
  mU8g2.drawBox(x, y, mU8g2.getWidth(), 2*h);
  mU8g2.setFont(u8g2_font_5x7_tr);
  mU8g2.setDrawColor(1);
  snprintf(buf, sizeof(buf), "E:%d", irqc.ERR);
  mU8g2.drawStr(x, y, buf);
  snprintf(buf, sizeof(buf), "S:%d", irqc.SSL);
  mU8g2.drawStr(x+w, y, buf);
  snprintf(buf, sizeof(buf), "R:%d", irqc.RXC);
  mU8g2.drawStr(x+2*w, y, buf);
  snprintf(buf, sizeof(buf), "T:%d", irqc.TXC);
  mU8g2.drawStr(x, y+h, buf);
  snprintf(buf, sizeof(buf), "D:%d", irqc.DRE);
  mU8g2.drawStr(x+w, y+h, buf);
  snprintf(buf, sizeof(buf), "C:%d", cmd);
  mU8g2.drawStr(x+2*w, y+h, buf);
  snprintf(buf, sizeof(buf), "S:%d", state);
  mU8g2.drawStr(x+2*w+w/2, y+h, buf);
}

void U8G2Display::drawCodeScreen(uint32_t code, uint8_t len)
{
  static char buf[8];
  u8g2_uint_t textWidth;
  mU8g2.setColorIndex(1);
  mU8g2.setFont(u8g2_font_10x20_tr);
  mU8g2.drawStr(0, 2, "PIN eingeben");
  snprintf(buf, sizeof(buf), "%.*lu", len, code);
  textWidth = mU8g2.getStrWidth(buf);
  //mU8g2.drawStr((mU8g2.getWidth() - textWidth)/2, 28, buf);
  if (len<sizeof(buf)) {
    memset(buf, '*', len);
    buf[len] = '\0';
  }
  textWidth = mU8g2.getStrWidth(buf);
  mU8g2.drawStr((mU8g2.getWidth() - textWidth)/2, 32, buf);
}

void U8G2Display::drawProgressScreen()
{
  uint16_t xOffset = (mU8g2.getWidth() - LOADER_WIDTH)/2;
  uint16_t yOffset = (mU8g2.getHeight() - LOADER_HEIGHT)/2;
  int frame = (millis() / LOADER_DELAY)  % LOADER_COUNT;
  mU8g2.drawXBMP(xOffset, yOffset, LOADER_WIDTH, LOADER_HEIGHT, loader_frames[frame]);
}

void U8G2Display::drawCodeVerifiedScreen(bool codeOk)
{
  uint16_t xOffset = (mU8g2.getWidth() - ok_width)/2;
  uint16_t yOffset = (mU8g2.getHeight() - ok_height)/2;
  mU8g2.setColorIndex(0);
  if (codeOk) {
    mU8g2.drawXBMP(xOffset, yOffset, ok_width, ok_height, ok_bits);
  }
  else {
    mU8g2.drawXBMP(xOffset, yOffset, fail_width, fail_height, fail_bits);
  }
}

void U8G2Display::drawLockedScreen(bool locked)
{
  uint16_t xOffset = (mU8g2.getWidth() - locked_width)/2;
  uint16_t yOffset = (mU8g2.getHeight() - locked_height)/2;
  mU8g2.setColorIndex(0);
  if (locked) {
    mU8g2.drawXBMP(xOffset, yOffset, locked_width, locked_height, locked_bits);
  }
  else {
    mU8g2.drawXBMP(xOffset, yOffset, unlocked_width, unlocked_height, unlocked_bits);
  }
}

void U8G2Display::drawString(const char* str)
{
  mU8g2.setFont(u8g2_font_6x10_tr);
  mU8g2.setColorIndex(1);
  mU8g2.drawUTF8(2, 32, str);
}