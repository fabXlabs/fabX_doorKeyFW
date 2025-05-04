#include <Arduino.h>

#define PIN_COL2   PIN_PA04
#define PIN_COL1   PIN_PA05
#define PIN_COL0   PIN_PA06
#define PIN_ROW3   PIN_PA16
#define PIN_ROW2   PIN_PA07
#define PIN_ROW1   PIN_PA08
#define PIN_ROW0   PIN_PA09

#define GET_ROW_PIN(x) ((x) == 0 ? PIN_ROW0 : (x) == 1 ? PIN_ROW1 : (x) == 2 ? PIN_ROW2 : (x) == 3 ? PIN_ROW3 : -1)
#define GET_COL_PIN(x) ((x) == 0 ? PIN_COL0 : (x) == 1 ? PIN_COL1 : (x) == 2 ? PIN_COL2 : -1)

#define KEY_NOKEY 0xff
#define KEY_BS 11
#define KEY_ENTER 10
uint8_t keypad_map[] = {KEY_NOKEY, 7, 8, 9, 4, 5, 6, 1, 2, 3, KEY_BS, 0, KEY_ENTER};
#define KEYMAP(x) (x >= 0 && x<sizeof(keypad_map) ? keypad_map[x] : 0xee);

class KeyPad
{
public:
	static const int number_of_rows = 4;
	static const int number_of_cols = 3;
	static const int number_of_keys = number_of_rows * number_of_cols;
	static const int debounce_time = 10; // ms

	/**
	 * brief Setup GPIO pins.
	 */
  void begin();
	/**
	 * @brief Scan keys, and report newly pressed key.
	 * 
	 * If multiple keys were pressed since last call the keys are returned one by one,
	 * i.e. the first call retuns the first key (in scan order), the next call the next one and so on.
	 */
  uint8_t readNewKey();

private:
	unsigned long mKeys[KeyPad::number_of_keys]; /** < Timestamp since when a key is pressed or 0 otherwise */
	uint16_t mLastKeyMask = 0; /** < Bit mask, which currently pressed keys have already been reported as 'new' keys */
};

void KeyPad::begin()
{
	// Keypad COL pins are inputs with pull-ups
	PORT->Group[PORTA].DIRCLR.reg = 1<<PIN_COL2 | 1<<PIN_COL1 | 1<<PIN_COL0;
	PORT->Group[PORTA].PINCFG[PIN_COL2].reg = PORT_PINCFG_INEN|PORT_PINCFG_PULLEN;
	PORT->Group[PORTA].PINCFG[PIN_COL1].reg = PORT_PINCFG_INEN|PORT_PINCFG_PULLEN;
	PORT->Group[PORTA].PINCFG[PIN_COL0].reg = PORT_PINCFG_INEN|PORT_PINCFG_PULLEN;
	PORT->Group[PORTA].PINCFG[PIN_ROW2].reg = PORT_PINCFG_INEN|PORT_PINCFG_PULLEN;
	PORT->Group[PORTA].PINCFG[PIN_ROW1].reg = PORT_PINCFG_INEN|PORT_PINCFG_PULLEN;
	PORT->Group[PORTA].PINCFG[PIN_ROW0].reg = PORT_PINCFG_INEN|PORT_PINCFG_PULLEN;
	PORT->Group[PORTA].OUTSET.reg = 1<<PIN_COL2 | 1<<PIN_COL1 | 1<<PIN_COL0; // enable pull-up
}

uint8_t KeyPad::readNewKey()
{
	uint8_t keyIdx = 0;
	uint8_t key = KEY_NOKEY;
	for (int row=0; row < number_of_rows; row++) {
		PORT->Group[PORTA].OUTSET.reg = 1<<PIN_ROW3 | 1<<PIN_ROW2 | 1<<PIN_ROW1 | 1<<PIN_ROW0;
		PORT->Group[PORTA].OUTCLR.reg = 1<<GET_ROW_PIN(row);
		for (int col=0; col < number_of_cols; col++, keyIdx++) {
			if ((PORT->Group[PORTA].IN.reg & (1<<GET_COL_PIN(col))) == 0) {
				if (mKeys[keyIdx] == 0) { // new key
					mKeys[keyIdx] = millis();
				}
			}
			else {
				mKeys[keyIdx] = 0;
				mLastKeyMask &= ~(1<<keyIdx);
			}

			if (mKeys[keyIdx] != 0
					&& millis() - mKeys[keyIdx] > debounce_time
					&& (mLastKeyMask&(1<<keyIdx)) == 0
				  && key == KEY_NOKEY) {
						key = KEYMAP(static_cast<size_t>(keyIdx+1));
				mLastKeyMask |= 1<<keyIdx;
			}
		}
	}
	return key;
}