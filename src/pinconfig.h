#pragma once

// don't use these PINs with pinMode or digitalWrite, they get re-mapped via variant.cpp to other pins
#define TXLED    PIN_PA27
#define RXLED    PIN_PA31
#define LED      PIN_PA17

#define TOGGLE_PIN(port,pin)     (((PORT->Group[port].OUT.reg&(1<<pin)) ? PORT->Group[port].OUTCLR.reg : PORT->Group[port].OUTSET.reg) = 1<<pin)