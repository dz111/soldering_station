#pragma once

#define SET(reg, n)    reg |= ((uint8_t)1 << n)
#define CLEAR(reg, n)  reg &= ~((uint8_t)1 << n)
#define TOGGLE(reg, n) reg ^= ((uint8_t)1 << n)
#define ISSET(reg, n)  ((reg >> n) & (uint8_t)1)
