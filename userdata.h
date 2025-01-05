#pragma once

#define WDT_FLAG 3
#define WDT_FLAG_TRUE  0x55
#define WDT_FLAG_FALSE 0xAA

uint8_t eeprom_read(uint16_t address) {
  while (EECR & _BV(EEPE));  // Wait for outstanding writes to complete
  EEAR = address;
  EECR |= _BV(EERE);  // Initiate EEPROM read
  return EEDR;        // Result is available on next CPU instruction
}

void eeprom_read(void* dest, size_t length, uint16_t src) {
  for (size_t i = 0; i < length; ++i) {
    *((uint8_t*)dest + i) = eeprom_read(src + i);
  }
}

void eeprom_write(uint16_t address, uint8_t data) {
  while (EECR & _BV(EEPE));  // Wait for outstanding writes to complete
  EEAR = address;
  EEDR = data;
  cli();  // EEPE must be set within 4 cycles of EEMPE
  EECR |= _BV(EEMPE);  // Enable write
  EECR |= _BV(EEPE);   // Initiate write
  sei();
}

void eeprom_write(uint16_t dest, size_t length, void* src) {
  for (size_t i = 0; i < length; ++i) {
    eeprom_write(dest + i, *((uint8_t*)src + i));
  }
}
