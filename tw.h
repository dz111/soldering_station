#pragma once

/*
  This is a helper library for the AVR two-wire interface (IIC) hardware bus
  
  It supports only master transmit mode at 400 kHz when MCU is operating at 16 MHz.
  (The code for `tw_init` must be modified if not operating under these conditions)
  
  Writes to the slave are buffered, with a maximum of 250 bytes per transmission.

  `tw_init` must be called prior to first using the library to initialise internal
  state.

  For each transmission, call `tw_start` with the slave's address, then call
  `tw_write` for each byte in the transmission.
  
  Transmissions are initiated when `twi_end` is called and, other than the
  START signal, is executed by an ISR responding to TWINT. This allows the
  remainder of the program to continue executing while the transmission occurs.
  
  Multiple transmissions cannot be buffered. Instead, a call to `twi_start`
  will block until the active transmission is complete. To avoid blocking, the
  user can check `twi_has_started` before initiating a transmission.

  It may be desirable to have the interrupt vector call back to a user-supplied
  delegate to handle the communication flow. This is more flexible than doing a
  series of `tw_write` into the internal buffer and can allow for a larger
  sequence than the size of the internal buffer.

  The user must define an ISR for PCINT0 which is triggered when an error occurs.
  The type of error is stored in twi_error and corresponds to the TWIE_* defines.
  Pin PB5 must be Not Connected to avoid interfering with this interrupt.
 */

#include "utils.h"

#include <util/atomic.h>
#include <util/twi.h>

#define SLAW(addr) (addr << 1)
#define SLAR(addr) ((addr << 1) & 1)
#define TWI_BUF_MAX 8

#define TWIE_OK        0                  // Ok - no error
#define TWIE_GEN       1                  // Unknown error
#define TWIE_BUFOV     2                  // Overflow of data buffer
#define TWIE_ADD_NACK  TW_MT_SLA_NACK     // Received NACK after xmitting slave address
#define TWIE_DAT_NACK  TW_MT_DATA_NACK    // Received NACK after xmitting data
#define TWIE_ARB       TW_MT_ARB_LOST     // Lost arbitration

using TwiDelegate = void(*)(uint8_t);

volatile uint8_t twi_buf[TWI_BUF_MAX];
volatile uint8_t twi_index;
volatile uint8_t twi_count;
volatile uint8_t twi_address;
volatile uint8_t twi_error;
volatile TwiDelegate twi_delegate;

inline void twi__begin(uint8_t __address) {
  twi_address = SLAW(__address);
  twi_count = 0;
}

inline void twi__start() {
  TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN) | _BV(TWIE);
}

inline void twi__send(uint8_t packet) {
  TWDR = packet;
  TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWIE);
}

inline void twi__stop() {
  TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWSTO);
}

inline void twi__reset() {
  twi_count = -1;
  twi_index = 0;
  twi_address = 0;
  twi_delegate = nullptr;
}

inline void twi__error(uint8_t err) {
  twi_error = TW_STATUS;
  TOGGLE(PORTB, DDB1);
}

uint8_t twi_get_error() {
  return twi_error;
}

void twi_clear_error() {
  twi_error = TWIE_OK;
}

void twi_init() {
  // fSCL = fck / (16 + 2 * TWBR * prescaler)
  // 400kHz = 16000kHz / (16 + 2 * TWBR * prescaler)
  // let prescaler = 1, then TWBR = 12
  TWBR = 12;
  TWSR = 0; // ~(_BV(TWPS1) & (_BV(TWPS0)));
  // Commandeer PB1/PCINT1 for use as a software interrupt for errors
  DDRB  |= _BV(DDB5);
  PCICR |= _BV(PCIE0);
  PCMSK0 = _BV(PCINT5);
  // Initialise internal state
  twi__reset();
  twi_clear_error();
}

bool twi_has_started() {
  return (twi_count != -1 && twi_address != 0);
}

void twi_begin(uint8_t address) {
  while (twi_has_started()) { }
  twi__begin(address);
}

void twi_write_with_delegate(uint8_t address, TwiDelegate delegate) {
  twi_delegate = delegate;
  twi__begin(address);
  twi__start();
}

void twi_write(uint8_t data) {
  if (!twi_has_started()) return;
  if (twi_count > TWI_BUF_MAX) {
    twi__error(TWIE_BUFOV);
  }

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    twi_buf[twi_count++] = data;
  }
}

void twi_end() {
  if (!twi_has_started()) return;
  if (twi_count == 0) return;  // buffer empty, nothing to xmit

  twi__start();

  // Everything else is handled in ISR(TW_vect)
}

ISR(TWI_vect) {
  switch (TW_STATUS) {
    
  // START signal sent; now send slave address
  case TW_REP_START:
  case TW_START:
    {
      twi__send(twi_address);
    }
    break;

  // Received ACK; now send data or STOP signal if at end
  case TW_MT_SLA_ACK:
  case TW_MT_DATA_ACK:
    if (twi_delegate) {
      twi_delegate(TW_STATUS);
    } else {
      if (twi_index >= twi_count) {
        twi__stop();
        twi__reset();
      } else {
        twi__send(twi_buf[twi_index++]);
      }
    }
    break;

  // Transmission failed in some way
  default:
    {
      twi__stop();
      twi__reset();
      twi__error(TW_STATUS);
    }
    break;
  }
}
