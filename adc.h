#pragma once

#include "utils.h"

using vuint16_t = volatile uint16_t;
using adc_dest_t = vuint16_t*;
adc_dest_t adc_destinations[9] = {nullptr};
uint8_t adc_curr_channel;
uint8_t adc_next_channel;

bool adc_is_converting() {
  return ADCSRA & _BV(ADSC);
}

//bool adc_start(uint8_t channel, volatile uint16_t* destination) {
//  while (adc_is_converting()) { }
//  adc_destination = destination;
//  CLEAR(PRR, PRADC);
//  channel &= 0x0F;  // mask bottom 4 bits
//  // VREF=AVcc; ADC Left adjust=false
//  ADMUX = _BV(REFS0) | channel;
//  // ADPS2:0 = 0b111, div factor 128, with fck=16MHz, ADC clock=125kHz
//  ADCSRA = _BV(ADEN) | _BV(ADSC) | _BV(ADIE) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);
//}

void adc_set_destination(uint8_t channel, adc_dest_t destination) {
  if (channel <= 8) {
    adc_destinations[channel] = destination;
  }
}

uint8_t adc_find_next_channel(uint8_t curr_channel) {
  uint8_t next_channel;
  for (uint8_t i = 0; i <= 8; ++i) {
    next_channel = (curr_channel + i + 1) % 9;
    if (adc_destinations[next_channel] != nullptr) {
      return next_channel;
    }
  }
  return 0xff;
}

void adc_set_channel(uint8_t channel) {
  ADMUX = (ADMUX & 0xF0) | (channel & 0x0F);
}

void adc_start() {
  CLEAR(PRR, PRADC);
  adc_curr_channel = adc_find_next_channel(8);
  // VREF=AVcc; ADC Left adjust=false
  ADMUX = _BV(REFS0) | adc_curr_channel;
  // ADTS2:0 = 0; free running mode
//  ADCSRB = 0;
  // ADPS2:0 = 0b111, div factor 128, with fck=16MHz, ADC clock=125kHz
  ADCSRA = _BV(ADEN) | _BV(ADSC) | _BV(ADATE) | _BV(ADIE) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);
  adc_next_channel = adc_find_next_channel(adc_curr_channel);
  adc_set_channel(adc_next_channel);
}

ISR(ADC_vect) {
  if (adc_destinations[adc_curr_channel]) {
    *adc_destinations[adc_curr_channel] = ADCW;
  }

  adc_curr_channel = adc_next_channel;
  adc_next_channel = adc_find_next_channel(adc_curr_channel);
  if (adc_next_channel == 0xff) {
    CLEAR(ADCSRA, ADEN);
  }
  adc_set_channel(adc_next_channel);
}
