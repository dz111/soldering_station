#define BOARD_REV 2

#if BOARD_REV == 2
#define OLED_RST_PORT PORTB
#define OLED_RST_PIN  PB2
#endif  // BOARD_REV == 2

#if BOARD_REV == 3
#define OLED_RST_PORT PORTB
#define OLED_RST_PIN  PB4
#endif  // BOARD_REV == 3

#include "adc.h"
#include "oled.h"
#include "tw.h"
#include "usart.h"
#include "utils.h"

#include <avr/wdt.h>

#if BOARD_REV == 2
void io_init() {
  DDRB  = 0b00000110;
  DDRC  = 0b00110000;
  DDRD  = 0b00001111;
  PORTB = 0b00111001;
  PORTC = 0b00000100;
  PORTD = 0b11111111;
  DIDR0 = 0b00001011;
}

#define BTN_FN1 (PIND & _BV(PD4))
#define BTN_FN2 (PIND & _BV(PD5))
#define BTN_INC (PIND & _BV(PD6))
#define BTN_DEC (PIND & _BV(PD7))

#endif  // BOARD_REV == 2

#if BOARD_REV == 3
void io_init() {
  DDRB  = 0b00011110;
  DDRC  = 0b00110000;
  DDRD  = 0b00000011;
  PORTB = 0b00001101;
  PORTC = 0b00000000;
  PORTD = 0b11111111;
  DIDR0 = 0b00001111;
}

#define BTN_FN1 (PORTD & _BV(PD2))
#define BTN_FN2 (PORTD & _BV(PD3))
#define BTN_INC (PORTD & _BV(PD4))
#define BTN_DEC (PORTD & _BV(PD5))

#endif  // BOARD_REV == 3

void pwm_init() {
/*
 * The heater PWM operates on Timer 1 (16-bit) and outputs
 * onto pin 15/OC1A/PB1. The data direction register (DDR)
 * for this pin is set in the main initialisation.
 * 
 * We use the Fast PWM mode here--the counter is incremented
 * from 0 to TOP. The output is set LOW when the counter 
 * overflows TOP and set HIGH when the counter equals OCR1A.
 * 
 * The counter increments at a rate of f_cpu / N (eg
 * 16 MHz / 8 = 2 MHz). The PWM frequency is this rate divided
 * by (TOP + 1), eg with TOP=127, f_pwm = 15,625 Hz.
 */
  // reset timer1
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1C = 0;
  OCR1A = 0;
  // set "TOP"
  ICR1 = 127;
  // set control registers
  // f_pwm = f_cpu / (N * (1 + TOP))  (Eq on p 135)
  //       = 16 MHz / (8 * 128)
  //       = 15,625 Hz (period = 64 us)
  // COM1A = 10; non-inverting mode, fast PWM (Tbl 16-2)
  // WGM13:10 = 1110; fast PWM, TOP=ICR1 (Tbl 16-4)
  // CS12:10 = 010; N=8 divisor (Tbl 16-5)
  TCCR1A = _BV(COM1A1) | _BV(WGM11);
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11);
}

void timer_init() {
/*
 * Timer2 (8-bit) is used to periodically switch off the
 * heater to allow the thermocouple to be read. Once the
 * heater is switched off, sufficient time must be given
 * for the current sense amplifier to recover before reading.
 * 
 * The counter increments from 0 to 255. When the counter
 * overflows 255, the OVF interrupt is triggered and the
 * relevant ISR switches off the heater. When the counter
 * equals OCR0A, the COMPA interrupt is triggered and the
 * relevant ISR reads the thermocouple, calls the heater
 * control to set the PWM duty cycle and switches on the
 * heater.
 * 
 * The effect of the above is that during the period
 * between 0 and OCR0A, the heater is off and the current
 * sense amplifier is recovering; between OCR0A and 255,
 * the heater is on. The effective heater power is factored
 * by (OCR0A + 1) / 256. So OCR0A should be reduced to the
 * minimum value that allows reliable temperature measurement.
 * 
 * The counter increments at a rate of f_cpu / N (eg
 * 16 MHz / 1024 = 15,625 Hz or 64 us per step). The full cycle
 * is this rated divided by 256 (eg approx 61 Hz or a period of
 * 16.384 ms).
 */
  //reset timer2
  TCCR2A = 0;
  TCCR2B = 0;
  TIMSK2 = 0;
  // set control registers
  // COM0A = 0; timer not connected to pins
  // WGM0  = 011;
  // f = c_cpu / (N * 256)
  //   = 16 MHz / (1024 * 256)
  //   = 61 Hz
  // CS0 = 101; N = 1024 (Tbl 14-9)
  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS22) | _BV(CS20);
  // op amp recovers in 14-18 us
  // timer period = 16.384ms, 256 steps, each step = 64us
  OCR2A = 12;  // trigger interrupt after n steps
  // interrupt mask
  TIMSK0 = _BV(OCIE2A) | _BV(TOIE2);
}

ISR(TIMER2_OVF_vect) {
//  read_iv();
//  pwm_off();
}

ISR(TIMER2_COMPA_vect) {
//  read_tip_temperature();
//  heat_controller();
//  pwm_on();
}

struct Button {
  bool is_pressed : 1;
  bool on_press   : 1;
  bool on_release : 1;
  uint16_t history;
};

struct ButtonStruct {
  Button fn1;
  Button fn2;
  Button inc;
  Button dec;
};

void process_button(struct Button& btn, bool pin) {
  btn.history <<= 1;
  btn.history |= pin;
  btn.on_press  = (!btn.is_pressed && (btn.history == 0));
  btn.on_release = (btn.is_pressed && (btn.history == 0xffff));
  if (btn.history == 0)      btn.is_pressed = true;
  if (btn.history == 0xffff) btn.is_pressed = false;
}

void process_buttons(struct ButtonStruct& btn) {
  process_button(btn.fn1, BTN_FN1);
  process_button(btn.fn2, BTN_FN2);
  process_button(btn.inc, BTN_INC);
  process_button(btn.dec, BTN_DEC);
}

void draw_power_meter(uint8_t duty) {
  for (int y = 0; y < 44; ++y) {
    oled_pixel(6, y, 1);
    oled_pixel(11, y, 1);

    if (y == 0 || y == 43 || (44 - y - 2) < (duty * 42 / 100)) {
      oled_pixel(7, y, 1);
      oled_pixel(8, y, 1);
      oled_pixel(9, y, 1);
      oled_pixel(10, y, 1);
    }
  }
}

class State {
public:
  virtual ~State() {}
  virtual void draw() = 0;
  virtual void button(const ButtonStruct& btn) = 0;
protected:
  uint16_t set_temperature = 300;
};

class StateWorking : public State {
public:
  ~StateWorking() override {}
  void draw() override {
    uint8_t duty = 75;

    String str_setpoint = "SET ";
    str_setpoint += set_temperature;
    str_setpoint += " C";
    
    oled_string(37, 0, str_setpoint.c_str(), 12, 1);
    oled_string(40, 16, "265", 32, 1);
    oled_string(23, 54, "OFF", 12, 1);
    oled_string(86, 54, "MENU", 12, 1);
    oled_string(102, 20, "AMB", 12, 1);
    oled_string(99,  30, "+25C", 12, 1);
    
    //oled_string(3, 44, "80", 12, 1);
    oled_string(0, 44, "120", 12, 1);

    draw_power_meter(duty);
  }
  void button(const ButtonStruct& btn) {
    if (btn.inc.on_release) {
      set_temperature += 5;
      if (set_temperature > 450) set_temperature = 450;
    }
    if (btn.dec.on_release) {
      set_temperature -= 5;
      if (set_temperature < 150) set_temperature = 150;
    }
  }
};

class StateSleep : public State {
public:
  ~StateSleep() override {}
  void draw() override {
    uint8_t duty = 3;

    oled_string(31, 0, "SLEEP 200 C", 12, 1);
    oled_string(40, 16, "265", 32, 1);
    oled_string(23, 54, "OFF", 12, 1);
    oled_string(86, 54, "MENU", 12, 1);
    oled_string(102, 20, "AMB", 12, 1);
    oled_string(99,  30, "+25C", 12, 1);
    
    //oled_string(3, 44, "80", 12, 1);
    oled_string(6, 44, "5", 12, 1);

    draw_power_meter(duty);
  }
};

class StateOff : public State {
public:
  ~StateOff() override {}
  void draw() override {
    oled_string(55, 0, "OFF", 12, 1);
    oled_string(40, 16, "265", 32, 1);
    oled_string(26, 54, "ON", 12, 1);
    oled_string(86, 54, "MENU", 12, 1);
    oled_string(102, 20, "AMB", 12, 1);
    oled_string(99,  30, "+25C", 12, 1);
    
    oled_char(0, 0, '0' + m_counter, 12, 1);
    m_counter++; if (m_counter > 9) m_counter = 0;
  }
private:
  uint8_t m_counter = 0;
};

class StateTipChange : public State {
public:
  ~StateTipChange() override {}
  void draw() override {
    oled_string(34, 27, "TIP CHANGE", 12, 1);
  }
};

class StateNoTip : public State {
public:
  ~StateNoTip() override {}
  void draw() override {
    oled_string(46, 27, "NO TIP", 12, 1);
  }
};

class StateMenu : public State {
public:
  ~StateMenu() override {}
  void draw() override {
    oled_string(40, 0, "SETTINGS", 12, 1);
    oled_string(20, 54, "EXIT", 12, 1);
    oled_string(80, 54, "SELECT", 12, 1);

    oled_string(0, 16, "DEFAULT", 12, 0);
    oled_string(98, 16, "300 C", 12, 1);

    oled_string(0, 28, "SLEEP", 12, 1);
    oled_string(98, 28, "200 C", 12, 1);

    oled_string(0, 40, "OFF TIME", 12, 1);
    oled_string(92, 40, "30 MIN", 12, 1);
  }
};

void spinner(uint8_t i) {
  for (int j = 0; j < 16; ++j) {
    oled_pixel(127-j, 0, 0);
  }
  if (i < 8) oled_pixel(127-i, 0, 1);
  else       oled_pixel(127-15+i, 0, 1);
}

int main() {
  sei();

  usart_init(3);
  usart_print("\nSoldering Station 245\n");

  io_init();

  pwm_init();
  timer_init();

  usart_print("# Init two wire interface...");
  twi_init();
  usart_print("  ok\n# Init OLED...");
  oled_init();
  usart_print("  ok\n# Switch on display...");
  oled_on();
  usart_print("  ok\n");

  oled_clear();
  oled_display();

  // Arduino UNO bootloader sets MCUSR to 0 :( :( :(
  // Apparently newer versions of the bootloader correct this??

  //auto mcu = MCUSR;
  //MCUSR = 0;

  //Serial.print("MCUSR: "); Serial.print(mcu, BIN); Serial.println();

//  if (mcu & _BV(WDRF)) {
//    oled_string(10, 52, "WATCHDOG RESET", 12, 1); 
//  } else {
//    oled_string(10, 52, "NO WARNINGS", 12, 1); 
//  }

//  adc_set_destination(0, &steps_thermo);
//  adc_set_destination(1, &steps_current);
//  adc_set_destination(3, &steps_voltage);
//  adc_set_destination(8, &steps_ambient);

  //wdt_init();
  adc_start();

  State* state = new StateWorking();

  String serial_input_buffer;

  ButtonStruct btn;

  usart_println("RDY");

  uint8_t i = 0;
  while (1) {
    wdt_reset();

    if (!usart_busy()) {
      uint8_t ch;
      while ((ch = usart_rx())) {
        if (ch == '\r' || ch == '\n') {
          String command = serial_input_buffer;
          command.toUpperCase();
          if (command == "SS") {
            usart_send_oled();
            //while(usart_busy());
          } else {
            usart_print("ERR unknown command: '");
            usart_print(serial_input_buffer.c_str());
            usart_print("'\nRDY\n");
          }
          serial_input_buffer = "";
        } else {
          serial_input_buffer += (char)ch;
        }
      }
    }

    process_buttons(btn);
    state->button(btn);
    if (!twi_has_started()) {
      state->draw();
      spinner(i);
      oled_display();
      i = (i+1)%16;
    }
  }
}

//#include "adc.h"
//#include "oled.h"
//#include "tw.h"
//#include "utils.h"
//
//#include <avr/boot.h>
//#include <avr/wdt.h>
//
//#define TS_OFFSET 5
//#define TS_GAIN 7
//


ISR(PCINT0_vect) {
  usart_println("TWI error: ");
  //Serial.print(twi_get_error(), HEX);
  //Serial.println();
  twi_clear_error();
}

//#define OVERHEAT_TEMP 400000  // 400 Kelvin
//#define OVERCURRENT   8000000 // 8 Amps
//#define IV_BLANKING   127     // no of steps out of 255
//#define TIP_DISCONNECTED 1020 // threshold value of thermocouple (in steps) when tip disconnected
//
//volatile uint16_t steps_voltage = 0;
//volatile uint16_t steps_current = 0;
//volatile uint16_t steps_ambient = 0;
//volatile uint16_t steps_thermo = 0;
//
//volatile int32_t thermocouple = 0;  // in mK (milliKelvin)
//volatile int32_t ambient = 0;
//volatile int32_t tip_temp = 0;
//volatile uint8_t thermo_status = 0;  // 0=ok; 1=overtemp; 2=disconnected
//
//uint32_t voltage = 0;
//uint32_t current = 0;
//
//volatile uint8_t power_disabled = 0;
//
//void read_iv() {
//  //voltage = (uint32_t)steps_voltage * 2452;  // 18V = 1,800,000
//  voltage = (uint32_t)steps_voltage * 2337;  // 18V = 1,800,000 scaled 95.3125%
//
//  //current = (uint32_t)steps_current * 9852;  // 7A = 7,000,000
//  current = (uint32_t)steps_current * 9390;  // 7A = 7,000,000 scaled 95.3125%
//
//  if (current > OVERCURRENT) {
//    power_disabled = 1;
//    pwm_off();
//  }
//}
//
//void read_tip_temperature() {
//  if (steps_thermo > TIP_DISCONNECTED) {
//    thermo_status = 2;
//  } else {
//    // TC characteristic after op-amp: 2.375mV/K
//    // adc 10-bits @ 5.00V, 4.883mV/step
//    // => 2.056K/step
//    thermocouple = (uint32_t)steps_thermo * 2056;
//    tip_temp = ambient + thermocouple;
//    if (tip_temp > OVERHEAT_TEMP) {
//      thermo_status = 1;
//      power_disabled = 2;
//      pwm_off();
//    } else {
//      thermo_status = 0;
//    }
//  }
//}
//
//inline void pwm_set(uint8_t duty) {
//  OCR1A = duty;
//}
//
//uint16_t set_temp = 300;
//int16_t last_error = 0;
//uint16_t power = 0;
//uint16_t last_power = 0;
//uint16_t ki = 6;
//
//void heat_controller() {
//  int16_t error = set_temp - (uint16_t)(tip_temp / 1000);
//
//  if (error < 0) {  // stop heating once set point exceeded
//    if (last_error > 0) {  // crossing above set point
//      last_power = power;
//    }
//    power = 0;
//  } else {
//    if (last_error < 0) {  // crossing below set point
//      power = last_power / 2;  // "take back half"
//    } else {  // already below set point
//      uint16_t margin_to_max = 0xffff - power;
//      uint16_t power_increment = ki * error;
//      if (power_increment > margin_to_max) {
//        power = 0xffff;
//      } else {
//        power += power_increment;
//      }
//    }
//  }
//
//  pwm_set(power >> 9);
//  last_error = error;
//}
//
//ISR(TIMER0_OVF_vect) {
//  read_iv();
//  pwm_off();
//}
//
//ISR(TIMER0_COMPA_vect) {
//  read_tip_temperature();
//  heat_controller();
//  pwm_on();
//}
//
//bool iv_blanking() {
//  return TCNT0 < IV_BLANKING;
//}



//void pwm_off() {
//  CLEAR(PORTB, PORTB1);
//  CLEAR(TCCR1A, COM1A1);
//}
//
//void pwm__on() {
//  SET(TCCR1A, COM1A1);
//}
//
//void pwm_on() {
//  if (power_disabled == 0) {
//    pwm__on();
//  } else {
//    pwm_off();
//  }
//}



void wdt_init() {
  cli();
  wdt_reset();
  
  WDTCSR = _BV(WDCE) | _BV(WDE);
  //WDTCSR |= _BV(WDP0);  // WDP3:0 = 0001; WDT time-out 64ms -> min loop rate of 15Hz
  WDTCSR = _BV(WDE) | _BV(WDP3); // timeout = 4s for testing
  //WDTCSR |= _BV(WDE);
//  wdt_reset();
//  __asm("wdr");
  sei();
}

//void itoa_rj(uint32_t value, char* str, uint8_t slen) {
//  char* tp = str + slen - 1;
//  for (uint8_t i = 0; i < slen; ++i) {
//    *tp-- = (value % 10) + '0';
//    value /= 10;
//  }
//}
//
//int main() {
//  sei();
//
//  int8_t ts_offset = boot_signature_byte_get(TS_OFFSET);
//  uint8_t ts_gain   = boot_signature_byte_get(TS_GAIN);
//  
//  Serial.begin(9600);
//  Serial.println("Soldering station debug");
//  Serial.println("David Zhong 2023");
//  
//  DDRB = _BV(DDB1);   // Set HEATER_PWM as output
//  //PORTB = _BV(DDB0);  // Set pull-up resistor for SLEEP
//  
//  DDRD = 0b00001111;  // Set PD0:3 as output; PD4:7 as input
//  PORTD = _BV(DDD0);  // Set PD0 on to indicate power up
//
//  DIDR0 = 0b00001011;  // Disable digit input for pins ADC0/PC0, ADC1/PC1, ADC3/PC3
//
//  pwm_init();
//  timer_init();
//
//  Serial.print("Init two wire interface..."); Serial.flush();
//  twi_init();
//  Serial.print("  ok\nInit OLED..."); Serial.flush();
// 
//  oled_init();
//
//  Serial.print("  ok\nSwitch on display..."); Serial.flush();
//
//  oled_on();
//
//  Serial.print("  ok\n"); Serial.flush();
//
//  oled_clear();
//  oled_display();
//  /* display images of bitmap matrix */
////  oled_bitmap(0, 2, Signal816, 16, 8); 
////  oled_bitmap(24, 2,Bluetooth88, 8, 8); 
////  oled_bitmap(40, 2, Msg816, 16, 8); 
////  oled_bitmap(64, 2, GPRS88, 8, 8); 
////  oled_bitmap(90, 2, Alarm88, 8, 8); 
////  oled_bitmap(112, 2, Bat816, 16, 8);
//
//  // Arduino UNO bootloader sets MCUSR to 0 :( :( :(
//  // Apparently newer versions of the bootloader correct this??
//
//  auto mcu = MCUSR;
//  MCUSR = 0;
//
//  Serial.print("MCUSR: "); Serial.print(mcu, BIN); Serial.println();
//
////  if (mcu & _BV(WDRF)) {
////    oled_string(10, 52, "WATCHDOG RESET", 12, 1); 
////  } else {
////    oled_string(10, 52, "NO WARNINGS", 12, 1); 
////  }
//
//  adc_set_destination(0, &steps_thermo);
//  adc_set_destination(1, &steps_current);
//  adc_set_destination(3, &steps_voltage);
//  adc_set_destination(8, &steps_ambient);
// 
//  wdt_init();
//  adc_start();
//
//  uint8_t counter = 0;
//  
//
//  while (1) {
//    wdt_reset();
//    //if (!adc_is_converting()) adc_start();
//    
//    //Serial.print("Steps: "); Serial.print(steps, DEC); Serial.print("\n");
//    //uint32_t voltage = ((uint32_t)steps * 2452 / 10);  //
//
//    if (ambient == 0 && steps_ambient != 0) {
//      //ambient = ((int32_t)steps_ambient - (373 - ts_offset)) * 128 / ts_gain + 25;
//      //ambient = (int32_t)steps_ambient * 7819 - 2502906;  // 25C = 25000
//      //ambient = (int32_t)steps_ambient * 35541 - 2502906;  // 25C = 25000
//      //ambient /= 10;
//      ambient = 25000;
//    }
//
//    if (!twi_has_started()) {
//      char voltage_s[8];  // 18V = 0180'0000
//      itoa_rj(voltage, voltage_s, 8);
//      char current_s[8];  // 7A = 0700'0000
//      itoa_rj(current, current_s, 8);
//
//      char lower[] = "+  qq.qV  q.qqA";
//      lower[0]  = counter + '0';
//      lower[3]  = voltage_s[1] != '0' ? voltage_s[1] : ' ';
//      lower[4]  = voltage_s[2];
//      lower[6]  = voltage_s[4];
//      lower[10] = current_s[1];
//      lower[12] = current_s[2];
//      lower[13] = current_s[3];
//
//      uint8_t amb_sign = (ambient >= 0);
//      char ambient_s[6];  // 25C = 025'000
//      itoa_rj((amb_sign ? (uint32_t)ambient : (uint32_t)-ambient),
//              ambient_s, 6);
//
////      char amb[] = "+1234567";
////      amb[0] = amb_sign ? '+' : '-';
////      for (uint8_t i = 0; i < 7; ++i) {
////        amb[i+1] = ambient_s[i];
////      }
//
//      char amb[] = "+yyC";
//      amb[0] = amb_sign ? '+' : '-';
//      amb[1] = ambient_s[1];
//      amb[2] = ambient_s[2];
//
//      char debug1[] = "ttt";
//      char steps1_s[8];
//      itoa_rj(tip_temp, steps1_s, 8);
//      for (uint8_t i = 0; i < 3; ++i) {
//        debug1[i] = steps1_s[i+2];
//      }
//      char debug2[] = "cccccccc";
//      char steps2_s[8];
//      itoa_rj(power, steps2_s, 8);
//      for (uint8_t i = 0; i < 8; ++i) {
//        debug2[i] = steps2_s[i];
//      }
//      char debug3[] = "cccccccc";
//      char steps3_s[8];
//      itoa_rj(last_power, steps3_s, 8);
//      for (uint8_t i = 0; i < 8; ++i) {
//        debug3[i] = steps3_s[i];
//      }
//      char debug4[] = "cccccccc";
//      char steps4_s[8];
//      itoa_rj(last_error, steps4_s, 8);
//      for (uint8_t i = 0; i < 8; ++i) {
//        debug4[i] = steps4_s[i];
//      }
//      
//      oled_string(10, 52, lower, 12, 1); 
//      oled_string(104, 24, amb, 12, 1);
//      oled_string(0, 0, debug1, 12, 1);
//      oled_string(0, 12, debug2, 12, 1);
//      oled_string(0, 24, debug3, 12, 1);
//      oled_string(0, 36, debug4, 12, 1);
//
//      oled_string(48, 0, debug1, 32, 1);
//      //oled_char(48, 0, '4', 32, 1);
//
//      switch (power_disabled) {
//      case 1:
//        oled_string(104, 36, "OC", 12, 1);
//        break;
//      case 2:
//        oled_string(104, 36, "OT", 12, 1);
//        break;
//      default:
//        break;
//      }
//  
////      oled_char3216(0, 16, (hundthousands + 48));
////      oled_char3216(24, 16, steps1_s[2]);
////      oled_char3216(40, 16, steps1_s[3]);
////      oled_char3216(56, 16, steps1_s[4]);
////      oled_char3216(72, 16, 'C');
////      oled_char3216(96, 16, (ones + 48));
//      
//      oled_display();
//
//      counter++;
//      if (counter > 9) counter = 0;
//    }
//
//    //_delay_ms(1);
//    //seconds++;
//    //steps++;
//  }
//}

//void blinky() {
//  PORTB |= _BV(DDB4);
//  _delay_ms(10);
//  PORTB &= ~_BV(DDB4);
//  _delay_ms(10);
//}
