#pragma once

template <typename T, size_t N, T sentinel>
class RingBuffer {
public:
  using data_type = T;
  using size_type = size_t;
  size_type capacity() const {
    return N;
  }
  size_type length() const volatile {
    if (write_index > read_index) return (write_index - read_index);
    else return (write_index + N - read_index);
  }
  bool full() const volatile {
    return (((write_index + 1) % N) == read_index);
  }
  bool empty() const volatile {
    return (write_index == read_index);
  }
  void push(data_type element) {
    if (full()) return;
    buf[write_index] = element;
    write_index = (write_index + 1) % N;
  }
  data_type pop() {
    if (empty()) return sentinel;
    data_type retval = buf[read_index];
    read_index = (read_index + 1) % N;
    return retval;
  }
private:
  volatile char buf[N];
  volatile size_type write_index = 0;
  volatile size_type read_index = 0;
};

RingBuffer<char, 32, 0> _usart_tx_buffer;


void usart_init(uint16_t ubrr) {
  // Set baud rate
  UBRR0H = (uint8_t)(ubrr >> 8);
  UBRR0L = (uint8_t)ubrr;
  // Enable transceiver
  UCSR0B = _BV(RXEN0) | _BV(TXEN0);
  // Set frame format for 8 bits with 2 stop bits
  UCSR0C = _BV(USBS0) | (3<<UCSZ00);
}

uint8_t usart_rx() {
  if (UCSR0A & _BV(RXC0)) {
    return UDR0;
  }
  return 0;
}

void _usart_interrupt_enable() {
  UCSR0B |= _BV(UDRIE0);
}

void _usart_interrupt_disable() {
  UCSR0B &= ~(_BV(UDRIE0));
}

void usart_char(char ch) {
  while (_usart_tx_buffer.full());
  cli();
  _usart_tx_buffer.push(ch);
  sei();
  _usart_interrupt_enable();
}

void usart_print(const char* str) {
  for (const char* cp = str; *cp != 0; ++cp) {
    usart_char(*cp);
  }
}

void usart_print_P(const char* str) {
  const char* cp = str;
  while (1) {
    char ch = pgm_read_byte(cp);
    if (ch == 0) break;
    usart_char(ch);
    ++cp;
  }
}

void usart_println(const char* str) {
  usart_print(str);
  usart_char('\n');
}

volatile uint8_t* _usart_cp = nullptr;

bool usart_busy() {
  return (UCSR0B & _BV(UDRIE0));
}

void usart_send_oled() {
  uint32_t len = sizeof(oled_buf) / sizeof(*oled_buf);
  String meta = "$";
  meta += len;
  meta += "\n";
  usart_print(meta.c_str());
  _usart_cp = oled_buf;
  
}

ISR(USART_UDRE_vect) {
  if (!_usart_tx_buffer.empty()) {
    UDR0 = _usart_tx_buffer.pop();
    return;
  }

  if (_usart_cp) {
    if (_usart_cp < (oled_buf + _oled_bufsiz)) {
      UDR0 = *_usart_cp++;
    } else {
      _usart_cp = nullptr;
      usart_print("\nRDY\n");
    }
    return;
  }
  
  _usart_interrupt_disable();
}
