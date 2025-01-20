#pragma once

#define SET(reg, n)    reg |= ((uint8_t)1 << n)
#define CLEAR(reg, n)  reg &= ~((uint8_t)1 << n)
#define TOGGLE(reg, n) reg ^= ((uint8_t)1 << n)
#define ISSET(reg, n)  ((reg >> n) & (uint8_t)1)

#define clamp(var, lower, upper) (((var) > (upper)) ? (upper) : (((var) < (lower)) ? (lower) : (var)))

void* operator new(size_t n) {
  void* const p = malloc(n);
  //@TODO: Handle p==nullptr
  return p;
}

void operator delete(void* p) {
  free(p);
}

void operator delete(void* p, size_t) {
  free(p);
}

namespace util {

namespace {
size_t strcpy_limited(char* dst, size_t dest_size, const char* src) {
  /*
     Copies null-terminated string `src` into `dst`, but only a maximum
     of (`dest_size` - 1) bytes. `dst` is null-terminated.
     Returns the number of (non-null) bytes copied.
  */
  for (size_t i = 0; i < (dest_size - 1); ++i) {
    dst[i] = src[i];
    if (src[i] == '\0') return i;
  }
  dst[dest_size] = '\0';
  return (dest_size - 1);
}

class RadixBin { };
class RadixDec { };
class RadixHex { };

template <size_t N>
class string {
public:
  // Constructors
  constexpr string() : m_buffer(0), m_length(0) { }
  constexpr string(const string& other) {
    m_length = strcpy_limited(this->m_buffer, N, other.m_buffer);
  }
  //constexpr string(const char (&s)[N]) {
  //  m_length = strcpy_limited(m_buffer, N, s);
  //}
  constexpr string(const char* s) {
    m_length = strcpy_limited(m_buffer, N, s);
  }
  string& operator=(const string& other) {
    m_length = strcpy_limited(this->m_buffer, N, other.m_buffer);
    return *this;
  }
  string& operator=(const char* s) {
    m_length = strcpy_limited(m_buffer, N, s);
    return *this;
  }

  // Accessors
  const char* data() const {
    return m_buffer;
  }
  const char* c_str() const {
    return m_buffer;
  }
  char operator[](size_t i) const {
    if (i >= m_length) return '\0';
    return m_buffer[i];
  }

  // Capacity
  constexpr bool empty() const {
    return m_length == 0;
  }
  constexpr bool full() const {
    return m_length >= (N - 1);
  }
  constexpr size_t size() const {
    return m_length;
  }
  constexpr size_t capacity() const {
    return N;
  }

  // Modifiers
  void clear() {
    m_buffer[0] = '\0';
    m_length = 0;
  }
  void insert(size_t pos, char ch) {
    if (pos >= (N - 1)) return;
    if (pos > m_length) return;
    if (!empty()) {
      size_t last = m_length - 1;
      append(m_buffer[last]);

      for (size_t i = last; i > pos; --i) {
        m_buffer[i] = m_buffer[i - 1];
      }
    }
    if (empty()) m_length = 1;
    m_buffer[pos] = ch;
  }
  template <size_t M>
  void append(const string<M>& s) {
    m_length += strcpy_limited(m_buffer + m_length, (N - m_length), s.data());
  }
  void append(char ch) {
    if (full()) return;
    m_buffer[m_length] = ch;
    m_buffer[m_length + 1] = '\0';
    ++m_length;
  }
  void append(const char *s) {
    m_length += strcpy_limited(m_buffer + m_length, (N - m_length), s);
  }
  void append(int16_t value) {
    append(value, RadixDec());
  }
  void append(int16_t value, RadixDec) {
    if (value < 0) {
      append('-');
      value = -value;
    }
    size_t insertion_point = m_length;
    while (value >= 10) {
      insert(insertion_point, (value % 10) + '0');
      value /= 10;
    }
    insert(insertion_point, value + '0');
  }
  void append(int16_t value, RadixBin) {
  }
  void append(int16_t value, RadixHex) {
  }
  void to_uppercase() {
    for (size_t i = 0; i < m_length; ++i) {
      if (m_buffer[i] >= 'a' && m_buffer[i] <= 'z') {
        m_buffer[i] -= ('a' - 'A');
      }
    }
  }
  void to_lowercase() {
    for (size_t i = 0; i < m_length; ++i) {
      if (m_buffer[i] >= 'A' && m_buffer[i] <= 'Z') {
        m_buffer[i] += ('a' - 'A');
      }
    }
  }

  // Comparators
  //constexpr bool operator==(const string& other) {
  //  if (this->m_length != other.m_length) return false;
  //  for (size_t i = 0; i < m_length; ++i) {
  //    if (this->m_buffer[i] != other.m_buffer[i]) return false;
  //  }
  //  return true;
  //}
  //constexpr bool operator!=(const string& other) {
  //  return !(*this == other);
  //}
  constexpr bool operator==(const char* other) {
    for (size_t i = 0; i < m_length; ++i) {
      if (other[i] == '\0') return false;
      if (m_buffer[i] != other[i]) return false;
    }
    return true;
  }
  constexpr bool operator!=(const char* other) {
    return !(*this == other);
  }

private:
  char m_buffer[N];
  size_t m_length;
};

}  // anonymous namespace

using string8 = string<8>;
using string16 = string<16>;

[[maybe_unused]] static RadixBin BIN;
[[maybe_unused]] static RadixDec DEC;
[[maybe_unused]] static RadixHex HEX;

}  // namespace util
