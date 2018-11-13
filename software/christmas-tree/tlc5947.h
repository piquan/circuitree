#ifndef TLC5947_H
#define TLC5947_H 1

// This is mostly compatible with the Adafruit TLC5947 driver, and is based
// on it, but uses templates to avoid the need to malloc.  In most cases,
// you know at runtime how many drivers you'll be using, and using malloc
// adds about 1k of flash and a little RAM.
//
// While I'm at it, I also use templates for the clock, MOSI, and latch,
// to avoid needing to store them in RAM.

template <uint16_t numdrivers, uint8_t clk, uint8_t dat, uint8_t lat>
class TLC5947 {
public:
  static constexpr uint16_t kPwmMax = 4095;
  static constexpr uint16_t kNumLeds = 24;
  void begin(void);
  void setPWM(uint16_t chan, uint16_t pwm);
  void setLED(uint16_t lednum, uint16_t r, uint16_t g, uint16_t b);
  void write(void);
private:
  uint16_t pwmbuffer[numdrivers * kNumLeds];
  friend void DoSanity(int lineno);
};

template <uint16_t numdrivers, uint8_t clk, uint8_t dat, uint8_t lat>
void TLC5947<numdrivers, clk, dat, lat>::write(void) {
  digitalWrite(lat, LOW);
  for (int16_t c = kNumLeds * numdrivers - 1; c >= 0; c--) {
    uint16_t pwm_value = pwmbuffer[c];
    for (int8_t b = 0; b < 12; b++) {
      digitalWrite(clk, LOW);
      digitalWrite(dat, !!(pwm_value & 2048));
      digitalWrite(clk, HIGH);
      pwm_value <<= 1;
    }
  }
  digitalWrite(clk, LOW);
  digitalWrite(lat, HIGH);
  digitalWrite(lat, LOW);
}

template <uint16_t numdrivers, uint8_t clk, uint8_t dat, uint8_t lat>
void TLC5947<numdrivers, clk, dat, lat>::setPWM(uint16_t chan, uint16_t pwm) {
  pwmbuffer[chan] = pwm;
}

template <uint16_t numdrivers, uint8_t clk, uint8_t dat, uint8_t lat>
void TLC5947<numdrivers, clk, dat, lat>::setLED(
    uint16_t lednum, uint16_t r, uint16_t g, uint16_t b) {
  setPWM(lednum*3, r);
  setPWM(lednum*3+1, g);
  setPWM(lednum*3+2, b);
}

template <uint16_t numdrivers, uint8_t clk, uint8_t dat, uint8_t lat>
void TLC5947<numdrivers, clk, dat, lat>::begin() {
  digitalWrite(lat, LOW);
  pinMode(clk, OUTPUT);
  pinMode(dat, OUTPUT);
  pinMode(lat, OUTPUT);
}

#endif
