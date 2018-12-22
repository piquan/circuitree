// tlc5947.h - A library for interfacing with the TLC5947 LED driver
//
// Copyright (C) 2018 Joel Ray Holveck
//
// Based on the Adafruit TLC5947 library at
// https://github.com/adafruit/Adafruit_TLC5947/
// which bears the following notice:
// ***************************************************
// This is a library for our Adafruit 24-channel PWM/LED driver
// Pick one up today in the adafruit shop!
// ------> http://www.adafruit.com/products/1429
// These drivers uses SPI to communicate, 3 pins are required to
// interface: Data, Clock and Latch. The boards are chainable
// Adafruit invests time and resources providing this open source code,
// please support Adafruit and open-source hardware by purchasing
// products from Adafruit!
// Written by Limor Fried/Ladyada for Adafruit Industries.
// BSD license, all text above must be included in any redistribution
// ***************************************************
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
//     (1) Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//
//     (2) Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in
//     the documentation and/or other materials provided with the
//     distribution.
//
//     (3)The name of the author may not be used to
//     endorse or promote products derived from this software without
//     specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
// IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT,
// INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
// STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
// IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef TLC5947_H
#define TLC5947_H 1

// This is mostly compatible with the Adafruit TLC5947 driver, and is based
// on it, but uses templates to avoid the need to malloc.  In most cases,
// you know at runtime how many drivers you'll be using, and using malloc
// adds about 1k of flash and a little RAM.
//
// While I'm at it, I also use templates for the clock, MOSI, and latch,
// to avoid needing to store them in RAM.  Instead, the compiler hard-codes
// them into the source code.

template <uint16_t numdrivers, uint8_t clk, uint8_t dat, uint8_t lat>
class TLC5947 {
public:
  static constexpr uint16_t kPwmMax = 4095;
  static constexpr uint16_t kNumLeds = 24;
  void begin(void);
  uint16_t& operator[](size_t chan) { return pwmbuffer[chan]; }
  void setPWM(uint16_t chan, uint16_t pwm) { (*this)[chan] = pwm; }
  void setLED(uint16_t lednum, uint16_t r, uint16_t g, uint16_t b);
  void write(void);
  void clear(void);
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

template <uint16_t numdrivers, uint8_t clk, uint8_t dat, uint8_t lat>
void TLC5947<numdrivers, clk, dat, lat>::clear() {
  memset(pwmbuffer, 0, numdrivers * kNumLeds * sizeof(*pwmbuffer));
}

#endif
