//#define NDEBUG 1

#include <avr/boot.h>
#include <avr/wdt.h>
#include <EEPROM.h>

#pragma GCC diagnostic warning "-Wall"
#pragma GCC diagnostic warning "-Wextra"

#ifdef NDEBUG
#define assert(e) do {} while (0)
#else
void AssertFail(int lineno);
#define assert(e) ((e) ? (void)0 : AssertFail(__LINE__))
#endif

class CTDebugger : public Print {
#ifdef NDEBUG
 public:
  size_t write(uint8_t) { return 0; }
  void print(float) {}
  void println(float) {}
#else
 public:
  virtual size_t write(uint8_t);
 private:
  void Clock(bool val);
#endif
} Debugger;

//#define DO_TRACE 1
#ifdef DO_TRACE
#define TRACELINE(arg) do {                             \
    Debugger.write('t'); Debugger.println(__LINE__);    \
  } while (0)
#else
#define TRACELINE(arg) do {} while (0)
#endif

#include "capsense.h"
#include "tlc5947.h"

// With a 1 MHz clock, random() can take a long time (relatively
// speaking), so we disable it then (and for anything under 4MHz).
// Note: random() is always type "long", so MyRandom behaves similarly.
inline long MyRandom(long max) {
#if F_CPU <= 4000000
  return max;
#else
  return random(max);
#endif
}
inline long MyRandom(long min, long max) {
#if F_CPU <= 4000000
  return max;
#else
  return random(min, max);
#endif
}

// The Arduino doesn't have a built-in random float function, so
// we use this instead.
float MyRandomFloat(float max) {
#if F_CPU <= 4000000
  return max;
#else
  return (max * static_cast<float>(random(LONG_MAX)) / 
          static_cast<float>(LONG_MAX));
#endif
}

constexpr uint16_t kNumGpios = 10;  // Is this in a #defined constant?
constexpr uint16_t kPwmMax = 4095;  // FIXME Push to a class constant upstream

constexpr float kBrightChangeRate = 1.0 / 32;

//#define CANARY 1
constexpr uint8_t kStackCanaryVal = 105;
constexpr uint8_t kStackCanarySweepFreq = 64;

// I use the gamma correction of 2.8, based on the quick tests at
// https://learn.adafruit.com/led-tricks-gamma-correction
// Enabling LED_GAMMA takes about 1k of flash space to get the
// floating point library; since we only have 8k, it ain't worth it!
//#define LED_GAMMA 2.8

// Regarding kCsSamps: Larger is more stable but slower.  The speed
// also depends on the capacitance present; lower capacitance resolves
// more quickly.  With four sensors (well, as of this writing, one
// sensor being probed 4 times), a 1M resistor, and a 3.3v supply,
// kCsSamps=16 samples gives a loop rate of about 42 Hz when there's
// no finger present, and about 20 Hz when all sensors are triggered.
constexpr uint8_t kCsSamps = 16;

// When tuning kCsThres, make sure your hand isn't over the laptop!
constexpr long kCsThres = (long)kCsSamps * 8;
constexpr long kCsThrL = kCsThres / 2;
constexpr long kCsThrH = kCsThres * 3 / 2;

// For mappings between package pin numbers and GPIO pin numbers,
// see http://highlowtech.org/?p=1695 specifically
// http://highlowtech.org/wp-content/uploads/2011/10/ATtiny44-84.png
constexpr uint8_t kPinCsChg = 0;
constexpr uint8_t kPinCsO1S = 1;
constexpr uint8_t kPinCsO2S = 2;
constexpr uint8_t kPinCsO3S = 3;
constexpr uint8_t kPinCsO4S = 7;
constexpr uint8_t kPinCsO5S = 4;
constexpr uint8_t kPinCsO6S = 6;

// We use the same pin for kPinDbgMosi as kPinLedMosi.
// This is the MISO, not MOSI, line on the ICSP header.
constexpr uint8_t kPinDbgMosi = 5;
constexpr uint8_t kPinDbgSclk = 4;

// The debug protocol seems stable at about 100uS hold time in initial
// experiments.  It may be related to the time I spend on the slave
// doing serial writes, since my writes are amplified a lot.  I might
// also try a direct port read in the slave, which is about 25x
// faster.  Note that this is only the hold time for high or low; the
// total clock period is about twice this.  Note that the debug host
// will resync if there's no clock for 10ms, to make sure it doesn't
// get its bits out of phase.
constexpr unsigned int kDbgClkHoldMicros = 250;

constexpr uint8_t kPinLedMosi = 5;
constexpr uint8_t kPinLedSclk = 8;
constexpr uint8_t kPinLedLat = 9;
constexpr uint8_t kPinLedBlank = 10;

constexpr int kPostFailBlinks = 3;

TLC5947<1,                   // Number of drivers
        kPinLedSclk,         // SPI clock pin
        kPinLedMosi,         // SPI data pin
        kPinLedLat> leds;    // Latch pin

#define LED_PHASE1 0
#define LED_PHASE2 1
#define LED_PHASE3 2
constexpr float kLoveStarMaxRamp = 0.75;
constexpr float kPhaseIncrPeace = 0.005;
constexpr float kPhaseIncrLove = 0.05;
//#define LED_SCHMITT 3
//#define LED_COUNT0 3  // Note that COUNT0 == Q
//#define LED_COUNT1 4
//#define LED_COUNT2 5
//#define LED_COUNT3 6
//#define LED_Q 9
//#define LED_NQ 10
//#define LED_CYCLE 11
//#define LED_CAP 12
#define LED_METER_START 15
constexpr uint16_t kLedMeterCount = 5;
constexpr long kLedMeterMax = kCsThres * 2;

constexpr int kNumLedColors = 4;
// The instance, and the referenced arrays, must be in PROGMEM.
class LedStars {
 private:
  class PgmArrayReader {
   public:
    explicit PgmArrayReader(uint8_t const* pgm_array_ptr)
        : pgm_array_ptr_(pgm_array_ptr) {}
    explicit PgmArrayReader(uint8_t const* const* pgm_array_pgm_ptr)
        : pgm_array_ptr_(pgm_read_word(pgm_array_pgm_ptr)) {}
    uint8_t operator[] (size_t i) const {
      return pgm_read_byte(pgm_array_ptr_ + i);
    }
    PgmArrayReader& operator+=(size_t increment) {
      pgm_array_ptr_ += increment;
      return *this;
    }
   private:
    const uint8_t* pgm_array_ptr_;
  };

 public:
  void Setup() const {
    count_ = pgm_read_byte(&pgm_count_);
  }

  PgmArrayReader by_pos() const {
    return PgmArrayReader(&pgm_by_pos_);
  }
  PgmArrayReader by_color() const {
    return PgmArrayReader(&pgm_by_color_);
  }
  PgmArrayReader color_counts() const {
    return PgmArrayReader(pgm_color_counts_);
  }
  uint8_t Count() const {
    return count_;
  }

 public: // FIXME Make these private
  const uint8_t* pgm_by_pos_;
  const uint8_t* pgm_by_color_;
  const uint8_t pgm_color_counts_[kNumLedColors];
  const uint8_t pgm_count_;
  static uint8_t count_;  // Updated once at start of program
};

uint8_t LedStars::count_;

#ifndef NDEBUG
const uint8_t led_stars_dev_by_pos[] PROGMEM = {
  7, 8, 9, 10, 11, 12, 14, 13, 23, 20, 21, 22 };
const uint8_t led_stars_dev_by_color[] PROGMEM = {
  10, 14, 22,  // green
  7, 11, 23,   // red
  8, 12, 20,   // yellow
  9, 13, 21    // blue
};
constexpr LedStars led_stars_dev PROGMEM = {
  .pgm_by_pos_ = led_stars_dev_by_pos,
  .pgm_by_color_ = led_stars_dev_by_color,
  .pgm_color_counts_ = { 3, 3, 3, 3 },
  .pgm_count_ = 12,
};
#endif
const uint8_t led_stars_norm_by_pos[] PROGMEM = {
  3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 14,
  13, 15, 16, 17, 18, 19, 23, 20, 21, 22 };
const uint8_t led_stars_norm_by_color[] PROGMEM = {
  6, 10, 14, 18, 22,    // green
  3, 7, 11, 15, 19, 23, // red
  4, 8, 12, 16, 20,     // yellow
  5, 9, 13, 17, 21,     // blue
};
constexpr LedStars led_stars_norm PROGMEM = {
  .pgm_by_pos_ = led_stars_norm_by_pos,
  .pgm_by_color_ = led_stars_norm_by_color,
  .pgm_color_counts_ = { 5, 6, 5, 5 },
  .pgm_count_ = 21,
};

#ifndef NDEBUG
const LedStars* led_stars;
#else
constexpr const LedStars* led_stars = &led_stars_norm;
#endif
constexpr int kNumLedStarsMax = 21;

#ifndef NDEBUG
bool dev_mode;
#endif
enum DisplayMode { kModeLove, kModePeace, kModeJoy } display_mode = kModePeace;

struct __attribute__ ((__packed__)) {
  bool uninitialized;  // Erased EEPROM reads as all 1s
  float bright_pct;
  DisplayMode display_mode;
} persistent_data;
unsigned long update_persistent_data_at_millis = ULONG_MAX;

// We use the LED on the debugging dongle to tell us if we've ever
// gotten a high reading.  That lets me leave it alone for a long time
// and see if it ever got a false reading.  This tells us if we need
// to send the latch signal.  To prevent spamming the LED latch
// signal, but still let us reset it just by resetting the debug board,
// we only send it at most once per second.
unsigned long debug_led_latch_at_millis;
constexpr unsigned long kDbgLedLatchDelayMs = 1000;

uint16_t bright; // Calculated during startup
float bright_pct = 1.0;

void TwinkleBool(uint16_t ledno, bool val) {
  long delta = display_mode == kModeLove ? MyRandom(bright/128) : 0;
  // The "16 * delta" is because we totally ignore the gamma curve here.
  leds.setPWM(ledno, val ? bright - (16 * delta) : delta);
}

#ifndef NDEBUG
void __attribute__((noinline)) CTDebugger::Clock(bool val) {
  digitalWrite(kPinDbgSclk, val);
  delayMicroseconds(kDbgClkHoldMicros);
}
size_t CTDebugger::write(uint8_t byte) {
  pinMode(kPinDbgSclk, OUTPUT);
  for (int i = 0; i < 8; i++) {
    digitalWrite(kPinDbgMosi, byte & 0x80);
    byte <<= 1;
    Clock(true);
    Clock(false);
  }
  return 1;
}
#endif

#ifndef NDEBUG
void DidFail(void) {
  //wdt_disable();
  //while(1);
}

void AssertFail(int lineno) {
  Debugger.write('A');
  Debugger.println(lineno);
  DidFail();
}
#else  // !NDEBUG
void DidFail(void) {}
#endif // !NDEBUG

void PostFail(uint8_t code, uint8_t aux) {
  uint16_t combined_code = aux << 8 | code;
  for (uint16_t i = 0; i < 16; i++) {
    leds.setPWM(i + 4, (combined_code & ((uint16_t)1 << i)) ? bright : 0);
  }
  digitalWrite(kPinLedBlank, LOW);

#ifndef NDEBUG
  Debugger.write('\x0c');
  Debugger.write('X');
  Debugger.print(code);
  Debugger.write(':');
  Debugger.println(aux);
  for (int i = 0; i < kPostFailBlinks; i++) {
    leds.setPWM(1, 0);  // Red LED
    leds.write();
    delay(1000);
    leds.setPWM(1, bright);
    leds.write();
    delay(1000);
  }
#endif
  DidFail();
}

class CapOrnament {
 public:
  CapOrnament(char name, uint16_t sense_pin) :
      cs_(sense_pin), name_(name), sense_pin_(sense_pin) {}
  bool level() const { return state_.level; };
  void set_led_diagnostic(bool d) { state_.led_diagnostic = d; }
  bool led_diagnostic() const { return state_.led_diagnostic; }
  bool offline() const { return state_.offline; }
  void set_offline(bool offline) { state_.offline = offline; }
  void Post(uint32_t code);
  bool SenseEdge();
  bool SenseLevel() { SenseEdge(); return level(); }
  static uint8_t count(void) { return count_; }
#ifndef NDEBUG
  static void SweepDbgSensor();
#endif

 private:
#ifndef NDEBUG
  static constexpr int kDbgSensorPinShift = 1;
  static constexpr uint16_t kDbgSensorSweepLength =
      kNumGpios << kDbgSensorPinShift;
  static uint8_t dbg_sensor_;
#endif
  static uint8_t count_;

  void PostOneWay(uint32_t code, bool value);
#ifndef NDEBUG
  bool Dbg() { return dbg_sensor_ == (sense_pin_ << kDbgSensorPinShift); }
#endif

  template<int threshold_l, int threshold_h>
  bool Schmitt(int input);
  bool CheckRisingEdge(bool input);

  CapacitiveSensor<kPinCsChg> cs_;
  char name_;
  uint8_t sense_pin_;
  struct {
    bool offline : 1;
    bool led_diagnostic : 1;
    bool level : 1;
  } state_;
};

#ifndef NDEBUG
uint8_t CapOrnament::dbg_sensor_ = 0;
#endif
uint8_t CapOrnament::count_ = 0;

template<int threshold_l, int threshold_h>
bool CapOrnament::Schmitt(int input) {
  if (state_.level) {
    return input > threshold_l;
  } else if (input >= threshold_h) {
    return true;
  } else {
    return false;
  }
}

#ifndef NDEBUG
void CapOrnament::SweepDbgSensor() {
  dbg_sensor_++;
  if (dbg_sensor_ >= kDbgSensorSweepLength) {
    dbg_sensor_ = 0;
  }
}
#endif

bool CapOrnament::SenseEdge() {
  if (state_.offline) {
#ifndef NDEBUG
    if (Dbg()) {
      Debugger.write(name_);
      Debugger.println('P');
    }
#endif
    return false;
  }

  // I tell the Sense routine that it doesn't need to bother after measuring
  // kLedMeterMax, which I assume is going to be greater than kCsThrH.
  long csval = cs_.Sense<kLedMeterMax, kCsSamps>();
  // Add a slight delay if we're using the debugging CLK line, to make
  // sure we didn't just clock in trash.  This will timeout the
  // debugging board's SPI reader.
  if (sense_pin_ == kPinDbgSclk)
      delay(1);
  bool new_level = Schmitt<kCsThrL, kCsThrH>(csval);
#ifndef NDEBUG
  bool any_edge = new_level != state_.level;
#endif
  bool rising_edge = new_level && !state_.level;
  state_.level = new_level;

#ifndef NDEBUG
  if (csval > kCsThres && debug_led_latch_at_millis <= millis()) {
    Debugger.write('\x0c');
    debug_led_latch_at_millis = millis() + kDbgLedLatchDelayMs;
  }
  if (rising_edge) {
    count_++;
    Debugger.write('#');
    Debugger.println(count_);
  }
  if (Dbg() || any_edge) {
    Debugger.write(name_);
    Debugger.write('F');
    Debugger.print(cs_.floor_);
    Debugger.write('C');
    Debugger.print(csval);
    Debugger.write(new_level ? '*' : '_');
    Debugger.write(rising_edge ? '*' : '_');
    Debugger.print("\r\n");
  }
#endif

#ifndef NDEBUG
#if defined(LED_CAP) || defined(LED_METER_START) || defined(LED_SCHMITT)
  if (state_.led_diagnostic) {
#ifdef LED_CAP
    long brightval = constrain(map(csval, 0, kLedMeterMax, 0, bright),
                               0, bright);
    leds.setPWM(LED_CAP, brightval);
#endif
#ifdef LED_SCHMITT
    TwinkleBool(LED_SCHMITT, triggered);
#endif
#ifdef LED_METER_START
    float meter_value = (float)csval * ((float)kLedMeterCount / kLedMeterMax);
    for (uint16_t ledno = 0; ledno < kLedMeterCount; ledno++) {
      if (meter_value < ledno) {
        TwinkleBool(LED_METER_START + ledno, false);
      } else if (meter_value >= ledno + 1) {
        TwinkleBool(LED_METER_START + ledno, true);
      } else {
        uint16_t pwm = bright * (meter_value - ledno);
        leds.setPWM(LED_METER_START + ledno, pwm);
      }
    }
#endif
    leds.write();
  }
#endif  // defined(LED_CAP) || defined(LED_METER_START) || defined(LED_SCHMITT)
#endif  // NDEBUG

  return rising_edge;
}

// Only call this in the POST; I haven't reviewed the CapacitiveSensor
// library to tell whether this will mess with the pin state it uses.
// (I did review the constructor.)
// Note that this will POST with code to code+3, depending on the
// failure mode.  (I haven't allocated all of these failure codes.)
void __attribute__((always_inline)) 
CapOrnament::PostOneWay(uint32_t code, bool val) {
  pinMode(sense_pin_, INPUT);
  digitalWrite(kPinCsChg, val);
  delay(1);
  if (digitalRead(sense_pin_) != val) {
    PostFail(code, name_);
    set_offline(true);
  }
}
void CapOrnament::Post(uint32_t code) {
  PostOneWay(code, HIGH);
  PostOneWay(code + 1, LOW);
}

CapOrnament csBrightUp('+', kPinCsO2S);
CapOrnament csBrightDn('-', kPinCsO1S);
CapOrnament csJoy('J', kPinCsO3S);
CapOrnament csOnOff('O', kPinCsO4S);
CapOrnament csPeace('P', kPinCsO5S);
CapOrnament csLove('L', kPinCsO6S);

uint16_t LedPctToPwm(float pct) {
  pct = constrain(pct, 0.0, 1.0);
#ifdef LED_GAMMA
  return kPwmMax * pow(pct * bright_pct, LED_GAMMA);
#else
  // Approximate
  pct *= bright_pct;
  return kPwmMax * pct * pct * pct;
#endif
}

void UpdateBrightnessFromPct() {
  // We keep bright_pct over 0.1 to prevent this from having an
  // apparent malfunction (that would persist once it writes to EEPROM)
  // if somebody holds it by the "BRIGHT-" button.
  bright_pct = constrain(bright_pct, 0.1, 1.0);
  bright = LedPctToPwm(1.0);
  Debugger.write('B');
  Debugger.println(bright);
}

void LedAll(uint16_t value) {
  for (uint16_t i = 0; i < 24; i++) {
    leds.setPWM(i, value);
  }
  leds.write();
}

#ifndef NDEBUG
void LedFastBurn() {
  // See if we can turn all the LEDs on at once.
  digitalWrite(kPinLedBlank, HIGH);
  delay(20);
  LedAll(bright);
  digitalWrite(kPinLedBlank, LOW);
  delay(3000);
}
void LedSlowBurn() {
  // (To be used to turn up the LEDs slowly during startup)
  // Turn off all LEDs before deasserting BLANK; the PWM controller
  // often sees them as all on during startup, and when it
  // immediately turns them on, it messes with our voltage and
  // causes a BOR.
  LedAll(0);
  digitalWrite(kPinLedBlank, LOW);
  // Turn them on slowly.
  for (uint16_t i = 0; i < 24; i++) {
    leds.setPWM(i, bright);
    leds.write();
    delay(10);
  }
}
#endif  // NDEBUG

#ifndef NDEBUG
// Various startup functions can be requested by holding a finger on
// ON/OFF at the same time as a different pad during reset.  This probes
// for those.
bool ProbeForStartupMode(uint8_t pin)
{
  // Normally, the setup will be:
  //     ON/OFF ---/\/\/\--- Charge ---/\/\/\--- Pin
  // Both resistors are 1M.  We'll put one signal on ON/OFF and a
  // different one on Charge, and see which one gets sensed on Pin.
  // If a user is holding a finger between two of these, we'll see
  // ON/OFF; otherwise, we'll see Charge.

  bool rv = false;
  pinMode(kPinCsO4S, OUTPUT);   // ON/OFF
  digitalWrite(kPinCsChg, LOW);
  digitalWrite(kPinCsO4S, HIGH);
  delay(1);
  if (digitalRead(pin)) {
      digitalWrite(kPinCsChg, HIGH);
      digitalWrite(kPinCsO4S, LOW);
      delay(1);
      if (!digitalRead(pin)) {
          rv = true;
      }
  }
  // Return the pins to their normal states.
  pinMode(pin, INPUT);
  return rv;
}
#endif

void setup() {
  // Save the reset cause, and disable the watchdog.  (We have to
  // clear MCUSR's WDT flag to disable the watchdog.)
  uint8_t old_mcusr = MCUSR;
  MCUSR = 0;
  wdt_disable();

#ifdef CANARY
  // Fill the canary space
  extern char *__bss_end;
  uint8_t *canary_ptr = (uint8_t *)&__bss_end;
  while (canary_ptr <= SP)
    *(canary_ptr++) = kStackCanaryVal;
#endif

  // Blank the LEDs, since they're undefined at startup
  digitalWrite(kPinLedBlank, HIGH);
  pinMode(kPinLedBlank, OUTPUT);

#ifndef NDEBUG
  // Set up the debugging output
  digitalWrite(kPinDbgSclk, LOW);
  //pinMode(kPinDbgSclk, OUTPUT); // Happens during the debugger write
  pinMode(kPinDbgMosi, OUTPUT);
#endif

  // Read the EEPROM so we can get our initial brightness.
  Debugger.write('E');
  EEPROM.get(0, persistent_data);
  if (persistent_data.uninitialized) {
    Debugger.println('-');
  } else {
    Debugger.println('+');
    bright_pct = persistent_data.bright_pct;
    display_mode = persistent_data.display_mode;
  }
  UpdateBrightnessFromPct();

  Debugger.write('P');
  leds.begin();

  // FIXME During the POST, instead of halting operation, we should
  // make a visual indication and then recover (if possible).  For
  // instance, if a particular touch sensor is inoperative, we should
  // just disable that sensor.

  // POST the reset cause
  Debugger.write('1');
  if ((!(old_mcusr & 0x01)) && (old_mcusr != 0x02)) {
    // Often, the brown-out reset will be asserted on the first
    // boot along with the power-on reset.  From what I can tell
    // from the datasheet, the power-on reset finishes above
    // Vcc=1.4V, while my BOR is programmed to 2.7V.  So, if
    // power-on reset is still asserted, I don't check any of the
    // other reset causes.
    PostFail(1, old_mcusr);
  }

  // POST the fuses
  Debugger.write('3');
  if ((CLKPR & 0x0f) != 0) {
    // Clock divider incorrect; set "clock speed" to "8 MHz internal"
    // and reburn bootloader.
    PostFail(3, CLKPR);
  }
  cli();
  uint8_t fuse_low = boot_lock_fuse_bits_get(GET_LOW_FUSE_BITS);
  uint8_t fuse_high = boot_lock_fuse_bits_get(GET_HIGH_FUSE_BITS);
  sei();
  Debugger.write('5');
  if (fuse_low != 0xe2) {
    // Other low fuse bits incorrect; probably reburn bootloader.
    PostFail(5, fuse_low);
  }
  Debugger.write('6');
  if (fuse_high != 0xdd) {
    // I changed the fuse values in the ATtiny defaults in
    // ~/.arduino15/packages/attiny/hardware/avr/1.0.2/boards.txt
    // from 0xdf to 0xdd, which enables the brown-out detector and
    // sets it to 2.7V.  (At 8 MHz, the ATtiny84A needs about
    // 2.5V.)

    // High fuse bits incorrect; probably reburn bootloader.
    // Also can use (untested):
    //     avrdude -p t84 -c usbtiny -U hfuse:w:0xdd:m -u
    PostFail(6, fuse_high);
  }
  // We don't care about the extended fuse bit.

  // Set up the charge line so we can probe for the startup mode.
  // It will remain in OUTPUT mode for the duration of the program.
  pinMode(kPinCsChg, OUTPUT);

#ifndef NDEBUG
  dev_mode = ProbeForStartupMode(kPinCsO3S);
  Debugger.print(dev_mode ? 'D' : 'd');
  if (dev_mode) {
    led_stars = &led_stars_dev;
  } else {
    led_stars = &led_stars_norm;
  }
#endif
  led_stars->Setup();

  // dim_mode determines the initial brightness (which is used for
  // the POST, and if the EEPROM is empty).
#ifndef NDEBUG
  bool dim_mode = ProbeForStartupMode(kPinCsO2S);
  Debugger.print(dim_mode ? 'B' : 'b');
  if (dim_mode) {
    bright_pct = 0.25;
    UpdateBrightnessFromPct();
  }
#endif

  // POST the capacitive sensor path, testing for shorts and opens.
  // Note that these will POST with codes +0 or +1 depending on the
  // failure.
  Debugger.write('+');
  csBrightUp.Post(8);
  Debugger.print('-');
  csBrightDn.Post(12);
#ifndef NDEBUG
  if (!dev_mode) {
#endif
    Debugger.print('J');
    csJoy.Post(16);
    Debugger.print('O');
    csOnOff.Post(20);
#ifndef NDEBUG
  }
#endif
  Debugger.print('L');
  csLove.Post(28);
  Debugger.print('P');
  csPeace.Post(36);

  // FIXME We don't have any feedback from the LED driver.  If we
  // connected its DOUT line to a GPIO, we could verify that we can
  // shift through it, but I don't want to waste a GPIO pin.

  // Only do the visible or slow tests in dev mode; otherwise, just
  // initialize to a blank display.
  LedAll(0);
#ifndef NDEBUG
  if (dev_mode) {
    // Test LEDs
    // In particular, I want to see if there's a voltage drop at Vcc
    // based on the LED power usage.  If there's over about 4 ohms DC
    // resistance on the ground line that's shared with the LED return,
    // then the ground pin can go up and we hit our BOR threshold.  The
    // PCB (as of rev 2 at least) has only 0.05 ohms on the ground line
    // between the PWM controller and the power regulator, but my
    // breadboard can hit 4 ohms pretty easily.  (As of this writing, I
    // only get about a 0.16V drop between all LEDs on and all LEDs off,
    // but an earlier breadboard with a dodgy jumper got a 1V drop.)
    Debugger.write('S');
    LedSlowBurn();
    delay(1000);
    Debugger.write('F');  // See if we can turn all the LEDs on at once.
    digitalWrite(kPinLedBlank, HIGH);
    delay(100);
  }
#endif  // NDEBUG
  digitalWrite(kPinLedBlank, LOW);

#ifndef NDEBUG  
  Debugger.write('Z');
#ifdef LED_Q
  leds.setPWM(LED_Q, 0);
#endif
#ifdef LED_NQ
  leds.setPWM(LED_NQ, 0);
#endif
#ifdef LED_CONST00
  leds.setPWM(LED_CONST00, 0);
  leds.setPWM(LED_CONST25, (bright+1)/4);
  leds.setPWM(LED_CONST50, (bright+1)/2);
  leds.setPWM(LED_CONST75, (3*(bright+1))/4);
  leds.setPWM(LED_CONST100, bright);
#endif
  leds.write();
#endif  // NDEBUG

#ifndef NDEBUG
  if (dev_mode)
    csLove.set_led_diagnostic(true);
#endif  // NDEBUG

  wdt_enable(WDTO_8S);

#ifndef NDEBUG
  Debugger.print(" OK\r\nT");
  Debugger.println(kCsThres);
#endif
}

static union {
  struct {
    float phase;
    int8_t direction;
    uint8_t cycle;
  } joy;
  struct {
    float phase;  // Range is [0,3)
    float star_brightness[kNumLedStarsMax];
    float star_motion[kNumLedStarsMax];
    uint8_t star_ramping_up[(kNumLedStarsMax+7)/8];
  } peace_love;
} display_state;

#ifndef NDEBUG
#define sanity() DoSanity(__LINE__)
void DoSanity(int lineno __attribute__((unused))) {
  //if (/* whatever you're troubleshooting */)
  //  AssertFail(lineno);
}
#else  // !NDEBUG
#define sanity() do {} while (0)
#endif

void __attribute__((noinline)) SetLoveRGBPin(uint8_t pin, float phase) {
  leds.setPWM(pin, LedPctToPwm(
      phase * 0.5 * kLoveStarMaxRamp * kLoveStarMaxRamp));
}

void loop() {
  wdt_reset();
  static uint8_t loop_count;
  loop_count++;
#ifndef NDEBUG
  CapOrnament::SweepDbgSensor();
#endif

  bool bright_down_edge = csBrightDn.SenseEdge();
  bool bright_up_edge = csBrightUp.SenseEdge();
  bool bright_down_level = csBrightDn.level();
  bool bright_up_level = csBrightUp.level();
  if (bright_down_level || bright_up_level) {
    if (bright_down_edge)
      bright_pct -= kBrightChangeRate * 5;
    if (bright_down_level)
      bright_pct -= kBrightChangeRate;
    if (bright_up_edge)
      bright_pct += kBrightChangeRate * 5;
    if (bright_up_level)
      bright_pct += kBrightChangeRate;
    UpdateBrightnessFromPct();
    update_persistent_data_at_millis = millis() + 2000;
  }

  bool toggle_onoff = csOnOff.SenseEdge();
  if (toggle_onoff) {
    static bool blank_state = false;
    blank_state = !blank_state;
    digitalWrite(kPinLedBlank, blank_state);
    Debugger.println(blank_state ? 'o' : 'O');
  }

  bool gotLove = csLove.SenseEdge();
  bool gotJoy = csJoy.SenseEdge();
  bool gotPeace = csPeace.SenseEdge();
  if (gotLove) {
    display_mode = kModeLove;
    memset(&display_state.peace_love, 0, sizeof(display_state.peace_love));
  } else if (gotPeace) {
    display_mode = kModePeace;
    memset(&display_state.peace_love, 0, sizeof(display_state.peace_love));
  } else if (gotJoy) {
    display_mode = kModeJoy;
    display_state.joy = {
      .phase = 0,
      .direction = 1,
      .cycle = 1,
    };
  }

  if (gotLove || gotPeace || gotJoy) {
    persistent_data.display_mode = display_mode;
    update_persistent_data_at_millis = millis() + 2000;
  }

#ifdef LED_CYCLE
  if (dev_mode)
    leds.setPWM(LED_CYCLE, (loop_count & 1) ? 0 : bright);
#endif

  if (display_mode == kModeJoy) {
    auto by_pos = led_stars->by_pos();
#if defined(JOY_BURNS_HOT)
    for (int i = 0; i < led_stars->Count(); i++) {
      uint8_t ledno = by_pos[i];
      leds.setPWM(ledno, bright);
    }
#elif defined(JOY_SHOWS_SCALE)
    for (int i = 0; i < led_stars->Count(); i++) {
      uint8_t ledno = by_pos[i];
      float pct = static_cast<float>(i) / led_stars->Count();
      leds.setPWM(ledno, LedPctToPwm(pct));
    }
#else
    for (int i = 0; i < led_stars->Count(); i++) {
      float pct = 1.0 - abs(display_state.joy.phase - i);
      int ledno = by_pos[i];
      leds.setPWM(ledno, LedPctToPwm(pct));
    }
    display_state.joy.phase += 0.6 * display_state.joy.direction;

    if (display_state.joy.direction == 1 &&
        display_state.joy.phase >= led_stars->Count() - 1) {
      display_state.joy.direction = -1;
      display_state.joy.phase = led_stars->Count() - 1;
      display_state.joy.cycle++;
    } else if (display_state.joy.direction == -1 &&
               display_state.joy.phase <= 0) {
      display_state.joy.direction = 1;
      display_state.joy.phase = 0;
      display_state.joy.cycle++;
    }
    if (display_state.joy.cycle == 8)  // Blackout is unappealing; skip it.
      display_state.joy.cycle = 1;
    leds.setPWM(LED_PHASE1, display_state.joy.cycle & 1 ? bright : 0);
    leds.setPWM(LED_PHASE2, display_state.joy.cycle & 2 ? bright : 0);
    leds.setPWM(LED_PHASE3, display_state.joy.cycle & 4 ? bright : 0);
#endif

  } else {  // PEACE and LOVE modes are handled by mostly-common code.

    // The RGB LED's phase goes through the half-open interval [0,3).
    // In each unit interval along there, two colors are on triangle
    // waves.  This is much, much cheaper than doing cosine waves,
    // since I don't need to link in all the floating-point math
    // stuff!  It also looks nice; if you've seen one of Adafruit's
    // sample "rotating NeoPixel" sketch that comes loaded in
    // CircuitPython from the factory, that's what it does; see
    // https://learn.adafruit.com/adafruit-circuit-playground-express/circuitpython-neopixel
    float phase_r, phase_g, phase_b; // Range is [0,1]
    if (display_state.peace_love.phase < 1.0) {
      phase_r = display_state.peace_love.phase;
      phase_g = 1.0 - display_state.peace_love.phase;
      phase_b = 0;
    } else if (display_state.peace_love.phase < 2.0) {
      phase_r = 2.0 - display_state.peace_love.phase;
      phase_g = 0;
      phase_b = display_state.peace_love.phase - 1.0;
    } else {
      phase_r = 0;
      phase_g = display_state.peace_love.phase - 2.0;
      phase_b = 3.0 - display_state.peace_love.phase;
    }
    if (display_mode == kModePeace) {
      leds.setPWM(LED_PHASE1, LedPctToPwm(phase_r * 0.6));
      leds.setPWM(LED_PHASE2, LedPctToPwm(phase_g * 0.6));
      leds.setPWM(LED_PHASE3, LedPctToPwm(phase_b * 0.6));
      display_state.peace_love.phase += kPhaseIncrPeace;
    } else {
      SetLoveRGBPin(LED_PHASE1, phase_r);
      SetLoveRGBPin(LED_PHASE2, phase_g);
      SetLoveRGBPin(LED_PHASE3, phase_b);
      display_state.peace_love.phase += kPhaseIncrLove;
    }
    if (display_state.peace_love.phase >= 3.0)
      display_state.peace_love.phase -= 3.0;

    constexpr float kLedStarMinMotion = 0.005;
    constexpr float kLedStarDeltaMotion = 0.01;
    constexpr long kLedStarPeriod = 64;
    constexpr long kLedStarBurstPeriod = 128;

    for (int i = 0; i < led_stars->Count(); i++) {
      bool swap_direction = MyRandom(kLedStarPeriod) == 0;
      if ((display_state.peace_love.star_ramping_up[i/8] & (1<<(i%8)))) {
        display_state.peace_love.star_brightness[i] +=
            display_state.peace_love.star_motion[i];
        if (display_state.peace_love.star_brightness[i] >= 1.0) {
          display_state.peace_love.star_brightness[i] = 1.0;
          swap_direction = true;
        }
      } else {
        display_state.peace_love.star_brightness[i] -=
            display_state.peace_love.star_motion[i];
        if (display_state.peace_love.star_brightness[i] <= 0) {
          display_state.peace_love.star_brightness[i] = 0;
          swap_direction = true;
        }
      }
      if (swap_direction) {
        display_state.peace_love.star_motion[i] =
            MyRandomFloat(kLedStarDeltaMotion) + kLedStarMinMotion;
        display_state.peace_love.star_ramping_up[i/8] ^= 1 << (i%8);
      }
    }

    auto by_color = led_stars->by_color();
    if (display_mode == kModePeace) {
      // In kModePeace, the goal is to not be distracting in a room
      // while you're reading a book or something.  One part of that
      // is that we have each color have a constant total
      // illumination.  FIXME We should do the averaging after gamma
      // correction (i.e., in PWM space) since the gamma correction is
      // only for human perception issues; with PWM modulation, LEDs
      // are (I think) pretty linear in actual power output.
      auto color_counts = led_stars->color_counts();
      uint8_t color_idx_ofs = 0;
      for (int color = 0; color < kNumLedColors; color++) {
        uint8_t color_count = color_counts[color];
        float total = 0;
        for (uint8_t star_idx = 0; star_idx < color_count; star_idx++) {
          float pct = display_state.peace_love.star_brightness[
              color_idx_ofs + star_idx];
          total += pct;
        }
        float bias = (color_count * 0.5 - total) / color_count;
        for (uint8_t star_idx = 0; star_idx < color_count; star_idx++) {
          float pct = display_state.peace_love.star_brightness[
              color_idx_ofs + star_idx];
          pct += bias;
          uint8_t ledno = by_color[star_idx + color_idx_ofs];
          uint16_t pwm = LedPctToPwm(pct);
          leds.setPWM(ledno, pwm);
        }
        color_idx_ofs += color_count;
      }
    } else {  // kModeLove
      for (int i = 0; i < led_stars->Count(); i++) {
        bool burst = MyRandom(kLedStarBurstPeriod) == 0;
        uint16_t half_bright = bright >> 1;
        uint16_t pwm;
        if (burst) {
          pwm = MyRandom(half_bright, bright);
        } else {
          uint16_t nominal = LedPctToPwm(
              kLoveStarMaxRamp * display_state.peace_love.star_brightness[i]);
          uint16_t half_nominal = nominal >> 1;
          pwm = MyRandom(half_nominal) + half_nominal;
          if (nominal < half_bright) {
            pwm += half_nominal;
          }
        }
        int ledno = by_color[i];
        leds.setPWM(ledno, pwm);
      }
    }
  }  // kModePeace and kModeLove
  
#ifndef NDEBUG
  if (dev_mode) {
#ifdef LED_Q
    TwinkleBool(LED_Q, CapOrnament::count() % 2);
#endif
#ifdef LED_NQ
    TwinkleBool(LED_NQ, !(CapOrnament::count() % 2));
#endif
#ifdef LED_COUNT4
    TwinkleBool(LED_COUNT4, CapOrnament::count() & 0x10);
#endif
#ifdef LED_COUNT3
    TwinkleBool(LED_COUNT3, CapOrnament::count() & 0x08);
#endif
#ifdef LED_COUNT2
    TwinkleBool(LED_COUNT2, CapOrnament::count() & 0x04);
#endif
#ifdef LED_COUNT1
    TwinkleBool(LED_COUNT1, CapOrnament::count() & 0x02);
#endif
#ifdef LED_COUNT0
    TwinkleBool(LED_COUNT0, CapOrnament::count() & 0x01);
#endif
  }
#endif  //NDEBUG

  leds.write();

  if (update_persistent_data_at_millis < millis()) {
    persistent_data.uninitialized = false;
    persistent_data.bright_pct = bright_pct;
    Debugger.println('E');
    EEPROM.put(0, persistent_data);
    update_persistent_data_at_millis = ULONG_MAX;
  }

#ifdef CANARY
  // Check the canary space
  if (loop_count % kStackCanarySweepFreq == 0) {
    extern char *__bss_end;
    extern uint8_t  __stack;
    uint8_t *canary_ptr = (uint8_t *)&__bss_end;
    uint16_t canary_bytes = 0;
    while (*canary_ptr == kStackCanaryVal &&
           canary_ptr <= (uint8_t *)&__stack) {
      canary_ptr++;
      canary_bytes++;
    }
    Debugger.write('C');
    Debugger.println(canary_bytes);
    assert(canary_bytes > 8);
  }
#endif

#if 0
  if (loop_count % 8 == 0) {
    for (int i = 0; i < 24; i++) {
      Debugger.print(i);
      Debugger.write(':');
      Debugger.println(leds.pwmbuffer[i]);
    }
  }
#endif
  sanity();
}

// Local Variables:
// compile-command: "~/arduino-1.8.3/arduino --upload --board attiny:avr:ATtinyX4:cpu=attiny84,clock=internal8 christmas-tree.ino"
// End:
