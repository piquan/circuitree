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

// I could make this inherit from Print to get printf, etc.  But the
// decimal print routines take up too much flash space (about 300
// bytes), so I print in hex, which is much cheaper.
class CTDebugger final {
#ifdef NDEBUG
 public:
  void write(uint8_t) {}
  template<typename T> void print(T) {}
  template<typename T> void println(T) {}
#else
 private:
  const char hexdigs[16] = {'0', '1', '2', '3', '4', '5', '6', '7',
                            '8', '9', 'a', 'b', 'c', 'd', 'e', 'f'};
  void PrintByte(uint8_t val) {
    write(hexdigs[val >> 4]);
    write(hexdigs[val & 0x0f]);
  }
 public:
  void write(uint8_t);
  void print(uint8_t val) {
    PrintByte(val);
  }
  void print(uint16_t val) {
    PrintByte(val >> 8);
    PrintByte(val);
  }
  void print(uint32_t val) {
    PrintByte(val >> 24);
    PrintByte(val >> 16);
    PrintByte(val >> 8);
    PrintByte(val);
  }
  void print(const char* val) {
    while (*val) {
      write(*(val++));
    }
  }
  void print(int8_t val) {
    if (val < 0) {
      write('-');
      print(static_cast<uint8_t>(-val));
    } else {
      print(static_cast<uint8_t>(val));
    }
  }
  void print(int16_t val) {
    if (val < 0) {
      write('-');
      print(static_cast<uint16_t>(-val));
    } else {
      print(static_cast<uint16_t>(val));
    }
  }
  void print(int32_t val) {
    if (val < 0) {
      write('-');
      print(static_cast<uint32_t>(-val));
    } else {
      print(static_cast<uint32_t>(val));
    }
  }
  void print(float val);
  void print(double val) { print(static_cast<float>(val)); }
  void println() {
    write('\r');
    write('\n');
  }
  template<typename T> void println(T val) {
    print(val);
    println();
  }
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

// The Arduino doesn't have a built-in random float function, so I
// have this instead.  It ain't perfect by a long shot, but it works
// for my needs.
float RandomFloat(float max) {
  return (max * static_cast<float>(random()) / static_cast<float>(LONG_MAX));
}

constexpr uint16_t kNumGpios = 10;  // Is this in a #defined constant?

constexpr float kBrightChangeRate = 1.0 / 32;

// Note that CANARY takes about 132 bytes.
#define CANARY 1
constexpr uint8_t kStackCanaryVal = 105;
constexpr uint8_t kStackCanarySweepFreq = 64;

// Regarding kCsSamps: Larger is more stable but slower.  The speed
// also depends on the capacitance present; lower capacitance resolves
// more quickly.  With four sensors (well, as of this writing, one
// sensor being probed 4 times), a 1M resistor, and a 3.3v supply,
// kCsSamps=16 samples gives a loop rate of about 42 Hz when there's
// no finger present, and about 20 Hz when all sensors are triggered.
constexpr uint8_t kCsSamps = 16;

// When tuning kCsThres, make sure your hand isn't over the laptop!
constexpr unsigned int kCsThres = kCsSamps * 6;
constexpr unsigned int kCsThrL = kCsThres / 2;
constexpr unsigned int kCsThrH = kCsThres * 3 / 2;

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
// This is the MISO (not MOSI) line on the ICSP header.
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
  3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 14, 13, 23, 20, 21, 22 };
const uint8_t led_stars_dev_by_color[] PROGMEM = {
  6, 10, 14, 22,  // green
  3, 7, 11, 23,   // red
  4, 8, 12, 20,   // yellow
  5, 9, 13, 21    // blue
};
constexpr LedStars led_stars_dev PROGMEM = {
  .pgm_by_pos_ = led_stars_dev_by_pos,
  .pgm_by_color_ = led_stars_dev_by_color,
  .pgm_color_counts_ = { 4, 4, 4, 4 },
  .pgm_count_ = 16,
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
constexpr int kNumLedStarsMax = led_stars_norm.pgm_count_;

#ifndef NDEBUG
bool dev_mode;
#endif
enum DisplayMode { kModeLove, kModePeace, kModeJoy } display_mode = kModeLove;
enum JoySubMode { kSubModeCylon, kSubModeDoubleChase, kSubModeLast };
JoySubMode joy_sub_mode = kSubModeCylon;

// FIXME Why are these separate from the top-level variables?
struct __attribute__ ((__packed__)) {
  bool uninitialized;  // Erased EEPROM reads as all 1s
  float bright_pct;
  DisplayMode display_mode;
  JoySubMode joy_sub_mode;
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
float bright_pct = 0.5;

void __attribute__((noinline)) TwinkleBool(uint16_t ledno, bool val) {
  // The twinkling of boolean outputs in debug mode is cute, but it takes
  // 90 bytes or so of code space.
#ifdef DO_TWINKLE_BOOL
  long delta = display_mode == kModeLove ? random(bright/128) : 0;
  // The "16 * delta" is because we totally ignore the gamma curve here.
  leds.setPWM(ledno, val ? bright - (16 * delta) : delta);
#else
  leds.setPWM(ledno, val ? bright : 0);
#endif
}

#ifndef NDEBUG
void __attribute__((noinline)) CTDebugger::Clock(bool val) {
  digitalWrite(kPinDbgSclk, val);
  delayMicroseconds(kDbgClkHoldMicros);
}
void CTDebugger::write(uint8_t byte) {
  pinMode(kPinDbgSclk, OUTPUT);
  for (int i = 0; i < 8; i++) {
    digitalWrite(kPinDbgMosi, byte & 0x80);
    byte <<= 1;
    Clock(true);
    Clock(false);
  }
}

// Takes about 186 bytes (if it's actually called)
void
CTDebugger::print(float value_in) {
  union {
    // Essentially defines the IEEE 784-2008 binary16 layout
    struct {
      unsigned int significand: 10;
      unsigned int exponent: 5;
      unsigned int sign: 1;
    } bits;
    float flo;
  } value;
  value.flo = value_in;

  // Print the sign even if it's a NaN.  This saves some code space,
  // and also distinguishes between quiet (negative) and signaling
  // (positive) NaNs.
  if (value.bits.sign) {
    write('-');
  }
  
  // Check NaN and infinity
  if (value.bits.exponent == 0x1f) {
    if (value.bits.significand != 0) {
      print("nan");
    } else {
      print("inf");
    }
    return;
  }
  
  if (value.bits.exponent == 0) {
    write('0');
  } else {
    write('1');
  }
  write('.');

  print(value.bits.significand >> 2);
  print(value.bits.significand << 6);
  
  // We use 'x' to separate the significand from the exponent.  The
  // C++ %a modifier uses 'p', but it prints the exponent in decimal,
  // so we avoid that.
  write('x');
  int8_t norm_exponent = value.bits.exponent - 15;
  print(norm_exponent);
}
#endif

void DidFail(void) {
  // Use this if you want to halt the board after failures (e.g., during
  // overnight testing).
  //wdt_disable();
  //while(1);
}

#ifndef NDEBUG
void AssertFail(int lineno) {
  Debugger.write('A');
  Debugger.println(lineno);
  DidFail();
}
#endif

void PostFail(uint8_t code, uint8_t aux) {
  uint16_t combined_code = (aux << 8) | code;
  for (uint16_t i = 4; i < 20; i++) {
    leds.setPWM(i, combined_code & 1 ? leds.kPwmMax : 0);
    combined_code >>= 1;
  }
  digitalWrite(kPinLedBlank, LOW);

#ifndef NDEBUG
  Debugger.write('\x0c');
  Debugger.write('X');
  Debugger.print(code);
  Debugger.write(':');
  Debugger.println(aux);
  for (uint8_t i = 0; i < kPostFailBlinks; i++) {
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
  void Post(uint8_t code);
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

  void PostOneWay(uint8_t code, bool value);
#ifndef NDEBUG
  bool Dbg() { return dbg_sensor_ == (sense_pin_ << kDbgSensorPinShift); }
#endif

  template<unsigned int threshold_l, unsigned int threshold_h>
  bool Schmitt(unsigned int input);
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

template<unsigned int threshold_l, unsigned int threshold_h>
bool CapOrnament::Schmitt(unsigned int input) {
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
  uint16_t csval = cs_.Sense<kLedMeterMax, kCsSamps>();
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
    Debugger.println();
  }
#endif

#if !defined(NDEBUG) && (defined(LED_CAP) || defined(LED_METER_START) || defined(LED_SCHMITT))
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
    float meter_value =
        static_cast<float>(csval) * kLedMeterCount / kLedMeterMax;
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
#endif  // !NDEBUG && (LED_CAP || LED_METER_START || LED_SCHMITT)

  return rising_edge;
}

// Only call this in the POST; I haven't reviewed the CapacitiveSensor
// library to tell whether this will mess with the pin state it uses.
// (I did review the constructor.)
// Note that this will POST with code to code+3, depending on the
// failure mode.  (I haven't allocated all of these failure codes.)
void __attribute__((always_inline))
CapOrnament::PostOneWay(uint8_t code, bool val) {
  pinMode(sense_pin_, INPUT);
  digitalWrite(kPinCsChg, val);
  delay(1);
  if (digitalRead(sense_pin_) != val) {
    PostFail(code, name_);
    set_offline(true);
  }
}
void CapOrnament::Post(uint8_t code) {
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
  // I use a gamma of 3, since I don't need this to be terribly good,
  // and a gamma of 3 can be implemented without needing pow() (which
  // takes a lot of space).
  pct *= bright_pct;
  return leds.kPwmMax * pct * pct * pct;
}

void UpdateBrightnessFromPct() {
  // We keep bright_pct over 0 to prevent this from having an apparent
  // malfunction (that would persist once it writes to EEPROM) if
  // somebody holds it by the "BRIGHT-" button.  0.15 is about the
  // minumum to ensure that the RGB LED is always on regardless of
  // mode.
  bright_pct = constrain(bright_pct, 0.15, 1.0);
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

  // We don't clear them intially, since our caller does that.
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
    joy_sub_mode = persistent_data.joy_sub_mode;
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
    // Also can use this, but I had problems with it on a
    // factory-fresh ATtiny84A:
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
  leds.clear();
  // This is mostly useful for testing how the power supply can manage
  // sudden demand changes, but it's also handy to get confirmation of
  // dev_mode.
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
  digitalWrite(kPinLedBlank, LOW);

#ifndef NDEBUG
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
  Debugger.write('T');
  Debugger.println(kCsThres);
#endif
}

static union {
  struct {
    float phase;
    int8_t direction;
    uint8_t cycle;
    uint8_t cycle_minor;
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
      phase * kLoveStarMaxRamp * kLoveStarMaxRamp));
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

  // We always reset the display, even if we're in the right mode, to
  // give visual feedback to the user.  Otherwise, the user might
  // start fiddling with buttons by pressing the current mode and
  // think that the button does nothing.
  if (gotLove) {
    display_mode = kModeLove;
  } else if (gotPeace) {
    display_mode = kModePeace;
  } else if (gotJoy) {
    if (display_mode == kModeJoy) {
      joy_sub_mode = 
          static_cast<JoySubMode>(static_cast<int>(joy_sub_mode) + 1);
      if (joy_sub_mode == kSubModeLast) {
        joy_sub_mode = static_cast<JoySubMode>(0);
      }
    }
    display_mode = kModeJoy;
  }

  if (gotLove || gotPeace || gotJoy) {
    memset(&display_state, 0, sizeof(display_state));
    persistent_data.display_mode = display_mode;
    persistent_data.joy_sub_mode = joy_sub_mode;
    update_persistent_data_at_millis = millis() + 2000;
  }

#ifdef LED_CYCLE
  if (dev_mode)
    leds.setPWM(LED_CYCLE, (loop_count & 1) ? 0 : bright);
#endif

  if (display_mode == kModeJoy) {
    auto by_pos = led_stars->by_pos();
    switch (joy_sub_mode) {
      case kSubModeCylon:
        {
          for (int i = 0; i < led_stars->Count(); i++) {
            float pct = 1.0 - abs(display_state.joy.phase - i);
            int ledno = by_pos[i];
            leds.setPWM(ledno, LedPctToPwm(pct));
          }
          display_state.joy.phase += (display_state.joy.direction ? 0.6 : -0.6);
          if (display_state.joy.direction == 1 &&
              display_state.joy.phase >= led_stars->Count() - 1) {
            display_state.joy.direction = 0;
            display_state.joy.phase = led_stars->Count() - 1;
            display_state.joy.cycle++;
          } else if (display_state.joy.direction == 0 &&
                     display_state.joy.phase <= 0) {
            display_state.joy.direction = 1;
            display_state.joy.phase = 0;
            display_state.joy.cycle++;
          }
          if (display_state.joy.cycle == 8) // Blackout is unappealing; skip it
            display_state.joy.cycle = 1;
          leds.setPWM(LED_PHASE1,
                      display_state.joy.cycle & 1 ? bright >> 2 : 0);
          leds.setPWM(LED_PHASE2,
                      display_state.joy.cycle & 2 ? bright >> 2 : 0);
          leds.setPWM(LED_PHASE3,
                      display_state.joy.cycle & 4 ? bright >> 2 : 0);
        }
        break;
      case kSubModeDoubleChase:
        {
          for (uint16_t i = 0; i < led_stars->Count(); i++) {
            uint8_t i_8bit = i * 256 / led_stars->Count();
            // Compute the absolute value of i_8bit - cycle
            uint8_t mag_major = i_8bit - display_state.joy.cycle;
            if (mag_major & 0x80)
              mag_major = ~mag_major;
            uint8_t mag_minor = i_8bit - display_state.joy.cycle_minor;
            if (mag_minor & 0x80)
              mag_minor = ~mag_minor;
            // mag_major and mag_minor now hold 7 bits of data each.  It's
            // a little tricky to do gamma correction in integer space
            // without losing too much precision.  Unfortunately, 32-bit
            // arithmetic is quite expensive on an AVR, and 16-bit only
            // slightly less so.  Our inputs are only 8 bits, so we're ok
            // with some loss of magnitude precision during this stage,
            // as long as we don't lose it all up-front.
            uint8_t bright_8bit = bright / 16;           // 0-255
            uint16_t mag_14bit = mag_major * mag_minor;  // 0-16129
            uint16_t mag_gamma = mag_14bit / 64;         // 0-252
            mag_gamma *= bright_8bit;                    // 0-64260
            mag_gamma /= 256;                            // 0-251
            mag_gamma *= bright_8bit;                    // 0-64005
            uint16_t pwm = mag_gamma / 16;               // 0-4000
            uint8_t ledno = by_pos[i];
            leds.setPWM(ledno, pwm);
          }

          // FIXME The LED_PHASE* stuff is about 230-240 bytes; can I
          // come up with something smaller?
          // This pretty much sets LED_PHASE1 to cycle^2 * bright^2,
          // scaled accordingly and in stages to prevent overflow.
          uint8_t bright15 = bright / 256;  // 0-15
          bright15 *= bright15;             // 0-225
          bright15 /= 16;                   // 0-14
          bright15 += 2;                    // 2-16
          uint16_t phase1 =
              ((display_state.joy.cycle * bright15)  // 0-4080
               / 16)                                 // 0-255
              * (display_state.joy.cycle / 8);       // 0-8128
          // Then, the sawtooth is transformed to a triangle: values
          // above the brightness become a descending value within the
          // brightness.
          if (phase1 > bright * 2)
            phase1 = bright;
          else if (phase1 > bright)
            phase1 = (bright * 2) - phase1;
          // We scale back the brightness since the RGB LED is pretty
          // bright.
          leds.setPWM(LED_PHASE1, phase1 / 4);
          // LED_PHASE2 is similar to LED_PHASE1, but uses cycle_minor.
          uint16_t phase2 =
              ((display_state.joy.cycle_minor * bright15) / 16)
              * (display_state.joy.cycle_minor / 8);
          if (phase2 > bright * 2)
            phase2 = bright;
          else if (phase2 > bright)
            phase2 = (bright * 2) - phase2;
          leds.setPWM(LED_PHASE2, phase2 / 4);
          // LED_PHASE3 roughly inverts the other two phases.
          uint16_t phase3 = bright - ((phase1 + phase2) / 2);
          leds.setPWM(LED_PHASE3, phase3 / 4);

          display_state.joy.cycle += 1;
          display_state.joy.cycle_minor -= 2;
        }
        break;
      case kSubModeLast:
        // CANTHAPPEN
        break;
    }

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
    display_state.peace_love.phase +=
        display_mode == kModePeace ? kPhaseIncrPeace : kPhaseIncrLove;
    // The color wheel looks better if we don't do the gamma
    // correction here.  However, we do back off by a constant factor,
    // since the RGB LED is otherwise too bright for comfort.  The
    // constant is different depending on the mode, since the ramp on
    // Peace mode is twice as high as the ramp on Love mode.
    float rgb_backoff = display_mode == kModePeace ? 0.4 : 0.2;
    leds.setPWM(LED_PHASE1, phase_r * rgb_backoff * bright);
    leds.setPWM(LED_PHASE2, phase_g * rgb_backoff * bright);
    leds.setPWM(LED_PHASE3, phase_b * rgb_backoff * bright);
    if (display_state.peace_love.phase >= 3.0)
      display_state.peace_love.phase -= 3.0;

    constexpr float kLedStarMinMotion = 0.005;
    constexpr float kLedStarDeltaMotion = 0.01;
    constexpr long kLedStarPeriod = 64;
    constexpr long kLedStarBurstPeriod = 128;

    for (int i = 0; i < led_stars->Count(); i++) {
      bool swap_direction = random(kLedStarPeriod) == 0;
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
            RandomFloat(kLedStarDeltaMotion) + kLedStarMinMotion;
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
        bool burst = random(kLedStarBurstPeriod) == 0;
        uint16_t half_bright = bright >> 1;
        uint16_t pwm;
        if (burst) {
          pwm = random(half_bright, bright);
        } else {
          uint16_t nominal = LedPctToPwm(
              kLoveStarMaxRamp * display_state.peace_love.star_brightness[i]);
          uint16_t half_nominal = nominal >> 1;
          pwm = random(half_nominal) + half_nominal;
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
// compile-command: "~/arduino-1.8.3/arduino --upload --board attiny:avr:ATtinyX4:cpu=attiny84,clock=internal8 --preserve-temp-files --pref build.path=build/ christmas-tree.ino"
// End:
