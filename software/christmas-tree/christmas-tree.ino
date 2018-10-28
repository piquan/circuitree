#include <avr/boot.h>
#include <avr/wdt.h>
#include <EEPROM.h>

#pragma GCC diagnostic warning "-Wall"
#pragma GCC diagnostic warning "-Wextra"

void AssertFail(int lineno);
#define assert(e) ((e) ? (void)0 : AssertFail(__LINE__))

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

#define CANARY 1
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
constexpr long kCsThres = (long)kCsSamps * 16;
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
constexpr float kPhaseIncr = 0.05;
//#define LED_SCHMITT 3
#define LED_COUNT0 3  // Note that COUNT0 == Q
#define LED_COUNT1 4
#define LED_COUNT2 5
#define LED_COUNT3 6
//#define LED_Q 9
//#define LED_NQ 10
//#define LED_CYCLE 11
//#define LED_CAP 12
#define LED_METER_START 15
constexpr uint16_t kLedMeterCount = 5;
constexpr long kLedMeterMax = kCsThres * 2;
const uint8_t led_stars_dev[] PROGMEM = {
  7, 8, 9, 10, 11, 12, 13, 14, 20, 21, 22, 23 };
constexpr int kNumLedStarsDev =
    sizeof(led_stars_dev) / sizeof(led_stars_dev[0]);
const uint8_t led_stars_norm[] PROGMEM = {
  3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13,
  14, 15, 16, 17, 18, 19, 20, 21, 22, 23 };
constexpr int kNumLedStarsNorm =
    sizeof(led_stars_norm) / sizeof(led_stars_norm[0]);
const uint8_t* led_stars;
int num_led_stars;
constexpr int kNumLedStarsMax = kNumLedStarsNorm;

bool dev_mode;
enum DisplayMode { kModeLove, kModePeace, kModeJoy } display_mode;

struct __attribute__ ((__packed__)) {
  bool uninitialized;  // Erased EEPROM reads as all 1s
  float bright_pct;
  DisplayMode display_mode;
} persistent_data;
unsigned long update_persistent_data_at_millis;

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

class CTDebugger : public Print {
 public:
  virtual size_t write(uint8_t);
 private:
  void Clock(bool val);
} Debugger;

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

void DidFail(void) {
  //wdt_disable();
  //while(1);
}

void AssertFail(int lineno) {
  Debugger.write('A');
  Debugger.println(lineno);
  DidFail();
}

void PostFail(uint8_t code, uint8_t aux) {
  uint16_t combined_code = aux << 8 | code;
  for (uint16_t i = 0; i < 16; i++) {
    leds.setPWM(i + 4, (combined_code & ((uint16_t)1 << i)) ? bright : 0);
  }
  digitalWrite(kPinLedBlank, LOW);

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
  static void SweepDbgSensor();
  static uint8_t count(void) { return count_; }

 private:
  static constexpr int kDbgSensorPinShift = 1;
  static constexpr uint16_t kDbgSensorSweepLength = kNumGpios << kDbgSensorPinShift;
  static uint8_t count_;
  static uint8_t dbg_sensor_;

  void PostOneWay(uint32_t code, bool value);
  bool Dbg() { return dbg_sensor_ == (sense_pin_ << kDbgSensorPinShift); }

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

uint8_t CapOrnament::dbg_sensor_ = 0;
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

void CapOrnament::SweepDbgSensor() {
  dbg_sensor_++;
  if (dbg_sensor_ >= kDbgSensorSweepLength) {
    dbg_sensor_ = 0;
  }
}

bool CapOrnament::SenseEdge() {
  if (state_.offline) {
    if (Dbg()) {
      Debugger.write(name_);
      Debugger.println('P');
    }
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
  bool any_edge = new_level != state_.level;
  bool rising_edge = new_level && !state_.level;
  state_.level = new_level;

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
#endif

  return rising_edge;
}

// Only call this in the POST; I haven't reviewed the CapacitiveSensor
// library to tell whether this will mess with the pin state it uses.
// (I did review the constructor.)
// Note that this will POST with code to code+3, depending on the
// failure mode.  (I haven't allocated all of these failure codes.)
void __attribute__((always_inline)) 
CapOrnament::PostOneWay(uint32_t code, bool val) {
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

// Note: the pad assignments of + and - flip in the next board revision!
CapOrnament csBrightUp('+', kPinCsO1S);
CapOrnament csBrightDn('-', kPinCsO2S);
CapOrnament csJoy('J', kPinCsO3S);
CapOrnament csOnOff('O', kPinCsO4S);
CapOrnament csPeace('P', kPinCsO5S);
CapOrnament csLove('L', kPinCsO6S);

uint16_t LedPctToPwm(float pct) {
#ifdef LED_GAMMA
  return kPwmMax * pow(pct * bright_pct, LED_GAMMA);
#else
  // Approximate
  return kPwmMax * bright_pct * bright_pct * bright_pct * pct * pct * pct;
#endif
}

void UpdateBrightnessFromPct() {
  bright_pct = constrain(bright_pct, 1.0 / kPwmMax, 1.0);
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
  pinMode(kPinCsChg, OUTPUT);
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
  pinMode(kPinCsO4S, INPUT);
  return rv;
}

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

  // Set up the debugging output
  digitalWrite(kPinDbgSclk, LOW);
  pinMode(kPinDbgSclk, OUTPUT);
  pinMode(kPinDbgMosi, OUTPUT);

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

  dev_mode = ProbeForStartupMode(kPinCsO3S);
  Debugger.print(dev_mode ? 'D' : 'd');
  if (dev_mode) {
    led_stars = &led_stars_dev[0];
    num_led_stars = kNumLedStarsDev;
  } else {
    led_stars = &led_stars_norm[0];
    num_led_stars = kNumLedStarsNorm;
  }

  // dim_mode determines the initial brightness (which is used for
  // the POST, and if the EEPROM is empty).
  bool dim_mode = ProbeForStartupMode(kPinCsO2S);
  Debugger.print(dim_mode ? 'B' : 'b');
  if (dim_mode) {
    bright_pct = 0.05;
    UpdateBrightnessFromPct();
  }

  // POST the capacitive sensor path, testing for shorts and opens.
  // Note that these will POST with codes +0 or +1 depending on the
  // failure.
  Debugger.write('+');
  csBrightUp.Post(8);
  Debugger.print('-');
  csBrightDn.Post(12);
  if (!dev_mode) {
    Debugger.print('J');
    csJoy.Post(16);
    Debugger.print('O');
    csOnOff.Post(20);
  }
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

  if (dev_mode)
    csLove.set_led_diagnostic(true);

  wdt_enable(WDTO_8S);

  Debugger.print(" OK\r\nT");
  Debugger.println(kCsThres);
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

#define sanity() DoSanity(__LINE__)
void DoSanity(int lineno __attribute__((unused))) {
  if (leds.pwmbuffer[0] == 0 && leds.pwmbuffer[1] == 0 && 
      leds.pwmbuffer[2] == 151)
    AssertFail(lineno);
}

void loop() {
  wdt_reset();
  static uint8_t loop_count;
  loop_count++;
  CapOrnament::SweepDbgSensor();

  bool bright_down_edge = csBrightDn.SenseEdge();
  bool bright_up_edge = csBrightUp.SenseEdge();
  bool bright_down_level = csBrightDn.level();
  bool bright_up_level = csBrightUp.level();
  assert(!(bright_up_level && bright_down_level));
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
    static bool blank_state = !blank_state;
    digitalWrite(kPinLedBlank, blank_state);
    Debugger.write(blank_state ? 'o' : 'O');
    Debugger.println(bright);
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
    display_state.joy.phase = 0;
    display_state.joy.direction = 1;
    display_state.joy.cycle = 1;
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
    for (int i = 0; i < num_led_stars; i++) {
      int ledno = pgm_read_byte(led_stars + i);
      leds.setPWM(ledno, bright);
    }

#if 0
    for (int i = 0; i < num_led_stars; i++) {
      float pct = 1.0 - abs(display_state.joy.phase - i);
      int ledno = pgm_read_byte(led_stars + i);
      leds.setPWM(ledno, pct <= 0 ? 0 : LedPctToPwm(pct));
    }
    display_state.joy.phase += 0.6 * display_state.joy.direction;

    if (display_state.joy.direction == 1 && display_state.joy.phase >= num_led_stars - 1) {
      display_state.joy.direction = -1;
      display_state.joy.phase = num_led_stars - 1;
      display_state.joy.cycle++;
    } else if (display_state.joy.direction == -1 && display_state.joy.phase <= 0) {
      display_state.joy.direction = 1;
      display_state.joy.phase = 0;
      display_state.joy.cycle++;
    }
    if (display_state.joy.cycle == 8)  // Blackout looks bad; skip it.
      display_state.joy.cycle = 1;
    leds.setPWM(LED_PHASE1, display_state.joy.cycle & 1 ? bright : 0);
    leds.setPWM(LED_PHASE2, display_state.joy.cycle & 2 ? bright : 0);
    leds.setPWM(LED_PHASE3, display_state.joy.cycle & 4 ? bright : 0);
#endif

  } else {  // JOY and PEACE modes are handled by mostly-common code.

    // FIXME These are modes that the user may want to leave it in as
    // a decoration in the corner of the living room.  To avoid being
    // a flashing distraction, I should normalize the brightness so
    // that every color always has a particular brightness.  I can
    // reasonably assume that the C150??KT LEDs all have a consistent
    // brightness / PWM curve, and can probably ignore the gamma.

    // The phase goes through the half-open interval [0,3).  In each
    // unit interval along there, two colors are on triangle waves.
    // This is much, much cheaper than doing cosine waves, since I
    // don't need to link in all the floating-point math stuff!  It
    // also looks nice; if you've seen one of Adafruit's sample
    // "rotating NeoPixel" sketch that comes loaded in CircuitPython
    // from the factory, that's what it does; see
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
    // We square the brightness curve, and drop it by a factor of 4,
    // since the RGB LED is really bright!
    leds.setPWM(LED_PHASE1, phase_r * bright * bright_pct * 0.25);
    leds.setPWM(LED_PHASE2, phase_g * bright * bright_pct * 0.25);
    leds.setPWM(LED_PHASE3, phase_b * bright * bright_pct * 0.25);
    display_state.peace_love.phase += kPhaseIncr;
    if (display_state.peace_love.phase >= 3.0)
      display_state.peace_love.phase -= 3.0;

    constexpr float kLedStarMaxRamp = 0.5;
    constexpr float kLedStarMinMotion = 0.005;
    constexpr float kLedStarDeltaMotion = 0.01;
    constexpr long kLedStarPeriod = 64;
    constexpr long kLedStarBurstPeriod = 128;
    for (int i = 0; i < num_led_stars; i++) {
      bool swap_direction = MyRandom(kLedStarPeriod) == 0;
      if ((display_state.peace_love.star_ramping_up[i/8] & (1<<(i%8)))) {
        display_state.peace_love.star_brightness[i] += display_state.peace_love.star_motion[i];
        if (display_state.peace_love.star_brightness[i] >= kLedStarMaxRamp) {
          display_state.peace_love.star_brightness[i] = kLedStarMaxRamp;
          swap_direction = true;
        }
      } else {
        display_state.peace_love.star_brightness[i] -= display_state.peace_love.star_motion[i];
        if (display_state.peace_love.star_brightness[i] <= 0) {
          display_state.peace_love.star_brightness[i] = 0;
          swap_direction = true;
        }
      }
      if (swap_direction) {
        display_state.peace_love.star_motion[i] = MyRandomFloat(kLedStarDeltaMotion) +
                                          kLedStarMinMotion;
        display_state.peace_love.star_ramping_up[i/8] ^= 1 << (i%8);
      }
      
      uint16_t pwm;
      if (display_mode == kModeLove) {
        uint16_t pmin, pmax;
        if (display_state.peace_love.star_brightness[i] < 0.5) {
          pmin = LedPctToPwm(display_state.peace_love.star_brightness[i]);
          pmax = LedPctToPwm(display_state.peace_love.star_brightness[i] * 1.5);
        } else {
          pmin = LedPctToPwm(display_state.peace_love.star_brightness[i] * 0.75);
          pmax = LedPctToPwm(display_state.peace_love.star_brightness[i]);
        }
        bool burst = MyRandom(kLedStarBurstPeriod) == 0;
        pwm = burst ? MyRandom(bright/2, bright) : MyRandom(pmin, pmax);
      } else {
        pwm = LedPctToPwm(display_state.peace_love.star_brightness[i]);
      }
      int ledno = pgm_read_byte(led_stars + i);
      leds.setPWM(ledno, pwm);
    }
  }
  
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
  
  leds.write();

  if (update_persistent_data_at_millis &&
      (update_persistent_data_at_millis <= millis())) {
    persistent_data.uninitialized = false;
    persistent_data.bright_pct = bright_pct;
    Debugger.println('E');
    //EEPROM.put(0, persistent_data);
    update_persistent_data_at_millis = 0;
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
