#define PIN_SCLK 6
#define PIN_MOSI 7
#define PIN_NRST 8

class RisingEdge {
 public:
  bool Value(bool input);
 private:
  bool triggered = false;
};

// Only returns true if the input moves from false to true
bool RisingEdge::Value(bool input) {
  if (input == false) {
    triggered = false;
    return false;
  } else if (triggered) {
    return false;
  } else {
    triggered = true;
    return true;
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  pinMode(PIN_SCLK, INPUT);
  pinMode(PIN_MOSI, INPUT);
  pinMode(PIN_NRST, INPUT);
}

void loop() {
  static uint8_t held_char;
  static int held_char_bits = 0;
  static RisingEdge clk_rise_detect;
  static RisingEdge rst_rise_detect;

#ifdef SHOW_BITS
  static char buf[70];
  static size_t buflen = 0;
#endif

  static bool led_rest_state = LOW;

  static unsigned long last_clk_at = 0;
  constexpr unsigned long kMinClkMicros = 800;
  bool rst = !digitalRead(PIN_NRST);
  bool rst_rise = rst_rise_detect.Value(rst);
  // Always run the clock signal through the rising edge detector,
  // even if RST is asserted, to make sure we don't lose an edge.
  bool clk = digitalRead(PIN_SCLK);
  bool clk_rise = clk_rise_detect.Value(clk);

  while (Serial.available()) {
    uint8_t byte = Serial.read();
    switch (byte) {
      case 'r':
        Serial.print(">>> RST");
        pinMode(PIN_NRST, OUTPUT);
        digitalWrite(LED_BUILTIN, HIGH);
        held_char_bits = 0;
#ifdef SHOW_BITS
        buflen = 0;
#endif
        delay(500);
        Serial.print("\r\n");
        digitalWrite(LED_BUILTIN, LOW);
        pinMode(PIN_NRST, INPUT);
        break;
      case '\r':
      case '\n':
        break;
      default:
        Serial.println(">?");
        break;
    }
  }

  if (rst_rise) {
    digitalWrite(LED_BUILTIN, HIGH);
    held_char_bits = 0;
#ifdef SHOW_BITS
    buflen = 0;
#endif
    Serial.println("*** RST");
  }
  if (rst) {
    return;
  }
  if (clk_rise) {
    unsigned long now = micros();
    if (held_char_bits && (last_clk_at + kMinClkMicros < now)) {
      //Serial.println("*** CLK");
      held_char_bits = 0;
    }
    last_clk_at = now;
    bool bit = digitalRead(PIN_MOSI);
#ifdef SHOW_BITS
    Serial.print(bit);
#endif
    held_char = (held_char << 1) | (bit ? 1 : 0);
    held_char_bits++;
    if (held_char_bits == 8) {
#ifdef SHOW_BITS
      Serial.write(' ');
      Serial.print(held_char, HEX);
      Serial.write(' ');
#endif
      switch (held_char) {
#ifdef SHOW_BITS
        // End of line characters
        case '\r':
          Serial.println("\\r");
          buflen = 0;
          break;
        case '\n':
          Serial.println("\\n\r\n");  // Make an extra line break
          buflen = 0;
          break;
#endif
        // Special control characters
        case 0:  // Noise from reusing the CLK line as a cap sensor
        case 0xff:
#ifdef SHOW_BITS
          Serial.println("----\r\n");
#endif
          break;
        case '\x0c':  // Turn LED on solid
          led_rest_state = HIGH;
          Serial.println("*** LED latched");
#ifdef SHOW_BITS
          buflen = 0;
#endif
          break;
        case '\x18':  // Turn LED to normal operation
          led_rest_state = LOW;
          Serial.println("*** LED unlatched");
#ifdef SHOW_BITS
          buflen = 0;
#endif
          break;
        // Normal characters
        default:
#ifdef SHOW_BITS
          buf[buflen] = held_char;
          buf[buflen + 1] = '\0';
          Serial.println(buf);
          buflen++;
          if (buflen >= sizeof(buf) - 2) {
            buflen = 0;
          }
#else
          Serial.write(held_char);
#endif
          break;
      }
      held_char_bits = 0;
    }
  }
  digitalWrite(LED_BUILTIN, held_char_bits ? HIGH : led_rest_state);
}

// Local Variables:
// compile-command: "~/arduino-1.8.3/arduino --upload --board arduino:avr:uno --port /dev/ttyACM0 spi-slave.ino"
// End:
