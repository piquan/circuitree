#ifndef CAPSENSE_H
#define CAPSENSE_H

// This CapacitiveSensor class is similar to the one at
// https://github.com/PaulStoffregen/CapacitiveSensor .
//
// Some differences:
//
// * Values that are unlikely to change within a program have been
//   moved to templated values, so they can be inlined and don't need
//   to take up memory.  For instance, the charge pin is typically
//   shared among all sensors in a program, so it's now a templated
//   constant.
//
// * A ceiling of the sensor reading has been added, so that the
//   program doesn't need to wait for a sensor to completely charge
//   before deciding that the capacitance is above its desired
//   threshold.  (This replaces the timeout value.)
//
// * The recalibration has been changed to be continual instead of
//   periodic, by having the floor slowly drift upwards each time
//   Sense() is called.  This is sometimes better than the
//   alternative, but not always.
//
// * Error management has been removed entirely.  The caller is
//   expected to know if the requested pins do not exist.

template<uint8_t charge_pin>
class CapacitiveSensor {
 public:
  // We take multiple samples as a simple low-pass filter, to smooth
  // out rapid spikes.
  static constexpr uint8_t kDefaultSampleCount = 16;
  CapacitiveSensor(uint8_t sense_pin) : sense_pin_(sense_pin) {}
  CapacitiveSensor(const CapacitiveSensor&) = delete;
  CapacitiveSensor& operator=(const CapacitiveSensor&) = delete;
  template<uint16_t ceiling = UINT_MAX,
           uint8_t sample_count = kDefaultSampleCount>
  uint16_t Sense();
  // floor_ is only public for debugging reasons.
  uint16_t floor_ = 0xffff;
 private:
  // The floor adjustment will cause buttons that are held down to
  // eventually drift to an "off" state.  Be sure to make the drift
  // sufficiently slow.
  static constexpr uint16_t kFloorDriftRate = 2;
  uint8_t sense_pin_;
};

template<uint8_t charge_pin>
template<uint16_t ceiling, uint8_t sample_count>
uint16_t CapacitiveSensor<charge_pin>::Sense() {
  // Set up the pins to the needed modes
  pinMode(charge_pin, OUTPUT);
  digitalWrite(charge_pin, LOW);
  digitalWrite(sense_pin_, LOW);

  uint8_t bit = digitalPinToBitMask(sense_pin_);
  uint8_t port = digitalPinToPort(sense_pin_);
  volatile uint8_t *input_register = portInputRegister(port);

  uint16_t actual_ceiling;
  if (floor_ <= 0xffffU - ceiling) {
    actual_ceiling = floor_ + ceiling;
  } else {
    actual_ceiling = 0xffff;
  }
  actual_ceiling = 0xffff;  

  uint16_t count = 0;
  for (uint8_t sample_i = 0; sample_i < sample_count; sample_i++) {
    // Alternate the target voltage
    bool target = sample_i & 1;

    // Discharge the sensor (the last iteration will have put us where
    // we want).
    digitalWrite(charge_pin, !target);
    digitalWrite(sense_pin_, !target);
    pinMode(sense_pin_, OUTPUT);
    delayMicroseconds(10);
    pinMode(sense_pin_, INPUT);
    
    // Turn on the charge pin
    digitalWrite(charge_pin, target);
    // See how long it takes before the sense pin detects the change
    while (1) {
      if (count >= actual_ceiling)
        break;
      if (target == !!(*input_register & bit))
        break;
      count++;
    }
  }
  // Leave it low for the next run.
  pinMode(charge_pin, INPUT);
  pinMode(sense_pin_, OUTPUT);
  digitalWrite(sense_pin_, LOW);

  if (count < floor_)
    floor_ = count;

  uint16_t rv = count - floor_;
  floor_ += kFloorDriftRate;
  return rv;
}

#endif
