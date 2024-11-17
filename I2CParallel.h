// (c) Copyright 2021-2024 Aaron Kimball
// This library is licensed under the terms of the BSD 3-Clause license.
// See the accompanying LICENSE.txt file for full license text.
//
// Library to use the PCF8574 / PCF8574A
// Also for pin-compatible fast-mode devices like NXP PCA8574[A].
//
// Remote 8-Bit I/O Expander for I2C Bus
// See datasheet: https://www.ti.com/lit/ds/symlink/pcf8574a.pdf
//
// This depends on the <Wire> library for I2C communication.
// This library is lightweight: just 4 bytes of memory and < 100 bytes of code.

#ifndef I2C_PARALLEL_H
#define I2C_PARALLEL_H

#include <Arduino.h>

#include <inttypes.h>

#include <Wire.h>

// The two chips are identical, except for their configurable address ranges:
// PCF8574  address bits: 0 1 0 0 A2 A1 A0 0
// PCF8574A address bits: 0 1 1 1 A2 A1 A0 0

// The PCF8574 uses addresses 0x20 .. 0x27
#define I2C_PCF8574_MIN_ADDR ((uint8_t)0x20)
#define I2C_PCF8574_MAX_ADDR ((uint8_t)0x27)

// The PCF8574A uses addresses 0x38 .. 0x3F
#define I2C_PCF8574A_MIN_ADDR ((uint8_t)0x38)
#define I2C_PCF8574A_MAX_ADDR ((uint8_t)0x3F)

#define I2C_SPEED_FAST ((uint32_t)400000L)
#define I2C_SPEED_STANDARD ((uint32_t)100000L)
// The PCF8574 standard is specified at 100kHz but TI-manufactured chips are
// rated for 400kHz "fast mode" I2C. (See p.12 of
// https://www.ti.com/lit/an/scpa032/scpa032.pdf) The pin-compatible NXP-mfr'd
// PCA8574 is also 400kHz device.
#ifndef I2C_PARALLEL_MAX_BUS_SPEED
#define I2C_PARALLEL_MAX_BUS_SPEED I2C_SPEED_FAST
#endif /* I2C_PARALLEL_MAX_BUS_SPEED */

// This is an 8-bit unsigned int output device.
#define I2C_PARALLEL_MAX_VAL ((uint8_t)0xFF)

// bitmasks can refer to bits 0--7 in the output byte.
#define I2C_MAX_BIT_POS ((uint8_t)7)

// The time delay from I2C acknowledge until the output is valid.
// (Also the hold time needed for driven inputs before the data can be reported
// back.)
#define I2C_PARALLEL_HOLD_TIME_MICROS ((unsigned int)4)

class I2CParallel {
public:
  I2CParallel();
  ~I2CParallel(){};

  // Configure the 8-bit parallel bus with its expected 7-bit I2C address.
  // This must be within one of the two supported MIN_ADDR .. MAX_ADDR ranges.
  void init(const uint8_t i2cAddr,
            const uint32_t busSpeed = I2C_PARALLEL_MAX_BUS_SPEED);

  // Configure the specified pin as the recipient of the INT_L signal from
  // the I2C parallel bus. The specified isr method will be called when INT_L
  // is pulled low by the PCF8574. Add a pull-up between this pin and Vcc.
  void initInterrupt(const uint8_t digitalPinNum, void (*isr)());

  // Set the value to emit on the 8-bit bus. This value is latched and held
  // until overwritten by another setByte() or driven by the other side of the
  // bus. Returns the number of bytes written (1 on success, 0 on failure).
  size_t setByte(const uint8_t val);
  // Synonym for setByte().
  size_t write(const uint8_t val) { return setByte(val); };

  // Read back the current contents of the 8-bit bus. The bus can be driven by
  // connected logic on any lines that we have set to HIGH via setByte().
  // On power-up, all data lines initialize to high (getByte() == 0xFF).
  // This returns the value received and sets nBytesRead to 1 or 0 depending on
  // whether or not it has successfully performed the I/O read. If bytesRead is
  // 0, the output should not be used.
  uint8_t getByte(uint8_t &nBytesRead);
  uint8_t getByte() {
    uint8_t numReceived = 0;
    return getByte(numReceived);
  };
  uint8_t read() { return getByte(); }; // synonym for getByte().

  // Read back the last known contents of the bus without actually reading over
  // i2c.
  uint8_t getLastState() const { return _state; };

  // Configure some data lines as inputs according to the specified mask.  The
  // masked data lines will be allowed to pull up to logic HIGH and can then be
  // driven by the connected device(s). Subsequently writing a logic LOW to any
  // bits with setByte() will drive those lines low and disable input mode.
  void enableInputs(const uint8_t mask) { setOr(mask); };

  // Apply a bitwise operation to the current bus state.
  size_t setOr(const uint8_t val) { return setByte(_state | val); };
  size_t setAnd(const uint8_t val) { return setByte(_state & val); };
  size_t setXor(const uint8_t val) { return setByte(_state ^ val); };

  /** Set the specified bit (0--7) high. */
  size_t setBit(const uint8_t bitPos) {
    if (bitPos > I2C_MAX_BIT_POS) {
      return 0; /* Nothing to do. */
    }
    uint8_t mask = 1 << bitPos;
    return setOr(mask);
  }

  /** Set the specified bit (0--7) low. */
  size_t clrBit(const uint8_t bitPos) {
    if (bitPos > I2C_MAX_BIT_POS) {
      return 0; /* Nothing to do. */
    }
    uint8_t mask = ~(1 << bitPos);
    return setAnd(mask);
  }

  /** Switch the state of the specified bit (0--7). */
  size_t toggleBit(const uint8_t bitPos) {
    if (bitPos > I2C_MAX_BIT_POS) {
      return 0; /* Nothing to do. */
    }
    uint8_t setMask = 1 << bitPos;
    if ((_state & setMask) != 0) {
      // Bit already set. Clear it.
      return clrBit(bitPos);
    } else {
      return setBit(bitPos);
    }
  }

  // Increment the bus arithmetically by 1; 0xFF + 1 rolls back to 0.
  size_t increment() { return setByte((_state + 1) & I2C_PARALLEL_MAX_VAL); };

  // Delay until the transmitted data is ready on the parallel bus,
  // or delay until parallel bus inputs can be queried.
  void waitForValid();

private:
  uint8_t _i2cAddr; // Address of chip on the I2C bus.
  uint8_t _state;   // State of the 8 data lines.
};

#endif /* I2C_PARALLEL_H */
