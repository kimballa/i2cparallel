// (C) Copyright 2021-2024 Aaron Kimball
// This library is licensed under the terms of the BSD 3-Clause license.
// See the accompanying LICENSE.txt file for full license text.
//
// Library to use the PCF8574 / PCF8574A
// Remote 8-Bit I/O Expander for I2C Bus
//
// Include I2CParallel.h to use.

#include <Arduino.h>

#include "I2CParallel.h"

// Always end our i2c transmissions with the STOP signal.
static const uint8_t SEND_STOP = 1;

// An "i2c address" that no PCF8574[A] can have. We use this to note that
// the device driver has not been initialized, and disable I/O until this
// condition is lifted.
static const uint8_t UNINITIALIZED_I2C_ADDR = 0;

// Use only the 7 less-significant bits of the address.
// This will be left-shifted by 1 and a r/w flag bit appended as the lsb
// for actual communication.
static const uint8_t I2C_PARALLEL_ADDR_MASK = ((uint8_t)0x7F);

// Timeout duration for I2C communications in microseconds; use 25 ms.
static const uint32_t I2C_PARALLEL_WIRE_TIMEOUT = ((uint32_t)25000);

// All bus lines begin at logic high.
static const uint8_t I2C_PARALLEL_DEFAULT_BUS_STATE = I2C_PARALLEL_MAX_VAL;

I2CParallel::I2CParallel() {
  _state = I2C_PARALLEL_DEFAULT_BUS_STATE;
  // Start with 'invalid' i2caddr to confirm we are not initialized yet.
  _i2cAddr = UNINITIALIZED_I2C_ADDR;
}

void I2CParallel::init(const uint8_t i2cAddr, const uint32_t busSpeed) {
  _i2cAddr = i2cAddr & I2C_PARALLEL_ADDR_MASK;

  if (_i2cAddr < I2C_PCF8574_MIN_ADDR ||
      (_i2cAddr > I2C_PCF8574_MAX_ADDR && _i2cAddr < I2C_PCF8574A_MIN_ADDR) ||
      _i2cAddr > I2C_PCF8574A_MAX_ADDR) {

    Serial.println(F("WARNING: I2C Parallel bus addr must be within 0x20..0x27 "
                     "or 0x38..0x3F"));
    Serial.print(F("Configured address: "));
    Serial.println(_i2cAddr, HEX);
  }

  Wire.setClock(busSpeed);
#if defined(ARDUINO_ARCH_SAMD) || defined(ARDUINO_TEENSY41)
  Wire.setTimeout(I2C_PARALLEL_WIRE_TIMEOUT);
#else
  // This is the default for the __ARCH_AVR__ Wire library.
  Wire.setWireTimeout(I2C_PARALLEL_WIRE_TIMEOUT, true);
#endif
}

void I2CParallel::initInterrupt(const uint8_t digitalPinNum, void (*isr)()) {
  pinMode(digitalPinNum, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(digitalPinNum), isr, FALLING);
}

size_t I2CParallel::setByte(const uint8_t val) {
  size_t numWritten = 0;
  if (_i2cAddr != UNINITIALIZED_I2C_ADDR) {
    // Only transmit if we have initialized i2c bus.
    Wire.beginTransmission(_i2cAddr);
    numWritten = Wire.write(val);
    Wire.endTransmission(SEND_STOP);
  }
  _state = val;
  return numWritten;
}

uint8_t I2CParallel::getByte(uint8_t &nBytesRead) {
  nBytesRead = 0;
  // Only actually perform I/O if I2C has been initialized.
  if (_i2cAddr != UNINITIALIZED_I2C_ADDR) {
    // Request 1 byte of data from the "read address" of the device
    // (@ write_addr + 1)
    nBytesRead = Wire.requestFrom((uint8_t)_i2cAddr, (uint8_t)1);
    if (nBytesRead != 1) {
      _state = 0xFF; // Unknown err state
    } else {
      _state = Wire.read();
    }
  }
  return _state;
}

void I2CParallel::waitForValid() {
  // Programmed delay unnecessary; even at 400KHz bus speed, we will
  // have already experienced 2.5us delay after the ACK, which is stable
  // in operational testing. (Despite being lower than the 4us requirement
  // specified on the datasheet.)
  //
  // delayMicroseconds(I2C_PARALLEL_HOLD_TIME_MICROS);
}
