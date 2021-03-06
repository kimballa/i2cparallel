// (C) Copyright 2021 Aaron Kimball
// This library is licensed under the terms of the BSD 3-Clause license.
// See the accompanying LICENSE.txt file for full license text.
//
// Library to use the PCF8574 / PCF8574A
// Remote 8-Bit I/O Expander for I2C Bus
//
// Include I2CParallel.h to use.

#include<Arduino.h>
#include "I2CParallel.h"

static const uint8_t SEND_STOP = 1; // Always end our i2c transmissions with the STOP signal.

I2CParallel::I2CParallel() {
  _state = I2C_PARALLEL_DEFAULT_BUS_STATE;
}

void I2CParallel::init(const uint8_t i2cAddr, const uint32_t busSpeed) {
  _i2cAddr = i2cAddr & I2C_PARALLEL_ADDR_MASK;

  if (_i2cAddr < I2C_PCF8574_MIN_ADDR
      || (_i2cAddr > I2C_PCF8574_MAX_ADDR && _i2cAddr < I2C_PCF8574A_MIN_ADDR)
      || _i2cAddr > I2C_PCF8574A_MAX_ADDR) {

    Serial.println(F("WARNING: I2C Parallel bus addr must be within 0x20..0x27 or 0x38..0x3F"));
    Serial.print(F("Configured address: "));
    Serial.println(_i2cAddr, HEX);
  }

  Wire.setClock(busSpeed);
#if defined(ARDUINO_ARCH_SAMD)
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

void I2CParallel::setByte(const uint8_t val) {
  Wire.beginTransmission(_i2cAddr);
  Wire.write(val);
  Wire.endTransmission(SEND_STOP);
  _state = val;
}

void I2CParallel::write(const uint8_t val) {
  setByte(val);
}

uint8_t I2CParallel::getByte() {
  // Request 1 byte of data from the "read address" of the device (@ write_addr + 1)
  uint8_t received = Wire.requestFrom((uint8_t)_i2cAddr, (uint8_t)1);
  if (received != 1) {
    _state = 0xFF;  // Unknown err state
  } else {
    _state = Wire.read();
  }
  return _state;
}

uint8_t I2CParallel::read() {
  return getByte();
}

uint8_t I2CParallel::getLastState() const {
  return _state;
}

void I2CParallel::enableInputs(const uint8_t mask) {
  setOr(mask);
}

void I2CParallel::setOr(const uint8_t val) {
  setByte(_state | val);
}

void I2CParallel::setAnd(const uint8_t val) {
  setByte(_state & val);
}

void I2CParallel::setXor(const uint8_t val) {
  setByte(_state ^ val);
}

void I2CParallel::increment() {
  setByte((_state + 1) & I2C_PARALLEL_MAX_VAL);
}

void I2CParallel::waitForValid() {
  // Programmed delay unnecessary; even at 400KHz bus speed, we will
  // have already experienced 2.5us delay after the ACK, which is stable
  // in operational testing. (Despite being lower than the 4us requirement
  // specified on the datasheet.)
  //
  //delayMicroseconds(I2C_PARALLEL_HOLD_TIME_MICROS);
}

