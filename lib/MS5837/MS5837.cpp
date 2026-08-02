//
//    FILE: MS5837.cpp
//  AUTHOR: Rob Tillaart
// VERSION: 0.3.3
//    DATE: 2023-11-12
// PURPOSE: Arduino library for MS5837 temperature and pressure sensor.
//     URL: https://github.com/RobTillaart/MS5837


#include "MS5837.h"

//  commands  (MS5611 alike)
constexpr uint8_t MS5837_CMD_READ_ADC   = 0x00;
constexpr uint8_t MS5837_CMD_READ_PROM  = 0xA0;
constexpr uint8_t MS5837_CMD_RESET      = 0x1E;
constexpr uint8_t MS5837_CMD_CONVERT_D1 = 0x40;
constexpr uint8_t MS5837_CMD_CONVERT_D2 = 0x50;


//  CONSTRUCTOR
MS5837::MS5837(TwoWire *wire)
{
  _wire = wire;
}

bool MS5837::begin(uint8_t mathMode)
{
  if (! isConnected()) return false;
  return reset(mathMode);
}

bool MS5837::isConnected()
{
  _wire->beginTransmission(_address);
  _error = _wire->endTransmission();
  return ( _error == MS5837_OK);
}

bool MS5837::reset(uint8_t mathMode)
{
  command(MS5837_CMD_RESET);
  if (_error != MS5837_OK)
  {
    return false;
  }
  //  ---- SROT DEVIATION 1 of 3 (2026-08-02) --------------------------------------
  //  Upstream reads `uint32_t start = millis();` and then compares `micros() - start`.
  //  micros() is ~1000x larger, so the condition is false on its FIRST evaluation and the
  //  loop body NEVER RUNS: the mandatory post-RESET delay was zero. The MS5837 needs ~2.8 ms
  //  after a RESET to reload its calibration PROM into the readable registers, so
  //  initConstants() below was racing that reload and the coefficients were timing luck at
  //  every boot. A corrupted C5 (Tref) of ~42220 instead of ~26646 reports -160 C; ~27833
  //  reports +10 C; an intact read reports correctly. That is exactly the "sometimes -160,
  //  sometimes 10, sometimes perfect" the vehicle showed.
  //
  //  20 ms against a ~2.8 ms requirement. This runs once at boot, so the margin is free.
  uint32_t start = millis();
  //  while loop prevents blocking RTOS
  while (millis() - start < 20)
  {
    yield();
    delay(1);
  }

  initConstants(mathMode);
  if (_error != MS5837_OK)
  {
    return false;
  }
  //  ---- SROT DEVIATION 2 of 3 ---------------------------------------------------
  //  Upstream says "SKIP CRC check (for now)". The PROM carries a 4-bit CRC for precisely
  //  the failure above, and skipping it is what let a garbage coefficient set present itself
  //  as a healthy sensor. A bad PROM corrupts dT, and dT feeds `offset` and `sens` — so it
  //  silently corrupts PRESSURE and DEPTH too, not just the temperature you can see.
  if (!promCrcOk())
  {
    _error = MS5837_ERROR_CRC;
    return false;
  }

  //  derive the type from mathMode instead of the other way around.
  //  user must provide correct mathMode.
  _type = MS5837_TYPE_30;
  if (mathMode == 1) _type = MS5837_TYPE_02;
  if (mathMode == 2) _type = MS5803_TYPE_01;
  return true;
}

uint8_t MS5837::getType()
{
  return _type;
}

uint8_t MS5837::getAddress()
{
  return _address;
}


//////////////////////////////////////////////////////////////////////
//
//  READ
//
//  datasheet page 7
//  bits determines OSR => nr of samples => accuracy etc
int MS5837::read(uint8_t bits)
{
  if (isConnected() == false)
  {
    return -1;
  }

  int OSR = constrain(bits, 8, 13);
  OSR -= 8;
  //  datasheet page 2 OSR, ADC maximum conversion time.
  uint8_t waitMillis[6] = { 1, 2, 3, 5, 9, 18 };
  uint8_t wait = waitMillis[OSR];

   //  D1 conversion
  _wire->beginTransmission(_address);
  //  datasheet page 7 adjust command byte based on OSR
  _wire->write(MS5837_CMD_CONVERT_D1 + OSR * 2);
  _error = _wire->endTransmission();
  if (_error != MS5837_OK)
  {
    //  _error = twoWire specific.
    return -2;
  }

  uint32_t start = millis();
  //  SROT: `<= wait`, not `< wait`. millis() ticks at an arbitrary phase relative to this
  //  call, so `< wait` can exit after as little as ~0.2 ms of real time at wait == 1 —
  //  before the ADC conversion has finished — and readADC() then returns 0 or the previous
  //  conversion while reporting success. One extra millisecond per read buys determinism.
  while (millis() - start <= wait)
  {
    yield();
    delay(1);
  }
  //  NOTE: names D1 and D2 are reserved in MBED (NANO BLE)
  uint32_t _D1 = readADC();
  if (_error != MS5837_OK)
  {
    //  _error = twoWire specific.
    return -3;
  }

   //  D2 conversion
  _wire->beginTransmission(_address);
  //  datasheet page 7 adjust command byte based on OSR
  _wire->write(MS5837_CMD_CONVERT_D2 + OSR * 2);
  _error = _wire->endTransmission();
  if (_error != MS5837_OK)
  {
    //  _error = twoWire specific.
    return -4;
  }

  start = millis();
  //  SROT: `<= wait` — see the D1 conversion above.
  while (millis() - start <= wait)
  {
    yield();
    delay(1);
  }

  //  NOTE: names D1 and D2 are reserved in MBED (NANO BLE)
  uint32_t _D2 = readADC();
  if (_error != MS5837_OK)
  {
    //  _error = twoWire specific.
    return -5;
  }

  //  determine temperature
  float dT = _D2 - C[5];
  _temperature = 2000 + dT * C[6];

  //  determine pressure
  float offset = C[2] + dT * C[4];
  float sens   = C[1] + dT * C[3];
  //                         1 / 2^21                    C[7] / 100
  _pressure = (_D1 * sens * 4.76837158203E-7 - offset) * C[7] * 0.01;
  if (_type == MS5837_TYPE_30) _pressure *= 10;

  //  Second order compensation
  //  Comment to save footprint (trade accuracy)
  if ((_temperature * 0.01) < 20)
  {
    float ti = 0, offset2 = 0, sens2 = 0;
    float t2 = (_temperature - 2000) * (_temperature - 2000);
    //  Math mode 0  Page 12
    if (_type == MS5837_TYPE_30)
    {
      ti = dT * dT * (3 * 1.164153218269E-10);  //  1 / 2^33
      offset2 = t2 * (3 * 0.5);  //  1 / 2^1
      sens2 = t2 * (5 * 0.125);  //  1 / 2^3
      if ((_temperature * 0.01) < -15)
      {
        t2 = (_temperature + 1500) * (_temperature + 1500);
        offset2 += 7 * t2;
        sens2   += 4 * t2;
      }
    }
    //  math mode 1
    if (_type == MS5837_TYPE_02)
    {
      ti = dT * dT * (11 * 2.91038304567E-11);  //  1 / 2^35
      offset2 = t2 * (31 * 0.125);  //  1 / 2^3
      sens2 = t2 * (63 * 0.03125);  //  1 / 2^5
    }
    //  math mode 2
    if (_type == MS5803_TYPE_01)
    {
      ti = dT * dT * (3 * 4.656612873077E-10);  //  1 / 2^31
      offset2 = t2 * (3 * 1.0);  //  1 / 2^0
      sens2 = t2 * (7 * 0.125);  //  1 / 2^3
      if ((_temperature * 0.01) < -15)
      {
        t2 = (_temperature + 1500) * (_temperature + 1500);
        sens2 += 2 * t2;
      }
    }
    else
    {
      // temperature > 20C MS5803 only
      if ((_type == MS5803_TYPE_01) && ( (_temperature * 0.01) > 45))
      {
        t2 = (_temperature - 4500) * (_temperature - 4500);
        sens2 -= t2 * 0.125;  //  1/2^3
      }
    }
    offset       -= offset2;
    sens         -= sens2;
    _temperature -= ti;
  }

  _temperature *= 0.01;

  _lastRead = millis();
  _error = MS5837_OK;
  return 0;
}

uint32_t MS5837::lastRead()
{
  return _lastRead;
}


float MS5837::getTemperature()
{
  return _temperature;
}


float MS5837::getPressure()
{
  return _pressure;
}


float MS5837::getPressurePascal()
{
  return _pressure * 100.0;
}


//  https://www.mide.com/air-pressure-at-altitude-calculator
//  https://community.bosch-sensortec.com/t5/Question-and-answers/How-to-calculate-the-altitude-from-the-pressure-sensor-data/qaq-p/5702 (stale link).
//  https://en.wikipedia.org/wiki/Pressure_altitude
float MS5837::getAltitude(float airPressure)
{
  //  _pressure is in mBar - differs from MS5611
  float ratio = _pressure / airPressure;
  return 44307.694 * (1 - pow(ratio, 0.190284));
}


float MS5837::getAltitudeFeet(float airPressure)
{
  float ratio = _pressure / airPressure;
  return 145366.45 * (1 - pow(ratio, 0.190284));
}


//  returns mBar; pressure == mBar; altitude == meter
float MS5837::getSeaLevelPressure(float pressure, float altitude)
{
  float x = 1 - altitude * 2.256944358E-5;  //  == altitude / 44307.694
  float ratio = pow(x, 5.2553026);          //  == (1.0 / 0.190284));
  float seaLevelPressure = pressure / ratio;
  return seaLevelPressure;
}


//////////////////////////////////////////////////////////////////////
//
//  DENSITY for depth
//
void MS5837::setDensity(float density)
{
  _density = density;
}

float MS5837::getDensity()
{
  return _density;
}

float MS5837::getDepth(float airPressure)
{
  //  9.80665 == Gravity constant.
  //  1 / (_density * 9.80665 * 10)  can be pre-calculated and cached in setDensity.
  //
  //  delta P = rho * g * h  => h = delta P / rho * g
  //  pressure = mbar,
  //  density grams/cm3 => correction factor 0.1 (= 1/10)
  return (_pressure - airPressure)/(_density * 9.80665 * 10);
}

float MS5837::getDepthFeet(float airPressure)
{
  //  optimized
  return 0.033455768 * (_pressure - airPressure)/_density;
  //  return getDepth() * 3.2808399;
}


//////////////////////////////////////////////////////////////////////
//
//  ERROR
//
int MS5837::getLastError()
{
  int e = _error;
  _error = 0;
  return e;
}


//////////////////////////////////////////////////////////////////////
//
//  PROM zero - meta info
//
uint16_t MS5837::getCRC()
{
  uint16_t value = C[0];
  return value >> 12;
}

uint16_t MS5837::getProduct()
{
  uint16_t value = C[0];
  return (value >> 5) & 0x007F;
}

uint16_t MS5837::getFactorySettings()
{
  uint16_t value = C[0];
  return value & 0x001F;
}


uint16_t MS5837::getPromZero()
{
  uint16_t value = C[0];
  return value;
}


/////////////////////////////////////////////////////
//
//  PROTECTED
//
int MS5837::command(uint8_t command)
{
  yield();
  _wire->beginTransmission(_address);
  _wire->write(command);
  _error = _wire->endTransmission();
  return _error;
}


void MS5837::initConstants(uint8_t mathMode)
{
  //  Constants that were multiplied in read() - datasheet page 8
  //  do this once and you save CPU cycles.
  //
  //                               datasheet MS5837_30  page 7
  //
  //                                mode = 0;         |  = 1     |   = 2    |
  C[0] = 1;               //  manufacturer
  C[1] = 32768L;          //  SENSt1   = C[1] * 2^15  |  * 2^16  |  * 2^15  |
  C[2] = 65536L;          //  OFFt1    = C[2] * 2^16  |  * 2^17  |  * 2^16  |
  C[3] = 3.90625E-3;      //  TCS      = C[3] / 2^8   |  / 2^7   |  / 2^8   |
  C[4] = 7.8125E-3;       //  TCO      = C[4] / 2^7   |  / 2^6   |  / 2^7   |
  C[5] = 256;             //  Tref     = C[5] * 2^8   |  * 2^8   |  * 2^8   |
  C[6] = 1.1920928955E-7; //  TEMPSENS = C[6] / 2^23  |  / 2^23  |  / 2^23  |
  C[7] = 1.220703125E-4;  //  compensate uses / 2^13  |  / 2^15  |  / 2^15  |  Pressure math (P)

  //  App note version for pressure.
  //  adjustments for MS5837_02
  if (mathMode == 1)
  {
    C[1] *= 2;  //  SENSt1
    C[2] *= 2;  //  OFFt1
    C[3] *= 2;  //  TCS
    C[4] *= 2;  //  TCO
    C[7] /= 4;  //  compensation code has factor 4.
  }

  //  adjustments for MS5803_01
  if (mathMode == 2)
  {
    C[7] /= 4;  //  compensation code has factor 4.
  }


  for (uint8_t i = 0; i < 7 ; i++)
  {
    _wire->beginTransmission(_address);
    _wire->write(MS5837_CMD_READ_PROM + i + i);
    _error = _wire->endTransmission();
    if (_error != MS5837_OK)
    {
      return;
    }
    uint8_t length = 2;
    if (_wire->requestFrom(_address, length) != length)
    {
      _error = MS5837_ERROR_REQUEST;
      return;
    }
    uint16_t tmp = _wire->read() << 8;
    tmp |= _wire->read();
    //  SROT: keep the RAW word. C[] is scaled by the mathMode multiplier, so it cannot be
    //  CRC'd after the fact; the CRC is defined over the unscaled PROM image.
    _prom[i] = tmp;
    C[i] *= tmp;
  }
  _error = MS5837_OK;
}


//  ---- SROT DEVIATION 3 of 3 -----------------------------------------------------
//  MS5837 PROM CRC-4, per the datasheet's AN520 reference implementation.
//  Word 0 holds the 4-bit CRC in bits 15..12; it is computed over words 0..6 with that
//  nibble zeroed, plus a zero-valued 8th word.
bool MS5837::promCrcOk()
{
  //  A bus that is dead or absent returns all-zeros or all-ones, and some CRC variants
  //  accept those. Reject them explicitly: a coefficient of 0 or 0xFFFF is not a sensor.
  bool all_zero = true, all_ones = true;
  for (uint8_t i = 1; i < 7; i++)
  {
    if (_prom[i] != 0x0000) all_zero = false;
    if (_prom[i] != 0xFFFF) all_ones = false;
  }
  if (all_zero || all_ones) return false;

  uint16_t n_rem = 0;
  const uint8_t crc_read = (uint8_t)(_prom[0] >> 12);
  uint16_t n_prom[8];
  for (uint8_t i = 0; i < 7; i++) n_prom[i] = _prom[i];
  n_prom[0] = (uint16_t)(n_prom[0] & 0x0FFF);   //  strip the CRC nibble
  n_prom[7] = 0;                                //  subsidiary value, per the datasheet

  for (uint8_t cnt = 0; cnt < 16; cnt++)
  {
    if (cnt % 2 == 1) n_rem ^= (uint16_t)(n_prom[cnt >> 1] & 0x00FF);
    else              n_rem ^= (uint16_t)(n_prom[cnt >> 1] >> 8);
    for (uint8_t n_bit = 8; n_bit > 0; n_bit--)
    {
      if (n_rem & 0x8000) n_rem = (uint16_t)((n_rem << 1) ^ 0x3000);
      else                n_rem = (uint16_t)(n_rem << 1);
    }
  }
  n_rem = (uint16_t)((n_rem >> 12) & 0x000F);
  return (uint8_t)n_rem == crc_read;
}


uint16_t MS5837::promRaw(uint8_t i)
{
  return (i < 7) ? _prom[i] : 0;
}


uint32_t MS5837::readADC()
{
  command(MS5837_CMD_READ_ADC);
  if (_error != MS5837_OK)
  {
    return 0UL;
  }
  uint8_t length = 3;
  if (_wire->requestFrom(_address, length) != length)
  {
    _error = MS5837_ERROR_REQUEST;
    return 0UL;
  }
  uint32_t value = _wire->read() * 65536UL;
  value += _wire->read() * 256UL;
  value += _wire->read();
  return value;
}


///////////////////////////////////////////////////////////////////
//
//  DERIVED CLASSES
//

//////////////////////////////////////////////////////////////////////
//
//  MS5803
//
MS5803::MS5803(TwoWire *wire) : MS5837(wire)
{
}

MS5803::MS5803(uint32_t address, TwoWire *wire)
{
  _address = address;
  _wire = wire;
}


//  -- END OF FILE --

