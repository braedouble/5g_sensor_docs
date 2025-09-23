#include "Arduino.h"
#include "ICM42688.h"
#include "registers.h"

using namespace ICM42688reg;

/* ICM42688 object, input the I2C bus and address */
ICM42688::ICM42688(TwoWire &bus, uint8_t address) {
  _i2c = &bus; // I2C bus
  _address = address; // I2C address
  _useSPI = false; // set to use I2C
}

/* ICM42688 object, input the SPI bus and chip select pin */
ICM42688::ICM42688(SPIClass &bus, uint8_t csPin, uint32_t SPI_HS_CLK) {
  _spi = &bus; // SPI bus
  _csPin = csPin; // chip select pin
  _useSPI = true; // set to use SPI
  SPI_HS_CLOCK = SPI_HS_CLK;
  _spiSettings = SPISettings(SPI_HS_CLK, MSBFIRST, SPI_MODE3);
}

/* starts communication with the ICM42688 */
int ICM42688::begin() {
  if( _useSPI ) { // using SPI for communication
    // use low speed SPI for register setting
    _useSPIHS = true;

    // setting CS pin to output
    pinMode(_csPin,OUTPUT);
    // setting CS pin high
    digitalWrite(_csPin,HIGH);
    // begin SPI communication
    _spi->begin(4, MISO, MOSI, 5);
  } else { // using I2C for communication
    // starting the I2C bus
    _i2c->begin();
    // setting the I2C clock
    _i2c->setClock(I2C_CLK);
  }

  // reset the ICM42688
  //reset();

  // check the WHO AM I byte
  if(whoAmI() != WHO_AM_I) {
    return whoAmI();
  }

  // turn on accel only Low Noise (LN) Mode
  if(writeRegister(UB0_REG_PWR_MGMT0, 0x03) < 0) {
    return -4;
  }

  // 16G is default -- do this to set up accel resolution scaling
  int ret = setAccelFS(gpm16);
  if (ret < 0) return ret;

  // 2000DPS is default -- do this to set up gyro resolution scaling
  ret = setGyroFS(dps2000);
  if (ret < 0) return ret;

  // // disable inner filters (Notch filter, Anti-alias filter, UI filter block)
  // if (setFilters(false, false) < 0) {
  //   return -7;
  // }

  // estimate gyro bias
  if (calibrateGyro() < 0) {
    return -8;
  }
  // successful init, return 1
  return 1;
}

/* sets the accelerometer full scale range to values other than default */
int ICM42688::setAccelFS(AccelFS fssel) {
  // use low speed SPI for register setting
  _useSPIHS = false;

  setBank(0);

  // read current register value
  uint8_t reg;
  if (readRegisters(UB0_REG_ACCEL_CONFIG0, 1, &reg) < 0) return -1;

  // only change FS_SEL in reg
  reg = (fssel << 5) | (reg & 0x1F);

  if (writeRegister(UB0_REG_ACCEL_CONFIG0, reg) < 0) return -2;

  _accelScale = static_cast<float>(1 << (4 - fssel)) / 32768.0f;
  _accelFS = fssel;

  return 1;
}

/* sets the gyro full scale range to values other than default */
int ICM42688::setGyroFS(GyroFS fssel) {
  // use low speed SPI for register setting
  _useSPIHS = false;

  setBank(0);

  // read current register value
  uint8_t reg;
  if (readRegisters(UB0_REG_GYRO_CONFIG0, 1, &reg) < 0) return -1;

  // only change FS_SEL in reg
  reg = (fssel << 5) | (reg & 0x1F);

  if (writeRegister(UB0_REG_GYRO_CONFIG0, reg) < 0) return -2;

  _gyroScale = (2000.0f / static_cast<float>(1 << fssel)) / 32768.0f;
  _gyroFS = fssel;

  return 1;
}

int ICM42688::setAccelODR(ODR odr) {
  // use low speed SPI for register setting
  //_useSPIHS = false;

  setBank(0);

  // read current register value
  uint8_t reg;
  if (readRegisters(UB0_REG_ACCEL_CONFIG0, 1, &reg) < 0) return -1;

  // only change ODR in reg
  reg = odr | (reg & 0xF0);

  if (writeRegister(UB0_REG_ACCEL_CONFIG0, reg) < 0) return -2;

  return 1;
}

int ICM42688::getAccelODR() {
  // use low speed SPI for register setting
  _useSPIHS = false;

  setBank(0);

  // read current register value
  uint8_t reg;
  if (readRegisters(UB0_REG_ACCEL_CONFIG0, 1, &reg) < 0)
    return -1;

  return reg;
}

int ICM42688::setGyroODR(ODR odr) {
  // use low speed SPI for register setting
  _useSPIHS = false;

  setBank(0);

  // read current register value
  uint8_t reg;
  if (readRegisters(UB0_REG_GYRO_CONFIG0, 1, &reg) < 0) return -1;

  // only change ODR in reg
  reg = odr | (reg & 0xF0);

  if (writeRegister(UB0_REG_GYRO_CONFIG0, reg) < 0) return -2;

  return 1;
}

int ICM42688::setFilters(bool gyroFilters, bool accFilters) {
  if (setBank(1) < 0) return -1;

  if (gyroFilters == true) {
    if (writeRegister(UB1_REG_GYRO_CONFIG_STATIC2, GYRO_NF_ENABLE | GYRO_AAF_ENABLE) < 0) {
      return -2;
    }
  }
  else {
    if (writeRegister(UB1_REG_GYRO_CONFIG_STATIC2, GYRO_NF_DISABLE | GYRO_AAF_DISABLE) < 0) {
      return -3;
    }
  }
  
  if (setBank(2) < 0) return -4;

  if (accFilters == true) {
    if (writeRegister(UB2_REG_ACCEL_CONFIG_STATIC2, ACCEL_AAF_ENABLE) < 0) {
      return -5;
    }
  }
  else {
    if (writeRegister(UB2_REG_ACCEL_CONFIG_STATIC2, ACCEL_AAF_DISABLE) < 0) {
      return -6;
    }
  }
  if (setBank(0) < 0) return -7;
  return 1;
}

int ICM42688::enableDataReadyInterrupt() {
  // use low speed SPI for register setting
  _useSPIHS = false;

  // push-pull, pulsed, active HIGH interrupts
  if (writeRegister(UB0_REG_INT_CONFIG, 0x18 | 0x03) < 0) return -1;

  // need to clear bit 4 to allow proper INT1 and INT2 operation
  // Read current value of INT_CONFIG1
  uint8_t reg;
  if (readRegisters(UB0_REG_INT_CONFIG1, 1, &reg) < 0)
    return -2;

  // Set bits 5 and 6 (0x60) to 1, leave other bits (including bit 4) unchanged
  // required for ODR > 4kHz, not required for ODR < 4kHz.
  reg |= 0x60;

  // Write modified value back to INT_CONFIG1
  if (writeRegister(UB0_REG_INT_CONFIG1, reg) < 0)
    return -3;
  // route UI data ready interrupt to INT1 // changed d ready clear (was 0x18)
  if (writeRegister(UB0_REG_INT_SOURCE0, 0x18) < 0) return -4;

  return 1;
}

int ICM42688::enableFifoThresholdInterrupt() {
  // Configure FIFO Threshold Interrupt clearing: clear on 1-byte FIFO read
  if (writeRegister(UB0_REG_INT_CONFIG, 0x08) < 0)
    return -1; // 0x08 = FIFO_THS_INT_CLEAR = 10

  // Read current value of INT_CONFIG1
  uint8_t reg;
  if (readRegisters(UB0_REG_INT_CONFIG1, 1, &reg) < 0)
    return -2;

  // Clear bit 4 (INT_ASYNC_RESET) and set bits 5 and 6 (8 µs pulse, disable
  // de-assertion)
  reg &= ~0x10; // Clear INT_ASYNC_RESET for proper INT1/INT2 operation
  reg |= 0x60;  // Set INT_TPULSE_DURATION = 1, INT_TDEASSERT_DISABLE = 1

  // Write modified value back to INT_CONFIG1
  if (writeRegister(UB0_REG_INT_CONFIG1, reg) < 0)
    return -3;

  // Route FIFO Threshold Interrupt to INT1
  if (writeRegister(UB0_REG_INT_SOURCE0, 0x04) < 0)
    return -4; // 0x04 = FIFO_THS_INT1_EN = 1

  return 1;
}

int ICM42688::resetInterrupts() {
  // use low speed SPI for register setting
  _useSPIHS = false;

  // set pin 4 to return to reset value
  uint8_t reg;
  if (readRegisters(UB0_REG_INT_CONFIG1, 1, &reg) < 0) return -1;
  reg |= 0x10;
  if (writeRegister(UB0_REG_INT_CONFIG1, reg) < 0) return -2;

  // return reg to reset value
  if (writeRegister(UB0_REG_INT_SOURCE0, 0x10) < 0) return -3;

  return 1;
}

int ICM42688::disableDataReadyInterrupt() {
  // use low speed SPI for register setting
  _useSPIHS = false;

  // set pin 4 to return to reset value
  uint8_t reg;
  if (readRegisters(UB0_REG_INT_CONFIG1, 1, &reg) < 0) return -1;
  reg |= 0x10;
  if (writeRegister(UB0_REG_INT_CONFIG1, reg) < 0) return -2;

  // return reg to reset value
  if (writeRegister(UB0_REG_INT_SOURCE0, 0x10) < 0) return -3;

  return 1;
}

/* reads the most current data from ICM42688 and stores in buffer */
int ICM42688::getAGT() { // modified to use getRawAGT()
  //if (getRawAGT() < 0) {
  //  return -1;
  //}

  //_t = (static_cast<float>(_rawT) / TEMP_DATA_REG_SCALE) + TEMP_OFFSET;

  _acc[0] = ((_rawAcc[0] * _accelScale) - _accB[0]) * _accS[0];
  _acc[1] = ((_rawAcc[1] * _accelScale) - _accB[1]) * _accS[1];
  _acc[2] = ((_rawAcc[2] * _accelScale) - _accB[2]) * _accS[2];

  //_gyr[0] = (_rawGyr[0] * _gyroScale) - _gyrB[0];
  //_gyr[1] = (_rawGyr[1] * _gyroScale) - _gyrB[1];
  //_gyr[2] = (_rawGyr[2] * _gyroScale) - _gyrB[2];

  return 1;
}

/* reads the most current data from ICM42688 and stores in buffer */
int16_t rawMeas[3];         // temp, accel xyz, gyro xyz
void IRAM_ATTR ICM42688::getRawAGT() { // Added to return raw data only
  //_useSPIHS = true;         // use the high speed SPI for data readout
  // grab the data from the ICM42688
  //if (readRegisters(UB0_REG_TEMP_DATA1, 14, _buffer) < 0) {
  //  return -1;
  //}

  // grab the data from the ICM42688
  //if (readRegisters(UB0_REG_ACCEL_DATA_X1, 6, _buffer) < 0) {
  //  //return -1;
  //}
  
  readRegisters(UB0_REG_ACCEL_DATA_X1, 6, _buffer);
  
  // combine bytes into 16 bit values

  for (size_t i = 0; i < 3; i++) {
    rawMeas[i] = ((int16_t)_buffer[i * 2] << 8) | _buffer[i * 2 + 1];
  }

  //_rawT = rawMeas[0];
  _rawAcc[0] = rawMeas[0];
  _rawAcc[1] = rawMeas[1];
  _rawAcc[2] = rawMeas[2];
  //_rawGyr[0] = rawMeas[4];
  //_rawGyr[1] = rawMeas[5];
  //_rawGyr[2] = rawMeas[6];
  

 // _rawAcc[0] = (int16_t)((_buffer[0] << 8) | _buffer[1]);
 // _rawAcc[1] = (int16_t)((_buffer[2] << 8) | _buffer[3]);
 // _rawAcc[2] = (int16_t)((_buffer[4] << 8) | _buffer[5]);

  //return 1;
}

void IRAM_ATTR ICM42688::getRawAGT(int *x, int *y, int *z) { // Added to return raw data only
  //_useSPIHS = true;         // use the high speed SPI for data readout
  // grab the data from the ICM42688
  // if (readRegisters(UB0_REG_TEMP_DATA1, 14, _buffer) < 0) {
  //  return -1;
  //}

  // grab the data from the ICM42688
  // if (readRegisters(UB0_REG_ACCEL_DATA_X1, 6, _buffer) < 0) {
  //  //return -1;
  //}
  readRegisters(UB0_REG_ACCEL_DATA_X1, 6, _buffer);

  // combine bytes into 16 bit values

  for (size_t i = 0; i < 3; i++) {
    rawMeas[i] = ((int16_t)_buffer[i * 2] << 8) | _buffer[i * 2 + 1];
  }

  //_rawT = rawMeas[0];
  *x = rawMeas[0];
  *y = rawMeas[1];
  *z = rawMeas[2];
  //_rawGyr[0] = rawMeas[4];
  //_rawGyr[1] = rawMeas[5];
  //_rawGyr[2] = rawMeas[6];

  // _rawAcc[0] = (int16_t)((_buffer[0] << 8) | _buffer[1]);
  // _rawAcc[1] = (int16_t)((_buffer[2] << 8) | _buffer[3]);
  // _rawAcc[2] = (int16_t)((_buffer[4] << 8) | _buffer[5]);

  // return 1;
}

/* configures and enables the FIFO buffer
  See https://invensense.tdk.com/wp-content/uploads/2020/04/ds-000347_icm-42688-p-datasheet.pdf, 6.1 for details
  Packet 1 - 1 byte Header, 6 bytes Accel, 1 byte Temp   
  Packet 2 - 1 byte Header, 6 bytes Gyro, 1 byte Temp
  Packet 3 - 1 byte Header, 6 bytes Accel, 6 bytes Gyro, 1 byte Temp, 2 bytes TimeStamp
  Packet 4 - 1 byte Header, 6 bytes Accel, 6 bytes Gyro, 2 bytes Temp, 2 bytes TimeStamp, 3 bytes 20-bit Extension

  When 20-bits data format is used, the only FSR settings that are
  operational are ±2000dps for gyroscope and ±16g for accelerometer, even if the FSR selection register settings are configured for
  other FSR values. 

  When 20-bits data format is used, gyroscope data consists of 19-bits of actual data and the LSB is always set to 0, accelerometer data consists of 18-bits
  of actual data and the two lowest order bits are always set to 0.
  */

int ICM42688_FIFO::enableFifo(bool accel,bool gyro,bool highRes) {
  // use low speed SPI for register setting
  //_useSPIHS = false;
  //if(writeRegister(FIFO_EN,(accel*FIFO_ACCEL)|(gyro*FIFO_GYRO)|(temp*FIFO_TEMP_EN)) < 0) {
  //  return -2;
  //}
  _enFifoAccel = accel || highRes; // highRes always includes accel
  _enFifoGyro = gyro || highRes; // highRes always includes gyro
  _enFifoTemp = accel || gyro; // or True, all 4 structures have temp
  _enFifoTimestamp = accel && gyro; // only if both accel and gyro are enabled (Structure 3 and 4)
  _enFifoHeader  = accel || gyro;

  _enFifoHighRes = highRes;

  if (highRes) {
    _fifoFrameSize = 20;
  } else { // Packet structure 1, 2, 3
    _fifoFrameSize = _enFifoHeader*1 + _enFifoAccel*6 + _enFifoGyro*6 + _enFifoTemp*1 + _enFifoTimestamp*2;
  }
  
  if (writeRegister(FIFO_EN, (_enFifoAccel * FIFO_ACCEL) | (_enFifoGyro * FIFO_GYRO) | (_enFifoTemp * FIFO_TEMP_EN) | (_enFifoHighRes * FIFO_HIGHRES_EN ))
	    < 0) {
		return -2;
	}
	return 1;
}

/* Set FIFO Mode / enable FIFO 
   bit 7:6 - FIFO_MODE
   00 = Bypass mode (Default)
   01 = Stream-to-FIFO mode
   10 = Stop-on-FULL mode
   11 = Stop-on-FULL mode
*/
int ICM42688_FIFO::streamToFifo() {
	if (writeRegister(ICM42688reg::UB0_REG_FIFO_CONFIG, 0b01000000) < 0) {
		return -2;
	}
	return 1;
}
int ICM42688_FIFO::getFifoSize() {
  readRegisters(UB0_REG_FIFO_CONFIG2, 2, _buffer);
  _fifoSize = (((uint16_t)(_buffer[1] & 0x0F)) << 8) + (((uint16_t)_buffer[0]));
  return _fifoSize;
}

bool ICM42688_FIFO::setFifoSize(uint16_t fifoSize) {
  if (fifoSize > 4095) { // 12-bit max value
    return false;        // Invalid size
  }
  if (fifoSize == 0) { // Non-zero required for interrupts
    return false;      // Invalid size
  }

  uint8_t lowByte = (uint8_t)(fifoSize & 0xFF); // FIFO_WM[7:0] (lower 8 bits)
  uint8_t highByte =
      (uint8_t)((fifoSize >> 8) & 0x0F); // FIFO_WM[11:8] (upper 4 bits)

  // Write FIFO_CONFIG2 (0x60)
  bool success = writeRegister(UB0_REG_FIFO_CONFIG2, lowByte);
  if (!success) {
    return false;
  }

  // Write FIFO_CONFIG3 (0x61)
  success = writeRegister(UB0_REG_FIFO_CONFIG3, highByte);
  return success;
}

  /* reads data from the ICM42688 FIFO and stores in buffer */
  int ICM42688_FIFO::readFifo() {

    // get the fifo size
    readRegisters(UB0_REG_FIFO_COUNTH, 2, _buffer);
    _fifoSize =
        (((uint16_t)(_buffer[0] & 0x0F)) << 8) + (((uint16_t)_buffer[1]));

    // precalculate packet structure as per-packet recalculation based on
    // headers isn't reliable
    size_t numFrames = _fifoSize / _fifoFrameSize;
    // start position + length
    size_t accIndex = 1; // header 1 length, accel starts at 1
    size_t gyroIndex =
        accIndex + _enFifoAccel * 6; // if accel enabled, 6 length
    size_t tempIndex = gyroIndex + _enFifoGyro * 6; // if gyro enabled, 6 length
    size_t timestampIndex =
        tempIndex +
        _enFifoTemp *
            (_enFifoHighRes + 1); // temp 1 unless HR enabled, then 2 length
    size_t extensionIndex =
        timestampIndex + _enFifoTimestamp * 2; // if on, 2 length, otherwise 0

    // read and parse the buffer
    for (size_t i = 0; i < _fifoSize / _fifoFrameSize; i++) {
      // read the FIFO data from the ICM42688
      if (readRegisters(UB0_REG_FIFO_DATA, _fifoFrameSize, _buffer) < 0) {
        return -1;
      }

      // if accel or gyro data is enabled, read header
      if (_enFifoHeader) {
        // read the header
        uint8_t rawMeas = _buffer[0];
        // transform and convert to float values
        _hFifo[i] = (rawMeas);
        _hSize = numFrames;
      }

      // if FIFO contains accel data
      if (_enFifoAccel) {
        // if not high resolution mode:
        if (!_enFifoHighRes) {
          // combine into 16 bit values
          int16_t rawMeas[3];

          // if there is accel data, always in the same positions
          // X - always 0x01 [15:8], 0x02 [7:0]
          rawMeas[0] =
              (((int16_t)_buffer[0 + accIndex]) << 8) | _buffer[1 + accIndex];
          // Y - always 0x03 [15:8], 0x04 [7:0]
          rawMeas[1] =
              (((int16_t)_buffer[2 + accIndex]) << 8) | _buffer[3 + accIndex];
          // Z - always 0x05 [15:8], 0x06 [7:0]
          rawMeas[2] =
              (((int16_t)_buffer[4 + accIndex]) << 8) | _buffer[5 + accIndex];
          // transform and convert to float values
          //_axFifo[i] = ((rawMeas[0] * _accelScale) - _accB[0]) * _accS[0];
          //_ayFifo[i] = ((rawMeas[1] * _accelScale) - _accB[1]) * _accS[1];
          //_azFifo[i] = ((rawMeas[2] * _accelScale) - _accB[2]) * _accS[2];
          _axFifo[i] = rawMeas[0]; // x
          _ayFifo[i] = rawMeas[1]; // y
          _azFifo[i] = rawMeas[2]; // z
          _aSize = _fifoSize / _fifoFrameSize;
        } else {
          // high resolution mode
          // combine into 20 bit values
          int32_t rawMeas[3]; // increase size for larger accel data

          // 20-bit data format, 18-bits of actual data and the two lowest order
          // bits are always set to 0 X - always 0x01 [19:12], 0x02 [11:4], 0x11
          // [3:0]
          rawMeas[0] = (((int32_t)_buffer[0 + accIndex]) << 10) |
                       ((_buffer[1 + accIndex] << 2) |
                        ((_buffer[0 + extensionIndex] >> 6)));
          // C/C++ does NOT automatically sign-extend non-standard bit widths
          // (like 18-bit). Sign-extend 18-bit value to 32-bit
          if (rawMeas[0] & (1 << 17)) {
            rawMeas[0] |= 0xFFFC0000;
          }
          // Y - always 0x03, 0x04, 0x12
          rawMeas[1] = (((int32_t)_buffer[2 + accIndex]) << 10) |
                       ((_buffer[3 + accIndex] << 2) |
                        ((_buffer[1 + extensionIndex] >> 6)));
          if (rawMeas[1] & (1 << 17)) {
            rawMeas[1] |= 0xFFFC0000;
          }
          // Z - always 0x05, 0x06, 0x13
          rawMeas[2] = (((int32_t)_buffer[4 + accIndex]) << 10) |
                       ((_buffer[5 + accIndex] << 2) |
                        ((_buffer[2 + extensionIndex] >> 6)));
          if (rawMeas[2] & (1 << 17)) {
            rawMeas[2] |= 0xFFFC0000;
          }
          // transform and convert to float values
          //_axFifo[i] = ((rawMeas[0] * _accelScale) - _accB[0]) * _accS[0];
          //_ayFifo[i] = ((rawMeas[1] * _accelScale) - _accB[1]) * _accS[1];
          //_azFifo[i] = ((rawMeas[2] * _accelScale) - _accB[2]) * _accS[2];
          _axFifo[i] = rawMeas[0]; // x
          _ayFifo[i] = rawMeas[1]; // y
          _azFifo[i] = rawMeas[2]; // z
          _aSize = _fifoSize / _fifoFrameSize;
        }
      }

      // if FIFO contains temperature data
      if (_enFifoTemp) {
        // if not high resolution mode, 8 bit value
        if (!_enFifoHighRes) {
          int8_t rawMeas = _buffer[0 + tempIndex];
          // transform and convert to float values
          //_tFifo[i] = (static_cast<float>(rawMeas) / TEMP_DATA_REG_SCALE) +
          //TEMP_OFFSET; _tSize    = numFrames;
          _tempFifo[i] = (rawMeas);
          _tempSize = numFrames;
        } else { // high resolution mode,  16 bit value
          // combine into 16 bit values
          int16_t rawMeas =
              (((int16_t)_buffer[0 + tempIndex] << 8) | _buffer[1 + tempIndex]);
          // transform and convert to float values
          //_tFifo[i] = (static_cast<float>(rawMeas) / TEMP_DATA_REG_SCALE) +
          //TEMP_OFFSET; _tSize = _fifoSize/_fifoFrameSize;
          _tempFifo[i] = (rawMeas);
          _tempSize = numFrames;
        }
      }

      // if FIFO contains gyro data
      if (_enFifoGyro) {
        // if not high resolution mode:
        if (!_enFifoHighRes) {
          // combine into 16 bit values
          int16_t rawMeas[3];
          // if there is gyro data, always in the same positions
          // X - always 0x01 [15:8], 0x02 [7:0]
          rawMeas[0] =
              (((int16_t)_buffer[0 + gyroIndex]) << 8) | _buffer[1 + gyroIndex];
          // Y - always 0x03 [15:8], 0x04 [7:0]
          rawMeas[1] =
              (((int16_t)_buffer[2 + gyroIndex]) << 8) | _buffer[3 + gyroIndex];
          // Z - always 0x05 [15:8], 0x06 [7:0]
          rawMeas[2] =
              (((int16_t)_buffer[4 + gyroIndex]) << 8) | _buffer[5 + gyroIndex];
          // transform and convert to float values
          //_gxFifo[i] = (rawMeas[0] * _gyroScale) - _gyrB[0];
          //_gyFifo[i] = (rawMeas[1] * _gyroScale) - _gyrB[1];
          //_gzFifo[i] = (rawMeas[2] * _gyroScale) - _gyrB[2];
          _gxFifo[i] = rawMeas[0]; // x
          _gyFifo[i] = rawMeas[1]; // y
          _gzFifo[i] = rawMeas[2]; // z
          _gSize = numFrames;
        } else {
          // high resolution mode
          // combine into 20 bit values
          int32_t rawMeas[3]; // increase size for larger gyro data
          // 20-bit data format, 19-bits of actual data and the LSB is always
          // set to 0 X - always 0x01 [19:12], 0x02 [11:4], 0x11 [3:0]
          rawMeas[0] = (((int32_t)_buffer[0 + gyroIndex]) << 12) |
                       ((_buffer[1 + gyroIndex] << 4) |
                        ((_buffer[0 + extensionIndex] >> 4) & 0x0F)) >>
                           2;
          // C/C++ does NOT automatically sign-extend non-standard bit widths
          // (like 18-bit). Sign-extend 18-bit value to 32-bit
          if (rawMeas[0] & (1 << 17)) {
            rawMeas[0] |= 0xFFFC0000;
          }
          // Y - always 0x03, 0x04, 0x12
          rawMeas[1] = (((int32_t)_buffer[2 + gyroIndex]) << 12) |
                       ((_buffer[3 + gyroIndex] << 4) |
                        ((_buffer[0 + extensionIndex] >> 4) & 0x0F)) >>
                           2;
          if (rawMeas[1] & (1 << 17)) {
            rawMeas[1] |= 0xFFFC0000;
          }
          // Z - always 0x05, 0x06, 0x13
          rawMeas[2] = (((int32_t)_buffer[4 + gyroIndex]) << 12) |
                       ((_buffer[5 + gyroIndex] << 4) |
                        ((_buffer[0 + extensionIndex] >> 4) & 0x0F)) >>
                           2;
          if (rawMeas[2] & (1 << 17)) {
            rawMeas[2] |= 0xFFFC0000;
          }
          // transform and convert to float values
          //_gxFifo[i] = (rawMeas[0] * _gyroScale) - _gyrB[0];
          //_gyFifo[i] = (rawMeas[1] * _gyroScale) - _gyrB[1];
          //_gzFifo[i] = (rawMeas[2] * _gyroScale) - _gyrB[2];
          _gxFifo[i] = rawMeas[0]; // x
          _gyFifo[i] = rawMeas[1]; // y
          _gzFifo[i] = rawMeas[2]; // z
          _gSize = numFrames;
        }
      }

      // if FIFO contains timestamp data
      if (_enFifoTimestamp) {
        // combine into 16 bit values
        int16_t rawMeas = (((int16_t)_buffer[0 + timestampIndex] << 8) |
                           _buffer[1 + timestampIndex]);
        // transform and convert to float values
        //_tFifo[i] = (static_cast<float>(rawMeas));
        //_tSize = _fifoSize/_fifoFrameSize;
        _timeFifo[i] = (rawMeas);
        _timeSize = numFrames;
      }
    }
    return 1;
  }
  /* returns the accelerometer FIFO size and data in the x direction */
  // int is int32 by default
  // tested - works for normal res, works now for high res (needed manual sign
  // extension)
  void ICM42688_FIFO::getFifoAccelX_raw(size_t *size, int *data) {
    *size = _aSize;
    memcpy(data, _axFifo, _aSize * sizeof(int));
  }

  /* returns the accelerometer FIFO size and data in the y direction */
  void ICM42688_FIFO::getFifoAccelY_raw(size_t *size, int *data) {
    *size = _aSize;
    memcpy(data, _ayFifo, _aSize * sizeof(int));
  }

  /* returns the accelerometer FIFO size and data in the z direction */
  void ICM42688_FIFO::getFifoAccelZ_raw(size_t *size, int *data) {
    *size = _aSize;
    memcpy(data, _azFifo, _aSize * sizeof(int));
  }

  /*
  ///* returns the accelerometer FIFO size and data in the x direction, m/s/s
  void ICM42688_FIFO::getFifoAccelX_mss(size_t *size,float* data) {
    *size = _aSize;
    memcpy(data,_axFifo,_aSize*sizeof(float));
  }

  ///* returns the accelerometer FIFO size and data in the y direction, m/s/s
  void ICM42688_FIFO::getFifoAccelY_mss(size_t *size,float* data) {
    *size = _aSize;
    memcpy(data,_ayFifo,_aSize*sizeof(float));
  }

  ///* returns the accelerometer FIFO size and data in the z direction, m/s/s
  void ICM42688_FIFO::getFifoAccelZ_mss(size_t *size,float* data) {
    *size = _aSize;
    memcpy(data,_azFifo,_aSize*sizeof(float));
  }

  */

  /* returns the gyroscope FIFO size and data in the x direction, dps */
  void ICM42688_FIFO::getFifoGyroX(size_t *size, float *data) {
    *size = _gSize;
    memcpy(data, _gxFifo, _gSize * sizeof(float));
  }

  /* returns the gyroscope FIFO size and data in the y direction, dps */
  void ICM42688_FIFO::getFifoGyroY(size_t *size, float *data) {
    *size = _gSize;
    memcpy(data, _gyFifo, _gSize * sizeof(float));
  }

  /* returns the gyroscope FIFO size and data in the z direction, dps */
  void ICM42688_FIFO::getFifoGyroZ(size_t *size, float *data) {
    *size = _gSize;
    memcpy(data, _gzFifo, _gSize * sizeof(float));
  }

  /* returns the die temperature FIFO size and data, C */
  void ICM42688_FIFO::getFifoTemperature_C(size_t *size, float *data) {
    *size = _tempSize;
    memcpy(data, _tempFifo, _tempSize * sizeof(float));
  }

  /* returns the header data */
  // 120 = high res
  // 104 = accel + gyro, normal res
  // 64 = accel only, normal res
  // 32 = gyro only, normal res

  // tested - works
  void ICM42688_FIFO::getFifoHeader(size_t *size, uint8_t *data) {
    *size = _hSize;
    memcpy(data, _hFifo, _hSize * sizeof(int8_t));
  }

  /* estimates the gyro biases */
  int ICM42688::calibrateGyro() {
    // set at a lower range (more resolution) since IMU not moving
    const GyroFS current_fssel = _gyroFS;
    if (setGyroFS(dps250) < 0)
      return -1;

    // take samples and find bias
    _gyroBD[0] = 0;
    _gyroBD[1] = 0;
    _gyroBD[2] = 0;
    for (size_t i = 0; i < NUM_CALIB_SAMPLES; i++) {
      getAGT();
      _gyroBD[0] += (gyrX() + _gyrB[0]) / NUM_CALIB_SAMPLES;
      _gyroBD[1] += (gyrY() + _gyrB[1]) / NUM_CALIB_SAMPLES;
      _gyroBD[2] += (gyrZ() + _gyrB[2]) / NUM_CALIB_SAMPLES;
      delay(1);
    }
    _gyrB[0] = _gyroBD[0];
    _gyrB[1] = _gyroBD[1];
    _gyrB[2] = _gyroBD[2];

    // recover the full scale setting
    if (setGyroFS(current_fssel) < 0)
      return -4;
    return 1;
  }

  /* returns the gyro bias in the X direction, dps */
  float ICM42688::getGyroBiasX() { return _gyrB[0]; }

  /* returns the gyro bias in the Y direction, dps */
  float ICM42688::getGyroBiasY() { return _gyrB[1]; }

  /* returns the gyro bias in the Z direction, dps */
  float ICM42688::getGyroBiasZ() { return _gyrB[2]; }

  /* sets the gyro bias in the X direction to bias, dps */
  void ICM42688::setGyroBiasX(float bias) { _gyrB[0] = bias; }

  /* sets the gyro bias in the Y direction to bias, dps */
  void ICM42688::setGyroBiasY(float bias) { _gyrB[1] = bias; }

  /* sets the gyro bias in the Z direction to bias, dps */
  void ICM42688::setGyroBiasZ(float bias) { _gyrB[2] = bias; }

  /* finds bias and scale factor calibration for the accelerometer,
  this should be run for each axis in each direction (6 total) to find
  the min and max values along each */
  int ICM42688::calibrateAccel() {
    // set at a lower range (more resolution) since IMU not moving
    const AccelFS current_fssel = _accelFS;
    if (setAccelFS(gpm2) < 0)
      return -1;

    // take samples and find min / max
    _accBD[0] = 0;
    _accBD[1] = 0;
    _accBD[2] = 0;
    for (size_t i = 0; i < NUM_CALIB_SAMPLES; i++) {
      getAGT();
      _accBD[0] += (accX() / _accS[0] + _accB[0]) / NUM_CALIB_SAMPLES;
      _accBD[1] += (accY() / _accS[1] + _accB[1]) / NUM_CALIB_SAMPLES;
      _accBD[2] += (accZ() / _accS[2] + _accB[2]) / NUM_CALIB_SAMPLES;
      delay(1);
    }
    if (_accBD[0] > 0.9f) {
      _accMax[0] = _accBD[0];
    }
    if (_accBD[1] > 0.9f) {
      _accMax[1] = _accBD[1];
    }
    if (_accBD[2] > 0.9f) {
      _accMax[2] = _accBD[2];
    }
    if (_accBD[0] < -0.9f) {
      _accMin[0] = _accBD[0];
    }
    if (_accBD[1] < -0.9f) {
      _accMin[1] = _accBD[1];
    }
    if (_accBD[2] < -0.9f) {
      _accMin[2] = _accBD[2];
    }

    // find bias and scale factor
    if ((abs(_accMin[0]) > 0.9f) && (abs(_accMax[0]) > 0.9f)) {
      _accB[0] = (_accMin[0] + _accMax[0]) / 2.0f;
      _accS[0] = 1 / ((abs(_accMin[0]) + abs(_accMax[0])) / 2.0f);
    }
    if ((abs(_accMin[1]) > 0.9f) && (abs(_accMax[1]) > 0.9f)) {
      _accB[1] = (_accMin[1] + _accMax[1]) / 2.0f;
      _accS[1] = 1 / ((abs(_accMin[1]) + abs(_accMax[1])) / 2.0f);
    }
    if ((abs(_accMin[2]) > 0.9f) && (abs(_accMax[2]) > 0.9f)) {
      _accB[2] = (_accMin[2] + _accMax[2]) / 2.0f;
      _accS[2] = 1 / ((abs(_accMin[2]) + abs(_accMax[2])) / 2.0f);
    }

    // recover the full scale setting
    if (setAccelFS(current_fssel) < 0)
      return -4;
    return 1;
  }

  /* returns the accelerometer bias in the X direction, m/s/s */
  float ICM42688::getAccelBiasX_mss() { return _accB[0]; }

  /* returns the accelerometer scale factor in the X direction */
  float ICM42688::getAccelScaleFactorX() { return _accS[0]; }

  /* returns the accelerometer bias in the Y direction, m/s/s */
  float ICM42688::getAccelBiasY_mss() { return _accB[1]; }

  /* returns the accelerometer scale factor in the Y direction */
  float ICM42688::getAccelScaleFactorY() { return _accS[1]; }

  /* returns the accelerometer bias in the Z direction, m/s/s */
  float ICM42688::getAccelBiasZ_mss() { return _accB[2]; }

  /* returns the accelerometer scale factor in the Z direction */
  float ICM42688::getAccelScaleFactorZ() { return _accS[2]; }

  /* sets the accelerometer bias (m/s/s) and scale factor in the X direction */
  void ICM42688::setAccelCalX(float bias, float scaleFactor) {
    _accB[0] = bias;
    _accS[0] = scaleFactor;
  }

  /* sets the accelerometer bias (m/s/s) and scale factor in the Y direction */
  void ICM42688::setAccelCalY(float bias, float scaleFactor) {
    _accB[1] = bias;
    _accS[1] = scaleFactor;
  }

  /* sets the accelerometer bias (m/s/s) and scale factor in the Z direction */
  void ICM42688::setAccelCalZ(float bias, float scaleFactor) {
    _accB[2] = bias;
    _accS[2] = scaleFactor;
  }

  /* writes a byte to ICM42688 register given a register address and data */
  int ICM42688::writeRegister(uint8_t subAddress, uint8_t data) {
    /* write data to device */
    if (_useSPI) {
      _spi->beginTransaction(SPISettings(SPI_LS_CLOCK, MSBFIRST,
                                         SPI_MODE3)); // begin the transaction
      digitalWrite(_csPin, LOW);  // select the ICM42688 chip
      _spi->transfer(subAddress); // write the register address
      _spi->transfer(data);       // write the data
      digitalWrite(_csPin, HIGH); // deselect the ICM42688 chip
      _spi->endTransaction();     // end the transaction
    } else {
      _i2c->beginTransmission(_address); // open the device
      _i2c->write(subAddress);           // write the register address
      _i2c->write(data);                 // write the data
      _i2c->endTransmission();
    }

    delay(10);

    /* read back the register */
    readRegisters(subAddress, 1, _buffer);
    /* check the read back register against the written register */
    if (_buffer[0] == data) {
      return 1;
    } else {
      return -1;
    }
  }

  /* reads registers from ICM42688 given a starting register address, number of
   * bytes, and a pointer to store data */
  int IRAM_ATTR ICM42688::readRegisters(uint8_t subAddress, uint8_t count,
                                        uint8_t *dest) {
    // if( _useSPI ) {
    //  begin the transaction
    // if(_useSPIHS) {
    _spi->beginTransaction(_spiSettings);
    //}
    //else{
    //  _spi->beginTransaction(SPISettings(SPI_LS_CLOCK, MSBFIRST, SPI_MODE3));
    //}
    
    //digitalWrite(_csPin,LOW); // select the ICM42688 chip
    REG_CLR_BIT(GPIO_OUT_REG, 1 << _csPin); // Set CS pin LOW
    
    _spi->transfer(subAddress | 0x80); // specify the starting register address

    
    for(uint8_t i = 0; i < count; i++) {
      dest[i] = _spi->transfer(0x00); // read the data
    }


    //digitalWrite(_csPin,HIGH); // deselect the ICM42688 chip
    REG_SET_BIT(GPIO_OUT_REG, 1 << _csPin); // Set CS pin HIGH

    _spi->endTransaction(); // end the transaction
    return 1;
  }
  /*
  else{
    _i2c->beginTransmission(_address); // open the device
    _i2c->write(subAddress); // specify the starting register address
    _i2c->endTransmission(false);
    _numBytes = _i2c->requestFrom(_address, count); // specify the number of bytes to receive if (_numBytes == count) { for(uint8_t i = 0; i < count; i++) {
        dest[i] = _i2c->read();
      }
      return 1;
    } else {
      return -1;
    }
  }
}
*/

  int ICM42688::setBank(uint8_t bank) {
    // if we are already on this bank, bail
    if (_bank == bank)
      return 1;

    _bank = bank;

    return writeRegister(REG_BANK_SEL, bank);
  }

void ICM42688::reset() {
  setBank(0);

  writeRegister(UB0_REG_DEVICE_CONFIG, 0x01);

  // wait for ICM42688 to come back up
  delay(1);
}

/* gets the ICM42688 WHO_AM_I register value */
uint8_t ICM42688::whoAmI() {
  setBank(0);

  // read the WHO AM I register
  if (readRegisters(UB0_REG_WHO_AM_I, 1, _buffer) < 0) {
    return -1;
  }
  // return the register value
  return _buffer[0];
}
