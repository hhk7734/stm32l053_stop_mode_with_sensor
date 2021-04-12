#include "hal_bme280.h"

#include <cmath>
#include <cstring>

/*!
 *  @brief  class constructor
 */
BME280::BME280()
    : _cs(-1)
    , _mosi(-1)
    , _miso(-1)
    , _sck(-1) {}

BME280::~BME280(void) {}

/*!
 *   @brief  Initialise sensor with given parameters / settings
 *   @param addr the I2C address the device can be found on
 *   @param theWire the I2C object to use, defaults to &Wire
 *   @returns true on success, false otherwise
 */
bool BME280::begin(I2C_HandleTypeDef *hi2c, uint8_t addr) {
    bool status = false;
    _i2caddr    = addr << 1;
    _hi2c       = hi2c;
    status      = init();

    return status;
}

/*!
 *   @brief  Initialise sensor with given parameters / settings
 *   @returns true on success, false otherwise
 */
bool BME280::init() {
    // init I2C or SPI sensor interface

    // check if sensor, i.e. the chip ID is correct
    _sensorID = read8(BME280_REGISTER_CHIPID);
    if(_sensorID != 0x60) return false;

    // reset the device using soft-reset
    // this makes sure the IIR is off, etc.
    write8(BME280_REGISTER_SOFTRESET, 0xB6);

    // wait for chip to wake up.
    HAL_Delay(10);

    // if chip is still reading calibration, HAL_Delay
    while(isReadingCalibration()) HAL_Delay(10);

    readCoefficients();    // read trimming parameters, see DS 4.2.2

    setSampling();    // use defaults

    HAL_Delay(100);

    return true;
}

/*!
 *   @brief  setup sensor with given parameters / settings
 *
 *   This is simply a overload to the normal begin()-function, so SPI users
 *   don't get confused about the library requiring an address.
 *   @param mode the power mode to use for the sensor
 *   @param tempSampling the temp samping rate to use
 *   @param pressSampling the pressure sampling rate to use
 *   @param humSampling the humidity sampling rate to use
 *   @param filter the filter mode to use
 *   @param duration the standby duration to use
 */
void BME280::setSampling(sensor_mode      mode,
                         sensor_sampling  tempSampling,
                         sensor_sampling  pressSampling,
                         sensor_sampling  humSampling,
                         sensor_filter    filter,
                         standby_duration duration) {
    _measReg.mode   = mode;
    _measReg.osrs_t = tempSampling;
    _measReg.osrs_p = pressSampling;

    _humReg.osrs_h    = humSampling;
    _configReg.filter = filter;
    _configReg.t_sb   = duration;

    // making sure sensor is in sleep mode before setting configuration
    // as it otherwise may be ignored
    write8(BME280_REGISTER_CONTROL, MODE_SLEEP);

    // you must make sure to also set REGISTER_CONTROL after setting the
    // CONTROLHUMID register, otherwise the values won't be applied (see
    // DS 5.4.3)
    write8(BME280_REGISTER_CONTROLHUMID, _humReg.get());
    write8(BME280_REGISTER_CONFIG, _configReg.get());
    write8(BME280_REGISTER_CONTROL, _measReg.get());
}

/*!
 *   @brief  Writes an 8 bit value over I2C or SPI
 *   @param reg the register address to write to
 *   @param value the value to write to the register
 */
void BME280::write8(uint8_t reg, uint8_t value) {
    uint8_t tx_buff[2] = {reg, value};
    HAL_I2C_Master_Transmit(_hi2c, _i2caddr, tx_buff, 2, 10);
}

/*!
 *   @brief  Reads an 8 bit value over I2C or SPI
 *   @param reg the register address to read from
 *   @returns the data uint8_t read from the device
 */
uint8_t BME280::read8(uint8_t reg) {
    uint8_t rx_buff[1];
    HAL_I2C_Mem_Read(
        _hi2c, _i2caddr, reg, I2C_MEMADD_SIZE_8BIT, rx_buff, 1, 10);
    return rx_buff[0];
}

/*!
 *   @brief  Reads a 16 bit value over I2C or SPI
 *   @param reg the register address to read from
 *   @returns the 16 bit data value read from the device
 */
uint16_t BME280::read16(uint8_t reg) {
    uint8_t rx_buff[2];
    HAL_I2C_Mem_Read(
        _hi2c, _i2caddr, reg, I2C_MEMADD_SIZE_8BIT, rx_buff, 2, 10);
    return rx_buff[0] << 8 | rx_buff[1];
}

/*!
 *   @brief  Reads a signed 16 bit little endian value over I2C or SPI
 *   @param reg the register address to read from
 *   @returns the 16 bit data value read from the device
 */
uint16_t BME280::read16_LE(uint8_t reg) {
    uint16_t temp = read16(reg);
    return (temp >> 8) | (temp << 8);
}

/*!
 *   @brief  Reads a signed 16 bit value over I2C or SPI
 *   @param reg the register address to read from
 *   @returns the 16 bit data value read from the device
 */
int16_t BME280::readS16(uint8_t reg) { return (int16_t)read16(reg); }

/*!
 *   @brief  Reads a signed little endian 16 bit value over I2C or SPI
 *   @param reg the register address to read from
 *   @returns the 16 bit data value read from the device
 */
int16_t BME280::readS16_LE(uint8_t reg) { return (int16_t)read16_LE(reg); }

/*!
 *   @brief  Reads a 24 bit value over I2C
 *   @param reg the register address to read from
 *   @returns the 24 bit data value read from the device
 */
uint32_t BME280::read24(uint8_t reg) {
    uint8_t rx_buff[3];
    HAL_I2C_Mem_Read(
        _hi2c, _i2caddr, reg, I2C_MEMADD_SIZE_8BIT, rx_buff, 3, 10);
    return rx_buff[0] << 16 | rx_buff[1] << 8 | rx_buff[2];
}
//
///*!
// *  @brief  Take a new measurement (only possible in forced mode)
//    @returns true in case of success else false
// */
// bool BME280::takeForcedMeasurement(void) {
//    bool return_value = false;
//    // If we are in forced mode, the BME sensor goes back to sleep after each
//    // measurement and we need to set it to forced mode once at this point, so
//    // it will take the next measurement and then return to sleep again.
//    // In normal mode simply does new measurements periodically.
//    if(_measReg.mode == MODE_FORCED) {
//        return_value = true;
//        // set to forced mode, i.e. "take next measurement"
//        write8(BME280_REGISTER_CONTROL, _measReg.get());
//        // Store current time to measure the timeout
//        uint32_t timeout_start = millis();
//        // wait until measurement has been completed, otherwise we would read
//        // the the values from the last measurement or the timeout occurred
//        // after 2 sec.
//        while(read8(BME280_REGISTER_STATUS) & 0x08) {
//            // In case of a timeout, stop the while loop
//            if((millis() - timeout_start) > 2000) {
//                return_value = false;
//                break;
//            }
//            HAL_Delay(1);
//        }
//    }
//    return return_value;
//}

/*!
 *   @brief  Reads the factory-set coefficients
 */
void BME280::readCoefficients(void) {
    _bme280_calib.dig_T1 = read16_LE(BME280_REGISTER_DIG_T1);
    _bme280_calib.dig_T2 = readS16_LE(BME280_REGISTER_DIG_T2);
    _bme280_calib.dig_T3 = readS16_LE(BME280_REGISTER_DIG_T3);

    _bme280_calib.dig_P1 = read16_LE(BME280_REGISTER_DIG_P1);
    _bme280_calib.dig_P2 = readS16_LE(BME280_REGISTER_DIG_P2);
    _bme280_calib.dig_P3 = readS16_LE(BME280_REGISTER_DIG_P3);
    _bme280_calib.dig_P4 = readS16_LE(BME280_REGISTER_DIG_P4);
    _bme280_calib.dig_P5 = readS16_LE(BME280_REGISTER_DIG_P5);
    _bme280_calib.dig_P6 = readS16_LE(BME280_REGISTER_DIG_P6);
    _bme280_calib.dig_P7 = readS16_LE(BME280_REGISTER_DIG_P7);
    _bme280_calib.dig_P8 = readS16_LE(BME280_REGISTER_DIG_P8);
    _bme280_calib.dig_P9 = readS16_LE(BME280_REGISTER_DIG_P9);

    _bme280_calib.dig_H1 = read8(BME280_REGISTER_DIG_H1);
    _bme280_calib.dig_H2 = readS16_LE(BME280_REGISTER_DIG_H2);
    _bme280_calib.dig_H3 = read8(BME280_REGISTER_DIG_H3);
    _bme280_calib.dig_H4 = ((int8_t)read8(BME280_REGISTER_DIG_H4) << 4)
                           | (read8(BME280_REGISTER_DIG_H4 + 1) & 0xF);
    _bme280_calib.dig_H5 = ((int8_t)read8(BME280_REGISTER_DIG_H5 + 1) << 4)
                           | (read8(BME280_REGISTER_DIG_H5) >> 4);
    _bme280_calib.dig_H6 = (int8_t)read8(BME280_REGISTER_DIG_H6);
}

/*!
 *   @brief return true if chip is busy reading cal data
 *   @returns true if reading calibration, false otherwise
 */
bool BME280::isReadingCalibration(void) {
    uint8_t const rStatus = read8(BME280_REGISTER_STATUS);

    return (rStatus & (1 << 0)) != 0;
}

/*!
 *   @brief  Returns the temperature from the sensor
 *   @returns the temperature read from the device
 */
float BME280::readTemperature(void) {
    int32_t var1, var2;

    int32_t adc_T = read24(BME280_REGISTER_TEMPDATA);
    if(adc_T == 0x800000)    // value in case temp measurement was disabled
        return NAN;
    adc_T >>= 4;

    var1 = ((((adc_T >> 3) - ((int32_t)_bme280_calib.dig_T1 << 1)))
            * ((int32_t)_bme280_calib.dig_T2))
           >> 11;

    var2 = (((((adc_T >> 4) - ((int32_t)_bme280_calib.dig_T1))
              * ((adc_T >> 4) - ((int32_t)_bme280_calib.dig_T1)))
             >> 12)
            * ((int32_t)_bme280_calib.dig_T3))
           >> 14;

    t_fine = var1 + var2 + t_fine_adjust;

    float T = (t_fine * 5 + 128) >> 8;
    return T / 100;
}

/*!
 *   @brief  Returns the pressure from the sensor
 *   @returns the pressure value (in Pascal) read from the device
 */
float BME280::readPressure(void) {
    int64_t var1, var2, p;

    readTemperature();    // must be done first to get t_fine

    int32_t adc_P = read24(BME280_REGISTER_PRESSUREDATA);
    if(adc_P == 0x800000)    // value in case pressure measurement was disabled
        return NAN;
    adc_P >>= 4;

    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)_bme280_calib.dig_P6;
    var2 = var2 + ((var1 * (int64_t)_bme280_calib.dig_P5) << 17);
    var2 = var2 + (((int64_t)_bme280_calib.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)_bme280_calib.dig_P3) >> 8)
           + ((var1 * (int64_t)_bme280_calib.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)_bme280_calib.dig_P1)
           >> 33;

    if(var1 == 0) {
        return 0;    // avoid exception caused by division by zero
    }
    p    = 1048576 - adc_P;
    p    = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)_bme280_calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)_bme280_calib.dig_P8) * p) >> 19;

    p = ((p + var1 + var2) >> 8) + (((int64_t)_bme280_calib.dig_P7) << 4);
    return (float)p / 256;
}

/*!
 *  @brief  Returns the humidity from the sensor
 *  @returns the humidity value read from the device
 */
float BME280::readHumidity(void) {
    readTemperature();    // must be done first to get t_fine

    int32_t adc_H = read16(BME280_REGISTER_HUMIDDATA);
    if(adc_H == 0x8000)    // value in case humidity measurement was disabled
        return NAN;

    int32_t v_x1_u32r;

    v_x1_u32r = (t_fine - ((int32_t)76800));

    v_x1_u32r = (((((adc_H << 14) - (((int32_t)_bme280_calib.dig_H4) << 20)
                    - (((int32_t)_bme280_calib.dig_H5) * v_x1_u32r))
                   + ((int32_t)16384))
                  >> 15)
                 * (((((((v_x1_u32r * ((int32_t)_bme280_calib.dig_H6)) >> 10)
                        * (((v_x1_u32r * ((int32_t)_bme280_calib.dig_H3)) >> 11)
                           + ((int32_t)32768)))
                       >> 10)
                      + ((int32_t)2097152))
                         * ((int32_t)_bme280_calib.dig_H2)
                     + 8192)
                    >> 14));

    v_x1_u32r = (v_x1_u32r
                 - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7)
                     * ((int32_t)_bme280_calib.dig_H1))
                    >> 4));

    v_x1_u32r = (v_x1_u32r < 0) ? 0 : v_x1_u32r;
    v_x1_u32r = (v_x1_u32r > 419430400) ? 419430400 : v_x1_u32r;
    float h   = (v_x1_u32r >> 12);
    return h / 1024.0;
}

/*!
 *   Calculates the altitude (in meters) from the specified atmospheric
 *   pressure (in hPa), and sea-level pressure (in hPa).
 *   @param  seaLevel      Sea-level pressure in hPa
 *   @returns the altitude value read from the device
 */
float BME280::readAltitude(float seaLevel) {
    // Equation taken from BMP180 datasheet (page 16):
    //  http://www.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf

    // Note that using the equation from wikipedia can give bad results
    // at high altitude. See this thread for more information:
    //  http://forums.adafruit.com/viewtopic.php?f=22&t=58064

    float atmospheric = readPressure() / 100.0F;
    return 44330.0 * (1.0 - pow(atmospheric / seaLevel, 0.1903));
}

/*!
 *   Calculates the pressure at sea level (in hPa) from the specified
 * altitude (in meters), and atmospheric pressure (in hPa).
 *   @param  altitude      Altitude in meters
 *   @param  atmospheric   Atmospheric pressure in hPa
 *   @returns the pressure at sea level (in hPa) from the specified altitude
 */
float BME280::seaLevelForAltitude(float altitude, float atmospheric) {
    // Equation taken from BMP180 datasheet (page 17):
    //  http://www.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf

    // Note that using the equation from wikipedia can give bad results
    // at high altitude. See this thread for more information:
    //  http://forums.adafruit.com/viewtopic.php?f=22&t=58064

    return atmospheric / pow(1.0 - (altitude / 44330.0), 5.255);
}

/*!
 *   Returns Sensor ID found by init() for diagnostics
 *   @returns Sensor ID 0x60 for BME280, 0x56, 0x57, 0x58 BMP280
 */
uint32_t BME280::sensorID(void) { return _sensorID; }

/*!
 *   Returns the current temperature compensation value in degrees Celcius
 *   @returns the current temperature compensation value in degrees Celcius
 */
float BME280::getTemperatureCompensation(void) {
    return float(((t_fine_adjust * 5) >> 8) / 100);
};

/*!
 *  Sets a value to be added to each temperature reading. This adjusted
 *  temperature is used in pressure and humidity readings.
 *  @param  adjustment  Value to be added to each tempature reading in Celcius
 */
void BME280::setTemperatureCompensation(float adjustment) {
    // convert the value in C into and adjustment to t_fine
    t_fine_adjust = ((int32_t(adjustment * 100) << 8)) / 5;
};
