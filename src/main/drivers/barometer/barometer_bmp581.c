/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 *
 * BMP581 driver (also detects the register-compatible BMP580)
 *
 * References:
 * BMP581 datasheet - https://www.bosch-sensortec.com/products/environmental-sensors/pressure-sensors/bmp581/
 * BMP5-Sensor-API - https://github.com/boschsensortec/BMP5-Sensor-API
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#if defined(USE_BARO) && (defined(USE_BARO_BMP581) || defined(USE_BARO_SPI_BMP581))

#include "build/build_config.h"
#include "build/debug.h"

#include "drivers/barometer/barometer.h"
#include "drivers/bus.h"
#include "drivers/bus_i2c.h"
#include "drivers/bus_i2c_busdev.h"
#include "drivers/bus_spi.h"
#include "drivers/io.h"
#include "drivers/time.h"

#include "barometer_bmp581.h"

// 10 MHz max SPI frequency
#define BMP581_MAX_SPI_CLK_HZ 10000000

// I2C addresses
#define BMP581_I2C_ADDR_PRIMARY     (0x46)  // SDO = LOW
#define BMP581_I2C_ADDR_SECONDARY   (0x47)  // SDO = HIGH

// Chip IDs - both are valid BMP58x parts with an identical register map
#define BMP580_CHIP_ID              (0x50)
#define BMP581_CHIP_ID              (0x51)

// Register addresses
#define BMP581_REG_CHIP_ID          (0x01)
#define BMP581_REG_REV_ID           (0x02)
#define BMP581_REG_CHIP_STATUS      (0x11)
#define BMP581_REG_DRIVE_CONFIG     (0x13)
#define BMP581_REG_INT_CONFIG       (0x14)
#define BMP581_REG_INT_SOURCE       (0x15)
#define BMP581_REG_FIFO_CONFIG      (0x16)
#define BMP581_REG_FIFO_COUNT       (0x17)
#define BMP581_REG_FIFO_SEL         (0x18)
#define BMP581_REG_TEMP_DATA_XLSB   (0x1D)
#define BMP581_REG_TEMP_DATA_LSB    (0x1E)
#define BMP581_REG_TEMP_DATA_MSB    (0x1F)
#define BMP581_REG_PRESS_DATA_XLSB  (0x20)
#define BMP581_REG_PRESS_DATA_LSB   (0x21)
#define BMP581_REG_PRESS_DATA_MSB   (0x22)
#define BMP581_REG_INT_STATUS       (0x27)
#define BMP581_REG_STATUS           (0x28)
#define BMP581_REG_FIFO_DATA        (0x29)
#define BMP581_REG_NVM_ADDR         (0x2B)
#define BMP581_REG_NVM_DATA_LSB     (0x2C)
#define BMP581_REG_NVM_DATA_MSB     (0x2D)
#define BMP581_REG_DSP_CONFIG       (0x30)
#define BMP581_REG_DSP_IIR          (0x31)
#define BMP581_REG_OOR_THR_P_LSB    (0x32)
#define BMP581_REG_OOR_THR_P_MSB    (0x33)
#define BMP581_REG_OOR_RANGE        (0x34)
#define BMP581_REG_OOR_CONFIG       (0x35)
#define BMP581_REG_OSR_CONFIG       (0x36)
#define BMP581_REG_ODR_CONFIG       (0x37)
#define BMP581_REG_OSR_EFF          (0x38)
#define BMP581_REG_CMD              (0x7E)

// Commands (CMD register)
#define BMP581_CMD_NOP              (0x00)
#define BMP581_CMD_FIFO_FLUSH       (0xB0)
#define BMP581_CMD_SOFT_RESET       (0xB6)

// ODR_CONFIG (0x37): [1:0] pwr_mode, [6:2] odr, [7] deep_dis
#define BMP581_MODE_STANDBY         (0x00)
#define BMP581_MODE_NORMAL          (0x01)  // continuous conversion at the configured ODR
#define BMP581_MODE_FORCED          (0x02)  // single shot, returns to standby
#define BMP581_MODE_NON_STOP        (0x03)

// Deep standby is entered from standby unless explicitly disabled. In deep standby the
// sensor ignores configuration writes, so this bit must be set whenever ODR_CONFIG is written.
#define BMP581_DEEP_STANDBY_DISABLE (0x01 << 7)

#define BMP581_ODR(x)               (((x) & 0x1F) << 2)
#define BMP581_ODR_240_HZ           BMP581_ODR(0x00)
#define BMP581_ODR_100_HZ           BMP581_ODR(0x0A)
#define BMP581_ODR_50_HZ            BMP581_ODR(0x0F)
#define BMP581_ODR_25_HZ            BMP581_ODR(0x14)
#define BMP581_ODR_10_HZ            BMP581_ODR(0x17)

// OSR_CONFIG (0x36): [2:0] osr_t, [5:3] osr_p, [6] press_en
#define BMP581_OSR_1X               (0x00)
#define BMP581_OSR_2X               (0x01)
#define BMP581_OSR_4X               (0x02)
#define BMP581_OSR_8X               (0x03)
#define BMP581_OSR_16X              (0x04)
#define BMP581_OSR_32X              (0x05)
#define BMP581_OSR_64X              (0x06)
#define BMP581_OSR_128X             (0x07)

#define BMP581_OSR_TEMP(x)          ((x) & 0x07)
#define BMP581_OSR_PRESS(x)         (((x) & 0x07) << 3)
#define BMP581_OSR_PRESS_EN         (0x01 << 6)

// DSP_CONFIG (0x30)
#define BMP581_DSP_COMP_PRESS_TEMP  (0x03)      // compensate both pressure and temperature
#define BMP581_DSP_SHDW_SEL_IIR_T   (0x01 << 3) // IIR filtered temperature into TEMP_DATA
#define BMP581_DSP_SHDW_SEL_IIR_P   (0x01 << 5) // IIR filtered pressure into PRESS_DATA

// DSP_IIR (0x31): [2:0] set_iir_t, [5:3] set_iir_p
// Coefficient encoding: 0=bypass, 1=1, 2=3, 3=7, 4=15, 5=31, 6=63, 7=127
#define BMP581_IIR_COEF_BYPASS      (0x00)
#define BMP581_IIR_COEF_1           (0x01)
#define BMP581_IIR_COEF_3           (0x02)
#define BMP581_IIR_COEF_7           (0x03)
#define BMP581_IIR_COEF_15          (0x04)
#define BMP581_IIR_COEF_31          (0x05)
#define BMP581_IIR_COEF_63          (0x06)
#define BMP581_IIR_COEF_127         (0x07)

#define BMP581_IIR_TEMP(x)          ((x) & 0x07)
#define BMP581_IIR_PRESS(x)         (((x) & 0x07) << 3)

// STATUS register (0x28) bitfields
#define BMP581_STATUS_NVM_RDY       (0x01 << 1)
#define BMP581_STATUS_NVM_ERR       (0x01 << 2)

// Measurement configuration.
// 64x pressure / 4x temperature oversampling is the fastest combination that still
// sustains a 50 Hz output data rate; asking for more pressure oversampling makes the
// sensor silently reduce the effective OSR because the conversion no longer fits in
// the ODR period (see OSR_EFF register in the datasheet).
#define BMP581_PRESSURE_OSR         BMP581_OSR_64X
#define BMP581_TEMPERATURE_OSR      BMP581_OSR_4X
#define BMP581_MEASUREMENT_ODR      BMP581_ODR_50_HZ

// Light IIR smoothing only - the group delay of a high coefficient (roughly
// coefficient+1 samples, so 320 ms at coefficient 15 and 50 Hz) hurts altitude hold
// more than the noise it removes. 64x oversampling already does the heavy lifting.
#define BMP581_IIR_COEFFICIENT      BMP581_IIR_COEF_3

// Data frame size: temperature (3 bytes) + pressure (3 bytes)
#define BMP581_DATA_FRAME_SIZE      6

// The sensor returns 0x7F in every data byte while a value is not yet available
#define BMP581_DATA_NOT_READY_BYTE  (0x7F)

// Uncompensated (raw 24-bit) pressure and temperature, last known good values
static uint32_t bmp581_up = 0;
static uint32_t bmp581_ut = 0;

static DMA_DATA_ZERO_INIT uint8_t sensor_data[BMP581_DATA_FRAME_SIZE];

static bool bmp581StartUT(baroDev_t *baro);
static bool bmp581GetUT(baroDev_t *baro);
static bool bmp581ReadUT(baroDev_t *baro);
static bool bmp581StartUP(baroDev_t *baro);
static bool bmp581GetUP(baroDev_t *baro);
static bool bmp581ReadUP(baroDev_t *baro);

static void bmp581Calculate(int32_t *pressure, int32_t *temperature);

static void bmp581BusInit(const extDevice_t *dev)
{
#ifdef USE_BARO_SPI_BMP581
    if (dev->bus->busType == BUS_TYPE_SPI) {
        IOHi(dev->busType_u.spi.csnPin);
        IOInit(dev->busType_u.spi.csnPin, OWNER_BARO_CS, 0);
        IOConfigGPIO(dev->busType_u.spi.csnPin, IOCFG_OUT_PP);
        spiSetClkDivisor(dev, spiCalculateDivider(BMP581_MAX_SPI_CLK_HZ));
    }
#else
    UNUSED(dev);
#endif
}

static void bmp581BusDeinit(const extDevice_t *dev)
{
#ifdef USE_BARO_SPI_BMP581
    if (dev->bus->busType == BUS_TYPE_SPI) {
        ioPreinitByIO(dev->busType_u.spi.csnPin, IOCFG_IPU, PREINIT_PIN_STATE_HIGH);
    }
#else
    UNUSED(dev);
#endif
}

/**
 * @brief Read the chip ID register
 * @param dev Pointer to the external device structure
 * @return Chip ID, or 0 if the read failed
 * @note The BMP58x needs one throw-away register read after power up before it
 *       responds correctly on SPI, so on SPI the register is read twice.
 */
static uint8_t bmp581ReadChipId(const extDevice_t *dev)
{
    uint8_t chipId = 0;

    if (dev->bus->busType == BUS_TYPE_SPI) {
        busReadRegisterBuffer(dev, BMP581_REG_CHIP_ID, &chipId, 1);
    }

    if (!busReadRegisterBuffer(dev, BMP581_REG_CHIP_ID, &chipId, 1)) {
        return 0;
    }

    return chipId;
}

static bool bmp581ChipIdValid(uint8_t chipId)
{
    return (chipId == BMP580_CHIP_ID) || (chipId == BMP581_CHIP_ID);
}

/**
 * @brief Detect and initialise a BMP581 (or BMP580) barometer
 * @param baro Pointer to barometer device structure to initialise
 * @return true if the sensor was detected and configured, false otherwise
 * @note Configures normal (continuous) mode at 50 Hz with 64x pressure and
 *       4x temperature oversampling, and a light IIR filter on both channels.
 */
bool bmp581Detect(baroDev_t *baro)
{
    delay(20);

    extDevice_t *dev = &baro->dev;
    bool defaultAddressApplied = false;

    bmp581BusInit(dev);

    if ((dev->bus->busType == BUS_TYPE_I2C) && (dev->busType_u.i2c.address == 0)) {
        dev->busType_u.i2c.address = BMP581_I2C_ADDR_PRIMARY;
        defaultAddressApplied = true;
    }

    uint8_t chipId = bmp581ReadChipId(dev);

    if (!bmp581ChipIdValid(chipId) && defaultAddressApplied) {
        // Retry on the alternate I2C address (SDO pulled the other way)
        dev->busType_u.i2c.address = BMP581_I2C_ADDR_SECONDARY;
        chipId = bmp581ReadChipId(dev);
    }

    if (!bmp581ChipIdValid(chipId)) {
        bmp581BusDeinit(dev);
        if (defaultAddressApplied) {
            dev->busType_u.i2c.address = 0;
        }
        return false;
    }

    busDeviceRegister(dev);

    // Soft reset, then wait for the NVM copy to complete (datasheet: 2 ms)
    busWriteRegister(dev, BMP581_REG_CMD, BMP581_CMD_SOFT_RESET);
    delay(5);

    uint8_t status = 0;
    if (!busReadRegisterBuffer(dev, BMP581_REG_STATUS, &status, 1) ||
        !(status & BMP581_STATUS_NVM_RDY) || (status & BMP581_STATUS_NVM_ERR)) {
        bmp581BusDeinit(dev);
        return false;
    }

    // Leave deep standby so that the configuration registers accept writes, and stay
    // in standby while they are written - OSR/IIR settings are only latched there.
    busWriteRegister(dev, BMP581_REG_ODR_CONFIG,
        BMP581_DEEP_STANDBY_DISABLE | BMP581_MEASUREMENT_ODR | BMP581_MODE_STANDBY);

    // Compensate both channels and route the IIR filter output into the data registers.
    // Without the shadow-select bits the data registers return unfiltered values.
    busWriteRegister(dev, BMP581_REG_DSP_CONFIG,
        BMP581_DSP_COMP_PRESS_TEMP | BMP581_DSP_SHDW_SEL_IIR_P | BMP581_DSP_SHDW_SEL_IIR_T);

    busWriteRegister(dev, BMP581_REG_DSP_IIR,
        BMP581_IIR_PRESS(BMP581_IIR_COEFFICIENT) | BMP581_IIR_TEMP(BMP581_IIR_COEFFICIENT));

    busWriteRegister(dev, BMP581_REG_OSR_CONFIG,
        BMP581_OSR_PRESS(BMP581_PRESSURE_OSR) | BMP581_OSR_TEMP(BMP581_TEMPERATURE_OSR) | BMP581_OSR_PRESS_EN);

    // Start continuous conversion at the configured ODR, so a fresh sample is always
    // waiting in the data registers when the state machine gets round to reading it.
    busWriteRegister(dev, BMP581_REG_ODR_CONFIG,
        BMP581_DEEP_STANDBY_DISABLE | BMP581_MEASUREMENT_ODR | BMP581_MODE_NORMAL);

    // Temperature and pressure come out of one 6 byte read, so the UT states are stubs
    // and combined_read makes the state machine skip them.
    baro->combined_read = true;
    baro->ut_delay = 0;
    baro->start_ut = bmp581StartUT;
    baro->get_ut = bmp581GetUT;
    baro->read_ut = bmp581ReadUT;

    baro->start_up = bmp581StartUP;
    baro->get_up = bmp581GetUP;
    baro->read_up = bmp581ReadUP;

    // One pass through PRESSURE_START/READ/SAMPLE costs up_delay + 2 ms of fixed 1 ms
    // inter-state waits, so 18 ms lines the polling loop up with the 50 Hz (20 ms) ODR.
    baro->up_delay = 18000;

    baro->calculate = bmp581Calculate;

    while (busBusy(&baro->dev, NULL));

    return true;
}

static bool bmp581StartUT(baroDev_t *baro)
{
    UNUSED(baro);
    // Dummy - temperature is read with pressure
    return true;
}

static bool bmp581ReadUT(baroDev_t *baro)
{
    UNUSED(baro);
    // Dummy - temperature is read with pressure
    return true;
}

static bool bmp581GetUT(baroDev_t *baro)
{
    UNUSED(baro);
    // Dummy - temperature is read with pressure
    return true;
}

static bool bmp581StartUP(baroDev_t *baro)
{
    UNUSED(baro);
    // In normal mode conversions run continuously - nothing to trigger
    return true;
}

/**
 * @brief Read raw pressure and temperature from the sensor
 * @param baro Pointer to barometer device
 * @return true on a successful bus read, false otherwise
 * @note Reads TEMP_XLSB..PRESS_MSB (0x1D..0x22) in one transfer. Synchronous read:
 *       asynchronous I2C reads have been seen to stall on PICO.
 */
static bool bmp581ReadUP(baroDev_t *baro)
{
    return busReadRegisterBuffer(&baro->dev, BMP581_REG_TEMP_DATA_XLSB,
        sensor_data, BMP581_DATA_FRAME_SIZE);
}

/**
 * @brief Assemble the 24-bit raw values from the read buffer
 * @param baro Pointer to barometer device
 * @return true always
 * @note A channel reading 0x7F7F7F has no sample ready yet; the previous value is kept.
 */
static bool bmp581GetUP(baroDev_t *baro)
{
    UNUSED(baro);

    if ((sensor_data[0] != BMP581_DATA_NOT_READY_BYTE) ||
        (sensor_data[1] != BMP581_DATA_NOT_READY_BYTE) ||
        (sensor_data[2] != BMP581_DATA_NOT_READY_BYTE)) {
        bmp581_ut = (uint32_t)sensor_data[0] |
                    ((uint32_t)sensor_data[1] << 8) |
                    ((uint32_t)sensor_data[2] << 16);
    }

    if ((sensor_data[3] != BMP581_DATA_NOT_READY_BYTE) ||
        (sensor_data[4] != BMP581_DATA_NOT_READY_BYTE) ||
        (sensor_data[5] != BMP581_DATA_NOT_READY_BYTE)) {
        bmp581_up = (uint32_t)sensor_data[3] |
                    ((uint32_t)sensor_data[4] << 8) |
                    ((uint32_t)sensor_data[5] << 16);
    }

    return true;
}

/**
 * @brief Convert the raw values to Pa and centidegrees C
 * @param pressure Pointer to store pressure in Pa (may be NULL)
 * @param temperature Pointer to store temperature in centidegrees C (may be NULL)
 * @note The sensor outputs already compensated data: temperature is a signed
 *       24-bit value in units of 1/65536 degC, pressure an unsigned 24-bit value
 *       in units of 1/64 Pa.
 */
static void bmp581Calculate(int32_t *pressure, int32_t *temperature)
{
    // Sign extend the 24-bit temperature to 32 bits
    int32_t tempRaw = (int32_t)bmp581_ut;
    if (tempRaw & 0x800000) {
        tempRaw |= ~0xFFFFFF;
    }

    if (temperature) {
        // 1/65536 degC per count -> centidegrees. Worst case 0x7FFFFF * 100 still fits in int32.
        *temperature = (tempRaw * 100) / 65536;
    }

    if (pressure) {
        *pressure = (int32_t)(bmp581_up / 64);
    }
}

#endif // defined(USE_BARO) && (defined(USE_BARO_BMP581) || defined(USE_BARO_SPI_BMP581))
