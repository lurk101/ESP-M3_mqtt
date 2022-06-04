#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_wifi.h"

#include "driver/gpio.h"
#include "driver/i2c.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "bme280.h"

#define ERROR_CHECK(a) ESP_ERROR_CHECK(a)

static const char* TAG = "BME280";

#define I2C_MASTER_SCL_IO GPIO_NUM_14
#define I2C_MASTER_SDA_IO GPIO_NUM_2
#define I2C_MASTER_NUM 0
#define I2C_MASTER_FREQ_HZ 40000
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0
#define I2C_MASTER_TIMEOUT_MS 1000

#define BME280_SENSOR_ADDR 0x76

#define BME280_REG_T1 0x88
#define BME280_REG_T2 0x8A
#define BME280_REG_T3 0x8C

#define BME280_REG_P1 0x8E
#define BME280_REG_P2 0x90
#define BME280_REG_P3 0x92
#define BME280_REG_P4 0x94
#define BME280_REG_P5 0x96
#define BME280_REG_P6 0x98
#define BME280_REG_P7 0x9A
#define BME280_REG_P8 0x9C
#define BME280_REG_P9 0x9E

#define BME280_REG_H1 0xA1
#define BME280_REG_H2 0xE1
#define BME280_REG_H3 0xE3
#define BME280_REG_H4 0xE4
#define BME280_REG_H5 0xE5
#define BME280_REG_H6 0xE7

#define BME280_REG_CHIPID 0xD0
#define BME280_REG_RESET 0xE0
#define BME280_REG_RESET_VALUE 0xB6

#define BME280_REG_CONTROLHUMID 0xF2
#define BME280_REG_STATUS 0XF3
#define BME280_REG_STATUS_IM_UPDATE_MASK (1 << 0)
#define BME280_REG_STATUS_MEASURING_MASK (1 << 3)
#define BME280_REG_CONTROL 0xF4
#define BME280_REG_CONTROL_MODE_MASK 3
#define BME280_REG_CONTROL_MODE_SLEEP 0
#define BME280_REG_CONTROL_MODE_FORCED 1
#define BME280_REG_CONTROL_MODE_NORMAL 3
#define BME280_REG_CONFIG 0xF5
#define BME280_REG_PRESSUREDATA 0xF7
#define BME280_REG_TEMPDATA 0xFA
#define BME280_REG_HUMIDDATA 0xFD

#define SAMPLING_NONE 0b000
#define SAMPLING_X1 0b001
#define SAMPLING_X2 0b010
#define SAMPLING_X4 0b011
#define SAMPLING_X8 0b100
#define SAMPLING_X16 0b101

#define FILTER_OFF 0b000
#define FILTER_X2 0b001
#define FILTER_X4 0b010
#define FILTER_X8 0b011
#define FILTER_X16 0b100

#define STANDBY_MS_0_5 0b000
#define STANDBY_MS_10 0b110
#define STANDBY_MS_20 0b111
#define STANDBY_MS_62_5 0b001
#define STANDBY_MS_125 0b010
#define STANDBY_MS_250 0b011
#define STANDBY_MS_500 0b100
#define STANDBY_MS_1000 0b101

// temperature compensation values
static uint16_t T1;
static int16_t T2, T3;
// pressure compensation values
static uint16_t P1;
static int16_t P2, P3, P4, P5, P6, P7, P8, P9;
// humidity compensation values
static uint8_t H1, H3;
static int16_t H2, H4, H5;
static int8_t H6;

// Register bit maps
static union {
    struct ctrl_meas {
        uint8_t mode : 2; // device mode
        // 00 = sleep, 01 or 10 = forced, 11 = normal
        uint8_t osrs_p : 3; // pressure oversampling
        // 000 = skipped, 001 = x1, 010 = x2, 011 = x4, 100 = x8, 101 and above = x16
        uint8_t osrs_t : 3; // temperature oversampling
        // 000 = skipped, 001 = x1, 010 = x2, 011 = x4, 100 = x8, 101 and above = x16
    } bits;
    uint8_t u;
} ctrlMeasReg; // measurement control register

static union {
    struct {
        uint8_t osrs_h : 3; // pressure oversampling
        // 000 = skipped, 001 = x1, 010 = x2, 011 = x4, 100 = x8, 101 and above = x16
    } bits;
    uint8_t u;
} ctrlHumReg; // humidity control register object

static union {
    struct {
        uint8_t spi3w_en : 1; // enable SPI mode
        uint8_t none : 1;     // unused - don't set
        uint8_t filter : 3;   // filter setting
        // 000 = filter off, 001 = 2x, 010 = 4x, 011 = 8x, 100 and above = 16x filter
        uint8_t t_sb : 3; // inactive duration (standby time) in normal mode
        // 000 = 0.5 ms, 001 = 62.5 ms, 010 = 125 ms, 011 = 250 ms, 100 = 500 ms
        // 101 = 1000 ms, 110 = 10 ms, 111 = 20 ms
    } bits;
    uint8_t u;
} configReg; //!< config register object

static void write8(uint16_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    ERROR_CHECK(i2c_master_write_to_device(I2C_MASTER_NUM, BME280_SENSOR_ADDR, buf, sizeof(buf),
                                           pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS)));
}

static uint8_t read8(uint16_t reg) {
    uint8_t buf[1] = {reg};
    ERROR_CHECK(i2c_master_write_to_device(I2C_MASTER_NUM, BME280_SENSOR_ADDR, buf, sizeof(buf),
                                           pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS)));
    ERROR_CHECK(i2c_master_read_from_device(I2C_MASTER_NUM, BME280_SENSOR_ADDR, buf, sizeof(buf),
                                            pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS)));
    return buf[0];
}

static int8_t readS8(uint16_t reg) { return (int8_t)read8(reg); }

static uint16_t read16(uint16_t reg) {
    uint16_t buf = reg;
    ERROR_CHECK(i2c_master_write_to_device(I2C_MASTER_NUM, BME280_SENSOR_ADDR, (uint8_t*)&buf, 1,
                                           pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS)));
    ERROR_CHECK(i2c_master_read_from_device(I2C_MASTER_NUM, BME280_SENSOR_ADDR, (uint8_t*)&buf,
                                            sizeof(buf), pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS)));
    return __builtin_bswap16(buf);
}

static int16_t readS16(uint16_t reg) { return (int16_t)read16(reg); }

static uint32_t read24(uint16_t reg) {
    uint32_t buf = reg;
    ERROR_CHECK(i2c_master_write_to_device(I2C_MASTER_NUM, BME280_SENSOR_ADDR, (uint8_t*)&buf, 1,
                                           pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS)));
    *((uint8_t*)buf) = 0;
    ERROR_CHECK(i2c_master_read_from_device(I2C_MASTER_NUM, BME280_SENSOR_ADDR,
                                            ((uint8_t*)&buf) + 1, 3,
                                            pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS)));
    return __builtin_bswap32(buf);
}

static bool isReadingCalibration(void) {
    return read8(BME280_REG_STATUS) & BME280_REG_STATUS_IM_UPDATE_MASK;
}

static void readCoefficients(void) {
    T1 = read16(BME280_REG_T1);
    T2 = readS16(BME280_REG_T2);
    T3 = readS16(BME280_REG_T3);
    P1 = read16(BME280_REG_P1);
    P2 = readS16(BME280_REG_P2);
    P3 = readS16(BME280_REG_P3);
    P4 = readS16(BME280_REG_P4);
    P5 = readS16(BME280_REG_P5);
    P6 = readS16(BME280_REG_P6);
    P7 = readS16(BME280_REG_P7);
    P8 = readS16(BME280_REG_P8);
    P9 = readS16(BME280_REG_P9);
    H1 = read8(BME280_REG_H1);
    H2 = readS16(BME280_REG_H2);
    H3 = read8(BME280_REG_H3);
    H4 = (readS8(BME280_REG_H4) << 4) | (read8(BME280_REG_H4 + 1) & 0xF);
    H5 = (readS8(BME280_REG_H5 + 1) << 4) | (read8(BME280_REG_H5) >> 4);
    H6 = readS8(BME280_REG_H6);
}

static void startMeasurement(int mode, int tempSampling, int pressSampling, int humSampling,
                             int filter, int duration) {
    ctrlMeasReg.bits.mode = mode;
    ctrlMeasReg.bits.osrs_t = tempSampling;
    ctrlMeasReg.bits.osrs_p = pressSampling;

    ctrlHumReg.u = 0;
    ctrlHumReg.bits.osrs_h = humSampling;

    configReg.u = 0;
    configReg.bits.filter = filter;
    configReg.bits.t_sb = duration;

    // making sure sensor is in sleep mode before setting configuration
    // as it otherwise may be ignored
    write8(BME280_REG_CONTROL, BME280_REG_CONTROL_MODE_SLEEP);

    // you must make sure to also set REGISTER_CONTROL after setting the
    // CONTROLHUMID register, otherwise the values won't be applied (see
    // DS 5.4.3)
    write8(BME280_REG_CONTROLHUMID, ctrlHumReg.u);
    write8(BME280_REG_CONFIG, configReg.u);

    write8(BME280_REG_CONTROL, ctrlMeasReg.u);
}

static int32_t t_fine;

// Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.
// t_fine carries fine temperature as global value
// Must read temp before others to set t_fine
static int32_t Temperature(void) {
    int32_t var1, var2, T, adc_T = read24(BME280_REG_TEMPDATA);
    if (adc_T == 0x800000) // value in case temp measurement was disabled
        return INT_MAX;
    var1 = ((((adc_T >> 3) - ((int32_t)T1 << 1))) * ((int32_t)T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)T1)) * ((adc_T >> 4) - ((int32_t)T1))) >> 12) *
            ((int32_t)T3)) >>
           14;
    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;
    return T;
}

// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8
// fractional bits). Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
static uint32_t Pressure(void) {
    int64_t var1, var2, p;
    int32_t adc_P = read24(BME280_REG_PRESSUREDATA);
    if (adc_P == 0x800000) // value in case pressure measurement was disabled
        return INT_MAX;
    var1 = t_fine - 128000;
    var2 = var1 * var1 * P6;
    var2 = var2 + ((var1 * P5) << 17);
    var2 = var2 + ((int64_t)P4 << 35);
    var1 = ((var1 * var1 * P3) >> 8) + ((var1 * P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * P1 >> 33;
    if (var1 == 0)
        return INT_MAX; // avoid exception caused by division by zero
    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (P9 * (p >> 13) * (p >> 13)) >> 25;
    var2 = (P8 * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)P7) << 4);
    return p;
}

// Returns humidity in %RH as unsigned 32 bit integer in Q22.10 format (22 integer and 10 fractional
// bits). Output value of “47445” represents 47445/1024 = 46.333 %RH
static uint32_t Humidity(void) {
    int32_t v_x1_u32r, adc_H = read16(BME280_REG_HUMIDDATA);
    if (adc_H == 0x8000) // value in case humidity measurement was disabled
        return INT_MAX;
    v_x1_u32r = t_fine - 76800;
    v_x1_u32r =
        (((((adc_H << 14) - ((int32_t)H4 << 20) - (H5 * v_x1_u32r)) + 16384) >> 15) *
         (((((((v_x1_u32r * H6) >> 10) * (((v_x1_u32r * H3) >> 11) + 32768)) >> 10) + 2097152) *
               H2 +
           8192) >>
          14));
    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * H1) >> 4));
    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
    return v_x1_u32r >> 12;
}

int getTempHumPress(int32_t* temp, int32_t* hum, int32_t* press) {
    int rc = -1;
    // Power the BME280
    gpio_set_direction(GPIO_NUM_4, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_4, 1);
    // Init I2C
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO, // select GPIO specific to your project
        .sda_pullup_en = GPIO_PULLUP_DISABLE,
        .scl_io_num = I2C_MASTER_SCL_IO, // select GPIO specific to your project
        .scl_pullup_en = GPIO_PULLUP_DISABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ // select frequency specific to your project
    };
    ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, I2C_MODE_MASTER, 0, 0, 0));
    // allow BME280 time to power up
    vTaskDelay(pdMS_TO_TICKS(20));
    // check if sensor, i.e. the chip ID is correct
    uint8_t _sensorID = read8(BME280_REG_CHIPID);
    if (_sensorID != 0x60) {
        ESP_LOGE(TAG, "Unknown sensor %02x", _sensorID);
        goto done;
    }
    // reset the device using soft-reset
    // this makes sure the IIR is off, etc.
    // write8(BME280_REG_RESET, BME280_REG_RESET_VALUE);
    // wait for chip to wake up.
    vTaskDelay(10);
    // if chip is still reading calibration, delay
    while (isReadingCalibration())
        vTaskDelay(pdMS_TO_TICKS(10));
    // get trim and set coefficient values
    readCoefficients(); // read trim parameters, see DS 4.2.2
    // take measurements
    startMeasurement(BME280_REG_CONTROL_MODE_FORCED, SAMPLING_X1, SAMPLING_X1, SAMPLING_X1,
                     FILTER_OFF, STANDBY_MS_0_5);
    uint32_t timeout_start = xTaskGetTickCount();
    while (read8(BME280_REG_STATUS) & BME280_REG_STATUS_MEASURING_MASK) {
        // In case of a timeout, stop the while loop
        if ((xTaskGetTickCount() - timeout_start) > pdMS_TO_TICKS(2 * I2C_MASTER_TIMEOUT_MS)) {
            ESP_LOGE(TAG, "Timeout waiting for measurement");
            goto done;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    // retrieve measurements
    *temp = Temperature();
    *press = Pressure();
    *hum = Humidity();
    rc = 0;
done:
    // kill the I2C pins
    i2c_driver_delete(I2C_MASTER_NUM);
    // power down
    gpio_set_level(GPIO_NUM_4, 0);
    return rc;
}
