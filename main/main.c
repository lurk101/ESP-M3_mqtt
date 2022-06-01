#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_sleep.h"
#include "esp_system.h"
#include "esp_wifi.h"

#include "driver/gpio.h"
#include "driver/i2c.h"

#include "nvs_flash.h"
#include "protocol_common.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "mqtt_client.h"

static const char *TAG = "MQTT_WEATHER";

#define I2C_MASTER_SCL_IO GPIO_NUM_14 /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO GPIO_NUM_2  /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM 0              /*!< I2C master i2c port number */
#define I2C_MASTER_FREQ_HZ 40000      /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0   /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0   /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS 1000

enum {
    BME280_SENSOR_ADDR = 0x60,
    BME280_REGISTER_DIG_T1 = 0x88,
    BME280_REGISTER_DIG_T2 = 0x8A,
    BME280_REGISTER_DIG_T3 = 0x8C,

    BME280_REGISTER_DIG_P1 = 0x8E,
    BME280_REGISTER_DIG_P2 = 0x90,
    BME280_REGISTER_DIG_P3 = 0x92,
    BME280_REGISTER_DIG_P4 = 0x94,
    BME280_REGISTER_DIG_P5 = 0x96,
    BME280_REGISTER_DIG_P6 = 0x98,
    BME280_REGISTER_DIG_P7 = 0x9A,
    BME280_REGISTER_DIG_P8 = 0x9C,
    BME280_REGISTER_DIG_P9 = 0x9E,

    BME280_REGISTER_DIG_H1 = 0xA1,
    BME280_REGISTER_DIG_H2 = 0xE1,
    BME280_REGISTER_DIG_H3 = 0xE3,
    BME280_REGISTER_DIG_H4 = 0xE4,
    BME280_REGISTER_DIG_H5 = 0xE5,
    BME280_REGISTER_DIG_H6 = 0xE7,

    BME280_REGISTER_CHIPID = 0xD0,
    BME280_REGISTER_VERSION = 0xD1,
    BME280_REGISTER_SOFTRESET = 0xE0,

    BME280_REGISTER_CAL26 = 0xE1, // R calibration stored in 0xE1-0xF0

    BME280_REGISTER_CONTROLHUMID = 0xF2,
    BME280_REGISTER_STATUS = 0XF3,
    BME280_REGISTER_CONTROL = 0xF4,
    BME280_REGISTER_CONFIG = 0xF5,
    BME280_REGISTER_PRESSUREDATA = 0xF7,
    BME280_REGISTER_TEMPDATA = 0xFA,
    BME280_REGISTER_HUMIDDATA = 0xFD
};

static struct {
    uint16_t dig_T1; ///< temperature compensation value
    int16_t dig_T2;  ///< temperature compensation value
    int16_t dig_T3;  ///< temperature compensation value
    uint16_t dig_P1; ///< pressure compensation value
    int16_t dig_P2;  ///< pressure compensation value
    int16_t dig_P3;  ///< pressure compensation value
    int16_t dig_P4;  ///< pressure compensation value
    int16_t dig_P5;  ///< pressure compensation value
    int16_t dig_P6;  ///< pressure compensation value
    int16_t dig_P7;  ///< pressure compensation value
    int16_t dig_P8;  ///< pressure compensation value
    int16_t dig_P9;  ///< pressure compensation value
    uint8_t dig_H1;  ///< humidity compensation value
    int16_t dig_H2;  ///< humidity compensation value
    uint8_t dig_H3;  ///< humidity compensation value
    int16_t dig_H4;  ///< humidity compensation value
    int16_t dig_H5;  ///< humidity compensation value
    int8_t dig_H6;   ///< humidity compensation value
} _bme280_calib;

static void log_error_if_nonzero(const char* message, int error_code) {
    if (error_code != 0)
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
}

static void write8(uint16_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    ESP_ERROR_CHECK(i2c_master_write_to_device(I2C_MASTER_NUM, BME280_SENSOR_ADDR, buf, sizeof(buf),
                                               pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS)));
}

static uint8_t read8(uint16_t reg) {
    uint8_t buf[1] = {reg};
    ESP_ERROR_CHECK(i2c_master_write_to_device(I2C_MASTER_NUM, BME280_SENSOR_ADDR, buf, sizeof(buf),
                                               pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS)));
    ESP_ERROR_CHECK(i2c_master_read_from_device(I2C_MASTER_NUM, BME280_SENSOR_ADDR, buf,
                                                sizeof(buf), pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS)));
    return buf[0];
}

static uint32_t read24(uint16_t reg) {
    uint32_t buf = reg;
    ESP_ERROR_CHECK(i2c_master_write_to_device(I2C_MASTER_NUM, BME280_SENSOR_ADDR, (uint8_t*)&buf,
                                               1, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS)));
    ESP_ERROR_CHECK(i2c_master_read_from_device(I2C_MASTER_NUM, BME280_SENSOR_ADDR, (uint8_t*)&buf,
                                                sizeof(buf), pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS)));
    return buf;
}

static uint16_t read16(uint16_t reg) {
    uint16_t buf = reg;
    ESP_ERROR_CHECK(i2c_master_write_to_device(I2C_MASTER_NUM, BME280_SENSOR_ADDR, (uint8_t*)&buf,
                                               1, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS)));
    ESP_ERROR_CHECK(i2c_master_read_from_device(I2C_MASTER_NUM, BME280_SENSOR_ADDR, (uint8_t*)&buf,
                                                sizeof(buf), pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS)));
    return buf;
}

static uint16_t read16_LE(uint16_t reg) { return __builtin_bswap16(read16(reg)); }

static int16_t readS16_LE(uint16_t reg) { return (int16_t)read16_LE(reg); }

static bool isReadingCalibration(void) {
    uint8_t rStatus = read8(BME280_REGISTER_STATUS);
    return (rStatus & (1 << 0)) != 0;
}

static void readCoefficients(void) {
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
    _bme280_calib.dig_H4 =
        ((int8_t)read8(BME280_REGISTER_DIG_H4) << 4) | (read8(BME280_REGISTER_DIG_H4 + 1) & 0xF);
    _bme280_calib.dig_H5 =
        ((int8_t)read8(BME280_REGISTER_DIG_H5 + 1) << 4) | (read8(BME280_REGISTER_DIG_H5) >> 4);
    _bme280_calib.dig_H6 = (int8_t)read8(BME280_REGISTER_DIG_H6);
}

enum sensor_mode { MODE_SLEEP = 0b00, MODE_FORCED = 0b01, MODE_NORMAL = 0b11 };

enum sensor_sampling {
    SAMPLING_NONE = 0b000,
    SAMPLING_X1 = 0b001,
    SAMPLING_X2 = 0b010,
    SAMPLING_X4 = 0b011,
    SAMPLING_X8 = 0b100,
    SAMPLING_X16 = 0b101
};

enum sensor_filter {
    FILTER_OFF = 0b000,
    FILTER_X2 = 0b001,
    FILTER_X4 = 0b010,
    FILTER_X8 = 0b011,
    FILTER_X16 = 0b100
};

enum standby_duration {
    STANDBY_MS_0_5 = 0b000,
    STANDBY_MS_10 = 0b110,
    STANDBY_MS_20 = 0b111,
    STANDBY_MS_62_5 = 0b001,
    STANDBY_MS_125 = 0b010,
    STANDBY_MS_250 = 0b011,
    STANDBY_MS_500 = 0b100,
    STANDBY_MS_1000 = 0b101
};

struct ctrl_meas {
    // temperature oversampling
    // 000 = skipped
    // 001 = x1
    // 010 = x2
    // 011 = x4
    // 100 = x8
    // 101 and above = x16
    unsigned int osrs_t : 3; ///< temperature oversampling

    // pressure oversampling
    // 000 = skipped
    // 001 = x1
    // 010 = x2
    // 011 = x4
    // 100 = x8
    // 101 and above = x16
    unsigned int osrs_p : 3; ///< pressure oversampling

    // device mode
    // 00       = sleep
    // 01 or 10 = forced
    // 11       = normal
    unsigned int mode : 2; ///< device mode
} _measReg;                //!< measurement register object

struct {
    /// unused - don't set
    unsigned int none : 5;

    // pressure oversampling
    // 000 = skipped
    // 001 = x1
    // 010 = x2
    // 011 = x4
    // 100 = x8
    // 101 and above = x16
    unsigned int osrs_h : 3; ///< pressure oversampling
} _humReg;                   //!< hum register object

struct {
    // inactive duration (standby time) in normal mode
    // 000 = 0.5 ms
    // 001 = 62.5 ms
    // 010 = 125 ms
    // 011 = 250 ms
    // 100 = 500 ms
    // 101 = 1000 ms
    // 110 = 10 ms
    // 111 = 20 ms
    unsigned int t_sb : 3; ///< inactive duration (standby time) in normal mode

    // filter settings
    // 000 = filter off
    // 001 = 2x filter
    // 010 = 4x filter
    // 011 = 8x filter
    // 100 and above = 16x filter
    unsigned int filter : 3; ///< filter settings

    // unused - don't set
    unsigned int none : 1;     ///< unused - don't set
    unsigned int spi3w_en : 1; ///< unused - don't set

    /// @return combined config register
} _configReg; //!< config register object

static uint8_t get_configReg(void) {
    return (_configReg.t_sb << 5) | (_configReg.filter << 2) | _configReg.spi3w_en;
}

static uint8_t get_humReg(void) { return (_humReg.osrs_h); }

static uint8_t get_measReg(void) {
    return (_measReg.osrs_t << 5) | (_measReg.osrs_p << 2) | _measReg.mode;
}

static void setSampling(enum sensor_mode mode, enum sensor_sampling tempSampling,
                        enum sensor_sampling pressSampling, enum sensor_sampling humSampling,
                        enum sensor_filter filter, enum standby_duration duration) {
    _measReg.mode = mode;
    _measReg.osrs_t = tempSampling;
    _measReg.osrs_p = pressSampling;

    _humReg.osrs_h = humSampling;
    _configReg.filter = filter;
    _configReg.t_sb = duration;
    _configReg.spi3w_en = 0;

    // making sure sensor is in sleep mode before setting configuration
    // as it otherwise may be ignored
    write8(BME280_REGISTER_CONTROL, MODE_SLEEP);

    // you must make sure to also set REGISTER_CONTROL after setting the
    // CONTROLHUMID register, otherwise the values won't be applied (see
    // DS 5.4.3)
    write8(BME280_REGISTER_CONTROLHUMID, get_humReg());
    write8(BME280_REGISTER_CONFIG, get_configReg());
    write8(BME280_REGISTER_CONTROL, get_measReg());
}

bool BME280_init() {
    // check if sensor, i.e. the chip ID is correct
    uint8_t _sensorID = read8(BME280_REGISTER_CHIPID);
    if (_sensorID != 0x60)
        return false;
    // reset the device using soft-reset
    // this makes sure the IIR is off, etc.
    write8(BME280_REGISTER_SOFTRESET, 0xB6);
    // wait for chip to wake up.
    vTaskDelay(1);
    // if chip is still reading calibration, delay
    while (isReadingCalibration())
        vTaskDelay(pdMS_TO_TICKS(10));
    readCoefficients(); // read trimming parameters, see DS 4.2.2
    setSampling(MODE_FORCED, SAMPLING_X16, SAMPLING_X16, SAMPLING_X16, FILTER_OFF, STANDBY_MS_0_5);
    vTaskDelay(pdMS_TO_TICKS(10));
    return true;
}

static int32_t t_fine;
static int32_t t_fine_adjust = 0;

static int32_t readTemperature(void) {
    int32_t var1, var2;
    int32_t adc_T = read24(BME280_REGISTER_TEMPDATA);
    if (adc_T == 0x800000) // value in case temp measurement was disabled
        return INT_MAX;
    adc_T >>= 4;
    var1 = (adc_T / 8) - ((int32_t)_bme280_calib.dig_T1 * 2);
    var1 = (var1 * ((int32_t)_bme280_calib.dig_T2)) / 2048;
    var2 = (adc_T / 16) - ((int32_t)_bme280_calib.dig_T1);
    var2 = (((var2 * var2) / 4096) * ((int32_t)_bme280_calib.dig_T3)) / 16384;
    t_fine = var1 + var2 + t_fine_adjust;
    int32_t T = (t_fine * 5 + 128) / 256;
    return T;
}

static int32_t readHumidity(void) {
    int32_t var1, var2, var3, var4, var5;
    int32_t adc_H = read16(BME280_REGISTER_HUMIDDATA);
    if (adc_H == 0x8000) // value in case humidity measurement was disabled
        return INT_MAX;
    var1 = t_fine - 76800;
    var2 = adc_H * 16384;
    var3 = ((int32_t)_bme280_calib.dig_H4) * 1048576;
    var4 = ((int32_t)_bme280_calib.dig_H5) * var1;
    var5 = (((var2 - var3) - var4) + (int32_t)16384) / 32768;
    var2 = (var1 * ((int32_t)_bme280_calib.dig_H6)) / 1024;
    var3 = (var1 * ((int32_t)_bme280_calib.dig_H3)) / 2048;
    var4 = ((var2 * (var3 + 32768)) / 1024) + 2097152;
    var2 = ((var4 * ((int32_t)_bme280_calib.dig_H2)) + 8192) / 16384;
    var3 = var5 * var2;
    var4 = ((var3 / 32768) * (var3 / 32768)) / 128;
    var5 = var3 - ((var4 * ((int32_t)_bme280_calib.dig_H1)) / 16);
    var5 = var5 < 0 ? 0 : var5;
    var5 = var5 > 419430400 ? 419430400 : var5;
    int32_t H = var5 / 4096;
    return H / 1024;
}

static int32_t readPressure(void) {
    int64_t var1, var2, var3, var4;
    int32_t adc_P = read24(BME280_REGISTER_PRESSUREDATA);
    if (adc_P == 0x800000) // value in case pressure measurement was disabled
        return INT_MAX;
    adc_P >>= 4;
    var1 = t_fine - 128000;
    var2 = var1 * var1 * _bme280_calib.dig_P6;
    var2 = var2 + var1 * _bme280_calib.dig_P5 * 131072;
    var2 = var2 + _bme280_calib.dig_P4 * 34359738368;
    var1 = (var1 * var1 * _bme280_calib.dig_P3 / 256) + (var1 * _bme280_calib.dig_P2 * 4096);
    var3 = 140737488355328;
    var1 = (var3 + var1) * _bme280_calib.dig_P1 / 8589934592;
    if (var1 == 0)
        return INT_MAX; // avoid exception caused by division by zero
    var4 = 1048576 - adc_P;
    var4 = (var4 * 2147483648 - var2) * 3125 / var1;
    var1 = (_bme280_calib.dig_P9 * (var4 / 8192) * (var4 / 8192)) / 33554432;
    var2 = _bme280_calib.dig_P8 * var4 / 524288;
    var4 = ((var4 + var1 + var2) / 256) + ((int64_t)_bme280_calib.dig_P7 * 16);
    return var4 / 256;
}

static void mqtt_event_handler(void* handler_args, esp_event_base_t base, int32_t event_id,
                               void* event_data) {
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        char tph[64];
        sprintf(tph, "%d,%d,%d", readTemperature(), readPressure(), readHumidity());
        msg_id =
            esp_mqtt_client_publish(client, "/weather/tph", (const char*)tph, strlen(tph), 1, 0);
        ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        break;
    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        gpio_set_level(GPIO_NUM_4, 0);
        esp_mqtt_client_disconnect(client);
        esp_wifi_disconnect();
        esp_sleep_enable_timer_wakeup(15 * 60 * 100);
        esp_deep_sleep_start();
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        printf("DATA=%.*s\r\n", event->data_len, event->data);
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
            log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
            log_error_if_nonzero("captured as transport's socket errno",  event->error_handle->esp_transport_sock_errno);
            ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));
        }
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}

static void mqtt_app_start(void) {
    esp_mqtt_client_config_t mqtt_cfg = {
        .uri = CONFIG_BROKER_URL,
    };
    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
}

static void app_i2c_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO, // select GPIO specific to your project
        .sda_pullup_en = GPIO_PULLUP_DISABLE,
        .scl_io_num = I2C_MASTER_SCL_IO, // select GPIO specific to your project
        .scl_pullup_en = GPIO_PULLUP_DISABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ // select frequency specific to your project
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, I2C_MODE_MASTER, 0, 0, 0));
}

void app_main(void) {
    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());
    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("MQTT_CLIENT", ESP_LOG_VERBOSE);
    esp_log_level_set("MQTT_EXAMPLE", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_BASE", ESP_LOG_VERBOSE);
    esp_log_level_set("esp-tls", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT", ESP_LOG_VERBOSE);
    esp_log_level_set("OUTBOX", ESP_LOG_VERBOSE);
    gpio_set_direction(GPIO_NUM_4, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_4, 1);
    app_i2c_init();
    BME280_init();
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(app_connect());
    mqtt_app_start();
}
