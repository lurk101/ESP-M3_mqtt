#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_wifi.h"

#include "driver/uart.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "bme280.h"
#include "esp_vfs_dev.h"
#include "mqtt_client.h"
#include "nvs_flash.h"
#include "protocol_common.h"

#define ERROR_CHECK(a) ESP_ERROR_CHECK(a)

static const char* TAG = "WEATHER";

#define SEM_TIMEOUT_MS 2000

// IPC
static StaticSemaphore_t staticSem;
static SemaphoreHandle_t sem;
static int temperature, humidity, pressure;

static void gotoSleep(esp_mqtt_client_handle_t client) {
    esp_mqtt_client_disconnect(client);
    esp_wifi_disconnect();
    esp_sleep_enable_timer_wakeup(15 * 60 * 100);
    esp_deep_sleep_start();
}

static void mqtt_event_handler(void* handler_args, esp_event_base_t base, int32_t event_id,
                               void* event_data) {
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    char txt[128];
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        xSemaphoreTake(sem, pdMS_TO_TICKS(SEM_TIMEOUT_MS));
        sprintf(txt, "{\"temperature\":%d,\"humidity\":%d,\"pressure\":%d}", temperature, humidity,
                pressure);
        ESP_LOGI(TAG, "publishing /weather as %s", txt);
        msg_id = esp_mqtt_client_publish(client, "/weather", (const char*)txt, strlen(txt), 1, 0);
        ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        gotoSleep(client);
        break;
    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        gotoSleep(client);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        printf("DATA=%.*s\r\n", event->data_len, event->data);
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT)
            if (event->error_handle->esp_transport_sock_errno != 0) {
                ESP_LOGE(TAG, "captured as transport's socket errno (%d)",
                         event->error_handle->esp_transport_sock_errno);
                ESP_LOGI(TAG, "errno string (%s)",
                         strerror(event->error_handle->esp_transport_sock_errno));
            }
        gotoSleep(client);
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}

// main thread
void app_main(void) {

    setvbuf(stdin, NULL, _IONBF, 0);
    ESP_ERROR_CHECK(
        uart_driver_install((uart_port_t)CONFIG_ESP_CONSOLE_UART_NUM, 256, 0, 0, NULL, 0));
    esp_vfs_dev_uart_use_driver(CONFIG_ESP_CONSOLE_UART_NUM);
    esp_vfs_dev_uart_port_set_rx_line_endings(CONFIG_ESP_CONSOLE_UART_NUM, ESP_LINE_ENDINGS_CR);
    esp_vfs_dev_uart_port_set_tx_line_endings(CONFIG_ESP_CONSOLE_UART_NUM, ESP_LINE_ENDINGS_CRLF);

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

    ERROR_CHECK(nvs_flash_init());
    ERROR_CHECK(esp_netif_init());
    ERROR_CHECK(esp_event_loop_create_default());
    ERROR_CHECK(app_connect());

    sem = xSemaphoreCreateBinaryStatic(&staticSem);

    // start the mqtt thread
    esp_mqtt_client_config_t mqtt_cfg = {
        .uri = CONFIG_BROKER_URL,
    };
    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    /* The last argument may be used to pass data to the event handler, in this example
     * mqtt_event_handler */
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);

    // main task continues with sensor management
    // retrieve measurements
    if (getTempHumPress(&temperature, &humidity, &pressure) == 0) {
        // mesurement done, release mqtt
        xSemaphoreGive(sem);
    }
    // done
}
