#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nvs.h"

#include "HD44780.h"

#define LCD_ADDR 0x27
#define SDA_PIN  21
#define SCL_PIN  22
#define LCD_COLS 16
#define LCD_ROWS 2

#define REED_GPIO GPIO_NUM_18
#define WHEEL_CIRCUMFERENCE 2.1  // metros
#define LAPS_PER_HUNDRED_METER 47
#define SEC_IN_US 1000000.0
#define TIMEOUT_TO_ZERO 5000000
#define DEBOUNCE_TIME 10000

static TaskHandle_t calc_task_handle = NULL;
static TaskHandle_t print_task_handle = NULL;

static volatile int64_t last_pulse_time = 0;
static volatile float current_speed_kmh = 0;

static volatile uint32_t lap_count = 0;
static uint32_t unsaved_laps = 0;

void save_lap_count_to_nvs(uint32_t count);

static void IRAM_ATTR reed_isr_handler(void* arg) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(calc_task_handle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}


void inc_and_save_lap_count() {
    lap_count++;
    unsaved_laps++;

    if (unsaved_laps >= LAPS_PER_HUNDRED_METER) {
        save_lap_count_to_nvs(lap_count);
        unsaved_laps = 0;
    }
}

void save_lap_count_to_nvs(uint32_t count) {
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err == ESP_OK) {
        nvs_set_u32(my_handle, "lap_count", count);
        nvs_commit(my_handle);
        nvs_close(my_handle);
    }
}

uint32_t load_lap_count_from_nvs() {
    nvs_handle_t my_handle;
    uint32_t count = 0;
    esp_err_t err = nvs_open("storage", NVS_READONLY, &my_handle);
    if (err == ESP_OK) {
        nvs_get_u32(my_handle, "lap_count", &count);
        nvs_close(my_handle);
    }
    return count;
}

void calc_speed_task(void *arg) {
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        int64_t now = esp_timer_get_time();
        int64_t delta_us = now - last_pulse_time;

        if (delta_us > DEBOUNCE_TIME) { // Debounce 10ms
            float delta_sec = delta_us / SEC_IN_US;
            current_speed_kmh = (WHEEL_CIRCUMFERENCE / delta_sec) * 3.6;
            last_pulse_time = now;
            inc_and_save_lap_count();

            xTaskNotifyGive(print_task_handle);
        }
    }
}

void print_speed_task(void *arg) {
    char speed_str[16] = "";
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        printf("Current speed: %.2f km/h\n", current_speed_kmh);
        sprintf(speed_str, "V:%.2f km/h", current_speed_kmh);
        LCD_home();
        LCD_writeStr(speed_str);
        LCD_setCursor(0, 1);
        sprintf(speed_str, "D: %.2f km", (lap_count*(WHEEL_CIRCUMFERENCE/1000)));
        LCD_writeStr(speed_str);
    }
}

void reed_switch_init() {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << REED_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_POSEDGE
    };
    gpio_config(&io_conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(REED_GPIO, reed_isr_handler, NULL);
}

void app_main() {
    last_pulse_time = esp_timer_get_time();
    LCD_init(LCD_ADDR, SDA_PIN, SCL_PIN, LCD_COLS, LCD_ROWS);

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    lap_count = load_lap_count_from_nvs();

    xTaskCreate(calc_speed_task, "CalcSpeedTask", 2048, NULL, 10, &calc_task_handle);
    xTaskCreate(print_speed_task, "PrintSpeedTask", 2048, NULL, 5, &print_task_handle);

    reed_switch_init();
    xTaskNotifyGive(print_task_handle);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(500));
        if (esp_timer_get_time() - last_pulse_time > TIMEOUT_TO_ZERO) {
            if (current_speed_kmh != 0) {
                current_speed_kmh = 0;
                xTaskNotifyGive(print_task_handle);
            }
        }
    }
}
