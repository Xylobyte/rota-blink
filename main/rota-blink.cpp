#include <esp_task_wdt.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <driver/gpio.h>
#include <esp_adc/adc_oneshot.h>
#include <freertos/task.h>

#define LED_1 GPIO_NUM_16
#define LED_2 GPIO_NUM_13
#define LED_3 GPIO_NUM_12
#define LED_4 GPIO_NUM_14
#define LED_5 GPIO_NUM_27
#define LED_6 GPIO_NUM_26
#define LED_7 GPIO_NUM_25
#define LED_8 GPIO_NUM_33
#define LED_9 GPIO_NUM_32

constexpr gpio_num_t leds[] = {LED_1, LED_2, LED_3, LED_4, LED_5, LED_6, LED_7, LED_8, LED_9};
constexpr uint16_t steps = 9;

TaskHandle_t adcTaskHandle;

volatile uint64_t lastTime = 0;
volatile uint32_t rotationTime = 0;

void ADCTask(void *arg) {
    adc_oneshot_unit_handle_t handle;
    constexpr adc_oneshot_unit_init_cfg_t init_conf = {
        .unit_id = ADC_UNIT_2,
        .clk_src = ADC_RTC_CLK_SRC_DEFAULT,
        .ulp_mode = ADC_ULP_MODE_DISABLE
    };

    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_conf, &handle));

    constexpr adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12
    };

    ESP_ERROR_CHECK(adc_oneshot_config_channel(handle, ADC_CHANNEL_0, &config));

    gpio_config_t gpio_conf = {
        .pin_bit_mask = 0,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    for (const auto led: leds) {
        gpio_conf.pin_bit_mask |= (1ULL << led);
    }

    ESP_ERROR_CHECK(gpio_config(&gpio_conf));

    int sensorValue = 0;
    bool sensorState = false;
    uint64_t currTime = 0;
    uint32_t delay = 0;
    uint16_t position = 0;

    while (true) {
        adc_oneshot_read(handle, ADC_CHANNEL_0, &sensorValue);

        if (sensorValue >= 2600 && !sensorState) {
            sensorState = true;
            currTime = esp_timer_get_time();

            if (lastTime > 0) {
                rotationTime = currTime - lastTime;
            }

            lastTime = currTime;
            delay = rotationTime / steps;
            position = 0;
            vTaskDelay(1);
        } else if (sensorValue < 2600 && sensorState) {
            sensorState = false;
        }

        if (position < steps && !sensorState && rotationTime > 0 && (rotationTime / 1000) < 400) {
            for (const auto led: leds) {
                gpio_set_level(led, false);
            }
            gpio_set_level(leds[position], true);

            position++;
            esp_rom_delay_us(delay);
        }
    }

    ESP_ERROR_CHECK(adc_oneshot_del_unit(handle));
}

extern "C" void app_main(void) {
    xTaskCreatePinnedToCore(ADCTask, "ADC Task", 4096, nullptr, 10, &adcTaskHandle, 0);
}
