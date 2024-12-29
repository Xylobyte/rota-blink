#include <esp_task_wdt.h>
#include <esp_timer.h>
#include <string>
#include <unordered_map>
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
constexpr uint8_t ledsCount = std::size(leds);
constexpr uint16_t steps = 110;
constexpr uint16_t charSteps = 10;

constexpr uint8_t char_0[charSteps][ledsCount] = {
    {0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 1, 1, 1, 1, 1, 1, 1, 0},
    {0, 1, 0, 0, 0, 0, 0, 1, 0},
    {0, 1, 0, 0, 0, 0, 0, 1, 0},
    {0, 1, 0, 0, 0, 0, 0, 1, 0},
    {0, 1, 0, 0, 0, 0, 0, 1, 0},
    {0, 1, 0, 0, 0, 0, 0, 1, 0},
    {0, 1, 1, 1, 1, 1, 1, 1, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0},
};

constexpr uint8_t char_1[charSteps][ledsCount] = {
    {0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 1, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 1, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 1, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 1, 1, 1, 1, 1, 1, 1, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0},
};

constexpr uint8_t char_2[charSteps][ledsCount] = {
    {0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 1, 1, 0, 0, 0, 1, 0, 0},
    {0, 1, 0, 1, 0, 0, 0, 1, 0},
    {0, 1, 0, 0, 1, 0, 0, 1, 0},
    {0, 1, 0, 0, 1, 0, 0, 1, 0},
    {0, 1, 0, 0, 1, 0, 0, 1, 0},
    {0, 1, 0, 0, 1, 0, 0, 1, 0},
    {0, 1, 0, 0, 0, 1, 1, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0},
};

constexpr uint8_t char_3[charSteps][ledsCount] = {
    {0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 1, 0, 0, 1, 0, 0, 1, 0},
    {0, 1, 0, 0, 1, 0, 0, 1, 0},
    {0, 1, 0, 0, 1, 0, 0, 1, 0},
    {0, 1, 0, 0, 1, 0, 0, 1, 0},
    {0, 1, 0, 0, 1, 0, 0, 1, 0},
    {0, 1, 0, 0, 1, 0, 0, 1, 0},
    {0, 1, 1, 1, 1, 1, 1, 1, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0},
};

constexpr uint8_t char_4[charSteps][ledsCount] = {
    {0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 1, 1, 0, 0, 0, 0, 0},
    {0, 0, 1, 0, 1, 0, 0, 0, 0},
    {0, 0, 1, 0, 0, 1, 0, 0, 0},
    {0, 0, 1, 0, 0, 0, 1, 0, 0},
    {0, 1, 1, 1, 1, 0, 0, 1, 0},
    {0, 0, 1, 0, 0, 0, 0, 0, 0},
    {0, 0, 1, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0},
};

constexpr uint8_t char_5[charSteps][ledsCount] = {
    {0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 1, 0, 0, 1, 1, 1, 1, 0},
    {0, 1, 0, 0, 1, 0, 0, 1, 0},
    {0, 1, 0, 0, 1, 0, 0, 1, 0},
    {0, 1, 0, 0, 1, 0, 0, 1, 0},
    {0, 1, 0, 0, 1, 0, 0, 1, 0},
    {0, 1, 0, 0, 1, 0, 0, 1, 0},
    {0, 0, 1, 1, 0, 0, 0, 1, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0},
};

constexpr uint8_t char_6[charSteps][ledsCount] = {
    {0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 1, 1, 1, 1, 1, 0, 0},
    {0, 1, 0, 0, 1, 0, 0, 1, 0},
    {0, 1, 0, 0, 1, 0, 0, 1, 0},
    {0, 1, 0, 0, 1, 0, 0, 1, 0},
    {0, 1, 0, 0, 1, 0, 0, 1, 0},
    {0, 1, 0, 0, 1, 0, 0, 1, 0},
    {0, 0, 1, 1, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0},
};

constexpr uint8_t char_7[charSteps][ledsCount] = {
    {0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 1, 0},
    {0, 1, 0, 0, 0, 0, 0, 1, 0},
    {0, 0, 1, 0, 0, 0, 0, 1, 0},
    {0, 0, 0, 1, 0, 0, 0, 1, 0},
    {0, 0, 0, 0, 1, 0, 0, 1, 0},
    {0, 0, 0, 0, 0, 1, 0, 1, 0},
    {0, 0, 0, 0, 0, 0, 1, 1, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0},
};

constexpr uint8_t char_8[charSteps][ledsCount] = {
    {0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 1, 1, 0, 1, 1, 0, 0},
    {0, 1, 0, 0, 1, 0, 0, 1, 0},
    {0, 1, 0, 0, 1, 0, 0, 1, 0},
    {0, 1, 0, 0, 1, 0, 0, 1, 0},
    {0, 1, 0, 0, 1, 0, 0, 1, 0},
    {0, 1, 0, 0, 1, 0, 0, 1, 0},
    {0, 0, 1, 1, 0, 1, 1, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0},
};

constexpr uint8_t char_9[charSteps][ledsCount] = {
    {0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 1, 1, 0, 0},
    {0, 1, 0, 0, 1, 0, 0, 1, 0},
    {0, 1, 0, 0, 1, 0, 0, 1, 0},
    {0, 1, 0, 0, 1, 0, 0, 1, 0},
    {0, 1, 0, 0, 1, 0, 0, 1, 0},
    {0, 1, 0, 0, 1, 0, 0, 1, 0},
    {0, 0, 1, 1, 1, 1, 1, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0},
};

constexpr uint8_t char_space[charSteps][ledsCount] = {
    {0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0},
};

std::unordered_map<char16_t, const uint8_t *> charMap = {
    {'0', &char_0[0][0]},
    {'1', &char_1[0][0]},
    {'2', &char_2[0][0]},
    {'3', &char_3[0][0]},
    {'4', &char_4[0][0]},
    {'5', &char_5[0][0]},
    {'6', &char_6[0][0]},
    {'7', &char_7[0][0]},
    {'8', &char_8[0][0]},
    {'9', &char_9[0][0]},
    {' ', &char_space[0][0]},
};

TaskHandle_t adcTaskHandle;

std::string text = "0";
size_t text_length = text.length();

[[noreturn]] void POVTask(void *arg) {
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
    uint64_t lastTime = 0;
    uint64_t currTime = 0;
    uint32_t stepTime = 0;
    uint64_t stepEndTime = 0;
    uint8_t position = 0;

    uint8_t actualTextIndex = 0;
    const uint8_t *actualChar = charMap[text[actualTextIndex]];
    uint8_t drawCharIndex = 0;

    while (true) {
        adc_oneshot_read(handle, ADC_CHANNEL_0, &sensorValue);

        if (sensorValue >= 2500 && !sensorState) {
            sensorState = true;
        } else if (sensorValue < 2500 && sensorState) {
            sensorState = false;
            currTime = esp_timer_get_time();

            const uint64_t elapsedTine = currTime - lastTime;
            if (lastTime > 0) {
                stepTime = elapsedTine / steps;

                const float tps = 1000000.0f / elapsedTine;
                text = std::to_string(static_cast<int>(tps * 60));
                text_length = text.length();
            }

            lastTime = currTime;
            stepEndTime = 0;
            actualTextIndex = 0;
            drawCharIndex = 0;
            position = 0;
        }

        if (position < steps && actualTextIndex < text_length && stepTime > 0 && esp_timer_get_time() > stepEndTime) {
            stepEndTime = esp_timer_get_time() + stepTime;

            for (const auto led: leds) {
                gpio_set_level(led, 0);
            }
            for (uint16_t i = 0; i < 9; ++i) {
                gpio_set_level(leds[i], *(actualChar + drawCharIndex * ledsCount + i));
            }

            if (++drawCharIndex >= charSteps) {
                if (++actualTextIndex >= text_length) {
                    actualChar = charMap[text[0]];
                    continue;
                }

                actualChar = charMap[text[actualTextIndex]];
                drawCharIndex = 0;
            }

            ++position;
        }
    }

    ESP_ERROR_CHECK(adc_oneshot_del_unit(handle));
}

extern "C" void app_main(void) {
    xTaskCreatePinnedToCore(POVTask, "POV Task", 4096, nullptr, 10, &adcTaskHandle, 0);
}
