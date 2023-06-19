/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "led_strip.h"
#include "sdkconfig.h"
#define LED_PIN GPIO_NUM_2
#define BUTTON_PIN GPIO_NUM_4
extern "C"
{
    void app_main(void)
    {
        esp_rom_gpio_pad_select_gpio(LED_PIN);
        gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);

        esp_rom_gpio_pad_select_gpio(BUTTON_PIN);
        gpio_set_direction(BUTTON_PIN, GPIO_MODE_INPUT);
        gpio_set_pull_mode(BUTTON_PIN, GPIO_PULLUP_ONLY);

        int led_state = 0;
        int button_state = 0;
        int last_button_state = 0;

        while (1)
        {
            button_state = gpio_get_level(BUTTON_PIN);

            if (button_state != last_button_state)
            {
                if (button_state == 0)
                {
                    led_state = !led_state; // Chuyển đổi trạng thái LED

                    if (led_state)
                    {
                        printf("LED is ON\n");
                        gpio_set_level(LED_PIN, 1); // Bật LED
                    }
                    else
                    {
                        printf("LED is OFF\n");
                        gpio_set_level(LED_PIN, 0); // Tắt LED
                    }
                }

                last_button_state = button_state;
            }

            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
    }
}