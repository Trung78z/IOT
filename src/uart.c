
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "driver/uart.h"
#include <string.h>
#define led_pin GPIO_NUM_2
#define button GPIO_NUM_0
#define ESP_INR_FLAG_DEFAULT 0
#define BUF_SIZE 1024
#define EX_UART_NUM UART_NUM_2
TaskHandle_t uart_task11 = NULL;
TaskHandle_t uart_task12 = NULL;

void uart_init()
{
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_driver_install(EX_UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_set_pin(EX_UART_NUM, 17, 16, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_param_config(EX_UART_NUM, &uart_config);
}
void uart_send(const char *data)
{
    uart_write_bytes(EX_UART_NUM, data, strlen(data));
}
void uart_task(void *pvParameters)
{
    uart_init();

    while (1)
    {
        // Gửi dữ liệu qua UART
        uart_send("ON");

        vTaskDelay(1000 / portTICK_PERIOD_MS);
        uart_send("OFF");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
void uart_receive(void *pvParameters)
{
    uart_init();
    while (1)
    {
        esp_rom_gpio_pad_select_gpio(led_pin);
        esp_rom_gpio_pad_select_gpio(button);
        gpio_set_direction(led_pin, GPIO_MODE_OUTPUT);
        gpio_set_direction(button, GPIO_MODE_INPUT);
        uint8_t receivedData[12]; // Định nghĩa một mảng để lưu dữ liệu nhận được
        int bytesRead = uart_read_bytes(EX_UART_NUM, receivedData, sizeof(receivedData) - 1, 10);

        if (bytesRead > 0)
        {
            receivedData[bytesRead] = '\0'; // Kết thúc mảng nhận được bằng ký tự null
            if (strcmp((char *)receivedData, "ON") == 0)
            {
                printf("Received: %s\n", receivedData);

                gpio_set_level(led_pin, 0);
            }
            else if (strcmp((char *)receivedData, "OFF") == 0)
            {
                printf("Received: %s\n", receivedData);

                gpio_set_level(led_pin, 1);
            }
        }
    }
}
TaskHandle_t ISR = NULL;
void IRAM_ATTR button_isr_handle(void *arg)
{
    xTaskResumeFromISR(ISR);
}
void buttontask(void *arg)
{
    bool led_status = false;
    while (1)
    {

        vTaskSuspend(NULL);
        led_status = !led_status;
        gpio_set_level(led_pin, led_status);
        printf("Button pressed!\n");
    }
}
void app_main(void)
{

    gpio_set_intr_type(button, GPIO_INTR_NEGEDGE);
    gpio_install_isr_service(ESP_INR_FLAG_DEFAULT);
    gpio_isr_handler_add(button, button_isr_handle, NULL);
    xTaskCreate(buttontask, "buttontask", 4096, NULL, 10, &ISR);
    xTaskCreate(uart_receive, "uart_receive", 4096, NULL, 10, &uart_task12);
    xTaskCreate(uart_task, "uart_task", 4096, NULL, 10, &uart_task11);
}
