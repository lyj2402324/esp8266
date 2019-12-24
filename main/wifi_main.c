/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#ifdef ECLIPSE
#include "build/include/sdkconfig.h"
#endif

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"

#include "driver/uart.h"
#include "esp_log.h"
#include "driver/gpio.h"

#include "driver/uart.h"

#define TAG "dwin"

#include "led.h"

#define GPIO_LED    GPIO_NUM_2
#define GPIO_OUTPUT_PIN_SEL    1ULL<<GPIO_LED
#define GPIO_BUTTON    GPIO_NUM_0
#define BUF_SIZE  256

static xQueueHandle gpio_evt_queue = NULL;

static int button = 0;



void init_uart()
{
    uart_config_t uart_config = {
        .baud_rate = 921600,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM_0, &uart_config);
    uart_driver_install(UART_NUM_0, BUF_SIZE * 2, 0, 0, NULL, 0);
}


static void gpio_isr_handler(void *arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void gpio_evt_task(void *arg)
{
    uint32_t  io_num;

    for (;;) {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            ESP_LOGI(TAG, "GPIO[%d] intr, val: %d\n", io_num, gpio_get_level(io_num));

            if(GPIO_BUTTON == io_num) {
                button = (button+1) % 4;
                toggle_led();
                ESP_LOGI(TAG, "button value is %d", button);
            }
        }
    }
}

void init_button()
{
    gpio_set_intr_type(GPIO_BUTTON,GPIO_INTR_NEGEDGE);
    gpio_set_direction(GPIO_BUTTON,GPIO_MODE_INPUT);
    gpio_set_pull_mode(GPIO_BUTTON,GPIO_PULLUP_ONLY);

    gpio_pulldown_dis(GPIO_BUTTON);

    /*gpio_config_t io_conf;

    io_conf.intr_type = GPIO_INTR_POSEDGE;
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = 1ULL<<GPIO_BUTTON;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);*/

    gpio_evt_queue = xQueueCreate(10,sizeof(uint32_t));

    xTaskCreate(gpio_evt_task, "gpio_evt_task", 2048, NULL, 10, NULL);

    gpio_install_isr_service(0);

    gpio_isr_handler_add(GPIO_BUTTON, gpio_isr_handler, (void *) GPIO_BUTTON );
}

void app_main()
{
    init_uart();

    init_led();

    init_button();



    ESP_LOGI(TAG,"Hello dwin!\n");

    /* Print chip information */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    ESP_LOGI(TAG,"This is ESP8266 chip with %d CPU cores, WiFi, ",
            chip_info.cores);

    ESP_LOGI(TAG,"silicon revision %d, ", chip_info.revision);

    ESP_LOGI(TAG,"%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
            (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);
    while (1) {
        int len = uart_read_bytes(UART_NUM_0, data, BUF_SIZE, 20 / portTICK_RATE_MS);

        if(len > 0) {

            for(int i=0;i<10;i++) {
                len++;
                data[len] = 0x31+i;
            }

            uart_write_bytes(UART_NUM_0, (const char *) data, len);
        }
    }
    ESP_LOGI(TAG,"Restarting now.\n");
    fflush(stdout);
    esp_restart();
}
