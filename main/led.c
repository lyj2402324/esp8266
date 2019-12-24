

#ifdef ECLIPSE
#include "build/include/sdkconfig.h"
#endif

#include "led.h"

#include "esp_log.h"
#include "driver/gpio.h"

#define TAG "led"

#define GPIO_LED    GPIO_NUM_2
#define GPIO_OUTPUT_PIN_SEL    1ULL<<GPIO_LED

static int led = 0;

void init_led()
{
	gpio_config_t io_conf = {
			.intr_type = GPIO_INTR_DISABLE,
			.mode = GPIO_MODE_OUTPUT,
			.pin_bit_mask = GPIO_OUTPUT_PIN_SEL,
			.pull_down_en = 0,
			.pull_up_en = 0
	};

	 gpio_config(&io_conf);
}

void toggle_led()
{
	led = (led+1) % 2;
	gpio_set_level(GPIO_LED,led);

	ESP_LOGI(TAG,"led=%d\n",led );
}
