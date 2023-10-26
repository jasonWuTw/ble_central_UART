#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrf_log.h"

#define MODE_UP NRF_GPIO_PIN_MAP(0,4)
#define RM_LED1 NRF_GPIO_PIN_MAP(0,6)

void gpio_init() {
    nrf_gpio_cfg_input(MODE_UP, NRF_GPIO_PIN_PULLUP);
    nrf_gpio_cfg_output(RM_LED1);
    nrf_gpio_pin_set(RM_LED1); // Turn the LED off by default
}

int main(void) {
	NRF_LOG_INFO("main...");
    gpio_init();
    while (1) {
        if (!nrf_gpio_pin_read(MODE_UP)) { // Button is active low
            nrf_gpio_pin_clear(RM_LED1); // Turn on LED
            nrf_delay_ms(500); // Delay for 500ms
            nrf_gpio_pin_set(RM_LED1); // Turn off LED
            nrf_delay_ms(500); // Delay for 500ms
			//NRF_LOG_INFO("button click...");
        }
    }
}

