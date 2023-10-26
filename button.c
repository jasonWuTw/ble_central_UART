#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrf_log.h"

#define BUTTON_PIN NRF_GPIO_PIN_MAP(0,4)
#define LED_PIN NRF_GPIO_PIN_MAP(0,6)

void gpio_init() {
    nrf_gpio_cfg_input(BUTTON_PIN, NRF_GPIO_PIN_PULLUP);
    nrf_gpio_cfg_output(LED_PIN);
    nrf_gpio_pin_set(LED_PIN); // Turn the LED off by default
}

int main(void) {
	NRF_LOG_INFO("main...");
    gpio_init();
    while (1) {
        if (!nrf_gpio_pin_read(BUTTON_PIN)) { // Button is active low
            nrf_gpio_pin_clear(LED_PIN); // Turn on LED
            nrf_delay_ms(500); // Delay for 500ms
            nrf_gpio_pin_set(LED_PIN); // Turn off LED
            nrf_delay_ms(500); // Delay for 500ms
			//NRF_LOG_INFO("button click...");
        }
    }
}

