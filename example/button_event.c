/** GitHub例子,使用裡面的event
 * This example demonstrates a way to enter system-off by pressing BUTTON_1 on the nRF5x-DK, 
 * and how to wake up again from system off pressing BUTTON_2. LED_1 on nRF5x-DK boards is 
 * lid when nRF5x is in System-On idle mode. LED_1 is turned off when nRF5x enters 
 * System-off mode.
 */

#include <stdbool.h>
#include "nrf.h"
#include "nrf_drv_gpiote.h"
#include "app_error.h"
#include "boards.h"

#define MODE_UP   4
//#define PIN_IN_WAKE_UP      5
#define RM_LED1             6

/**
 * @brief Interrupt handler for wakeup pins
 */
void in_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
		//ret_code_t err_code;
	
		if(pin == MODE_UP)
		{
            nrf_gpio_pin_toggle(RM_LED1);
		}
}

/**
 * @brief Function for configuring: MODE_UP pin (BUTTON_1) to enter System-off, 
 * PIN_IN_WAKE_UP pin (BUTTON_2) to wake up from System-off, and RM_LED1 pin for LED_1 output
 */
static void gpio_init(void)
{
    ret_code_t err_code;

    //Initialize gpiote module
    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);
    
	//Initialize output pin
    nrf_drv_gpiote_out_config_t out_config = GPIOTE_CONFIG_OUT_SIMPLE(false);        //Configure output button
    err_code = nrf_drv_gpiote_out_init(RM_LED1, &out_config);                        //Initialize output button
    APP_ERROR_CHECK(err_code);                                                       //Check potential error
    nrf_drv_gpiote_out_clear(RM_LED1);                                               //Turn on LED to indicate that nRF5x is not in System-off mode

		//Configure sense input pin to enable wakeup and interrupt on button press.
    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_HITOLO(false);     //Configure to generate interrupt and wakeup on pin signal low. "false" means that gpiote will use the PORT event, which is low power, i.e. does not add any noticable current consumption (<<1uA). Setting this to "true" will make the gpiote module use GPIOTE->IN events which add ~8uA for nRF52 and ~1mA for nRF51.
    in_config.pull = NRF_GPIO_PIN_PULLUP;                                            //Configure pullup for input pin to prevent it from floting. Pin is pulled down when button is pressed on nRF5x-DK boards, see figure two in http://infocenter.nordicsemi.com/topic/com.nordic.infocenter.nrf52/dita/nrf52/development/dev_kit_v1.1.0/hw_btns_leds.html?cp=2_0_0_1_4		
    err_code = nrf_drv_gpiote_in_init(MODE_UP, &in_config, in_pin_handler);   //Initialize the pin with interrupt handler in_pin_handler
    APP_ERROR_CHECK(err_code);                                                          //Check potential error
    nrf_drv_gpiote_in_event_enable(MODE_UP, true);                            //Enable event and interrupt for the wakeup pin
}

/**
 * @brief Main function
 */
int main(void)
{
    gpio_init();

    while (true)
    {
				//Enter System-on idle mode
				__WFE();
				__SEV();
				__WFE();			
    }
}
