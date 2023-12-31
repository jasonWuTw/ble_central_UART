/**
 * Copyright (c) 2023 - 2021, Matsutek
 *
 * All rights reserved.
 * Handlebar - BLE (perpheral-Device)
 *
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "app_timer.h"
#include "ble_nus.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "bsp_btn_ble.h"
#include "nrf_pwr_mgmt.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrf.h"
#include "nrf_drv_timer.h"
#include "bsp.h"

#include <stdbool.h>
#include "nrf_drv_gpiote.h"
#include "app_error.h"
#include "boards.h"

#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

#define APP_BLE_CONN_CFG_TAG            1                                           /**< A tag identifying the SoftDevice BLE configuration. */

#define DEVICE_NAME                     "Matsutek_E-Bike"                               /**< Name of device. Will be included in the advertising data. */
#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_BLE_OBSERVER_PRIO           3                                           /**< Application's BLE observer priority. You shouldn't need to modify this value. */

#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */

#define APP_ADV_DURATION                18000                                       /**< The advertising duration (180 seconds) in units of 10 milliseconds. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(75, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                       /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */

/** gwell test zone*/
/*
#define BLE_LED 6	// P0.17

#define MODE_LED1 17 // P0.06
#define MODE_LED2 7 // P0.07
#define MODE_LED3 8 // P0.08

#define RM_LED1 22
#define RM_LED2 23
#define RM_LED3 24
*/
/** gwell test zone*/

/** production zone*/

#define MODE_LED1 24
#define MODE_LED2 23
#define MODE_LED3 22
#define BLE_LED 17 // P0.17
#define RM_LED1 6
#define RM_LED2 7
#define RM_LED3 8

/** production zone*/

#define MCU_POWER_HOLD 2	//PO.02
#define POWER_ON 3	//PO.03
#define MODE_UP 4	//PO.04
#define MODE_DOWN 5	//PO.05
#define CELL_V 29	// P0.29
#define MAX_RECEIVED_BLE_ARRAY_SIZE 50

const nrf_drv_timer_t TIMER_4 = NRF_DRV_TIMER_INSTANCE(4);
APP_TIMER_DEF(m_repeated_timer_id_ble_led_blink);
APP_TIMER_DEF(m_repeated_timer_id_low_power_led_blink);
APP_TIMER_DEF(m_repeated_timer_id_query_power);
APP_TIMER_DEF(m_single_shot_timer_id);  /**< Handler for single shot timer used to mode switch. */
APP_TIMER_DEF(m_single_shot_timer_id_ble_connected_query_mode);  /**< Handler for single shot timer used to query mode status after BLE connected. */
APP_TIMER_DEF(m_single_shot_timer_id_power_off);  
APP_TIMER_DEF(m_single_shot_timer_id_power_off_step2);  

/* Define the transmission buffer, which is a buffer to hold the data to be sent over UART */
static uint8_t mode_command_1[] =   {(char)0x67,(char)0x00,(char)0x00,(char)0x01,(char)0x01}; 
static uint8_t mode_command_2[] =   {(char)0x67,(char)0x00,(char)0x00,(char)0x03,(char)0x03}; 
static uint8_t mode_command_3[] =   {(char)0x67,(char)0x00,(char)0x00,(char)0x05,(char)0x05}; 
static uint8_t mode_query[] =  {(char)0x66,(char)0x00,(char)0x00,(char)0x00,(char)0x00}; 
static uint8_t query_response_mode_command_1[] =   {(char)0x66,(char)0x00,(char)0x00,(char)0x01,(char)0x01}; 
static uint8_t query_response_mode_command_2[] =   {(char)0x66,(char)0x00,(char)0x00,(char)0x03,(char)0x03}; 
static uint8_t query_response_mode_command_3[] =   {(char)0x66,(char)0x00,(char)0x00,(char)0x05,(char)0x05}; 

//battery power 電量
static uint8_t power_query[] =  {(char)0x80,(char)0x01,(char)0x1f,(char)0x00,(char)0x1e};  //80 01 1f 00 1e
static uint8_t power_query_response_prefix_four[] =  {(char)0x80,(char)0x01,(char)0x1f,(char)0x00};  //80 01 1f 00 前四碼
static int seconds_query_power=15;	//second
static int seconds_of_inactivity_before_going_to_sleep=3*60;	//second
static int high_power=80;//高電量
static int low_power=30;//低電量
static int blink_power=25;//電量燈閃爍

//static bool usingRX_TX_for_debug = false; // is using RX and TX
static bool usingRX_TX_for_debug = true; // is using RX and TX

//static bool gwell_debug = true;
//static bool gwell_debug = false;

//received_ble_data_array (FIFO)
static uint8_t received_ble_data_array[MAX_RECEIVED_BLE_ARRAY_SIZE];
static int received_ble_data_length = 0;
static int timeout_mode = 0; //timer;
static int timeout_ble_connected = 0; //timer
static int query_mode_before_mode_switch = 50;//million second(ms)
static int query_mode_after_ble_connected = 500;//million second(ms)
static int ble_led_blink_ms = 1000;//million second(ms)
static int ble_led_blink_ms_fast = 100;//million second(ms)
static bool is_GATT_EVT_ATT_MTU_UPDATED_once = false;
static bool isPairing = false;

//mode up / down
static bool left_side_or_right_side = false;
static int mode_status = 0;					//1,2,3,0(0:unknown)
static char mode_button =' '; //' '  'u'  'd'

BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);                                   /**< BLE NUS service instance. */
NRF_BLE_GATT_DEF(m_gatt);                                                           /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                             /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                                 /**< Advertising module instance. */

static uint16_t   m_conn_handle          = BLE_CONN_HANDLE_INVALID;                 /**< Handle of the current connection. */
static uint16_t   m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;            /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
static ble_uuid_t m_adv_uuids[]          =                                          /**< Universally unique service identifier. */
{
    {BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}
};

static void turn_on_BLE_LED(void) {
	nrf_gpio_pin_clear(BLE_LED);
}

static void blink_BLE_LED(void) {
	uint32_t err_code;
	if(isPairing){
		err_code = app_timer_start(m_repeated_timer_id_ble_led_blink, APP_TIMER_TICKS(ble_led_blink_ms), NULL);
	}else{
		err_code = app_timer_start(m_repeated_timer_id_ble_led_blink, APP_TIMER_TICKS(ble_led_blink_ms_fast), NULL);
	}
	APP_ERROR_CHECK(err_code);
}

static void led_blink(void) {
	//LED blink  
	for(int i = 0; i <6; i++) {
		nrf_gpio_pin_toggle(RM_LED1);
		nrf_gpio_pin_toggle(RM_LED2);
		nrf_gpio_pin_toggle(RM_LED3);
		nrf_gpio_pin_toggle(BLE_LED);
		nrf_gpio_pin_toggle(MODE_LED1);
		nrf_gpio_pin_toggle(MODE_LED2);
		nrf_gpio_pin_toggle(MODE_LED3);
		nrf_delay_ms(500);
	}
}

static void led_off_all_mode_led(void) {
    nrf_gpio_pin_set(MODE_LED1); // Turn off LED
    nrf_gpio_pin_set(MODE_LED2); // Turn off LED
    nrf_gpio_pin_set(MODE_LED3); // Turn off LED
}

static void led_off_all_rm_led(void) {
    nrf_gpio_pin_set(RM_LED1); // Turn off LED
    nrf_gpio_pin_set(RM_LED2); // Turn off LED
    nrf_gpio_pin_set(RM_LED3); // Turn off LED
}

static void led_off_all(void) {
    //Turn off all LED
    nrf_gpio_pin_set(BLE_LED); // Turn off LED
    led_off_all_mode_led();
    led_off_all_rm_led();
}

static void switch_mode_led(uint32_t led_pin){
	led_off_all_mode_led();
	nrf_gpio_pin_toggle(led_pin);
}

static void power_led(uint32_t percentage){
	led_off_all_rm_led();
    uint32_t err_code;
    err_code = app_timer_stop(m_repeated_timer_id_low_power_led_blink);
    APP_ERROR_CHECK(err_code);
    if(percentage>=high_power){
        nrf_gpio_pin_toggle(RM_LED1);
        nrf_gpio_pin_toggle(RM_LED2);
        nrf_gpio_pin_toggle(RM_LED3);
    }
    if(percentage>low_power && percentage<high_power){
        nrf_gpio_pin_toggle(RM_LED1);
        nrf_gpio_pin_toggle(RM_LED2);
    }
    if(percentage<=low_power && percentage>blink_power){
        nrf_gpio_pin_toggle(RM_LED1);
    }
    if(percentage<=blink_power){
        //blink for low power
        err_code = app_timer_start(m_repeated_timer_id_low_power_led_blink, APP_TIMER_TICKS(ble_led_blink_ms_fast), NULL);
        APP_ERROR_CHECK(err_code);
    }
}

static void received_ble_data_array_handle(uint8_t *dst, uint8_t *src, uint8_t len, uint8_t x) {
    memmove(dst, src, len);
	received_ble_data_length -= x;
}

/**
 * Function to send a mode command over BLE NUS 
 */
static void send_mode_cmd(uint8_t *data, uint16_t length)
{
  uint32_t err_code;
  
  err_code = ble_nus_data_send(&m_nus, data, &length, m_conn_handle);
  if ((err_code != NRF_ERROR_INVALID_STATE) && 
      (err_code != NRF_ERROR_RESOURCES) &&
      (err_code != NRF_ERROR_NOT_FOUND))
  {
       APP_ERROR_CHECK(err_code);
  }
}

void ble_query_timer_enabled(void) {
	uint32_t err_code;
	timeout_ble_connected += query_mode_after_ble_connected;
	err_code = app_timer_start(m_single_shot_timer_id_ble_connected_query_mode, APP_TIMER_TICKS(timeout_ble_connected), NULL);
	APP_ERROR_CHECK(err_code);
}

void mode_timer_enabled(void) {
	uint32_t err_code;
	timeout_mode += query_mode_before_mode_switch;
	err_code = app_timer_start(m_single_shot_timer_id, APP_TIMER_TICKS(timeout_mode), NULL);
	APP_ERROR_CHECK(err_code);
}

/**
 * @brief Interrupt handler for button click
 */
void in_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
	if (pin == MODE_UP || pin == MODE_DOWN ){
		//m_single_shot_timer_id_power_off
		uint32_t err_code;	
		// timer stop
		err_code = app_timer_stop(m_single_shot_timer_id_power_off);
		APP_ERROR_CHECK(err_code);
		// timer start
		timeout_ble_connected += query_mode_after_ble_connected;
		err_code = app_timer_start(m_single_shot_timer_id_power_off, APP_TIMER_TICKS(1000*seconds_of_inactivity_before_going_to_sleep), NULL);
		APP_ERROR_CHECK(err_code);
	}
	
	if(left_side_or_right_side){  
		if (pin == MODE_UP){
			mode_button='u';
			send_mode_cmd(mode_query, sizeof(mode_query));
			mode_timer_enabled();
		} else if (pin == MODE_DOWN) {	
			mode_button='d';
			send_mode_cmd(mode_query, sizeof(mode_query));
			mode_timer_enabled();
		}
	}else{
		if (pin == MODE_UP){
			mode_button='d';
			send_mode_cmd(mode_query, sizeof(mode_query));
			mode_timer_enabled();
		} else if (pin == MODE_DOWN) {	
			mode_button='u';
			send_mode_cmd(mode_query, sizeof(mode_query));
			mode_timer_enabled();
		}
	}
}

/**
 * @brief Function for configuring: Button and LED
 */
static void gpio_init(void)
{
    ret_code_t err_code;

    //Initialize gpiote module
    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_HITOLO(false);     //Configure to generate interrupt and wakeup on pin signal low. "false" means that gpiote will use the PORT event, which is low power, i.e. does not add any noticable current consumption (<<1uA). Setting this to "true" will make the gpiote module use GPIOTE->IN events which add ~8uA for nRF52 and ~1mA for nRF51.
	
	//MODE_UP - Configure sense input pin to enable wakeup and interrupt on button press.
    in_config.pull = NRF_GPIO_PIN_PULLUP;                                            //Configure pullup for input pin to prevent it from floting. Pin is pulled down when button is pressed on nRF5x-DK boards, see figure two in http://infocenter.nordicsemi.com/topic/com.nordic.infocenter.nrf52/dita/nrf52/development/dev_kit_v1.1.0/hw_btns_leds.html?cp=2_0_0_1_4		
    err_code = nrf_drv_gpiote_in_init(MODE_UP, &in_config, in_pin_handler);   //Initialize the pin with interrupt handler in_pin_handler
    APP_ERROR_CHECK(err_code);                                                          //Check potential error
    nrf_drv_gpiote_in_event_enable(MODE_UP, true);                            //Enable event and interrupt for the wakeup pin

	//MODE_DOWN - Configure sense input pin to enable wakeup and interrupt on button press.
    in_config.pull = NRF_GPIO_PIN_PULLUP;                                            //Configure pullup for input pin to prevent it from floting. Pin is pulled down when button is pressed on nRF5x-DK boards, see figure two in http://infocenter.nordicsemi.com/topic/com.nordic.infocenter.nrf52/dita/nrf52/development/dev_kit_v1.1.0/hw_btns_leds.html?cp=2_0_0_1_4		
    err_code = nrf_drv_gpiote_in_init(MODE_DOWN, &in_config, in_pin_handler);   //Initialize the pin with interrupt handler in_pin_handler
    APP_ERROR_CHECK(err_code);                                                          //Check potential error
    nrf_drv_gpiote_in_event_enable(MODE_DOWN, true); 
		
    //POWER_ON ***會不能開機
/*    in_config.pull = NRF_GPIO_PIN_PULLDOWN;                                            //Configure pullup for input pin to prevent it from floting. Pin is pulled down when button is pressed on nRF5x-DK boards, see figure two in http://infocenter.nordicsemi.com/topic/com.nordic.infocenter.nrf52/dita/nrf52/development/dev_kit_v1.1.0/hw_btns_leds.html?cp=2_0_0_1_4		
    err_code = nrf_drv_gpiote_in_init(POWER_ON, &in_config, in_pin_handler);   //Initialize the pin with interrupt handler in_pin_handler
    APP_ERROR_CHECK(err_code);                                                          //Check potential error
    nrf_drv_gpiote_in_event_enable(POWER_ON, true); */
		
	//other
    nrf_gpio_cfg_output(RM_LED1);  
    nrf_gpio_cfg_output(RM_LED2);
    nrf_gpio_cfg_output(RM_LED3); 
    nrf_gpio_cfg_output(BLE_LED); 
    nrf_gpio_cfg_output(MODE_LED1); 
    nrf_gpio_cfg_output(MODE_LED2); 
    nrf_gpio_cfg_output(MODE_LED3); 
    nrf_gpio_cfg_input(MCU_POWER_HOLD, NRF_GPIO_PIN_PULLUP);
}


/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyse
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Function for initializing the timer module.
 */
static void timers_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of
 *          the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *) DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}
/**
 * Checks if the prefix of `arr1` is equal to `arr2`.

 * @param arr1 A pointer to the first array.
 * @param arr2 A pointer to the second array.

 * @return `true` if the prefix of `arr1` is equal to `arr2`, `false` otherwise.
 */
bool is_prefix_equal(uint8_t* arr1, uint8_t* arr2) {
  if (sizeof(arr1)>sizeof(arr2)) return false;
  for (int i = 0; i < sizeof(arr1); i++) {
    if (arr1[i] != arr2[i]) {
      return false; 
    }
  }
  return true;
}

/**
 * Checks if the given array is equal to either the mode command array.

 * @param arr2 A pointer to the array to check.

 * @return `'1'` if the array is equal to `mode_command_1`, `'2'` if the array is equal to `mode_command_2`, or `' '` otherwise.
 */
char is_equal_command( uint8_t* received_ble_data_array) {
	if(is_prefix_equal(power_query_response_prefix_four,received_ble_data_array)
    && received_ble_data_length == 10){
        NRF_LOG_INFO("power:%d",received_ble_data_array[4]);
        NRF_LOG_INFO("power:0x%x",received_ble_data_array[4]);
        power_led(received_ble_data_array[4]);
        return 'p'; 
	}
	if(is_prefix_equal(mode_command_1,received_ble_data_array)){
      return 'm'; 
	}
	if(is_prefix_equal(mode_command_2,received_ble_data_array)){
      return 'm'; 
	}
	if(is_prefix_equal(mode_command_3,received_ble_data_array)){
      return 'm'; 
	}
	if(is_prefix_equal(query_response_mode_command_1,received_ble_data_array)){
      return '1'; 
	}
	if(is_prefix_equal(query_response_mode_command_2,received_ble_data_array)){
      return '2'; 
	}
	if(is_prefix_equal(query_response_mode_command_3,received_ble_data_array)){
      return '3'; 
	}
  return ' ';
}

/**
 * @brief Handle received BLE NUS data
 * 
 * This function handles the received BLE NUS data and copies it into 
 * the global received_ble_data_array buffer.
 * 
 * If the buffer is full, log an error message.
 * 
 * @param[in] p_evt   The BLE NUS event containing RX data
 */
static void handle_received_nus_data(ble_nus_evt_t * p_evt) {
  if (received_ble_data_length == 0) {
    if (p_evt->params.rx_data.length <= MAX_RECEIVED_BLE_ARRAY_SIZE) {
      memcpy(received_ble_data_array, p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);
      received_ble_data_length = p_evt->params.rx_data.length;
    } else {
      NRF_LOG_INFO("Data too long to fit in the array.");
    }
  } else {
    if (received_ble_data_length + p_evt->params.rx_data.length <= MAX_RECEIVED_BLE_ARRAY_SIZE) {
      memcpy(received_ble_data_array + received_ble_data_length, p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);
      received_ble_data_length += p_evt->params.rx_data.length;
    } else {
      NRF_LOG_INFO("Data too long to fit in the array.");
    }
  }
}


/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_evt       Nordic UART Service event.
 */
/**@snippet [Handling the data received over BLE] */
static void nus_data_handler(ble_nus_evt_t * p_evt)
{
    if (p_evt->type == BLE_NUS_EVT_RX_DATA)
    {
        uint32_t err_code;
        if(usingRX_TX_for_debug){
            for (uint32_t i = 0; i < p_evt->params.rx_data.length; i++)
            {
                    do
                    {
                            err_code = app_uart_put(p_evt->params.rx_data.p_data[i]);
                            if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_BUSY))
                            {
                                    NRF_LOG_ERROR("Failed receiving NUS message. Error 0x%x. ", err_code);
                                    APP_ERROR_CHECK(err_code);
                            }
                    } while (err_code == NRF_ERROR_BUSY);
            }
        }
				
        //NRF_LOG_INFO("Received data from BLE NUS. Writing data on UART.");
        //NRF_LOG_HEXDUMP_INFO(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);
        handle_received_nus_data(p_evt);
        // NRF_LOG_INFO("received_ble_data_length:%d",received_ble_data_length);
        // NRF_LOG_HEXDUMP_INFO(received_ble_data_array, received_ble_data_length);
        if(received_ble_data_length>4){
            //do {
                switch ( is_equal_command(received_ble_data_array) )
                {
                    case '1'://query moode結果為1
                        //NRF_LOG_INFO("is_equal_command : 1");
                        mode_status = 1;
                        received_ble_data_array_handle(&received_ble_data_array[0], &received_ble_data_array[5], received_ble_data_length - 5,5);
                        switch_mode_led(MODE_LED1);
                        break;
                    case '2':
                        //NRF_LOG_INFO("is_equal_command : 2");
                        mode_status = 2;
                        received_ble_data_array_handle(&received_ble_data_array[0], &received_ble_data_array[5], received_ble_data_length - 5,5);
                        switch_mode_led(MODE_LED2);	
                        break;
                    case '3':
                        //NRF_LOG_INFO("is_equal_command : 3");
                        mode_status = 3;
                        received_ble_data_array_handle(&received_ble_data_array[0], &received_ble_data_array[5], received_ble_data_length - 5,5);
                        switch_mode_led(MODE_LED3);		
                        break;
                    case 'm': //click up/down button.從received_ble_data_array刪掉,不用處理.
                        //NRF_LOG_INFO("is_equal_command : m");
                        received_ble_data_array_handle(&received_ble_data_array[0], &received_ble_data_array[5], received_ble_data_length - 5,5);
                        break;
                    case 'p': //電量
                        //NRF_LOG_INFO("is_equal_command : m");
                        received_ble_data_array_handle(&received_ble_data_array[0], &received_ble_data_array[10], received_ble_data_length - 10,10);
                        break;
                    default:
                        // 判斷結尾是否有 : 0d 0a ?
                        if(received_ble_data_length>1){
                            if(received_ble_data_array[received_ble_data_length-2] == 0x0d 
                            && received_ble_data_array[received_ble_data_length-1] == 0x0a){
                                received_ble_data_array_handle(&received_ble_data_array[0], &received_ble_data_array[received_ble_data_length],0, received_ble_data_length );
                            }
                        }
                        // NRF_LOG_INFO("case default");
                    }
                    // NRF_LOG_INFO("2. received_ble_data_length:%d",received_ble_data_length);
                    // NRF_LOG_HEXDUMP_INFO(received_ble_data_array, received_ble_data_length);
                //} while (received_ble_data_length > 4);
                // NRF_LOG_INFO("3. received_ble_data_length:%d",received_ble_data_length);
                // NRF_LOG_HEXDUMP_INFO(received_ble_data_array, received_ble_data_length);
            }
    }
}
/**@snippet [Handling the data received over BLE] */


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t           err_code;
    ble_nus_init_t     nus_init;
    nrf_ble_qwr_init_t qwr_init = {0};

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    // Initialize NUS.
    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;

    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module
 *          which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply setting
 *       the disconnect_on_fail config parameter, but instead we use the event handler
 *       mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;
        case BLE_ADV_EVT_IDLE:
            NRF_LOG_INFO("BLE_ADV_EVT_IDLE");
            printf("\r\n BLE_ADV_EVT_IDLE \r\n");
            err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST); 
            APP_ERROR_CHECK(err_code);
            break;
        default:
            break;
    }
}

static void query_mode(void){	
	send_mode_cmd(mode_query, sizeof(mode_query));
}

static void query_power(void){	
	send_mode_cmd(power_query, sizeof(power_query));
}

static bool is_ble_while_list(ble_evt_t const * p_ble_evt){
	if(p_ble_evt->evt.gap_evt.params.connected.peer_addr.addr[0]!= 0x5C){
		return false;
	}
	if(p_ble_evt->evt.gap_evt.params.connected.peer_addr.addr[1]!= 0x32){
		return false;
	}
	if(p_ble_evt->evt.gap_evt.params.connected.peer_addr.addr[2]!= 0x5E){
		return false;
	}
	if(p_ble_evt->evt.gap_evt.params.connected.peer_addr.addr[3]!= 0x52){
		return false;
	}
	if(p_ble_evt->evt.gap_evt.params.connected.peer_addr.addr[4]!= 0x3B){
		return false;
	}
	if(p_ble_evt->evt.gap_evt.params.connected.peer_addr.addr[5]!= 0xE6){
		return false;
	}
	return true;
}
/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    uint32_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected");
            printf(" Connected.");
            //get remote MAC address
            printf("Connected to device with MAC address: %02X:%02X:%02X:%02X:%02X:%02X\n",
                   p_ble_evt->evt.gap_evt.params.connected.peer_addr.addr[0],
                   p_ble_evt->evt.gap_evt.params.connected.peer_addr.addr[1],
                   p_ble_evt->evt.gap_evt.params.connected.peer_addr.addr[2],
                   p_ble_evt->evt.gap_evt.params.connected.peer_addr.addr[3],
                   p_ble_evt->evt.gap_evt.params.connected.peer_addr.addr[4],
                   p_ble_evt->evt.gap_evt.params.connected.peer_addr.addr[5]);
				
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
				
            //check white list
            if(! is_ble_while_list(p_ble_evt)){
                // 主動斷線
                uint32_t err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            //停止低電量閃爍
		    err_code = app_timer_stop(m_repeated_timer_id_low_power_led_blink);
            APP_ERROR_CHECK(err_code);
            //停止電量查詢
		    err_code = app_timer_stop(m_repeated_timer_id_query_power);
            APP_ERROR_CHECK(err_code);
            
            NRF_LOG_INFO("Disconnected");
            printf("Disconnected.");
            // LED indication will be changed when advertising starts.
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
						
            blink_BLE_LED();
            led_off_all();
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for the SoftDevice initialization.
 *
 * @details This function initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
    {
        m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        NRF_LOG_INFO("Data len is set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
				
        is_GATT_EVT_ATT_MTU_UPDATED_once = true;
        ble_query_timer_enabled();
        uint32_t err_code;
        err_code = app_timer_stop(m_repeated_timer_id_ble_led_blink);  //藍芽燈不再閃爍
        APP_ERROR_CHECK(err_code);
        turn_on_BLE_LED();//藍芽燈打開
        
        //電量查詢
        err_code = app_timer_start(m_repeated_timer_id_query_power, APP_TIMER_TICKS(seconds_query_power*1000), NULL);
        APP_ERROR_CHECK(err_code);
        
        // mode_timer_enabled();
        nrf_delay_ms(5000);
        query_mode(); //查詢模式
    }
    // NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x",
    //               p_gatt->att_mtu_desired_central,
    //               p_gatt->att_mtu_desired_periph);
}


/**@brief Function for initializing the GATT library. */
void gatt_init(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    uint32_t err_code;
    switch (event)
    {
        case BSP_EVENT_SLEEP:
            NRF_LOG_INFO("BSP_EVENT_SLEEP");
            printf("\r\n BSP_EVENT_SLEEP \r\n");
            err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST); 
            APP_ERROR_CHECK(err_code);
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BSP_EVENT_WHITELIST_OFF:
            if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                err_code = ble_advertising_restart_without_whitelist(&m_advertising);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            break;

        default:
            break;
    }
}


/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to
 *          a string. The string will be be sent over BLE when the last character received was a
 *          'new line' '\n' (hex 0x0A) or if the string has reached the maximum data length.
 */
/**@snippet [Handling the data received over UART] */
void uart_event_handle(app_uart_evt_t * p_event)
{
    static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
    static uint8_t index = 0;
    uint32_t       err_code;

    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
            if(usingRX_TX_for_debug){
                UNUSED_VARIABLE(app_uart_get(&data_array[index]));
                index++;
                /*if ((data_array[index - 1] == '\n') ||
                (data_array[index - 1] == '\r') ||
                (index >= m_ble_nus_max_data_len))
                {*/
                if (index > 0)
                {
                    NRF_LOG_DEBUG("Ready to send data over BLE NUS");
                    NRF_LOG_HEXDUMP_DEBUG(data_array, index);
                    do
                    {
                        uint16_t length = (uint16_t)index;
                        err_code = ble_nus_data_send(&m_nus, data_array, &length, m_conn_handle);
                        if ((err_code != NRF_ERROR_INVALID_STATE) &&
                            (err_code != NRF_ERROR_RESOURCES) &&
                            (err_code != NRF_ERROR_NOT_FOUND))
                        {
                            APP_ERROR_CHECK(err_code);
                        }
                    } while (err_code == NRF_ERROR_RESOURCES);
                }
                index = 0;
								//}
            }
            break;

        case APP_UART_COMMUNICATION_ERROR:
            if(usingRX_TX_for_debug){
							APP_ERROR_HANDLER(p_event->data.error_communication); //[NRF_ERROR_DATA_SIZE] when RT/TX no connect.
						}
            break;

        case APP_UART_FIFO_ERROR:
            if(usingRX_TX_for_debug){
							APP_ERROR_HANDLER(p_event->data.error_code);
						}
            break;

        default:
            break;
    }
}
/**@snippet [Handling the data received over UART] */


/**@brief  Function for initializing the UART module.
 */
/**@snippet [UART Initialization] */
static void uart_init(void)
{
    uint32_t                     err_code;
    app_uart_comm_params_t const comm_params =
    {
        .rx_pin_no    = RX_PIN_NUMBER,
        .tx_pin_no    = TX_PIN_NUMBER,
        // .rx_pin_no    = 25,
        // .tx_pin_no    = 26,
        .rts_pin_no   = RTS_PIN_NUMBER,
        .cts_pin_no   = CTS_PIN_NUMBER,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
        .use_parity   = false,
#if defined (UART_PRESENT)
        .baud_rate    = NRF_UART_BAUDRATE_9600
#else
        .baud_rate    = NRF_UARTE_BAUDRATE_115200
#endif
    };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);
    APP_ERROR_CHECK(err_code);
}
/**@snippet [UART Initialization] */


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    uint32_t               err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance = false;
    //init.advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;
    init.advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

    init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.srdata.uuids_complete.p_uuids  = m_adv_uuids;

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;
    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    bsp_event_t startup_event;

    // NULL --> NRF_ERROR_NO_MEM]
    uint32_t err_code = bsp_init(NULL, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    uint32_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
}

/**@brief Timeout handler for the repeated timer.
 */
static void repeated_timer_handler_ble_led_blink(void * p_context)
{
	nrf_gpio_pin_toggle(BLE_LED);
}

static void repeated_timer_handler_low_power_led_blink(void * p_context)
{
	nrf_gpio_pin_toggle(RM_LED1);
}

static void single_shot_timer_handler_ble_connected_query_mode(void * p_context)
{
	query_mode();
}

static void repeated_timer_handler_query_power(void * p_context)
{
    // printf("\r\n query_power \r\n");
    // NRF_LOG_INFO("query_power");
	query_power();
}

static void single_shot_timer_handler_power_off(void * p_context)
{
    
    // nrf_delay_ms(2000);
    printf("\r\n single_shot_timer_handler_power_off \r\n");
    NRF_LOG_INFO("single_shot_timer_handler_power_off");

    // nrf_delay_ms(2000);
    //? ping ??
	// nrf_gpio_cfg_input(MODE_LED1, GPIO_PIN_CNF_PULL_Pulldown);
	// nrf_gpio_cfg_input(MODE_LED2, GPIO_PIN_CNF_PULL_Pulldown);
	// nrf_gpio_cfg_input(MODE_LED3, GPIO_PIN_CNF_PULL_Pulldown);
    
	// nrf_gpio_cfg_input(RM_LED1, GPIO_PIN_CNF_PULL_Pulldown);
	// nrf_gpio_cfg_input(RM_LED2, GPIO_PIN_CNF_PULL_Pulldown);
	// nrf_gpio_cfg_input(RM_LED3, GPIO_PIN_CNF_PULL_Pulldown);
    
	// nrf_gpio_cfg_input(BLE_LED, GPIO_PIN_CNF_PULL_Pulldown);

	// nrf_gpio_cfg_input(MCU_POWER_HOLD, GPIO_PIN_CNF_PULL_Pulldown);
	// nrf_gpio_cfg_input(POWER_ON, GPIO_PIN_CNF_PULL_Pulldown);
	// nrf_gpio_cfg_input(MODE_UP, GPIO_PIN_CNF_PULL_Pulldown);
	// nrf_gpio_cfg_input(MODE_DOWN, GPIO_PIN_CNF_PULL_Pulldown);
	// nrf_gpio_cfg_input(CELL_V, GPIO_PIN_CNF_PULL_Pulldown);

    // nrf_delay_ms(2000);

    //? ping ??
	// nrf_gpio_cfg_input(MODE_LED1, NRF_GPIO_PIN_PULLUP);
	// nrf_gpio_cfg_input(MODE_LED2, NRF_GPIO_PIN_PULLUP);
	// nrf_gpio_cfg_input(MODE_LED3, NRF_GPIO_PIN_PULLUP);
    
	// nrf_gpio_cfg_input(RM_LED1, NRF_GPIO_PIN_PULLUP);
	// nrf_gpio_cfg_input(RM_LED2, NRF_GPIO_PIN_PULLUP);
	// nrf_gpio_cfg_input(RM_LED3, NRF_GPIO_PIN_PULLUP);
    
	// nrf_gpio_cfg_input(BLE_LED, NRF_GPIO_PIN_PULLUP);

	// nrf_gpio_cfg_input(MCU_POWER_HOLD, NRF_GPIO_PIN_PULLUP);
	// nrf_gpio_cfg_input(POWER_ON, NRF_GPIO_PIN_PULLUP);
	// nrf_gpio_cfg_input(MODE_UP, NRF_GPIO_PIN_PULLUP);
	// nrf_gpio_cfg_input(MODE_DOWN, NRF_GPIO_PIN_PULLUP);
	// nrf_gpio_cfg_input(CELL_V, NRF_GPIO_PIN_PULLUP);
    
    //不可刪掉NRF_GPIO_PIN_NOPULL 
	nrf_gpio_cfg_input(MODE_LED1, NRF_GPIO_PIN_NOPULL);
	nrf_gpio_cfg_input(MODE_LED2, NRF_GPIO_PIN_NOPULL);
	nrf_gpio_cfg_input(MODE_LED3, NRF_GPIO_PIN_NOPULL);
    
	nrf_gpio_cfg_input(RM_LED1, NRF_GPIO_PIN_NOPULL);
	nrf_gpio_cfg_input(RM_LED2, NRF_GPIO_PIN_NOPULL);
	nrf_gpio_cfg_input(RM_LED3, NRF_GPIO_PIN_NOPULL);
    
	nrf_gpio_cfg_input(BLE_LED, NRF_GPIO_PIN_NOPULL);

	nrf_gpio_cfg_input(MCU_POWER_HOLD, NRF_GPIO_PIN_NOPULL);
	nrf_gpio_cfg_input(POWER_ON, NRF_GPIO_PIN_NOPULL);
	nrf_gpio_cfg_input(MODE_UP, NRF_GPIO_PIN_NOPULL);
	nrf_gpio_cfg_input(MODE_DOWN, NRF_GPIO_PIN_NOPULL);
	nrf_gpio_cfg_input(CELL_V, NRF_GPIO_PIN_NOPULL);

    // nrf_delay_ms(2000);
    
    // printf("\r\n NRF_GPIO_PIN_PULLUP \r\n");
    // NRF_LOG_INFO("NRF_GPIO_PIN_PULLUP");

    // nrf_delay_ms(2000);

    // nRF52832 ??,??
    // NVIC_SystemReset();
    
    uint32_t err_code;	
    // timer stop
    err_code = app_timer_start(m_single_shot_timer_id_power_off_step2, APP_TIMER_TICKS(1000*3), NULL);
    APP_ERROR_CHECK(err_code);
}

static void single_shot_timer_handler_power_off_step2(void * p_context)
{
    
    // nrf_delay_ms(2000);
    // printf("\r\n single_shot_timer_handler_power_off_step2 \r\n");
    // NRF_LOG_INFO("single_shot_timer_handler_power_off_step2");

    // nrf_delay_ms(2000);
    
    // nRF52832 ??,??
    NVIC_SystemReset();
}

static void single_shot_timer_handler_mode_switch(void * p_context)
{
/*	NRF_LOG_INFO("mode_status:%d",mode_status);
	NRF_LOG_INFO("mode_button:%c",mode_button);
	if(left_side_or_right_side){NRF_LOG_INFO("left_side_or_right_side:true");}
	else{NRF_LOG_INFO("left_side_or_right_side:false");}	*/
	switch(mode_status){
		case 1:
			if(mode_button=='d'){
				//Send mode command 2
				send_mode_cmd(mode_command_2, sizeof(mode_command_2));
				query_mode();
			}
			break;
		case 2:
			if(mode_button=='u'){
				//Send mode command 1
				send_mode_cmd(mode_command_1, sizeof(mode_command_1));
				query_mode();
			}
			if(mode_button=='d'){
				//Send mode command 3
				send_mode_cmd(mode_command_3, sizeof(mode_command_3));
				query_mode();
			}
			break;
		case 3:
			if(mode_button=='u'){
				//Send mode command 2
				send_mode_cmd(mode_command_2, sizeof(mode_command_2));
				query_mode();
			}
			break;
		default:
			//donothing
			break;
	}
}

/**@brief Create timers.
 */
static void create_timers()
{
	ret_code_t err_code;
	// Create timers
	err_code = app_timer_create(&m_single_shot_timer_id,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                single_shot_timer_handler_mode_switch);
	
	err_code = app_timer_create(&m_single_shot_timer_id_ble_connected_query_mode,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                single_shot_timer_handler_ble_connected_query_mode);
	
	err_code = app_timer_create(&m_single_shot_timer_id_power_off,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                single_shot_timer_handler_power_off);

	err_code = app_timer_create(&m_repeated_timer_id_low_power_led_blink,
                                APP_TIMER_MODE_REPEATED,
                                repeated_timer_handler_low_power_led_blink);
	
	err_code = app_timer_create(&m_repeated_timer_id_ble_led_blink,
                                APP_TIMER_MODE_REPEATED,
                                repeated_timer_handler_ble_led_blink);
	
	err_code = app_timer_create(&m_repeated_timer_id_query_power,
                                APP_TIMER_MODE_REPEATED,
                                repeated_timer_handler_query_power);
	
	err_code = app_timer_create(&m_single_shot_timer_id_power_off_step2,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                single_shot_timer_handler_power_off_step2);

	APP_ERROR_CHECK(err_code);
}

/**@brief Application main function.
 */
int main(void)
{
    bool erase_bonds;
    gpio_init();

    //LED test
    led_blink();
    led_off_all();
		
    // Initialize.
    uart_init();
    log_init();
    timers_init();
    buttons_leds_init(&erase_bonds);
    power_management_init();
    ble_stack_init();
    gap_params_init();
    gatt_init();
    services_init();
    advertising_init();
    conn_params_init();

    // Start execution.
    //printf("\r\nUART started.\r\n");
    NRF_LOG_INFO("Debug logging for UART over RTT started.");
    advertising_start();
		
    //get local MAC address
    ble_gap_addr_t addr;
    uint32_t err_code = sd_ble_gap_addr_get(&addr);
    if (err_code == NRF_SUCCESS)
    {
        printf("My MAC address: %02x:%02x:%02x:%02x:%02x:%02x\n",
                        addr.addr[0], addr.addr[1], addr.addr[2],
                        addr.addr[3], addr.addr[4], addr.addr[5]);
    }
	
    //Timer
    create_timers();
		//m_repeated_timer_id_ble_led_blink
    if(!is_GATT_EVT_ATT_MTU_UPDATED_once){				
        uint32_t err_code;
        if(isPairing){
                err_code = app_timer_start(m_repeated_timer_id_ble_led_blink, APP_TIMER_TICKS(ble_led_blink_ms), NULL); 
        }else{
                err_code = app_timer_start(m_repeated_timer_id_ble_led_blink, APP_TIMER_TICKS(ble_led_blink_ms_fast), NULL); 
        }
        APP_ERROR_CHECK(err_code);
    }
		
    //m_single_shot_timer_id_power_off
    timeout_ble_connected += query_mode_after_ble_connected;
    err_code = app_timer_start(m_single_shot_timer_id_power_off, APP_TIMER_TICKS(1000*seconds_of_inactivity_before_going_to_sleep), NULL);
    
    /* FLASH */
    // uint32_t addr_data; 
    // uint32_t patwr [3]; 
    // uint32_t pg_size; 
    // uint32_t pg_num; 

    // patwr [0]=222;
    // patwr [1]=555; 
    // patwr [2]=333; 

    // pg_size = NRF_FICR->CODEPAGESIZE; 
    // pg_num = NRF_FICR->CODESIZE - 1; // ??????Flash
		
    // addr_data = (pg_size * pg_num); 

    // // Read  
    // nrf_delay_ms(200);
    // uint32_t* p_data = (uint32_t*)addr_data;  
    // for(int i = 0; i < 3; i++) {
	// 	uint32_t data_read = p_data[i];  
	// 	NRF_LOG_INFO("Data at index %d: %u", i, data_read);
    // }

    // //erase
    // nrf_delay_ms(200);
    // patwr [0]=777;
    // sd_flash_page_erase(pg_num);
				
    // // Read from ?Flash????
    // nrf_delay_ms(200);
    // uint32_t* p_data_2 = (uint32_t*)addr_data; // ????????????
    // uint32_t data_read_2 = *p_data_2; // ????????
    // NRF_LOG_INFO("Data at address %u: %u", addr_data, data_read_2);
		
    // //write
    // nrf_delay_ms(200);
    // sd_flash_write((uint32_t*)addr_data, patwr, 3);
		
    // // Read from ?Flash????
    // nrf_delay_ms(200);
    // uint32_t* p_data_3 = (uint32_t*)addr_data; // ????????????
    // uint32_t data_read_3 = *p_data_3; // ????????
    // NRF_LOG_INFO("Data at address %u: %u", addr_data, data_read_3);
	
	

    //開機後,第一次電量查詢
    nrf_delay_ms(1000);
    query_power();
    
    //每隔多久電量查詢
    err_code = app_timer_start(m_repeated_timer_id_query_power, APP_TIMER_TICKS(seconds_query_power*1000), NULL);
    APP_ERROR_CHECK(err_code);

    // Enter main loop.
    for (;;)
    {
        //Enter System-on idle mode
        __WFE();
        __SEV();
        __WFE();	
			
        idle_state_handle();
    }
}


/**
 * @}
 */
