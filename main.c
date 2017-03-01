/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
 *
 * @defgroup ble_sdk_uart_over_ble_main main.c
 * @{
 * @ingroup  ble_sdk_app_nus_eval
 * @brief    UART over BLE application main file.
 *
 * This file contains the source code for a sample application that uses the Nordic UART service.
 * This application uses the @ref srvlib_conn_params module.
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf_adc.h"
#include "nrf.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "app_button.h"
#include "ble_nus.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "bsp.h"
#include "bsp_btn_ble.h"
#include "nrf_delay.h"
#include "pins.h"

#define IS_SRVC_CHANGED_CHARACT_PRESENT 0                                           /**< Include the service_changed characteristic. If not enabled, the server's database cannot be changed for the lifetime of the device. */

#define DEVICE_NAME                     "FretX"                               			/**< Name of device. Will be included in the advertising data. */
#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      180                                         /**< The advertising timeout (in units of seconds). */

#define APP_TIMER_PRESCALER             0                                           /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE         4                                           /**< Size of timer operation queues. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(75, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER) /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define START_STRING                    "Start...\n"                                /**< The string that will be sent over the UART when the application starts. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */

static ble_nus_t                        m_nus;                                      /**< Structure to identify the Nordic UART Service. */
static uint16_t                         m_conn_handle = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */

static ble_uuid_t                       m_adv_uuids[] = {{BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}};  /**< Universally unique service identifier. */

uint8_t battery = 0;

bool push = true;
bool leds_running = false;
bool connected = false;
bool data_received = false;
#define VBAT_MAX_IN_MV                  3000

volatile int32_t adc_sample;

//const uint8_t led = 22;				/** LED connected to p0.21 **/

bool OFF = 1;
bool ON = 0;
bool led1,led2,led3,led4,led5,led6,led11,led12,led13,led14,led15,led16,led21,led22,led23,led24,led25,led26,led31,led32,led33,led34,led35,led36,led41,led42,led43,led44,led45,led46;
int next = 0;

const uint8_t row1 = 0;
const uint8_t row2 = 1;
const uint8_t row3 = 2;
const uint8_t row4 = 3;
const uint8_t row5 = 4;
const uint8_t row6 = 5;
const uint8_t col1 = 10;
const uint8_t col2 = 9;
const uint8_t col3 = 8;
const uint8_t col4 = 7;
const uint8_t col5 = 6;

/******************************************************************************************/

uint8_t battery_level_get(void)
{
// Configure ADC
NRF_ADC->CONFIG     = (ADC_CONFIG_RES_8bit                        << ADC_CONFIG_RES_Pos)     |
                      (ADC_CONFIG_INPSEL_SupplyOneThirdPrescaling << ADC_CONFIG_INPSEL_Pos)  |
                      (ADC_CONFIG_REFSEL_VBG                      << ADC_CONFIG_REFSEL_Pos)  |
                      (ADC_CONFIG_PSEL_Disabled                   << ADC_CONFIG_PSEL_Pos)    |
                      (ADC_CONFIG_EXTREFSEL_None                  << ADC_CONFIG_EXTREFSEL_Pos);
NRF_ADC->EVENTS_END = 0;
NRF_ADC->ENABLE     = ADC_ENABLE_ENABLE_Enabled;

NRF_ADC->EVENTS_END  = 0;    // Stop any running conversions.
NRF_ADC->TASKS_START = 1;

while (!NRF_ADC->EVENTS_END)
{
	
}

uint16_t vbg_in_mv = 1200;
uint8_t adc_max = 255;
uint16_t vbat_current_in_mv = (NRF_ADC->RESULT * 3 * vbg_in_mv) / adc_max;

NRF_ADC->EVENTS_END     = 0;
NRF_ADC->TASKS_STOP     = 1;

return (uint8_t) ((vbat_current_in_mv * 100) / VBAT_MAX_IN_MV);
       
}


/******************************************************************************************/


void gpio_output(void)
{
	nrf_gpio_cfg_output(row1);
	nrf_gpio_cfg_output(row2);
	nrf_gpio_cfg_output(row3);
	nrf_gpio_cfg_output(row4);
	nrf_gpio_cfg_output(row5);
	nrf_gpio_cfg_output(row6);
	nrf_gpio_cfg_output(col1);
	nrf_gpio_cfg_output(col2);
	nrf_gpio_cfg_output(col3);
	nrf_gpio_cfg_output(col4);
	nrf_gpio_cfg_output(col5);
}

/*******************************************************************************************/

void match(int integer)
{
    switch (integer)
    {
    case 1:
        led6 = 1;
        break;
    case 2:
        led5 = 1;
        break;
    case 3:
        led4 = 1;
        break;
    case 4:
        led3 = 1;
        break;
    case 5:
        led2 = 1;
        break;
    case 6:
        led1 = 1;
        break;
    
    case 11:
        led16 = 1;
        break;
    case 12:
        led15 = 1;
        break;
    case 13:
        led14 = 1;
        break;
    case 14:
        led13 = 1;
        break;
    case 15:
        led12 = 1;
        break;
    case 16:
        led11 = 1;
        break;
        
    case 21:
        led26 = 1;
        break;
    case 22:
        led25 = 1;
        break;
    case 23:
        led24 = 1;
        break;
    case 24:
        led23 = 1;;
        break;
    case 25:
        led22 = 1;
        break;
    case 26:
        led21 = 1;
        break;
        
    case 31:
        led36 = 1;
        break;
    case 32:
        led35 = 1;
        break;
    case 33:
        led34 = 1;
        break;
    case 34:
        led33 = 1;;
        break;
    case 35:
        led32 = 1;
        break;
    case 36:
        led31 = 1;
        break;
        
    case 41:
        led46 = 1;
        break;
    case 42:
        led45 = 1;
        break;
    case 43:
        led44 = 1;
        break;
    case 44:
        led43 = 1;
        break;
    case 45:
        led42 = 1;
        break;
    case 46:
        led41 = 1;
        break;
    }
}

void all_clear()
{
    led1 = 0;
    led2 = 0;
    led3 = 0;
    led4 = 0;
    led5 = 0;
    led6 = 0;
    led11 = 0;
    led12 = 0;
    led13 = 0;
    led14 = 0;
    led15 = 0;
    led16 = 0;
    led21 = 0;
    led22 = 0;
    led23 = 0;
    led24 = 0;
    led25 = 0;
    led26 = 0;
    led31 = 0;
    led32 = 0;
    led33 = 0;
    led34 = 0;
    led35 = 0;
    led36 = 0;
    led41 = 0;
    led42 = 0;
    led43 = 0;
    led44 = 0;
    led45 = 0;
    led46 = 0;
}

void leds_off()
{
  nrf_gpio_pin_clear(row1);
	nrf_gpio_pin_clear(row2);
	nrf_gpio_pin_clear(row3);
	nrf_gpio_pin_clear(row4);
	nrf_gpio_pin_clear(row5);
	nrf_gpio_pin_clear(row6);
	nrf_gpio_pin_set(col1);
	nrf_gpio_pin_set(col2);
	nrf_gpio_pin_set(col3);
	nrf_gpio_pin_set(col4);
	nrf_gpio_pin_set(col5);
}

void off()
{
  nrf_gpio_pin_set(col1);
	nrf_gpio_pin_set(col2);
	nrf_gpio_pin_set(col3);
	nrf_gpio_pin_set(col4);
	nrf_gpio_pin_set(col5);
}

void col1_on()
{
	nrf_gpio_pin_clear(col1);
	nrf_gpio_pin_set(col2);
	nrf_gpio_pin_set(col3);
	nrf_gpio_pin_set(col4);
	nrf_gpio_pin_set(col5);
}

void col2_on()
{
  nrf_gpio_pin_set(col1);
	nrf_gpio_pin_clear(col2);
	nrf_gpio_pin_set(col3);
	nrf_gpio_pin_set(col4);
	nrf_gpio_pin_set(col5);
}

void col3_on()
{
	nrf_gpio_pin_set(col1);
	nrf_gpio_pin_set(col2);
	nrf_gpio_pin_clear(col3);
	nrf_gpio_pin_set(col4);
	nrf_gpio_pin_set(col5);
}

void col4_on()
{
  nrf_gpio_pin_set(col1);
	nrf_gpio_pin_set(col2);
	nrf_gpio_pin_set(col3);
	nrf_gpio_pin_clear(col4);
	nrf_gpio_pin_set(col5);
}

void col5_on()
{
  nrf_gpio_pin_set(col1);
	nrf_gpio_pin_set(col2);
	nrf_gpio_pin_set(col3);
	nrf_gpio_pin_set(col4);
	nrf_gpio_pin_clear(col5);
}

void set_row1()
{
    if(led1 != 1)
    {
        nrf_gpio_pin_clear(row1);
    }
    else
    {
        nrf_gpio_pin_set(row1);
    }
    
    if(led2 != 1)
    {
        nrf_gpio_pin_clear(row2);
    }
    else
    {
        nrf_gpio_pin_set(row2);
    }
    
    if(led3 != 1)
    {
        nrf_gpio_pin_clear(row3);
    }
    else
    {
        nrf_gpio_pin_set(row3);
    }
    
    if(led4 != 1)
    {
        nrf_gpio_pin_clear(row4);
    }
    else
    {
        nrf_gpio_pin_set(row4);
    }
    
    if(led5 != 1)
    {
        nrf_gpio_pin_clear(row5);
    }
    else
    {
        nrf_gpio_pin_set(row5);
    }
		if(led6 != 1)
    {
        nrf_gpio_pin_clear(row6);
    }
    else
    {
        nrf_gpio_pin_set(row6);
    }
}

void set_row2()
{
    if(led11 != 1)
    {
        nrf_gpio_pin_clear(row1);
    }
    else
    {
        nrf_gpio_pin_set(row1);
    }
    
    if(led12 != 1)
    {
        nrf_gpio_pin_clear(row2);
    }
    else
    {
        nrf_gpio_pin_set(row2);
    }
    
    if(led13 != 1)
    {
        nrf_gpio_pin_clear(row3);
    }
    else
    {
        nrf_gpio_pin_set(row3);
    }
    
    if(led14 != 1)
    {
        nrf_gpio_pin_clear(row4);
    }
    else
    {
        nrf_gpio_pin_set(row4);
    }
    
    if(led15 != 1)
    {
        nrf_gpio_pin_clear(row5);
    }
    else
    {
        nrf_gpio_pin_set(row5);
    }
		if(led16 != 1)
    {
        nrf_gpio_pin_clear(row6);
    }
    else
    {
        nrf_gpio_pin_set(row6);
    }
}

void set_row3()
{
    if(led21 != 1)
    {
        nrf_gpio_pin_clear(row1);
    }
    else
    {
        nrf_gpio_pin_set(row1);
    }
    
    if(led22 != 1)
    {
        nrf_gpio_pin_clear(row2);
    }
    else
    {
        nrf_gpio_pin_set(row2);
    }
    
    if(led23 != 1)
    {
        nrf_gpio_pin_clear(row3);
    }
    else
    {
        nrf_gpio_pin_set(row3);
    }
    
    if(led24 != 1)
    {
        nrf_gpio_pin_clear(row4);
    }
    else
    {
        nrf_gpio_pin_set(row4);
    }
    
    if(led25 != 1)
    {
        nrf_gpio_pin_clear(row5);
    }
    else
    {
        nrf_gpio_pin_set(row5);
    }
		if(led26 != 1)
    {
        nrf_gpio_pin_clear(row6);
    }
    else
    {
        nrf_gpio_pin_set(row6);
    }
}

void set_row4()
{
    if(led31 != 1)
    {
        nrf_gpio_pin_clear(row1);
    }
    else
    {
        nrf_gpio_pin_set(row1);
    }
    
    if(led32 != 1)
    {
        nrf_gpio_pin_clear(row2);
    }
    else
    {
        nrf_gpio_pin_set(row2);
    }
    
    if(led33 != 1)
    {
        nrf_gpio_pin_clear(row3);
    }
    else
    {
        nrf_gpio_pin_set(row3);
    }
    
    if(led34 != 1)
    {
        nrf_gpio_pin_clear(row4);
    }
    else
    {
        nrf_gpio_pin_set(row4);
    }
    
    if(led35 != 1)
    {
        nrf_gpio_pin_clear(row5);
    }
    else
    {
        nrf_gpio_pin_set(row5);
    }
		if(led36 != 1)
    {
        nrf_gpio_pin_clear(row6);
    }
    else
    {
        nrf_gpio_pin_set(row6);
    }
}

void set_row5()
{
    if(led41 != 1)
    {
        nrf_gpio_pin_clear(row1);
    }
    else
    {
        nrf_gpio_pin_set(row1);
    }
    
    if(led42 != 1)
    {
        nrf_gpio_pin_clear(row2);
    }
    else
    {
        nrf_gpio_pin_set(row2);
    }
    
    if(led43 != 1)
    {
        nrf_gpio_pin_clear(row3);
    }
    else
    {
        nrf_gpio_pin_set(row3);
    }
    
    if(led44 != 1)
    {
        nrf_gpio_pin_clear(row4);
    }
    else
    {
        nrf_gpio_pin_set(row4);
    }
    
    if(led45 != 1)
    {
        nrf_gpio_pin_clear(row5);
    }
    else
    {
        nrf_gpio_pin_set(row5);
    }
		if(led46 != 1)
    {
        nrf_gpio_pin_clear(row6);
    }
    else
    {
        nrf_gpio_pin_set(row6);
    }
}

void set_row6()
{
    if(led6 != 1)
    {
        nrf_gpio_pin_clear(row1);
    }
    else
    {
        nrf_gpio_pin_set(row1);
    }
    
    if(led16 != 1)
    {
        nrf_gpio_pin_clear(row2);
    }
    else
    {
        nrf_gpio_pin_set(row2);
    }
    
    if(led26 != 1)
    {
        nrf_gpio_pin_clear(row3);
    }
    else
    {
        nrf_gpio_pin_set(row3);
    }
    
    if(led36 != 1)
    {
        nrf_gpio_pin_clear(row4);
    }
    else
    {
        nrf_gpio_pin_set(row4);
    }
    
    if(led46 != 1)
    {
        nrf_gpio_pin_clear(row5);
    }
    else
    {
        nrf_gpio_pin_set(row5);
    }
}

/*******************************************************************************************/


void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;
		ble_advdata_t advdata;
    ble_advdata_t scanrsp;

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


/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_nus    Nordic UART Service structure.
 * @param[in] p_data   Data to be send to UART module.
 * @param[in] length   Length of the data.
 */
/**@snippet [Handling the data received over BLE] */
static void nus_data_handler(ble_nus_t * p_nus, uint8_t * p_data, uint16_t length)
{
    /*for (uint32_t i = 0; i < length; i++)
    {
        while(app_uart_put(p_data[i]) != NRF_SUCCESS);
    }
    while(app_uart_put('\n') != NRF_SUCCESS);*/
	data_received = true;
	all_clear();
	/** get byte 1 **/
	if (length > 0) {
            switch (p_data[0]) {
            case 0:
                off();
                break;
            default:
                match(p_data[0]);
                break;
            }
        }
	/** get byte 2 **/	
	if (length > 1) {
            switch (p_data[1]) {
            case 0:
                off();
                break;
            default:
                match(p_data[1]);
                break;
            }
        }
	/** get byte 3 **/	
	if (length > 2) {
            switch (p_data[2]) {
            case 0:
                off();
                break;
            default:
                match(p_data[2]);
                break;
            }
        }
	/** get byte 4 **/	
	if (length > 3) {
            switch (p_data[3]) {
            case 0:
                off();
                break;
            default:
                match(p_data[3]);
                break;
            }
        }
	/** get byte 5 **/	
	if (length > 4) {
            switch (p_data[4]) {
            case 0:
                off();
                break;
            default:
                match(p_data[4]);
                break;
            }
        }
	/** get byte 6 **/	
	if (length > 5) {
            switch (p_data[5]) {
            case 0:
                off();
                break;
            default:
                match(p_data[5]);
                break;
            }
        }
	/** get byte 7 **/	
	if (length > 6) {
            switch (p_data[6]) {
            case 0:
                off();
                break;
            default:
                match(p_data[6]);
                break;
            }
        }
	/** get byte 8 **/	
	if (length > 7) {
            switch (p_data[7]) {
            case 0:
                off();
                break;
            default:
                match(p_data[7]);
                break;
            }
        }
	/** get byte 9 **/	
	if (length > 8) {
            switch (p_data[8]) {
            case 0:
                off();
                break;
            default:
                match(p_data[8]);
                break;
            }
        }
	/** get byte 10 **/	
	if (length > 9) {
            switch (p_data[9]) {
            case 0:
                off();
                break;
            default:
                match(p_data[9]);
                break;
            }
        }
	/** get byte 11 **/	
	if (length > 10) {
            switch (p_data[10]) {
            case 0:
                off();
                break;
            default:
                match(p_data[10]);
                break;
            }
        }
	/** get byte 12 **/	
	if (length > 11) {
            switch (p_data[11]) {
            case 0:
                off();
                break;
            default:
                match(p_data[11]);
                break;
            }
        }
	/** get byte 13 **/	
	if (length > 12) {
            switch (p_data[12]) {
            case 0:
                off();
                break;
            default:
                match(p_data[12]);
                break;
            }
        }
	/** get byte 14 **/	
	if (length > 13) {
            switch (p_data[13]) {
            case 0:
                off();
                break;
            default:
                match(p_data[13]);
                break;
            }
        }
	/** get byte 15 **/	
	if (length > 14) {
            switch (p_data[14]) {
            case 0:
                off();
                break;
            default:
                match(p_data[14]);
                break;
            }
        }
	/** get byte 16 **/	
	if (length > 15) {
            switch (p_data[15]) {
            case 0:
                off();
                break;
            default:
                match(p_data[15]);
                break;
            }
        }
		
}
/**@snippet [Handling the data received over BLE] */


static void services_init(void)
{
    uint32_t       err_code;
    ble_nus_init_t nus_init;
    
    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;
    
    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
}


static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;
    
    if(p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


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


static void sleep_mode_enter(void)
{
    uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}

void shut_down(void)
{
	sleep_mode_enter();
}

void check_battery_status(void)
{
			/** check battery status after each 5 sec **/
			static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
			uint8_t level = battery_level_get();
			if(level < 80)
			{
				if(level < 75)
				{
					uint8_t index = 0;
					data_array[index] = 'S';
					index++;
					data_array[index] = 'H';
					index++;
					data_array[index] = 'U';
					index++;
					data_array[index] = 'T';
					index++;
					data_array[index] = 'D';
					index++;
					data_array[index] = 'O';
					index++;
					data_array[index] = 'W';
					index++;
					data_array[index] = 'N';
					index++;
					data_array[index] = '\n';
					uint32_t       err_code;
					err_code = ble_nus_string_send(&m_nus, data_array, index);
					if (err_code != NRF_ERROR_INVALID_STATE)
					{
						APP_ERROR_CHECK(err_code);
					}
					nrf_delay_ms(500);
					shut_down();
				}
				else
				{
					uint8_t index = 0;
					data_array[index] = 'L';
					index++;
					data_array[index] = 'O';
					index++;
					data_array[index] = 'W';
					index++;
					data_array[index] = '\n';
					uint32_t       err_code;
					err_code = ble_nus_string_send(&m_nus, data_array, index);
					if (err_code != NRF_ERROR_INVALID_STATE)
					{
						APP_ERROR_CHECK(err_code);
					}
				}
			}
}

/******************************************************************************************/


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
						leds_off();																												/** switch off all the LEDs before going to sleep **/
            sleep_mode_enter();
            break;
        default:
            break;
    }
}


static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t                         err_code;
    
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
						connected = true;
						led1 = 1;
						led2 = 1;
						led3 = 1;
						led4 = 1;
						led5 = 1;
						led6 = 1;
            break;
            
        case BLE_GAP_EVT_DISCONNECTED:
            err_code = bsp_indication_set(BSP_INDICATE_IDLE);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
						connected = false;
						leds_off();
						sleep_mode_enter();
            break;

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

        default:
            // No implementation needed.
            break;
    }
}


static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    ble_conn_params_on_ble_evt(p_ble_evt);
    ble_nus_on_ble_evt(&m_nus, p_ble_evt);
    on_ble_evt(p_ble_evt);
    ble_advertising_on_ble_evt(p_ble_evt);
    bsp_btn_ble_on_ble_evt(p_ble_evt);
    
}


static void ble_stack_init(void)
{
    uint32_t err_code;
    
    // Initialize SoftDevice.
		SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_RC_250_PPM_4000MS_CALIBRATION, NULL); /** uncomment this line to use internal crystal(32kHz) **/
    //SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM, NULL);

    // Enable BLE stack.
    ble_enable_params_t ble_enable_params;
    memset(&ble_enable_params, 0, sizeof(ble_enable_params));
#if (defined(S130) || defined(S132))
    ble_enable_params.gatts_enable_params.attr_tab_size   = BLE_GATTS_ATTR_TAB_SIZE_DEFAULT;
#endif
    ble_enable_params.gatts_enable_params.service_changed = IS_SRVC_CHANGED_CHARACT_PRESENT;
    err_code = sd_ble_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);
    
    // Subscribe for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
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
					if(leds_running == true)
					{
						leds_off();
					}
						sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BSP_EVENT_WHITELIST_OFF:
            err_code = ble_advertising_restart_without_whitelist();
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        default:
            break;
    }
}


void uart_event_handle(app_uart_evt_t * p_event)
{
    static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
    static uint8_t index = 0;
    uint32_t       err_code;

    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
            UNUSED_VARIABLE(app_uart_get(&data_array[index]));
            index++;

            if ((data_array[index - 1] == '\n') || (index >= (BLE_NUS_MAX_DATA_LEN)))
            {
                err_code = ble_nus_string_send(&m_nus, data_array, index);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
                
                index = 0;
            }
            break;

        case APP_UART_COMMUNICATION_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}

static void uart_init(void)
{
    uint32_t                     err_code;
    const app_uart_comm_params_t comm_params =
    {
        RX_PIN_NUMBER,
        TX_PIN_NUMBER,
        RTS_PIN_NUMBER,
        CTS_PIN_NUMBER,
        APP_UART_FLOW_CONTROL_ENABLED,
        false,
        UART_BAUDRATE_BAUDRATE_Baud38400
    };

    APP_UART_FIFO_INIT( &comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOW,
                       err_code);
    APP_ERROR_CHECK(err_code);
}


static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;
    ble_advdata_t scanrsp;

    // Build advertising data struct to pass into @ref ble_advertising_init.
    memset(&advdata, 0, sizeof(advdata));
    advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance = false;
    advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;

    memset(&scanrsp, 0, sizeof(scanrsp));
    scanrsp.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    scanrsp.uuids_complete.p_uuids  = m_adv_uuids;

    ble_adv_modes_config_t options = {0};
    options.ble_adv_fast_enabled  = BLE_ADV_FAST_ENABLED;
    options.ble_adv_fast_interval = APP_ADV_INTERVAL;
    options.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = ble_advertising_init(&advdata, &scanrsp, &options, on_adv_evt, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    bsp_event_t startup_event;

    uint32_t err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS,
                                 APP_TIMER_TICKS(100, APP_TIMER_PRESCALER), 
                                 bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


/**@brief Function for placing the application in low power state while waiting for events.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}


/**@brief Application main function.
 */
int main(void)
{
    uint32_t err_code;
    bool erase_bonds;
    uint8_t  start_string[] = START_STRING;
	
    // Initialize.
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);
    //uart_init();
    buttons_leds_init(&erase_bonds);
		gpio_output();
    ble_stack_init();
    gap_params_init();
    services_init();
    advertising_init();
    conn_params_init();
    //printf("%s",start_string);
    err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
		power_manage();
		all_clear();
		leds_off();
		nrf_gpio_pin_set(row1);
		nrf_gpio_pin_set(row2);
		nrf_gpio_pin_set(row3);
		nrf_gpio_pin_set(row4);
		nrf_gpio_pin_set(row5);
		nrf_gpio_pin_set(row6);
    int x = 2;
		int count = 0;
while(1)
{
if(connected == false)
{
		leds_running = true;
    nrf_gpio_pin_clear(col1);
    nrf_delay_ms(300);
    nrf_gpio_pin_set(col1);
    nrf_gpio_pin_clear(col2);
    nrf_delay_ms(300);
    nrf_gpio_pin_set(col2);
    nrf_gpio_pin_clear(col3);
    nrf_delay_ms(300);
    nrf_gpio_pin_set(col3);
    nrf_gpio_pin_clear(col4);
    nrf_delay_ms(300);
    nrf_gpio_pin_set(col4);
    nrf_gpio_pin_clear(col5);
    nrf_delay_ms(300);
    nrf_gpio_pin_set(col5);
    x--;
}

if(connected == true)
{
	if(data_received == false)
	{
		nrf_gpio_pin_clear(col1);
		nrf_gpio_pin_clear(col2);
		nrf_gpio_pin_clear(col3);
		nrf_gpio_pin_clear(col4);
		nrf_gpio_pin_clear(col5);
		nrf_gpio_pin_clear(row1);
		nrf_gpio_pin_clear(row2);
		nrf_gpio_pin_clear(row3);
		nrf_gpio_pin_clear(row4);
		nrf_gpio_pin_clear(row5);
		nrf_gpio_pin_clear(row6);
		leds_running = true;
    nrf_gpio_pin_set(row1);
    nrf_delay_ms(300);
    nrf_gpio_pin_clear(row1);
    nrf_gpio_pin_set(row2);
    nrf_delay_ms(300);
    nrf_gpio_pin_clear(row2);
    nrf_gpio_pin_set(row3);
    nrf_delay_ms(300);
    nrf_gpio_pin_clear(row3);
    nrf_gpio_pin_set(row4);
    nrf_delay_ms(300);
    nrf_gpio_pin_clear(row4);
    nrf_gpio_pin_set(row5);
    nrf_delay_ms(300);
    nrf_gpio_pin_clear(row5);
		nrf_gpio_pin_set(row6);
		nrf_delay_ms(300);
    nrf_gpio_pin_clear(row6);
		check_battery_status();
    x--;
	}
	if(data_received == true)
	{
			nrf_delay_ms(1);
			//nrf_delay_us(100);
      off();
			//power_manage();
      switch(next)
      {
        case 0:
        //off();
        set_row1();
        col1_on();
        next = 1;
        //off();
        break;
        
        case 1:
        //off();
        set_row2();
        col2_on();
        next = 2;
        //off();
        break;
        
        case 2:
        //off();
        set_row3();
        col3_on();
        next = 3;
        //off();
        break;
        
        case 3:
        //off();
        set_row4();
        col4_on();
        next = 4;
        //off();
        break;
        
        case 4:
        //off();
        set_row5();
        col5_on();
        next = 0;
        //off();
        break;
      }
		}
		
		count ++;
		if(count == 2000)
		{
			check_battery_status();
			count = 0;
		}
	}
}
}


/** 
 * @}
 */
