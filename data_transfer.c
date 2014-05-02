/*
* Copyright (c) 2013 Sherwin_Yan. All Rights Reserved.
* This file is dealing data transfer for W1-project . 
* main.c will call transfer_config and transfer_data .
*/

#include <stdint.h>
#include <string.h>
#include "ble.h"
#include "rtc.h"
#include "data_transfer.h"
//#include "ble_W1_pins.h"
#include "ble_hci.h"
#include "app_timer.h"
#include "ble_flash.h"
#include "ble_advdata.h"
#include "ble_srv_common.h"
#include "ble_conn_params.h"
#include "softdevice_handler.h"
#include "step.h"
#include "main.h"

#define DEVICE_NAME                     "W1_STEPS_TEST"                           	/**< Name of device. Will be included in the advertising data. */

// YOUR_JOB: Modify these according to requirements.
#define APP_TIMER_PRESCALER             0                                           /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS            4                                           /**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE         5                                           /**< Size of timer operation queues. */

#define MIN_CONN_INTERVAL               16            //seattle-test why change to this value                              /**< Minimum acceptable connection interval (7.5 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               32                                          /**< Maximum acceptable connection interval (7.5 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                400                                         /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(20000, APP_TIMER_PRESCALER) /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (15 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)  /**< Time between each call to sd_ble_gap_conn_param_update after the first (5 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */


#define SEC_PARAM_TIMEOUT               30                                          /**< Timeout for Pairing Request or Security Request (in seconds). */
#define SEC_PARAM_BOND                  1                                           /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                           /**< Man In The Middle protection not required. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                        /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                           /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                           /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                          /**< Maximum encryption key size. */

#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      180                                         /**< The advertising timeout (in units of seconds). */

static ble_gap_sec_params_t             m_sec_params;                               /**< Security requirements for this application. */
static uint16_t                         m_conn_handle = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */
extern ble_nus_t                        m_nus;	
/**@brief GAP initialization.
 *
 * @details This function shall be used to setup all the necessary GAP (Generic Access Profile) 
 *          parameters of the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
    
    err_code = sd_ble_gap_device_name_set(&sec_mode, (const uint8_t *) DEVICE_NAME, strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


void uart_data_handler(ble_nus_t * p_nus, uint8_t * data, uint16_t length)
{
    data[length] = '\0';
    W1_PRINT(data);
    W1_PRINT("\n");
		on_command_handler((char *)data);
}

/**@brief Initialize services that will be used by the application.
 */
static void transfer_services_init(void)
{
    uint32_t err_code;
    static ble_nus_init_t nus_init;
    
    memset(&nus_init, 0, sizeof nus_init);
    nus_init.data_handler = uart_data_handler;
    
    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
}

/**@brief Advertising functionality initialization.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;
    ble_advdata_t scanrsp;
    uint8_t       flags = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;
    
    ble_uuid_t adv_uuids[] = {{BLE_UUID_NUS_SERVICE, m_nus.uuid_type}};

    memset(&advdata, 0, sizeof(advdata));
    advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance      = false;
    advdata.flags.size              = sizeof(flags);
    advdata.flags.p_data            = &flags;

    memset(&scanrsp, 0, sizeof(scanrsp));
    scanrsp.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
    scanrsp.uuids_complete.p_uuids  = adv_uuids;
    
    err_code = ble_advdata_set(&advdata, &scanrsp);
    APP_ERROR_CHECK(err_code);
}

/**@brief Connection Parameters Module handler.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in]   p_evt   Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;
    
    if(p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}

/**@brief Connection Parameters module error handler.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Initialize the Connection Parameters module.
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

/**@brief Initialize security parameters.
 */
static void sec_params_init(void)
{
    m_sec_params.timeout      = SEC_PARAM_TIMEOUT;
    m_sec_params.bond         = SEC_PARAM_BOND;
    m_sec_params.mitm         = SEC_PARAM_MITM;
    m_sec_params.io_caps      = SEC_PARAM_IO_CAPABILITIES;
    m_sec_params.oob          = SEC_PARAM_OOB;  
    m_sec_params.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
    m_sec_params.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
}

/**@brief Start advertising.
 */
static void advertising_start(void)
{
    uint32_t             err_code;
    ble_gap_adv_params_t adv_params;
    
    // Start advertising
    memset(&adv_params, 0, sizeof(adv_params));
    
    adv_params.type        = BLE_GAP_ADV_TYPE_ADV_IND;
    adv_params.p_peer_addr = NULL;
    adv_params.fp          = BLE_GAP_ADV_FP_ANY;
    adv_params.interval    = APP_ADV_INTERVAL;
    adv_params.timeout     = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = sd_ble_gap_adv_start(&adv_params);
    APP_ERROR_CHECK(err_code);
    //nrf_gpio_pin_set(ADVERTISING_LED_PIN_NO);
}


bool transfer_config(void)
{
	bool ret = true;
	
	gap_params_init();
	transfer_services_init();
  advertising_init();
  conn_params_init();
  sec_params_init();
	
	return ret;
}

bool transfer_start(void)
{
	bool ret = true;
	
	advertising_start();
	
	return ret;
}


bool W1_transfer_data(struct S_device_config* device_config_transfer, uint8_t TransMode)
{
	uint32_t buffer_page[256];
	char str[50];
	int i;
	char *tmp;
	
	tmp = (char *)buffer_page;
	// Transfer next package
	if(TransMode == TRANS_NEXT)
	{
		device_config_transfer->error_times = 0;
		// One page is over, start another page
		if(device_config_transfer->word_offset == 0){	
			// All page is over, set transfer over flag
			if(device_config_transfer->page_now == device_config_transfer->page_end){
				device_config_transfer->info_changed |= BIT_DATA_TRANS_OVER;			
			}
			else{
				device_config_transfer->page_now++;
				device_config_transfer->word_offset = W1_MAX_WORD_OFFSET;
			}
		}
		if(!(device_config_transfer->info_changed & BIT_DATA_TRANS_OVER)){
			device_config_transfer->word_offset -= (STORAGE_BUFFER_LENGTH/4);
		}
	}
	// Transfer last package
	else if(TransMode == TRANS_LAST){
		device_config_transfer->error_times ++;
		if(device_config_transfer->error_times >= 3)
		{
			device_config_transfer->info_changed |= BIT_ERROR_OCCURED;	
		}
	}

	W1_flash_words_read2( device_config_transfer->page_now, buffer_page, device_config_transfer->word_offset, STORAGE_BUFFER_LENGTH/4);
	//seattle-debug >>>
	sprintf(str,"page now:%02x ,word offset:%02x \n", device_config_transfer->page_now, device_config_transfer->word_offset);
	W1_DEBUG(str);
	for(i=0; i<40; i++){
		sprintf(str,"%02x ",*(tmp+i));
		W1_DEBUG(str);
	}
	//seattle-debug <<<
	W1_DEBUG("\ntransfer one package data begin \n");
	ble_transfer_data((uint8_t *)buffer_page,20);
	ble_transfer_data((uint8_t *)buffer_page+20,20);
	W1_DEBUG("transfer one package data end \n");
	
	return true;
}

/*
* here to deal with command 
* the command list is as follow :
*
* identify
* user_init
*	account
*	password
*	transfer_start
*	transfer_end
*	transfer_fail
*	time_read
*	time_set
*
*/

/*seattle-test not used
int never_used(void)
{
	int ret = 0;
	
	return ret;
}

int first_init(void)
{
	int ret = 0;
	
	return ret;
}



bool transfer_identify(void)
{
	bool ret = true;
	
	return ret;
}

bool transfer_data(uint8_t* page_num,uint8_t* end_roll)
{
	uint32_t buffer_page[256];
	uint8_t word_count;
	
	uint8_t high_position,low_position;
	
	W1_DEBUG("transfer data begin \n");
	ble_flash_page_read(*page_num,buffer_page,&word_count);
	W1_DEBUG("transfer data end \n");
	ble_transfer_data((uint8_t *)buffer_page,20);
	ble_transfer_data((uint8_t *)buffer_page+20,20);
	
	get_storage_range(&high_position,&low_position);
	if((*page_num == low_position)&&(*end_roll))
	{
		*page_num = high_position;
		*end_roll = 0;
	}else if(*page_num > low_position)
	{
		*page_num = *page_num - 1;
	}
	
	return true;
}



bool transfer_end(void)
{
	bool ret = true;
	
	return ret;
}

int account_check(void)
{
	int ret = 0;
	
	if(never_used())
	{
		first_init();
	}
	
	return ret;
}


void transfer_test(void)
{
	uint32_t buffer_page[256];
	uint8_t word_count;
	
	W1_DEBUG("transfer test begin \n");
	ble_flash_page_read(PAGE_SIZE_END,buffer_page,&word_count);
	W1_DEBUG("transfer test end \n");
	ble_transfer_data((uint8_t *)buffer_page,20);
	ble_transfer_data((uint8_t *)buffer_page+20,20);
	
}
*/
