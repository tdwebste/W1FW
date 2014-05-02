/* 
 * Copyright (c) 2013 W1FW. All Rights Reserved.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */
 
/*****************************************************************************
* Head file
*****************************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "nrf.h"
#include "nrf_gpio.h"
#include "nrf_soc.h"
#include "ble_conn_params.h"
#include "ble_nus.h"
#include "ble_dis.h"
#include "ble_bas.h"
#include "ble_debug_assert_handler.h"
#include "ble_flash.h"
#include "pstorage.h"
#include "app_timer.h"
#include "nordic_common.h"
#include "softdevice_handler.h"
#include "pca10001.h"
#include "nrf51_bitfields.h"

#include "main.h"
#include "sensor.h"
#include "step.h"
#include "rtc.h"
#include "data_transfer.h"


/*****************************************************************************
* Macro definition
*****************************************************************************/
#define DEAD_BEEF                       0xDEADBEEF 


#define APP_TIMER_PRESCALER                 0                                         /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS                4                                         /**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE             5                                         /**< Size of timer operation queues. */

#define SENSOR_ADC_MEAS_INTERVAL        		APP_TIMER_TICKS(30, APP_TIMER_PRESCALER)	/**< sensor ADC measurement interval (ticks). */
#define BATTERY_LEVEL_MEAS_INTERVAL         APP_TIMER_TICKS(2000, APP_TIMER_PRESCALER)/**< Battery level measurement interval (ticks). */
#define STEP_LEVEL_MEAS_INTERVAL         		APP_TIMER_TICKS(3000, APP_TIMER_PRESCALER)/**< Battery level measurement interval (ticks). */
#define MANUFACTURER_NAME                   "W1FW"                     								/**< Manufacturer. Will be passed to Device Information Service. */

#define APP_GPIOTE_MAX_USERS            1  
#define APP_ADV_INTERVAL                64                                          	/**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */

#define ACK_ID_INIT											"ACK-ID-INIT"
#define ACK_ID_AGREE										"ACK-ID-AGREE"
#define ACK_ID_DENT											"ACK-ID-DENT"

#define	UNACTIVE_DEVICE_ENABLE								0																				/*Unactive device, only for debug use!*/
#define ENABLE_UART_FUNCTION									1																				/*Enable uart function, only for debug use!*/
/*****************************************************************************
* Global variable
*****************************************************************************/
static ble_gap_sec_params_t             m_sec_params;                               	/**< Security requirements for this application. */
static uint16_t                         m_conn_handle = BLE_CONN_HANDLE_INVALID;    	/**< Handle of the current connection. */
static app_timer_id_t                   m_sensor_timer_id; 
static app_timer_id_t                   m_battery_timer_id; 
static app_timer_id_t                   m_step_timer_id;
ble_bas_t                               m_bas;                                     		/**< Structure used to identify the battery service. */
ble_nus_t                        				m_nus;																				/**< Structure to identify the Nordic UART Service. */

struct S_time time;
struct S_step step;
struct S_step_buffer step_buffer;
struct S_step_count step_count;
//struct S_data_storage data_storage;
struct S_device_config *device_config_main = NULL;

bool power_off_flag;
bool permission_agree;
bool normal_time_flag;
bool real_time_flag;
bool allow_transfer_flag = true;
uint8_t next_transfer_page = 127;
uint8_t last_transfer_page = 127;
uint8_t T_end_roll = 0;

static void ble_evt_dispatch(ble_evt_t * p_ble_evt);
static void sys_evt_dispatch(uint32_t sys_evt);
static void transfer_mode_run(void);
static void power_manage(void);
/*****************************************************************************
* Error Handling Functions
*****************************************************************************/

/**@brief Error handler function, which is called when an error has occurred. 
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze 
 *          how your product is supposed to react in case of error.
 *
 * @param[in] error_code  Error code supplied to the handler.
 * @param[in] line_num    Line number where the handler is called.
 * @param[in] p_file_name Pointer to the file name. 
 */
void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{
    nrf_gpio_pin_set(ASSERT_LED_PIN_NO);		/*seattle-test	this function need add*/

    // This call can be used for debug purposes during development of an application.
    // @note CAUTION: Activating this code will write the stack to flash on an error.
    //                This function should NOT be used in a final product.
    //                It is intended STRICTLY for development/debugging purposes.
    //                The flash write will happen EVEN if the radio is active, thus interrupting
    //                any communication.
    //                Use with care. Un-comment the line below to use.
    ble_debug_assert_handler(error_code, line_num, p_file_name);

    // On assert, the system can only recover with a reset.
    //NVIC_SystemReset();				/*seattle-test need open*/
}


/** @brief Function for sending ' Exit!' string to UART.
Execution is blocked until UART peripheral detects all characters have been sent.
 */
static __INLINE void uart_quit()
{
  W1_DEBUG(" \n\rExit!\n\r");
}


/** @brief Function for sending 'Start: ' string to UART.
Execution is blocked until UART peripheral detects all characters have been sent.
 */
static __INLINE void uart_start()
{
  W1_DEBUG(" \n\rStart: this is project W1 \n");
}


/*
* if try_count get to too large , we will say it goes timeout
*/
void check_timeout(bool timeout)
{
  while(timeout)
  {
    uint8_t cr = simple_uart_get();
    simple_uart_put(cr);

    if(cr == 'q' || cr == 'Q')
    {
      uart_quit();
      while(1){}
    }
  }
}


// add by sherwin for number print
void print_error_code(uint32_t error_code)
{
	char str[8];
	W1_DEBUG("ERROR:0x");
	sprintf(str,"%08x",error_code);
	W1_DEBUG(str);
	W1_DEBUG("\n");
}


// no range detect , so please try to use strncpy first
void array_ncpy(char *array1,char *array2,uint8_t num)
{
	uint8_t i = 0;
	for(i = 0; i < num; i++)
	{
		array1[i]=array2[i];
	}
}

void ble_transfer_data(uint8_t * data, uint16_t length)
{
    uint32_t err_code;
	
    err_code = ble_nus_send_string(&m_nus, data, length);
    if (err_code != NRF_ERROR_INVALID_STATE)
    {
        APP_ERROR_CHECK(err_code);
    }		
}


/*
* In this funtion , wakeup because of button should be not concerned as first start
*/
/*seattle-test
static int is_first_start()
{
	int ret = 0;
	
	return ret;
}
seattle-test*/
/*  this function is called when ADC get its data 
*   here we can transfer data either by uart or ble
*/
void check_step(uint16_t data)
{
	
  //char str[20];
	uint8_t step_true = 1;

	step.is_step = false;
	step.current_voltage = data;
	
	// Check if a step is complete and update the direction
	if(!is_step2(&step, &step_count)&&(step.is_step))
	{
		if(!real_time_flag){
			W1_DEBUG("step count++ \n");
			if(step_count.step_count < 0xFF)
				step_count.step_count ++;
			else
				step_count.step_count=0xFF;
		}
		else
		{
			W1_DEBUG("ble realtime\n");
			ble_transfer_data(&step_true, 1);
		}
		
	}
	//get_time(step_count.now_time);
	//sprintf(str,"\n\rd:%d",data);
	//W1_SENSOR_DEBUG(str);
}

void check_step2(int LpcompResult)
{
	char str[20];
	//uint8_t tmpstep[2];
	//sprintf(str,"result: %d\n", LpcompResult);
	//W1_DEBUG(str);
	step.current_voltage = LpcompResult;
	if((!step.current_voltage) && step.last_voltage){
		W1_DEBUG("step count++ \n");
		step_count.step_count ++;
		//tmpstep[0] = step_count.step_count;
		if(permission_agree)
		{	W1_DEBUG("ble transfer \n");
			//ble_transfer_data(tmpstep, 1);
		}
		sprintf(str,"count: %d\n", step_count.step_count);
		W1_DEBUG(str);
	}
	step.last_voltage = step.current_voltage;
	
}


/*****************************************************************************
* Static Timeout Handling Functions
*****************************************************************************/

/**@brief Function for handling the Battery measurement timer timeout.
 *
 * @details This function will be called each time the battery level measurement timer expires.
 *          This function will start the ADC.
 *
 * @param[in]   p_context   Pointer used for passing some arbitrary information (context) from the
 *                          app_start_timer() call to the timeout handler.
 */
static void battery_level_meas_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    battery_start();
}


/**@brief Function for handling the sensor measurement timer timeout.
 *
 * @details This function will be called each time the sensor voltage measurement timer expires.
 *          This function will start the ADC.
 *
 * @param[in]   p_context   Pointer used for passing some arbitrary information (context) from the
 *                          app_start_timer() call to the timeout handler.
 */
static void sensor_level_meas_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
		step_count.adc_count++;
		W1_SENSOR_DEBUG(" \n\rsensor adc handler");
    sensor_start();
		if(!real_time_flag){
			step_mode_run(&step_count,&step_buffer);
	
			if(step_count.switch_mode == DATA_TRANSFERER_MODE &&  allow_transfer_flag){
				transfer_mode_run();
				allow_transfer_flag = false;
			}
		}
		power_manage();
}

static void step_level_meas_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
	
		if(step_count.step_count==0)
		{
			step_count.count_mul ++;
			if(step_count.count_mul >= 60){
				W1_DEBUG("it will shutdown since not running in 3 minutes\n");
				power_off_flag = true;
			}
		}
		else
		{
			if((step_count.step_count >= 10) && (step_count.switch_mode == STEP_COUNTER_MODE)){
				W1_DEBUG("LPCOMP transfer\n");
				step_count.switch_mode = DATA_TRANSFERER_MODE;
				transfer_mode_run();
			}
			step_count.count_mul = 0;
		}
		step_count.step_count = 0;
    
}
/**@brief Assert macro callback function.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze 
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/*****************************************************************************
* Static Initialization Functions
*****************************************************************************/

/**@brief Function for the Timer initialization.
 *
* @details Initializes the timer module. This creates and starts application timers.
*/
static void timers_init(void)
{
    uint32_t err_code;

    // Initialize timer module
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, false);

    // Create timers
    err_code = app_timer_create(&m_sensor_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                sensor_level_meas_timeout_handler);
		APP_ERROR_CHECK(err_code);
	
    err_code = app_timer_create(&m_battery_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                battery_level_meas_timeout_handler);	
    APP_ERROR_CHECK(err_code);
	
		err_code = app_timer_create(&m_step_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                step_level_meas_timeout_handler);
		APP_ERROR_CHECK(err_code);
}

/**@brief  Function for configuring the buttons.
 */
static void buttons_init(void)
{
		//seattle-test why need this
    nrf_gpio_cfg_sense_input(WAKEUP_BUTTON_PIN_NO,
                             BUTTON_PULL, 
                             NRF_GPIO_PIN_SENSE_LOW);    
}

/**@brief  Function for initializing the UART module.
 */
static void uart_init(void)
{
    simple_uart_config(RTS_PIN_NUMBER, TX_PIN_NUMBER, CTS_PIN_NUMBER, RX_PIN_NUMBER, HWFC);
}

void LPCOMP_COMP_IRQHandler(void)
{

	//LPCOMP handler
	if (NRF_LPCOMP->EVENTS_CROSS != 0)
	{
		//W1_DEBUG("LPCOMP IRQHandler \n");
		NRF_LPCOMP->EVENTS_CROSS = 0;
		
		// Trigering SAMPLE task to generate RESULT
		NRF_LPCOMP->TASKS_SAMPLE = 1;
		//check_step2(NRF_LPCOMP->RESULT);
  }

}

/**@brief  Function for initializing the LPCOMP module.
 */
static void LPCOMP_start(void)
{
	
	W1_DEBUG("lpcomp start \n");
	
	// Enable interrupt on LPCOMP CROSS event
	NRF_LPCOMP->INTENSET = LPCOMP_INTENSET_CROSS_Msk;
	NVIC_EnableIRQ(LPCOMP_COMP_IRQn);
	
	NRF_LPCOMP->PSEL = LPCOMP_PSEL_PSEL_AnalogInput2;
	NRF_LPCOMP->REFSEL = LPCOMP_REFSEL_REFSEL_SupplyFiveEighthsPrescaling;
	NRF_LPCOMP->ANADETECT = LPCOMP_ANADETECT_ANADETECT_Cross;
	
	// Enable and start the low power comparator
	NRF_LPCOMP->ENABLE = LPCOMP_ENABLE_ENABLE_Enabled;
  NRF_LPCOMP->TASKS_START = 1;

}
static void LPCOMP_end(void)
{
	// Enable and start the low power comparator
	NRF_LPCOMP->ENABLE = LPCOMP_ENABLE_ENABLE_Disabled;
  NRF_LPCOMP->TASKS_START = 0;
}
/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;
    
    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM, false);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);
    
    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}
//seattle-test need modify
void W1_softdevice_handler_sd_enable()
{		
		ble_stack_init();
}

/**@brief Function for initializing the services that will be used by the application.
 *
 * @details Initialize the Battery and Device Information services.
 */
static void services_init(void)
{
    uint32_t       err_code;
    ble_bas_init_t bas_init;
    ble_dis_init_t dis_init;


    // Initialize Battery Service
    memset(&bas_init, 0, sizeof(bas_init));

    // Here the sec level for the Battery Service can be changed/increased.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&bas_init.battery_level_char_attr_md.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_report_read_perm);

    bas_init.evt_handler          = NULL;
    bas_init.support_notification = true;
    bas_init.p_report_ref         = NULL;
    bas_init.initial_batt_level   = 100;

    err_code = ble_bas_init(&m_bas, &bas_init);
    APP_ERROR_CHECK(err_code);

    // Initialize Device Information Service
    memset(&dis_init, 0, sizeof(dis_init));

    ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, MANUFACTURER_NAME);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&dis_init.dis_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&dis_init.dis_attr_md.write_perm);

    err_code = ble_dis_init(&dis_init);
    APP_ERROR_CHECK(err_code);
}


/*****************************************************************************
* Static Start Functions
*****************************************************************************/
/**@brief Start timers.
*/
static void sensor_timers_start(void)
{
    uint32_t err_code;

		W1_DEBUG("sensor timers start\n");
    // Start application timers
    err_code = app_timer_start(m_sensor_timer_id, SENSOR_ADC_MEAS_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
	
		if(err_code==NRF_SUCCESS)
		{
			W1_DEBUG("no error\n");
		}
}

static void battery_timers_start(void)
{
    uint32_t err_code;

		W1_DEBUG("battery timers start\n");
    // Start application timers
    err_code = app_timer_start(m_battery_timer_id, BATTERY_LEVEL_MEAS_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
	
		if(err_code==NRF_SUCCESS)
		{
			W1_DEBUG("no error\n");
		}
}

static void step_timers_start(void)
{
    uint32_t err_code;

		W1_DEBUG("step timers start\n");
    // Start application timers
    err_code = app_timer_start(m_step_timer_id, STEP_LEVEL_MEAS_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
	
		if(err_code==NRF_SUCCESS)
		{
			W1_DEBUG("no error\n");
		}
}

static void sensor_timers_stop(void)
{
	uint32_t err_code;
	
	W1_DEBUG(" sensor timers stop\n");
	// Stop applicatin timers
	err_code = app_timer_stop(m_sensor_timer_id);
	APP_ERROR_CHECK(err_code);
	
	if(err_code == NRF_SUCCESS)
	{
		W1_DEBUG("no error\n");
	}
}

/**@brief Start advertising.
 */
/*seattle-test redefinition
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
    nrf_gpio_pin_set(ADVERTISING_LED_PIN_NO);
}
*/
void W1_flash_write_prepare(void)
{
		sensor_timers_stop();
  	softdevice_handler_sd_disable();
}

void W1_flash_write_restore(void)
{//W1_DEBUG("--14 \n");
		W1_softdevice_handler_sd_enable();	
	//W1_DEBUG("--15 \n");
		sensor_timers_start();
}

static void transfer_mode_run(void)
{
		W1_DEBUG("\n go to DATA_TRANSFERER_MODE \n");
		services_init();		//SZ0021
		sensor_timers_stop();
		transfer_config();
		transfer_start();
//seattle-test not used		transfer_identify();
//seattle-test not used		transfer_data(device_config_main);
//seattle-test not used		transfer_end();
}

/**@brief Function for putting the chip in System OFF Mode
 */
static void system_off_mode_enter(void)
{
    uint32_t err_code;
	
		// Update device info before power off	
		device_info_update(device_config_main, &step_count);		
	
		ble_stack_init();			//seattle-test why need this
	
		//ADC_Disable();
		LPCOMP_start();

	
		W1_DEBUG("going to system off! \n");
    // Configure buttons with sense level low as wakeup source.					//seattle-test why need this
	/*
    nrf_gpio_cfg_sense_input(WAKEUP_BUTTON_PIN_NO,
                             BUTTON_PULL,
                             NRF_GPIO_PIN_SENSE_LOW);
	*/
     //nrf_gpio_cfg_sense_input(ADC_SENSOR_PIN_NO,
     //                        NRF_GPIO_PIN_PULLUP,
     //                        NRF_GPIO_PIN_SENSE_LOW);        
    // Go to system-off mode (this function will not return; wakeup will cause a reset)
    err_code = sd_power_system_off(); 
    APP_ERROR_CHECK(err_code);
}

/**@brief Power manager.
 */
static void power_manage(void)
{
		if(power_off_flag)
		{
			system_off_mode_enter();
		}
		
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}

/*****************************************************************************
* Static Event Handling Functions
*****************************************************************************/

/**@brief Application's BLE Stack event handler.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t                         err_code = NRF_SUCCESS;
    static ble_gap_evt_auth_status_t m_auth_status;
    ble_gap_enc_info_t *             p_enc_info;
    
    switch (p_ble_evt->header.evt_id)
    {
				/**< Connection established. */
        case BLE_GAP_EVT_CONNECTED:
						W1_DEBUG("connected ok\n");
            nrf_gpio_pin_set(CONNECTED_LED_PIN_NO);				/*not used*/
            nrf_gpio_pin_clear(ADVERTISING_LED_PIN_NO);		/*not used*/
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
				
						//LPCOMP_end();
						// Start timers used to generate battery 
						//battery_timers_start();
            /* YOUR_JOB: Uncomment this part if you are using the app_button module to handle button
                         events (assuming that the button events are only needed in connected
                         state). If this is uncommented out here,
                            1. Make sure that app_button_disable() is called when handling
                               BLE_GAP_EVT_DISCONNECTED below.
                            2. Make sure the app_button module is initialized.
            err_code = app_button_enable();
            */
            break;
				
        /**< Disconnected from peer. */    
        case BLE_GAP_EVT_DISCONNECTED:
						W1_DEBUG("disconnected ok\n");
						power_off_flag = true;
            nrf_gpio_pin_clear(CONNECTED_LED_PIN_NO);			/*not used*/
            m_conn_handle = BLE_CONN_HANDLE_INVALID;

						//LPCOMP_start();
            /* YOUR_JOB: Uncomment this part if you are using the app_button module to handle button
                         events. This should be done to save power when not connected
                         to a peer.
            err_code = app_button_disable();
            */
						/*seattle-test
            if (err_code == NRF_SUCCESS)
            {
                advertising_start();
            }
						*/
            break;
            
        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, 
                                                   BLE_GAP_SEC_STATUS_SUCCESS, 
                                                   &m_sec_params);
            break;
            
        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0);
            break;

        case BLE_GAP_EVT_AUTH_STATUS:
            m_auth_status = p_ble_evt->evt.gap_evt.params.auth_status;
            break;
            
        case BLE_GAP_EVT_SEC_INFO_REQUEST:
            p_enc_info = &m_auth_status.periph_keys.enc_info;
            if (p_enc_info->div == p_ble_evt->evt.gap_evt.params.sec_info_request.div)
            {
                err_code = sd_ble_gap_sec_info_reply(m_conn_handle, p_enc_info, NULL);
            }
            else
            {
                // No keys found for this device
                err_code = sd_ble_gap_sec_info_reply(m_conn_handle, NULL, NULL);
            }
            break;
						
				/**< Timeout expired. */
        case BLE_GAP_EVT_TIMEOUT:
            if (p_ble_evt->evt.gap_evt.params.timeout.src == BLE_GAP_TIMEOUT_SRC_ADVERTISEMENT)
            {
                nrf_gpio_pin_clear(ADVERTISING_LED_PIN_NO);		/*not used*/
							
                // Configure buttons with sense level low as wakeup source.				//seattle-test why need this
                nrf_gpio_cfg_sense_input(WAKEUP_BUTTON_PIN_NO,
                                         BUTTON_PULL, 
                                         NRF_GPIO_PIN_SENSE_LOW);

                power_off_flag = true;
            }
            break;

        default:
            break;
    }

    APP_ERROR_CHECK(err_code);
}

/**@brief Function for uart command handler
 * @detail 
 */
void on_command_handler(char * data)
{
		uint8_t tmp_data[20];
		uint16_t package_num;
	
		// Command "ID:"
		if(!strncmp(data,"ID:",3))
		{
			if(device_config_main->is_active)
			{
				if(!strncmp(data+3, device_config_main->user_name, USERNAME_LENGTH)){
					device_config_main->info_changed |= BIT_CHECK_ID_OK;
					W1_DEBUG("\nActived, Check ID ok, then SET-TIME\n");
				}
				else {
					device_config_main->info_changed &= ~BIT_CHECK_ID_OK;
					W1_DEBUG("\nActived, Check ID fail!!!\n");
				}
			}
			else
			{
				array_ncpy(device_config_main->user_name, data+3, USERNAME_LENGTH);
				device_config_main->info_changed |= BIT_SET_ID_DONE;
				W1_DEBUG_LEN(device_config_main->user_name, USERNAME_LENGTH);
				W1_DEBUG("\nUnacitved, Set ID ok, then SET-TIME\n");
			}
		}
		// Command "SET-TIME:"
		else if(!strncmp(data,"SET-TIME:",9))
		{
			if(device_config_main->is_active)
			{	// If actived and check ID ok, then set permission agree
				if(device_config_main->info_changed & BIT_CHECK_ID_OK){
					array_ncpy(step_count.set_time, data+9, TIME_LENGTH);
					ble_transfer_data((uint8_t *)ACK_ID_AGREE, sizeof(ACK_ID_AGREE));
					permission_agree = true;
					W1_DEBUG("Actived, SET-TIME ok, then DEVICE-INFO\n");
				}
				else{
					ble_transfer_data((uint8_t *)ACK_ID_DENT, sizeof(ACK_ID_DENT));
					permission_agree = false;
					W1_DEBUG("Actived, SET-TIME fail, first check ID:\n");
				}
			}
			else
			{	// If unactived and set ID done, then set login time.
				if(device_config_main->info_changed & BIT_SET_ID_DONE){
					array_ncpy(device_config_main->login_time, data+9, TIME_LENGTH);
					device_config_main->info_changed |= BIT_SET_TIME_DONE;
					time_display((struct S_time *)(device_config_main->login_time));
					ble_transfer_data((uint8_t *)ACK_ID_INIT, sizeof(ACK_ID_INIT));
					W1_DEBUG("Unactived, SET-TIME ok\n");
				}
				else{
					W1_DEBUG("Unacitved, SET-TIME fail, SET-ID first please!\n");
				}
			}
		}
		// Command "DEVICE-INFO"
		else if(!strncmp(data,"DEVICE-INFO",11))
		{
			if(permission_agree){
				array_ncpy((char*)tmp_data, device_config_main->login_time, TIME_LENGTH); //transfer the login time
				tmp_data[TIME_LENGTH] = 50; 		//transfer the battery level
				tmp_data[TIME_LENGTH+3] = 0x1;  //transfer the device type				
				package_num = ((device_config_main->page_end - device_config_main->page_now) * W1_MAX_WORD_OFFSET + device_config_main->word_offset) / 10 ;
				tmp_data[TIME_LENGTH+1] = package_num % 0x100;
				tmp_data[TIME_LENGTH+2] = package_num / 0x100;	//transfer the step data size
				ble_transfer_data(tmp_data, 11);
				// Use for error deal
				device_config_main->page_now_record = device_config_main->page_now;
				device_config_main->word_offset_record = device_config_main->word_offset;
				W1_DEBUG("Permission agree, DEVICE-INFO ok, then TRANSFER-DATA \n");
				time_display((struct S_time *)(tmp_data));
			}
			else{
				W1_DEBUG("\nPermission deny, No action\n");
			}
		}
		else if(!strncmp(data,"NONREAL-TIME",12))
		{
			normal_time_flag = true;
			real_time_flag = false;
		}
		else if(!strncmp(data,"REAL-TIME",9))
		{
			real_time_flag = true;
			normal_time_flag = false;
		}
		else if(!strncmp(data,"DATA-BEGIN",10))
		{
			if(permission_agree && real_time_flag){
				sensor_timers_start();
			}
		}
		else if(!strncmp(data,"DATA-END",8))
		{
			if(permission_agree && real_time_flag){
				sensor_timers_stop();
			}
		}
		// Command "TRANSFER-DATA"
		else if(!strncmp(data,"TRANSFER-DATA",13))
		{
			if(permission_agree && normal_time_flag){
				W1_DEBUG("TRANSFER-DATA ,one package start\n");
				W1_transfer_data(device_config_main, TRANS_NEXT);
			}
			else{
				W1_DEBUG("\nPermission deny, No action\n");
			}
		}
		// Command "ERROR"
		else if(!strncmp(data,"ERROR",13))
		{
			W1_DEBUG("ERROR ,retry last package \n");
			W1_transfer_data(device_config_main, TRANS_LAST);
		}
		// Command "TRANSFER-OVER"
		else if(!strncmp(data,"TRANSFER-OVER",14))
		{
			W1_DEBUG("TRANSFER-OVER ok\n");
			device_config_main->info_changed |= BIT_DATA_TRANS_OVER;
			rtc_show_time();
		}
		// Command "show_time"
		else if(!strncmp(data,"show_time",9))
		{
			//W1_DEBUG("show_time ok\n");
			//rtc_show_time();
		}
		// Command "shutdown"
		else if(!strncmp(data,"shutdown",8))
		{
			//W1_DEBUG("shutdown now!\n");
			//power_off_flag = true;
		}
		// Command "timer_start"
		else if(!strncmp(data,"timer_start",11))
		{
			//W1_DEBUG("timer_start ok\n");
			//timers_start();
		}
		// Command "timer_stop"
		else if(!strncmp(data,"timer_stop",10))
		{
			//W1_DEBUG("timer_stop ok\n");
			//timers_stop();
		}
		// Command "transfer_test"
		else if(!strncmp(data,"transfer_test",13))
		{
			//W1_DEBUG("transfer_test begin\n");
			//transfer_test();
		}
		// Command "test1"
		else if(!strncmp(data,"test1",5))
		{
			//W1_DEBUG("test1 ok\n");
		}
		// Command fail
		else{
			W1_DEBUG("command failed!!\n");
		}
}

/**@brief Dispatches a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the scheduler in the main loop after a BLE stack
 *          event has been received.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
 void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    ble_conn_params_on_ble_evt(p_ble_evt);
    ble_nus_on_ble_evt(&m_nus, p_ble_evt);
	  ble_bas_on_ble_evt(&m_bas, p_ble_evt);
    on_ble_evt(p_ble_evt);
}

/**@brief Function for dispatching a system event to interested modules.
 *
 * @details This function is called from the System event interrupt handler after a system
 *          event has been received.
 *
 * @param[in]   sys_evt   System stack event.
 */
static void sys_evt_dispatch(uint32_t sys_evt)
{
    pstorage_sys_event_handler(sys_evt);
}

void unactive_device(void)
{
	uint8_t word_count;
	uint32_t buffer_page[256];
	char str[50];
	
	W1_DEBUG("Unactive device, only for debug!!! \n");
	
	W1_flash_write_prepare();
	//ble_flash_page_erase(PAGE_SIZE_END);
	ble_flash_page_write(PAGE_SIZE_END, 0, (sizeof(FLASH_MAGIC_NUMBER))/4);
	W1_flash_write_restore();

	ble_flash_page_read(PAGE_SIZE_END, buffer_page, &word_count);
	sprintf(str,"word count: %02x ", word_count);
	W1_DEBUG(str);

	rtc_reset_time();
}
/*****************************************************************************
* Main Function
*****************************************************************************/
/**
 * @brief Function for application main entry. 
 * @return 0. int return type required by ANSI/ISO standard.
 */
int main(void)
{
	power_off_flag = false;
	permission_agree = false;
	
	step_count.step = &step;		//seattle-test need modify
	
#if (ENABLE_UART_FUNCTION == 1)		
	uart_init();
#endif
	
  uart_start();
	//buttons_init(); 
	timers_init();
  ble_stack_init();
	rtc_init();

	step_init(&step_count, &step_buffer);		
	device_config_main = device_init();		//seattle-test need modify
  //ble_stack_init();				//seattle-test why need this
	//services_init();
	sensor_timers_start();
	//step_timers_start();
#if (UNACTIVE_DEVICE_ENABLE == 1)	
	unactive_device();
#else
	mode_switch(&step_count);	
	//go to STEP_COUNTER_MODE	
	/*seattle-test
	while(step_count.switch_mode == STEP_COUNTER_MODE)
	{
		step_mode_run(&step_count,&step_buffer);
		power_manage();
	}

	//go to DATA_TRANSFERER_MODE
	if(step_count.switch_mode == DATA_TRANSFERER_MODE)
	{		
		transfer_mode_run();
	}
	*/
#endif //end UNACTIVE_DEVICE_ENABLE
	
	W1_DEBUG("\n End: this is project W1  \n");
	
	

#if defined(END_WITH_UART)
	//the code below can be used to test if the cpu is run or not
  while(true)
  {
    uint8_t cr = simple_uart_get();
    simple_uart_put(cr);

    if(cr == 'q' || cr == 'Q')
    {
      uart_quit();
      while(1){}
    }
    if(cr == 't' || cr == 'T')
    {
      W1_DEBUG("\n");
			rtc_get_time(&time);
    }
  }
#else
	
		//ADC_Disable();
		//LPCOMP_start();
  // Enter main loop
  for (;;)
  {
      power_manage();
  }
#endif

}

/**
 *@}
 **/
