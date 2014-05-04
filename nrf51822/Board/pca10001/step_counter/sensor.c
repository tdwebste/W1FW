/*
* Copyright (c) 2013 Sherwin_Yan. All Rights Reserved.
* This file is dealing sensor for W1-project . 
* main.c will call sensor_start and the handle when timer is expired .
*/

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "nrf_gpio.h"
#include "nrf51_bitfields.h"
#include "softdevice_handler.h"
#include "ble_bas.h"
#include "sensor.h"
#include "app_util.h"
#include "main.h"

#define ADC_REF_VOLTAGE_IN_MILLIVOLTS        1200                                      /**< Reference voltage (in milli volts) used by ADC while doing conversion. */
#define ADC_PRE_SCALING_COMPENSATION         3                                         /**< The ADC is configured to use VDD with 1/3 prescaling as input. And hence the result of conversion is to be multiplied by 3 to get the actual value of the battery voltage.*/
#define DIODE_FWD_VOLT_DROP_MILLIVOLTS       270                                       /**< Typical forward voltage drop of the diode (Part no: SD103ATW-7-F) that is connected in series with the voltage supply. This is the voltage drop when the forward current is 1mA. Source: Data sheet of 'SURFACE MOUNT SCHOTTKY BARRIER DIODE ARRAY' available at www.diodes.com. */
extern ble_bas_t                                    m_bas;                                      /**< Structure used to identify the battery service. */

/**@brief Macro to convert the result of ADC conversion in millivolts.
 *
 * @param[in]  ADC_VALUE   ADC result.
 * @retval     Result converted to millivolts.
 */
#define ADC_RESULT_IN_MILLI_VOLTS(ADC_VALUE)\
        ((((ADC_VALUE) * ADC_REF_VOLTAGE_IN_MILLIVOLTS) / 255) * ADC_PRE_SCALING_COMPENSATION)
/*seattle-test>>>	
#define ADC_RESULT_IN_MILLI_VOLTS2(ADC_VALUE)\
        (((ADC_VALUE) * ADC_REF_VOLTAGE_IN_MILLIVOLTS) / 255)

//add by sherwin for set sensor pin as adc input
void ADC_GPIO_set(uint8_t rxd_pin_number)
{
		nrf_gpio_cfg_input(rxd_pin_number, NRF_GPIO_PIN_NOPULL); 
}
seattle-test<<<*/
/**@brief Function for handling the ADC interrupt.
 * @details  This function will fetch the conversion result from the ADC, convert the value into
 *           percentage and send it to peer.
 */
void ADC_IRQHandler(void)
{
		//Sensor adc handler
		if ((NRF_ADC->EVENTS_END != 0) && 
			 (((NRF_ADC->CONFIG & ADC_CONFIG_PSEL_Msk) >> ADC_CONFIG_PSEL_Pos) == ADC_CONFIG_PSEL_AnalogInput2))
    //if (NRF_ADC->EVENTS_END != 0)
		{
        uint8_t     adc_result;
        uint16_t    batt_lvl_in_milli_volts;
//char str[20];

        NRF_ADC->EVENTS_END     = 0;
        adc_result              = NRF_ADC->RESULT;
        NRF_ADC->TASKS_STOP     = 1;

        batt_lvl_in_milli_volts = ADC_RESULT_IN_MILLI_VOLTS(adc_result);
//sprintf(str,"%d mv\n",batt_lvl_in_milli_volts);			
//W1_DEBUG(str);			
				check_step(batt_lvl_in_milli_volts);
				
				W1_SENSOR_DEBUG("\n\rADC_IRQHandler");
    }
		
		//Battery adc handler
		else if ((NRF_ADC->EVENTS_END != 0) && 
						((((NRF_ADC->CONFIG & ADC_CONFIG_PSEL_Msk) >> ADC_CONFIG_PSEL_Pos)) == ADC_CONFIG_PSEL_AnalogInput4))
		{
				uint8_t     adc_result;
				uint16_t    batt_lvl_in_milli_volts;
				uint8_t     percentage_batt_lvl;
				uint32_t    err_code;
//char str[20];

        NRF_ADC->EVENTS_END     = 0;
        adc_result              = NRF_ADC->RESULT;
        NRF_ADC->TASKS_STOP     = 1;

        //batt_lvl_in_milli_volts = ADC_RESULT_IN_MILLI_VOLTS(adc_result) +
        //                          DIODE_FWD_VOLT_DROP_MILLIVOLTS;
			batt_lvl_in_milli_volts = ADC_RESULT_IN_MILLI_VOLTS(adc_result);
//sprintf(str,"%d mv\n",batt_lvl_in_milli_volts);			
//W1_DEBUG(str);
        percentage_batt_lvl     = battery_level_in_percent(batt_lvl_in_milli_volts);

        err_code = ble_bas_battery_level_update(&m_bas, percentage_batt_lvl);
        if (
            (err_code != NRF_SUCCESS)
            &&
            (err_code != NRF_ERROR_INVALID_STATE)
            &&
            (err_code != BLE_ERROR_NO_TX_BUFFERS)
            &&
            (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
        )
        {
            APP_ERROR_HANDLER(err_code);
        }
		}
		
}

void sensor_start(void)
{
    uint32_t err_code;

    // Configure ADC
    NRF_ADC->INTENSET   = ADC_INTENSET_END_Msk;
    NRF_ADC->CONFIG     = (ADC_CONFIG_RES_8bit                        			<< ADC_CONFIG_RES_Pos)     |
                          (ADC_CONFIG_INPSEL_AnalogInputOneThirdPrescaling 	<< ADC_CONFIG_INPSEL_Pos)  |
                          (ADC_CONFIG_REFSEL_VBG                      			<< ADC_CONFIG_REFSEL_Pos)  |
                          (ADC_CONFIG_PSEL_AnalogInput2                   	<< ADC_CONFIG_PSEL_Pos)    |
                          (ADC_CONFIG_EXTREFSEL_None                  			<< ADC_CONFIG_EXTREFSEL_Pos);
    NRF_ADC->EVENTS_END = 0;
    NRF_ADC->ENABLE     = ADC_ENABLE_ENABLE_Enabled;

		// Config ADC gpio input
		nrf_gpio_cfg_input(ADC_SENSOR_PIN_NO, NRF_GPIO_PIN_NOPULL); 
	
    // Enable ADC interrupt
    err_code = sd_nvic_ClearPendingIRQ(ADC_IRQn);
    APP_ERROR_CHECK(err_code);

    err_code = sd_nvic_SetPriority(ADC_IRQn, NRF_APP_PRIORITY_LOW);
    APP_ERROR_CHECK(err_code);

    err_code = sd_nvic_EnableIRQ(ADC_IRQn);
    APP_ERROR_CHECK(err_code);

    NRF_ADC->EVENTS_END  = 0;    // Stop any running conversions.
    NRF_ADC->TASKS_START = 1;
}

void battery_start(void)
{
    uint32_t err_code;

    // Configure ADC
    NRF_ADC->INTENSET   = ADC_INTENSET_END_Msk;
    NRF_ADC->CONFIG     = (ADC_CONFIG_RES_8bit                        << ADC_CONFIG_RES_Pos)     |
                          (ADC_CONFIG_INPSEL_AnalogInputOneThirdPrescaling << ADC_CONFIG_INPSEL_Pos)  |
                          (ADC_CONFIG_REFSEL_VBG                      << ADC_CONFIG_REFSEL_Pos)  |
                          (ADC_CONFIG_PSEL_AnalogInput4               << ADC_CONFIG_PSEL_Pos)    |
                          (ADC_CONFIG_EXTREFSEL_None                  << ADC_CONFIG_EXTREFSEL_Pos);
    NRF_ADC->EVENTS_END = 0;
    NRF_ADC->ENABLE     = ADC_ENABLE_ENABLE_Enabled;
	
		//config ADC gpio input
		nrf_gpio_cfg_input(ADC_BATTERY_PIN_NO, NRF_GPIO_PIN_NOPULL); 
	
    // Enable ADC interrupt
    err_code = sd_nvic_ClearPendingIRQ(ADC_IRQn);
    APP_ERROR_CHECK(err_code);

    err_code = sd_nvic_SetPriority(ADC_IRQn, NRF_APP_PRIORITY_LOW);
    APP_ERROR_CHECK(err_code);

    err_code = sd_nvic_EnableIRQ(ADC_IRQn);
    APP_ERROR_CHECK(err_code);

    NRF_ADC->EVENTS_END  = 0;    // Stop any running conversions.
    NRF_ADC->TASKS_START = 1;
}

void ADC_Disable(void)
{
	NRF_ADC->ENABLE     = ADC_ENABLE_ENABLE_Disabled;
}

/**
 * @}
 */
