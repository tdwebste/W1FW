#ifndef MAIN_H__
#define MAIN_H__

#include <stdint.h>
#include "simple_uart.h"
#include "ble_bas.h"
#include "ble_nus.h"

// Debug log enble control
//#if ENABLE_UART_FUNCTION == 1
#define DEBUG_EN
//#endif

#ifdef DEBUG_EN
		#define W1_DEBUG(x)		simple_uart_putstring((const uint8_t *) x)
		#define W1_PRINT(x)		simple_uart_putstring((const uint8_t *) x)
		#define W1_DEBUG_LEN(x,y)	simple_uart_putlen((const uint8_t *) x, (uint8_t) y)
#else
		#define W1_DEBUG(x)		do{} while(0);
		#define W1_PRINT(x)		do{} while(0);
		#define W1_DEBUG_LEN(x,y)	do{} while(0);
#endif
			

			
#define BUFFER_HEAD_LENGTH				2
#define TIME_LENGTH								7
#define BUFFER_COUNT_OFFSET				BUFFER_HEAD_LENGTH + TIME_LENGTH
#define STORAGE_BUFFER_LENGTH			40
			
#define USERNAME_LENGTH						13
#define PASSWORD_LENGTH						8
#define MAGICNUM_LENGTH						9
#define STORAGE_INFO_LENGTH				5
#define STORAGE_INFO_OFFSET 			MAGICNUM_LENGTH + USERNAME_LENGTH + PASSWORD_LENGTH + TIME_LENGTH
#define DEVICE_INFO_LENGTH				48
			



#define BIT0								0x01
#define BIT1								0x02
#define BIT2								0x04
#define BIT3								0x08	
#define BIT4								0x10
#define BIT5								0x20
#define BIT6								0x40
#define BIT7								0x80

#define BIT_SET_ID_DONE					BIT0
#define BIT_SET_TIME_DONE				BIT1
#define BIT_STORAGE_INFO_OK			BIT2
#define BIT_CHECK_ID_OK					BIT3
#define	BIT_DATA_TRANS_OVER			BIT4
#define	BIT_ERROR_OCCURED				BIT5

//seatttle-test>>>
#define WAKEUP_BUTTON_PIN_NO							10			/* wakeup the device from power off */
#define ADC_BATTERY_PIN_NO								6				/* battery detection use analog input 4 as analog input */	
#define ADC_SENSOR_PIN_NO									4			 	/* piezoelectric sensor use analog input 2 as analog input */

#define CONNECTED_LED_PIN_NO							8U			/*not used*/
#define ASSERT_LED_PIN_NO									8U			/*not used*/
#define ADVERTISING_LED_PIN_NO						8U			/*not used*/
//seattle-test<<<

/**@brief External reference to the Battery Service. */
extern ble_bas_t                            m_bas;
extern ble_nus_t                        		m_nus;																				/**< Structure to identify the Nordic UART Service. */

void W1_flash_write_prepare(void);
void W1_flash_write_restore(void);

struct S_time
{
	
	uint8_t sec;
	uint8_t min;
	uint8_t hour;
	uint8_t dayofweek;
	uint8_t dayofmonth;
	uint8_t mon;
	uint8_t year;
};
			
struct S_step
{
	
	char last_direction;
	char new_direction;
	uint16_t last_voltage;
	uint16_t current_voltage;
	uint16_t last_adc;
	uint16_t new_adc;
	bool is_step;
	
};

struct S_step_count
{
	
	//struct S_time now_time;
	char set_time[TIME_LENGTH];
	//struct S_time *start_time;
	struct S_step *step;
	uint8_t step_count;
	uint16_t adc_count;
	uint16_t count_mul;
	int switch_mode;
	
};

struct S_step_buffer
{
	char 		head;
	uint8_t	index;
	struct S_time time;
	//uint16_t count_base;
	uint8_t buffer[STORAGE_BUFFER_LENGTH-TIME_LENGTH-BUFFER_HEAD_LENGTH-1];
	char 		stepend;
	uint8_t offset;
	
};

struct S_data_storage
{
	
	uint8_t start_offset;
	uint8_t end_offset;
	uint8_t length;																															// if equal with "start_offset>end_offset?start_offset-end_offset;end_offset-start_offset"
	uint8_t end_rool;																															//if the storage erea is from the end to the start , go over the edge
	struct S_step_count *step_count;
	
};

typedef struct S_device_config
{
	char magic_number[MAGICNUM_LENGTH];
	char user_name[USERNAME_LENGTH];
	char password[PASSWORD_LENGTH];
	char login_time[TIME_LENGTH]; //time that active
	uint8_t page_end;
	uint8_t page_now;
	uint8_t word_offset;
	uint8_t is_active;
	uint8_t info_changed; // |set-id-done|set-time-done|storage-info-ok|check-id-ok|data-transfer-over|error-occurd|
	char reserved[DEVICE_INFO_LENGTH-PASSWORD_LENGTH-USERNAME_LENGTH-MAGICNUM_LENGTH-TIME_LENGTH-STORAGE_INFO_LENGTH];
	
	uint8_t error_times;
	uint8_t page_now_record;
	uint8_t word_offset_record;
	
}S_device_config;

void print_error_code(uint32_t error_code);
void array_ncpy(char *array1,char *array2,uint8_t num);

#endif // MAIN_H__

