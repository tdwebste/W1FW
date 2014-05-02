#ifndef RTC_H__
#define RTC_H__

#include "sensor.h"
#include "twi_master.h"

#define RS5C372_BASE_ADDRESS  			0x64 	//7 MSBs of the rs5c372a TWI address(01100100)
#define RS5C372_CTLREG2							0x0F	//Control Register 2
#define USE_24HOUR_MODE 						0x20	//24 Hour mode
#define RS5C372_SECONDS							0x00	//Second Count offset

// debug log enble control
#define RTC_DEBUG_EN
//#undef RTC_DEBUG_EN

#ifdef RTC_DEBUG_EN
		#define W1_RTC_DEBUG(x)		W1_DEBUG(x)
#else
		#define W1_RTC_DEBUG(x)		do{} while(0);
#endif


int rtc_init(void);
//seattle-test	void get_second_display(void);
int rtc_get_time(struct S_time *time);
bool rtc_set_time(struct S_time *time);
bool rtc_reset_time(void);
void rtc_show_time(void);
void time_display(struct S_time *time);			
			
bool rs5c372a_init(uint8_t device_address);
bool rs5c372a_register_write(uint8_t register_address, const uint8_t value);
bool rs5c372a_register_read(uint8_t register_address,uint8_t *value);

#endif // RTC_H__

