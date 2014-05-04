#ifndef STEP_H__
#define STEP_H__

#include "sensor.h"

#define STEP_COUNTER_MODE 			1
#define DATA_TRANSFERER_MODE 		2

#define STEP_DERICTIONG_NULL 		0
#define STEP_DERICTIONG_UP 			1
#define STEP_DERICTIONG_DOWN 		2

#define FLASH_MAGIC_NUMBER			"00PROJECTUSERNAMEandIDPASSWORDSETTIMESTORAGEINFO"	//9+13+8+7+11=48Byte
#define FLASH_MAGICNUM_EMPTY		"00PROJECT"
#define FLASH_MAGICNUM_USING		"W1PROJECT"
#define FLASH_DEFAULT_PASSWORD	"PASSWORD"


#define STEP_REF_VOLTAGE				1500				/*Define the reference voltage of a step. 1500mV=1.5V */
#define STEP_REF_TIME						100					/*Define the reference time to record a step data. 100*30ms=3000ms=3s*/
#define SWITCH_REF_COUNT				5						/*Define the reference step count to switch mode*/
#define SLEEP_REF_TIME					60					/*Define the reference time to go power off. 60*3s=180s=3min*/

// the time per count to note step count , here is 90 seconds
#define NOTE_REF_TIME						3000

#define PAGE_ACCOUNT						100
#define PAGE_SIZE_END						BLE_FLASH_PAGE_END-1


#define BYTE2DWORD(X,Y)					((X[Y/4]>>(8*(Y%4)))&0xFF)

// debug log enble control
#define STEP_DEBUG_EN
//#undef STEP_DEBUG_EN

#ifdef STEP_DEBUG_EN
		#define W1_STEP_DEBUG(x)		W1_DEBUG(x)
#else
		#define W1_STEP_DEBUG(x)		do{} while(0);
#endif

int step_init(struct S_step_count *step_count, struct S_step_buffer *step_buffer);
int is_step(struct S_step *step, struct S_step_count *step_count);
int is_step2(struct S_step *step, struct S_step_count *step_count);
int mode_switch(struct S_step_count *step_count);
struct S_device_config* device_init(void);
int device_info_update(struct S_device_config* device_config_update, struct S_step_count *step_count);
int step_mode_run(struct S_step_count *step_count,struct S_step_buffer *step_buffer);
//void W1_softdevice_handler_sd_enable(void);
			
//void W1_flash_write_prepare(void);
//void W1_flash_write_restore(void);
//void get_storage_range(uint8_t* high_position,uint8_t* low_position);
			
#endif // STEP_H__

