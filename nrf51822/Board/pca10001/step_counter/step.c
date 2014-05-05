/*
* Copyright (c) 2013 Sherwin_Yan. All Rights Reserved.
* This file is dealing step for W1-project . 
* main.c will call setp_init and mode_switch .
*/

#include <stdint.h>
#include <string.h>
#include "step.h"
#include "rtc.h"
#include "data_transfer.h"
#include "ble_flash.h"
#include "softdevice_handler.h"
#include "main.h"

uint8_t *W1_step_buffer;
struct S_device_config device_config;

int step_init(struct S_step_count *step_count, struct S_step_buffer *step_buffer)
{
	char ret=0;
	
	// Init some global variable	
	step_buffer->offset = TIME_LENGTH+BUFFER_HEAD_LENGTH; // offset of step_buffer to store the next uint8
	W1_step_buffer = (uint8_t *)step_buffer;
	rtc_get_time(&(step_buffer->time));
	
	// because the last_voltage is 0V ,then the direction should not be UP! 
	// it will conflict in logic so that we can know that it's the first time to check step!
	step_count->step->current_voltage = 0;
	step_count->step->last_voltage = 0;
	step_count->step->last_direction = STEP_DERICTIONG_NULL;
	step_count->step->new_direction = STEP_DERICTIONG_NULL;
	
	step_count->adc_count = 0;
	step_count->step_count = 0;
	step_count->count_mul = 0;
	
	return ret;
}

/*
* this funciton is to check is a step is complete & update the direction
*/
int is_step(struct S_step *step, struct S_step_count *step_count)
{
	char ret=0;
	/*seattle-test not used
	if((step->last_voltage == 0) && (step->last_direction == STEP_DERICTIONG_NULL))
	{
		step->last_voltage = step->current_voltage;
		step->is_step = false;
		return 1;
	}
	
	if(step->current_voltage == step->last_voltage)
	{	
		step->last_voltage = step->current_voltage;
		return 2;
	}
	*/
	//if((step->current_voltage > step->last_voltage) && (step->last_voltage >= STEP_REF_VOLTAGE)){	
	if(step->current_voltage > step->last_voltage){
		step->new_direction = STEP_DERICTIONG_UP;
	}
	else if((step->current_voltage < step->last_voltage) && (step->current_voltage >= STEP_REF_VOLTAGE)){	
		step->new_direction = STEP_DERICTIONG_DOWN;
	}
	else{
		step->new_direction = STEP_DERICTIONG_NULL;
	}
	
	/*seattle-test not used
	//step->new_direction = (step->current_voltage > step->last_voltage)?STEP_DERICTIONG_UP:STEP_DERICTIONG_DOWN;

	//if((step->new_direction == STEP_DERICTIONG_DOWN)&&(step->current_voltage > STEP_REF_VOLTAGE))
	if(step->new_direction == STEP_DERICTIONG_DOWN)
	{
		//W1_STEP_DEBUG("direction download\n");
	}
		
	if((step->current_voltage > STEP_REF_VOLTAGE))
	{
		//W1_STEP_DEBUG("large voltage\n");
	}
	*/
	if((step->last_direction == STEP_DERICTIONG_UP) && (step->new_direction == STEP_DERICTIONG_DOWN))
	{	step->new_adc = step_count->adc_count;
		//if(3 < abs(step->new_adc - step->last_adc) < 2995){
			step->is_step = true;
			W1_DEBUG("is step \n");
		//}
		//step->last_adc = step->new_adc;
	}
	else
	{
		step->is_step = false;
	}
	
	step->last_voltage = step->current_voltage;
	step->last_direction = step->new_direction;
	
	return ret;
}

int is_step2(struct S_step *step, struct S_step_count *step_count)
{
	char ret=0, update_direction=0;
	
	if((step->last_voltage < STEP_REF_VOLTAGE) && (step->current_voltage >= STEP_REF_VOLTAGE)){
		step->new_direction = STEP_DERICTIONG_UP;
		update_direction = 1;
	}
	else if((step->last_voltage >= STEP_REF_VOLTAGE) && (step->current_voltage < STEP_REF_VOLTAGE)){
		step->new_direction = STEP_DERICTIONG_DOWN;
		update_direction = 1;
	}
	
	if((step->last_direction == STEP_DERICTIONG_UP) && (step->new_direction == STEP_DERICTIONG_DOWN))
	{	step->new_adc = step_count->adc_count;
		//if(3 < abs(step->new_adc - step->last_adc) < 2995){
			step->is_step = true;
			W1_DEBUG("is step \n");
		//}
		//step->last_adc = step->new_adc;
	}
	else
	{
		step->is_step = false;
	}
	step->last_voltage = step->current_voltage;
	if(update_direction){
		step->last_direction = step->new_direction;
	}
	
	return ret;
}

/*
*  this function will return 
*/
int mode_switch(struct S_step_count *step_count)
{
//	char ret=0;
	
	if(device_config.is_active){
		//is active, return STEP_COUNTER_MODE;
		W1_DEBUG("\n go to STEP_COUNTER_MODE \n");
		step_count->switch_mode = STEP_COUNTER_MODE;
		return STEP_COUNTER_MODE;
	}
	else{
		//not active, return DATA_TRANSFERER_MODE;
		W1_DEBUG("\n go to DATA_TRANSFERER_MODE \n");
		step_count->switch_mode = DATA_TRANSFERER_MODE;
		return DATA_TRANSFERER_MODE;
	}

//	return ret;
}
/*seattle-test not used
int array_8to32(uint8_t *buffer8,uint32_t *buffer32,uint8_t length8)
{
	int ret = 0;
	
	return ret;
}
int array_32to8(uint8_t *buffer8,uint32_t *buffer32,uint8_t length8)
{
	int ret = 0;
	int i=0;
	
	for(i=0;i<length8;i++)
	{
		//buffer8[i]=(buffer32[i/4]>>(8*(i%4)))&0xFF;
		buffer8[i]=BYTE2DWORD(buffer32,i);
	}
	
	return ret;
}

*/

struct S_device_config* device_init(void)
{
	uint32_t buffer_page[256];
	char *buffer8;
	uint8_t word_count;
	char tmp_buffer[sizeof(FLASH_MAGIC_NUMBER)] = FLASH_MAGIC_NUMBER;
	//uint8_t storage_info_init[5]={PAGE_SIZE_END-1,PAGE_SIZE_END-1,0,0,0};
	struct S_device_config *p_device_config;
	
	buffer8 = (char *)buffer_page;
	p_device_config = (S_device_config *)buffer_page;
	
	// Read the END_PAGE
	ble_flash_page_read(PAGE_SIZE_END, buffer_page, &word_count);

	
	//seattle-test-	device_config.info_changed = 0;
	if(!strncmp(buffer8,FLASH_MAGICNUM_EMPTY,9))
	{	
		W1_STEP_DEBUG("device_config read over:\n");
		W1_DEBUG_LEN(p_device_config->magic_number, MAGICNUM_LENGTH);
		W1_DEBUG_LEN(p_device_config->user_name, USERNAME_LENGTH);
		W1_DEBUG_LEN(p_device_config->password, PASSWORD_LENGTH);
		W1_DEBUG_LEN(p_device_config->login_time, TIME_LENGTH);
		W1_STEP_DEBUG("\ndevice not active now!\n");
		device_config.is_active = false;		
	}
	else if(!strncmp(buffer8,FLASH_MAGICNUM_USING,9))
	{	
		W1_STEP_DEBUG("device_config read over:\n");
		W1_DEBUG_LEN(p_device_config->magic_number, MAGICNUM_LENGTH);
		W1_DEBUG_LEN(p_device_config->user_name, USERNAME_LENGTH);
		W1_DEBUG_LEN(p_device_config->password, PASSWORD_LENGTH);
		time_display((struct S_time *)(p_device_config->login_time));
		W1_STEP_DEBUG("\ndevice active!\n");
		device_config.is_active = true;
	}
	else
	{
		W1_STEP_DEBUG("\ndevice first use!\n");
		device_config.is_active = false;
		/*seattle-test
		array_ncpy(tmp_buffer+sizeof(FLASH_MAGIC_NUMBER)-1,(char *)(&step_buffer->time),TIME_LENGTH);	
		array_ncpy(tmp_buffer+sizeof(FLASH_MAGIC_NUMBER)+TIME_LENGTH-1,(char *)storage_info_init,STORAGE_INFO_LENGTH);	
		ble_flash_page_write(PAGE_SIZE_END,(uint32_t *)tmp_buffer,(sizeof(FLASH_MAGIC_NUMBER)+TIME_LENGTH+STORAGE_INFO_LENGTH+3)/4);
		seattle-test*/
		// Flash device info into the END_PAGE
		W1_flash_write_prepare();
		ble_flash_page_write(PAGE_SIZE_END, (uint32_t *)tmp_buffer, (sizeof(FLASH_MAGIC_NUMBER))/4);
		W1_flash_write_restore();
		W1_STEP_DEBUG("first ble_flash_page_write over \n");
				
	}
	

	if(device_config.is_active){
		device_config.info_changed = 0;
		array_ncpy(device_config.magic_number, buffer8, MAGICNUM_LENGTH);
		array_ncpy(device_config.user_name, buffer8+MAGICNUM_LENGTH, USERNAME_LENGTH);
		array_ncpy(device_config.password, buffer8+USERNAME_LENGTH+MAGICNUM_LENGTH, PASSWORD_LENGTH);
		array_ncpy(device_config.login_time, buffer8+USERNAME_LENGTH+MAGICNUM_LENGTH+PASSWORD_LENGTH, TIME_LENGTH);		
		
		device_config.page_end = buffer8[STORAGE_INFO_OFFSET];
		device_config.page_now = buffer8[STORAGE_INFO_OFFSET+1];
		device_config.word_offset = buffer8[STORAGE_INFO_OFFSET+2];	
		/*seattle-test
		device_config.page_end = PAGE_SIZE_END-1;
		device_config.page_now = PAGE_SIZE_END-1;
		device_config.word_offset = 0;
		*/
	}
	//else{
		//device_config.page_end = PAGE_SIZE_END-1;
		//device_config.page_now = PAGE_SIZE_END-1;
		//device_config.word_offset = 0;
	//}
	
	return &device_config;
}

int step_write2flash(uint8_t *buffer8,uint8_t length8)
{
	int ret=0,i;
	char *tmp;
	//uint8_t  word_count;
	//uint32_t buffer_page[STORAGE_BUFFER_LENGTH/4];
	uint32_t buffer_page[256];
	char str[50];
	
	tmp = (char *)buffer_page;
	
	array_ncpy(tmp, (char *)buffer8 ,length8);
	
	//if(W1_flash_page_validate(device_config.page_now))
	if(device_config.word_offset == 0){
		W1_DEBUG(" A new flash page\n");
		W1_flash_write_prepare();
		ble_flash_page_write(device_config.page_now, buffer_page, STORAGE_BUFFER_LENGTH/4);
		W1_flash_write_restore();
	}
	else {		
		W1_DEBUG(" Continue flash\n");
		W1_flash_write_prepare();
		W1_flash_words_write(device_config.page_now, device_config.word_offset, buffer_page, STORAGE_BUFFER_LENGTH/4);	
		W1_flash_write_restore();		
	}
	
	device_config.word_offset += (STORAGE_BUFFER_LENGTH/4);
	//One page is full, start another page
	if(device_config.word_offset >= W1_MAX_WORD_OFFSET){
		device_config.page_now--;
		//All page is full, start around
		if(device_config.page_now <= (device_config.page_end - PAGE_ACCOUNT))
		{
			device_config.page_now = device_config.page_end;				
		}
		device_config.word_offset = 0;
	}
	
	W1_flash_words_read(device_config.page_now, buffer_page, device_config.word_offset);
	W1_DEBUG("begin print flash\n");	
	
	//seattle-debug
	sprintf(str,"page now:%02x ,word offset:%02x \n", device_config.page_now, device_config.word_offset);
	W1_DEBUG(str);
	for(i=0; i<(device_config.word_offset)*4; i++){
		if(i%40==0)
			W1_DEBUG("\n");
		sprintf(str,"%02x ",*(tmp+i));
		W1_DEBUG(str);
	}
	
	//seattle-debug
	//time_display((struct S_time *)(buffer_page));
	W1_DEBUG("\nend print flash\n");
	
	return ret;

}
/**@brief Function for device configuration update before going to power off
 *
 * @details Update device configuration
 */
int device_info_update(struct S_device_config* device_config_update, struct S_step_count *step_count)
{
	uint32_t buffer_page[256]={0};
	char *buffer8;
	uint16_t package_num;

	buffer8 = (char *)buffer_page;
	W1_STEP_DEBUG("device config update begin:\n");
	
	// If unactived and set id ok set time ok, then active the device
	if((!(device_config_update->is_active)) && ((device_config_update->info_changed & 0x03) == 0x03))
	{	
		W1_STEP_DEBUG("Active the device\n");
		device_config_update->is_active = true;
		device_config_update->page_end = PAGE_SIZE_END-1;
		device_config_update->page_now = PAGE_SIZE_END-1;
		device_config_update->word_offset = 0;
		array_ncpy(device_config_update->magic_number, FLASH_MAGICNUM_USING, MAGICNUM_LENGTH);
		array_ncpy(device_config_update->password, FLASH_DEFAULT_PASSWORD, PASSWORD_LENGTH);
		array_ncpy(buffer8, device_config_update->magic_number, DEVICE_INFO_LENGTH);
		
		W1_flash_write_prepare();
		ble_flash_page_write(PAGE_SIZE_END,buffer_page,DEVICE_INFO_LENGTH/4);
		W1_flash_write_restore();
			// Set login time to RTC
		rtc_set_time((struct S_time *)(device_config_update->login_time));
		rtc_show_time();
	}
	// If actived, update device configuration info
	else if(device_config_update->is_active)
	{
		if(device_config_update->info_changed & BIT_CHECK_ID_OK)
		{
			rtc_set_time((struct S_time *)(step_count->set_time));
			package_num = ((device_config_update->page_end - device_config_update->page_now) * W1_MAX_WORD_OFFSET + device_config_update->word_offset) / 10 ;
			// If error occured
			if((device_config_update->info_changed & BIT_ERROR_OCCURED) || package_num){
				W1_STEP_DEBUG("\nError occured \n");
				device_config_update->page_now = device_config_update->page_now_record;
				device_config_update->word_offset = device_config_update->word_offset_record;
			}
		}
		// Update device configuration info
		W1_STEP_DEBUG("Update the device info\n");
		array_ncpy(buffer8, device_config_update->magic_number, DEVICE_INFO_LENGTH);
		W1_flash_write_prepare();
		ble_flash_page_write(PAGE_SIZE_END, buffer_page, DEVICE_INFO_LENGTH/4);
		W1_flash_write_restore();
	}
	
	W1_STEP_DEBUG("device config update over:\n");
	
	return 0;
}
/**@brief Function for step mode run
 *
 * @details 
 */
int step_mode_run(struct S_step_count *step_count, struct S_step_buffer *step_buffer)
{
	int ret = 0;
	char str[50];
	uint8_t offset, i;
	uint8_t increment = 0;
	static uint8_t last_increment = 0;
	bool buffer_full_flag = 0, running_flag = 1;
	extern bool power_off_flag;

	offset = W1_step_buffer[STORAGE_BUFFER_LENGTH];
	
	// Every 3 seconds, record step once
	if((!(step_count->adc_count % STEP_REF_TIME)) && (step_count->adc_count))
	{
		W1_DEBUG("\n Every 3 seconds, record step once \n");
		W1_step_buffer[offset] = step_count->step_count;
				
		// Caculate the increment
		increment = W1_step_buffer[offset];
		/*
		if(offset == BUFFER_COUNT_OFFSET){
			last_increment = W1_step_buffer[STORAGE_BUFFER_LENGTH - 2];
		}
		else{
			last_increment = W1_step_buffer[offset-1];
		}
		*/	
		// Judge the increment		
		if(increment == 0){
			running_flag = 0;
		}
		//else if(increment >= SWITCH_REF_COUNT){
		if(increment <= 2 && last_increment >= SWITCH_REF_COUNT){
			W1_DEBUG("\n ADC transfer \n");
			step_count->switch_mode = DATA_TRANSFERER_MODE;
			//transfer_mode_run();
		}
				
		if(!running_flag){
			step_count->count_mul ++;
			// It will shutdown if not running in 3 minutes
			if(step_count->count_mul >= SLEEP_REF_TIME){
				W1_STEP_DEBUG("it will shutdown since not running in 3 minutes\n");
				power_off_flag = true;
			}
		}
		else {
			step_count->count_mul = 0;
		}
		
		offset ++;
		W1_step_buffer[STORAGE_BUFFER_LENGTH] = offset;
		last_increment = increment;
		step_count->step_count = 0;
	}
	

	//Every 90 seconds, set buffer_full_flag
	if(offset == STORAGE_BUFFER_LENGTH-1)
	{
		W1_STEP_DEBUG("another 90 seconds \n");
		buffer_full_flag = 1;
		//step_buffer->count_base += step_count->step_count;
		step_count->adc_count = 0;
		//step_count->step_count = 0;
		step_buffer->offset = TIME_LENGTH+BUFFER_HEAD_LENGTH;
		offset = TIME_LENGTH+BUFFER_HEAD_LENGTH;
	}
	
	//write to flash when buffler full or power off
	if(buffer_full_flag || power_off_flag)
	{
		W1_STEP_DEBUG("write to flash \n");	
		for(i=0;i<STORAGE_BUFFER_LENGTH;i++){
			sprintf(str,"%02x ",W1_step_buffer[i]);
			W1_DEBUG(str);
		}
		W1_DEBUG("\n");
		/*seattle-test
		sprintf(str,"BLE_FLASH_PAGE_SIZE:0x%x ", BLE_FLASH_PAGE_SIZE);
		W1_DEBUG(str);
		sprintf(str,"BLE_FLASH_PAGE_END:0x%x ", BLE_FLASH_PAGE_END);
		W1_DEBUG(str);
		seattle-test*/
		step_buffer->head='H';
		step_buffer->stepend='E';
		//before write flash ,we should disable SoftDevice first
		//time_display((struct S_time *)(W1_step_buffer+BUFFER_HEAD_LENGTH));
		W1_DEBUG("\n\rdisable before flash \n");		
		step_write2flash(W1_step_buffer,STORAGE_BUFFER_LENGTH);	
		W1_DEBUG("enable after flash \n\n");
		
		rtc_get_time(&step_buffer->time);
		
		//step_buffer->head='H';
		if(*(W1_step_buffer+2)<0xFF)
		{
			step_buffer->index=*(W1_step_buffer+1)+1;
		}else
		{
			step_buffer->index=0;
		}
		
	}
	
	return ret;
}

