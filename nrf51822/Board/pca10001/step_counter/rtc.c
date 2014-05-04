/*
* Copyright (c) 2013 Sherwin_Yan. All Rights Reserved.
* This file is rtc driver for W1-project . 
* main.c will call sensor_start and the handle when timer is expired .
*/

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "rtc.h"
#include "nrf_delay.h"

static uint8_t m_device_address; //!< Device address in bits [7:1]

bool rs5c372a_init(uint8_t device_address)
{	
	bool transfer_succeeded = false;
	m_device_address = (uint8_t)(device_address);
	
	// Set control register2 to make 24h mode
	transfer_succeeded = rs5c372a_register_write(RS5C372_CTLREG2, USE_24HOUR_MODE);
  //seattle-test	transfer_succeeded = rs5c372a_register_write(0x4,0x45);
	return transfer_succeeded;
	
}


bool rs5c372a_register_write(uint8_t register_address, uint8_t value)
{	
	uint8_t data_buffer[2];
	
	register_address &= 0x0F;
	data_buffer[0] = (register_address << 4);
	data_buffer[1] = value;
  return twi_master_transfer(m_device_address, data_buffer, 2, TWI_ISSUE_STOP);	
}


bool rs5c372a_register_read(uint8_t register_address, uint8_t *value)
{
  bool transfer_succeeded = false;
	
  if(twi_master_transfer(m_device_address, &register_address, 1, TWI_DONT_ISSUE_STOP))
	{	
	  if(twi_master_transfer(m_device_address | TWI_READ_BIT, value, 1, TWI_ISSUE_STOP))
		{
			transfer_succeeded = true;
		}
	}
	
  return transfer_succeeded;
}

bool rs5c372a_registers_write(uint8_t register_address, uint8_t *value, uint8_t number_of_bytes)
{	
	uint8_t data_buffer[number_of_bytes+1];
	uint8_t i = 0;
/*	
	register_address &= 0x0F;
	data_buffer[0] = (register_address << 4);
*/
	data_buffer[0] = register_address;
	for(i=0;i<number_of_bytes+1;i++)
	data_buffer[i+1] = value[i];
	
  return twi_master_transfer(m_device_address, data_buffer, number_of_bytes+1, TWI_ISSUE_STOP);	
}

bool rs5c372a_registers_read(uint8_t register_address, uint8_t *value, uint8_t number_of_bytes)
{
  bool transfer_succeeded=false;
	//seattle-test??why down here??
	//W1_RTC_DEBUG("seattle-11 \n");
	//seattle-test??why down here??
	register_address &= 0x0F;
	register_address = (register_address << 4);
  if(twi_master_transfer(m_device_address, &register_address, 1, TWI_DONT_ISSUE_STOP))
	{	//W1_RTC_DEBUG("seattle-12 \n");
	  if(twi_master_transfer(m_device_address | TWI_READ_BIT, value, number_of_bytes, TWI_ISSUE_STOP))
		{	//W1_RTC_DEBUG("seattle-13 \n");
			transfer_succeeded = true;
		}
	}
	//W1_RTC_DEBUG("seattle-14 \n");
  return transfer_succeeded;
}

int rtc_init(void)
{
	int ret = 0;
	int retry_count = 0;
	struct S_time time_tmp;
	
	twi_master_init();
	while((!rs5c372a_init(RS5C372_BASE_ADDRESS)) && (retry_count<10))
	{	
		retry_count ++;
		W1_DEBUG("retry init rtc!\n");
	}
	if(retry_count >= 10)
	{
		return ret;
	}
	
	rtc_get_time(&time_tmp);
	if((time_tmp.year>30)||(time_tmp.mon>12)||(time_tmp.dayofweek>7))
	{
		rtc_reset_time();
		W1_RTC_DEBUG("time_reset well\n");
	}
	
	return 1;
}
/*seattle-test
void get_second_display(void)
{
	char str[20];
	uint8_t value;
	
	if(rs5c372a_register_read(0x0,&value))
	{
			sprintf(str,"\nSEC: 0x%x",value);
			W1_DEBUG(str);
	}
}
*/
void time_display(struct S_time *time)
{
	uint8_t *buffer;
	char str[30];
	
	buffer = (uint8_t*)time;
	sprintf(str,"20%02x-%02x-%02x %02x %02x:%02x:%02x\n",buffer[6],buffer[5],buffer[4],buffer[3],buffer[2],buffer[1],buffer[0]);
	W1_DEBUG(str);
}

void rtc_show_time(void)
{
	uint8_t *buffer;
	struct S_time time;
	
	W1_RTC_DEBUG("Show RTC time \n");
	buffer = (uint8_t*)&time;
	//W1_RTC_DEBUG("seattle-1 \n");
	if(rs5c372a_registers_read(RS5C372_SECONDS, buffer, sizeof(struct S_time)))
	{
		//W1_RTC_DEBUG("seattle-2 \n");
		time_display(&time);
		//W1_RTC_DEBUG("seattle-3 \n");
	}
	//W1_RTC_DEBUG("seattle-4 \n");
}

int rtc_get_time(struct S_time *time)
{
	int ret = 0;
	uint8_t *buffer;
	
	buffer = (uint8_t*)time;
	if(rs5c372a_registers_read(RS5C372_SECONDS, buffer, sizeof(struct S_time)))
	{
		time_display(time);
		return ret;
	}
	
	return 1;
}

bool rtc_reset_time(void)
{
	W1_RTC_DEBUG("Reset RTC time to {2000-01-01 6 00:00:00} \n");
	uint8_t time_init[sizeof(struct S_time)] = 
	{
		0x00,
		0x00,
		0x00,
		0x06,
		0x01,
		0x01,
		0x00
	};
	
	//if(!rs5c372a_register_write(RS5C372_CTLREG2, 0x20))  // set control register2 to make 24h mode
	//	return false;
	
	return rs5c372a_registers_write(RS5C372_SECONDS, time_init, sizeof(struct S_time));
}

bool rtc_set_time(struct S_time *time)
{
	uint8_t *time_tmp;
	char str[20];
	char i;
	
	W1_RTC_DEBUG("Set RTC time \n");
	//if(!rs5c372a_register_write(RS5C372_CTLREG2, 0x20))  // set control register2 to make 24h mode
	//	return false;
	time_tmp = (uint8_t *)time;
	for(i=0;i<7;i++)
	{
		sprintf(str,"%02x ",*(time_tmp+i));
		W1_RTC_DEBUG(str);
	}
	return rs5c372a_registers_write(RS5C372_SECONDS, time_tmp, sizeof(struct S_time));
}
