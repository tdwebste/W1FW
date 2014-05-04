#ifndef DATA_TRANSFER_H__
#define DATA_TRANSFER_H__

#include <stdint.h>
#include "main.h"
#include "ble_nus.h"

#define TRANS_NEXT	1
#define TRANS_LAST	0

bool transfer_config(void);
bool transfer_start(void);
//bool transfer_identify(void);
//bool transfer_data(uint8_t* page_num,uint8_t* end_roll);
bool W1_transfer_data(struct S_device_config* device_config_transfer, uint8_t TransMode);
//bool transfer_end(void);

void ble_transfer_data(uint8_t * data, uint16_t length);
void on_command_handler(char * data);


//void transfer_test(void);
#endif // DATA_TRANSFER_H__

