#ifndef __ADS1113_H_
#define __ADS1113_H_

#include <stdint.h>


#define ADS_1113_ADDR 0x48

void ads1113_init();
void ex_adc_task(void* ignore);


#endif