#ifndef __SWITCH_H_
#define __SWITCH_H_

#include <driver/gpio.h>
#include <stdint.h>

typedef enum state
{
    VAC_STATE,
    VDC_STATE,
    I_STATE,
    R_STATE,
    NULL_STATE
} state_t;

void gpio_task(void *ignore);
void setting_channel(uint8_t mask);
void switch_init();

uint8_t get_mask(int pin);
state_t mask2state(uint8_t mask);
#endif