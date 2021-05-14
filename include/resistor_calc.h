#ifndef __RESISTOR_CALC_H_
#define __RESISTOR_CALC_H_


#include <stdbool.h>

typedef enum R_SET
{
    R_2M,
    R_200K,
    R_20K,
    R_2K,
    R_200,
    R_NULL
} RESISTOR_SET;


double get_resistor_adc();
bool resistor_calc(double input);
void cmp_control(RESISTOR_SET set);

#endif