     
/*******************************************************************
 *                        GPIO USEAGE SHEET
 * | PIN | USEAGE      | DIRECTION | PIN | USEAGE      | DIRECTION |
 * | P34 | VDC_SETTING | IN_PULLUP | P4  | RESISTOR_S4 | OUTPUT    |
 * | P35 | VAC_SETTING | IN_PULLUP | P0  | RESISTOR_S3 | OUTPUT    | 
 * | P32 | I_SETTING   | IN_PULLUP | P19 | LED_I2C_SCL | I2C       |
 * | P33 | R_SETTING   | IN_PULLUP | P18 | LED_I2C_SDA | I2C       |
 * | P27 | ADC_I2C_RDY | INPUT     | P5  | LED_I2C_RES | OUTPUT    |
 * | P14 | ADC_I2C_SCL | I2C       | P17 | LED_I2C_D/C | OUTPUT    |
 * | P26 | ADC_I2C_SDA | I2C       | P23 | MEASURE_SWB | OUTPUT    |
 * | P13 | ONCHIP_ADC  | ADC_INPUT | P22 | MEASURE_SWA | OUTPUT    |
 * |     |             |           | P2  | RESISTOR_S2 | OUTPUT    |
 * |     |             |           | P15 | RESISTOR_S1 | OUTPUT    |
 * 
 *******************************************************************/

#ifndef __PINCONF_H_
#define __PINCONF_H_

#include <stdint.h>
#include <esp_err.h>


/* Right Side's Pins */
#define MEASURE_SWB     (int)23
#define MEASURE_SWA     (int)22
#define LED_I2C_SCL     (int)19
#define LED_I2C_SDA     (int)18
#define LED_I2C_RES     (int)5
#define LED_I2C_DC      (int)17
#define RESISTOR_S4     (int)4
#define RESISTOR_S3     (int)0
#define RESISTOR_S2     (int)2
#define RESISTOR_S1     (int)15

/* Left Side's Pins */
#define VDC_SETTING     (int)34
#define VAC_SETTING     (int)35
#define I_SETTING       (int)32
#define R_SETTING       (int)33
#define ADC_I2C_RDY     (int)27
#define ADC_I2C_SCL     (int)14
#define ADC_I2C_SDA     (int)26
#define ONCHIP_ADC      (int)13


/* Setting Mask */
#define VAC_SETTING_MASK (uint8_t)0b0111
#define VDC_SETTING_MASK (uint8_t)0b1011
#define I_SETTING_MASK   (uint8_t)0b1101
#define R_SETTING_MASK   (uint8_t)0b1110
#define NULL_MASK        (uint8_t)0b1111
#define ERROR_MASK       (uint8_t)0


#define ADC_I2C_PORT 0

/* Pin Utility Init Functions */
void GPIO_Init();
esp_err_t I2C_Init();
esp_err_t ADC_Init();

#endif