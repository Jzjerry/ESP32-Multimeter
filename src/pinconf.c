#include "pinconf.h"

#include <driver/gpio.h>
#include <driver/i2c.h>
#include <driver/adc.h>
#include <esp_adc_cal.h>
#include <esp_log.h>

#define OUTPUT_PIN_SEL ((1ULL<<MEASURE_SWB)|(1ULL<<MEASURE_SWA)|\
                        (1ULL<<RESISTOR_S2)|(1ULL<<RESISTOR_S1)|\
                        (1ULL<<RESISTOR_S4)|(1ULL<<RESISTOR_S3))

#define INPUT_PU_PIN_SEL ((1ULL<<VDC_SETTING)|(1ULL<<VAC_SETTING)|\
                          (1ULL<<I_SETTING)|(1ULL<<R_SETTING))



#define USING_ADC2

#define ESP_INTR_FLAG_DEFAULT 0

extern void IRAM_ATTR gpio_isr_handler(void* arg);

static const adc_channel_t adc_channel = ADC_CHANNEL_4;
// static const adc_bits_width_t adc_width = ADC_WIDTH_BIT_12;
static const char* INIT = "Initialize";

esp_adc_cal_value_t val_type;
esp_adc_cal_characteristics_t *adc_chars;


void GPIO_Init()
{
    gpio_config_t io_conf;

    //disable interrupt
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set
    io_conf.pin_bit_mask = OUTPUT_PIN_SEL;/* OUTPUT_PINS */
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    io_conf.intr_type = GPIO_PIN_INTR_NEGEDGE;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //bit mask of the pins that you want to set
    io_conf.pin_bit_mask = INPUT_PU_PIN_SEL;/* INPUT_PINS */
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 1;
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);

    gpio_isr_handler_add(VAC_SETTING, gpio_isr_handler, (void*)VAC_SETTING);
    gpio_isr_handler_add(VDC_SETTING, gpio_isr_handler, (void*)VDC_SETTING);
    gpio_isr_handler_add(I_SETTING, gpio_isr_handler, (void*)I_SETTING);
    gpio_isr_handler_add(R_SETTING, gpio_isr_handler, (void*)R_SETTING);

    // RDY unavailable for ADS1113
    // /* Set intrrupt Enable */ 
    // io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    // //bit mask of the pins that you want to set
    // io_conf.pin_bit_mask = (1ULL<<ADC_I2C_RDY);/* ADC_RDY_PIN */
    // //disable pull-down mode
    // io_conf.pull_down_en = 0;
    // //disable pull-up mode
    // io_conf.pull_up_en = 0;
    // ESP_ERROR_CHECK(gpio_config(&io_conf));

    ESP_LOGI(INIT,"GPIO_Init Complete");
}


esp_err_t I2C_Init()
{
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = ADC_I2C_SDA;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = ADC_I2C_SCL;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 300000; // 300KHz

    i2c_param_config(ADC_I2C_PORT, &conf);
    return i2c_driver_install(ADC_I2C_PORT, conf.mode, 0, 0, 0);
}

esp_err_t ADC_Init()
{
    gpio_num_t adc2_io;
    adc2_pad_get_io_num(adc_channel, &adc2_io);
    if(adc2_io == ONCHIP_ADC)
    {
        adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
        val_type = esp_adc_cal_characterize(ADC_UNIT_2, ADC_ATTEN_11db, ADC_WIDTH_BIT_12, 3900 , adc_chars);
        return adc2_config_channel_atten( adc_channel, ADC_ATTEN_11db );
    }
    ESP_LOGI(INIT,"io_num doesn't match adc2, expected: %d, used: %d",adc2_io,ONCHIP_ADC);
    return ESP_FAIL;
}