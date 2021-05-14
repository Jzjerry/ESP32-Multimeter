
#include "resistor_calc.h"
#include "pinconf.h"

#include <driver/adc.h>
#include <esp_adc_cal.h>
#include <esp_log.h>
#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static const adc_bits_width_t width = ADC_WIDTH_BIT_12;

static const double res_0[] = {1e6, 1e5, 1e4, 1e3, 100};

volatile double res_data;
RESISTOR_SET res_state;

// extern esp_adc_cal_value_t val_type;
extern esp_adc_cal_characteristics_t *adc_chars;

static double get_ref_v();

bool resistor_calc(double input)
{
    double adc_in;
    // if(res_state == R_2M || res_state == R_200K) adc_in = 2.064;
    // else if (res_state == R_20K) adc_in = 1.876;
    // else adc_in = get_resistor_adc() + get_ref_v();
    // adc_in = get_ref_v();
    adc_in = get_resistor_adc();
    ESP_LOGI("Resistor","Resistor Max: %d",res_state);
    ESP_LOGI("Resistor","ads1113:%lf,adc:%lf",input,adc_in);
    if(input > 0.8*adc_in || res_state == R_NULL)
    {
        if( res_state != R_2M)
            cmp_control( res_state - 1 );
        else
            cmp_control( R_200 );
        return false;
    }
    else if(input < 0.1*adc_in || res_state == R_NULL)
    {
        if( res_state != R_200)
            cmp_control( res_state + 1 );
        else
            cmp_control( R_2M );
        return false;
    }
    else
    {
        res_data = input * res_0[res_state] / (adc_in - input);
        return true;
    }
    vTaskDelay( 10 / portTICK_RATE_MS);
}

double get_resistor_adc()
{
    uint32_t avg_raw = 0;
    double read_out;
    esp_err_t r;
    for(int i = 0;i < 10; i++)
    {   
        int read_raw;
        r = adc2_get_raw(ADC_CHANNEL_4, width, &read_raw);
        if(r != ESP_OK) ESP_LOGW("ADC2","ADC2 Get Data Fail");
        avg_raw += read_raw;
    }
    avg_raw /= 10;
    uint32_t voltage = esp_adc_cal_raw_to_voltage(avg_raw, adc_chars);
    read_out = voltage/1000.0;
    // read_out = (double)avg_raw*3.9/(double)0xfff;
    return read_out;
}

void cmp_control(RESISTOR_SET set)
{
    res_state = set;
    switch (set)
    {
    case R_2M:
        /* code */
        gpio_set_level(RESISTOR_S4, 1);
        gpio_set_level(RESISTOR_S3, 0);
        gpio_set_level(RESISTOR_S2, 0);
        gpio_set_level(RESISTOR_S1, 0);
        break;
    case R_200K:
        gpio_set_level(RESISTOR_S4, 0);
        gpio_set_level(RESISTOR_S3, 1);
        gpio_set_level(RESISTOR_S2, 0);
        gpio_set_level(RESISTOR_S1, 0);
        break;
    case R_20K:
        gpio_set_level(RESISTOR_S4, 0);
        gpio_set_level(RESISTOR_S3, 0);
        gpio_set_level(RESISTOR_S2, 0);
        gpio_set_level(RESISTOR_S1, 0);
        break;
    case R_2K:
        gpio_set_level(RESISTOR_S4, 1);
        gpio_set_level(RESISTOR_S3, 1);
        gpio_set_level(RESISTOR_S2, 1);
        gpio_set_level(RESISTOR_S1, 0);
        break;
    case R_200:
        gpio_set_level(RESISTOR_S4, 1);
        gpio_set_level(RESISTOR_S3, 1);
        gpio_set_level(RESISTOR_S2, 0);
        gpio_set_level(RESISTOR_S1, 1);
        break;

    case R_NULL:
    default:
        gpio_set_level(RESISTOR_S4, 1);
        gpio_set_level(RESISTOR_S3, 1);
        gpio_set_level(RESISTOR_S2, 0);
        gpio_set_level(RESISTOR_S1, 0);
        break;
    }
}

static double get_ref_v()
{
    return (res_state == R_2M) ? 0.07 :
            (res_state == R_200K) ? 0.07 :
            (res_state == R_20K) ? 0.07 :
            (res_state == R_2K) ? 0.04:
            (res_state == R_200) ? 0.04 :
            0.00;
}