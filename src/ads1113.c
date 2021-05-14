
#include "ads1113.h"
#include "switch.h"
#include "pinconf.h"
#include "ac_calc.h"
#include "resistor_calc.h"

#include <esp_err.h>
#include <esp_log.h>
#include <driver/i2c.h>
#include <driver/adc.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define WRITE_BIT I2C_MASTER_WRITE
#define READ_BIT I2C_MASTER_READ

#define ACK_CHECK_EN 1
#define ACK_VAL 0x0
#define NACK_VAL 0x1

#define CONFIG_REG_ADDR 0x01 

#define ADS1113_READ_PERIOD_MS 500
#define ADS1113_READ_PERIOD_MS_FAST 2

/* Private Function */
static esp_err_t ads1113_get(uint8_t *data_h,uint8_t *data_l);

/* ADS1113 config register content */
static uint8_t config_reg[]={0x84,0x83};
// static uint8_t config_reg[]={0xC4,0x83};

/* ADS1113 Address Pointer register content */
static uint8_t addr_ptr_reg = 0x00;

static const char* ADSTAG = "ads1113";
/* unit string list */
static const char* unit_V = " V";
static const char* unit_mV = "mV";
static const char* unit_A = " A";
static const char* unit_mA = "mA";
static const char* unit_O = " O";
static const char* unit_KO = "KO";
static const char* unit_MO = "MO";

// static const double one_sqrttwo = 0.707106;

/* External Global Varibales */
extern volatile uint8_t setting_state;
extern volatile double ac_result;
extern volatile double res_data;

/* Local Global Variables */
volatile double ex_adc_data;
const char* unit_str = "";

void ads1113_init()
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    esp_err_t ret;
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ADS_1113_ADDR<<1| WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, CONFIG_REG_ADDR, ACK_CHECK_EN);
    i2c_master_write(cmd, config_reg, sizeof(config_reg), ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(ADC_I2C_PORT, cmd, 1000 / portTICK_RATE_MS);
    if(ret != ESP_OK) 
    {
        ESP_LOGE(ADSTAG,"i2c communicate error while init config_reg");
        return;
    }
    i2c_cmd_link_delete(cmd);

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ADS_1113_ADDR<<1| WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, addr_ptr_reg, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(ADC_I2C_PORT, cmd, 1000 / portTICK_RATE_MS);
    if(ret != ESP_OK) 
    {
        ESP_LOGE(ADSTAG,"i2c communicate error while init ptr_reg");
        return;
    }
    i2c_cmd_link_delete(cmd);
}

static esp_err_t ads1113_get(uint8_t *data_h,uint8_t *data_l)
{

    esp_err_t ret = ESP_OK;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ADS_1113_ADDR<<1| READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, data_h, ACK_VAL);
    i2c_master_read_byte(cmd, data_l, NACK_VAL);
    i2c_master_stop(cmd);
    ret = (i2c_master_cmd_begin(ADC_I2C_PORT, cmd, 1000 / portTICK_RATE_MS));
    i2c_cmd_link_delete(cmd);
    return ret;
}


void ex_adc_task(void* ignore)
{
    double bias = 0;
    // double bias_p = 0.002;
    // double bias_n = 0.0034;
    double gain = 10.0;
    double v_ref = 2.048;
    int16_t adc_out = 0;
    double basic_adc_data;
    cmp_control(R_NULL);

    while(1)
    {
        uint8_t data_h,data_l;
        uint8_t ads_timeout = 10;
        while(ads1113_get(&data_h, &data_l) != ESP_OK)
        {
            ads_timeout--;
            if(ads_timeout == 0)
            {
                ESP_LOGW(ADSTAG,"ADS Timeout occured, data is aborted!");
                break;
            }
            vTaskDelay( 5 / portTICK_RATE_MS);
        }
        if( ads_timeout != 0 )
        {
            adc_out = (int16_t)data_h<<8|(int16_t)data_l;
            adc_out = 0 - adc_out;
            // ESP_LOGI(ADSTAG,"ADC read %d,MSB: %o, LSB: %o", adc_out, data_h, data_l);
            basic_adc_data = gain*((double)adc_out*(double)v_ref/(double)0x7fff) - bias; // basic voltage value
            // ESP_LOGI(ADSTAG,"ADC read as %lf", basic_adc_data);
            basic_adc_data = 1.01*basic_adc_data - 0.03; // correction 
            switch (setting_state)
            {
            case VAC_STATE:
                if(wave_eff_value(basic_adc_data))
                {
                    ESP_LOGI(ADSTAG,"AC Completed!");
                    double ac_data;
                    ac_data = avg_filter(ac_result);
                    if(ac_data < 1.0 )
                    {
                        ex_adc_data = 1000.0*ac_data;
                        unit_str = unit_mV;
                    }
                    else
                    {
                        ex_adc_data = ac_data;
                        unit_str = unit_V;
                    }
                }
                break;
            case VDC_STATE:
                if(basic_adc_data < 1.0 && basic_adc_data > -1.0)
                {
                    ex_adc_data = 1000.0*basic_adc_data;
                    unit_str = unit_mV; 
                }
                else
                {
                    ex_adc_data = basic_adc_data;
                    unit_str = unit_V;
                }
                break;
            case I_STATE:
                basic_adc_data = basic_adc_data/20;
                if(basic_adc_data < 1.0 && basic_adc_data > -1.0)
                {
                    ex_adc_data = 1000.0*basic_adc_data;
                    unit_str = unit_mA;
                }
                else
                {
                    ex_adc_data = basic_adc_data;
                    unit_str = unit_A;
                }
                break;
            case R_STATE:
                basic_adc_data = (double)adc_out*(double)v_ref/(double)0x7fff;
                if(resistor_calc(basic_adc_data))
                {
                    ESP_LOGI(ADSTAG,"Resistor Measure Completed!");
                    if(res_data > 1e6 )
                    {
                        ex_adc_data = res_data/1e6;
                        unit_str = unit_MO;
                    }
                    else if(res_data > 1e3 )
                    {
                        ex_adc_data = res_data/1e3;
                        unit_str = unit_KO;
                    }
                    else
                    {
                        ex_adc_data = res_data;
                        unit_str = unit_O;
                    }
                }
                break;
            default:
                ex_adc_data = basic_adc_data;
                unit_str = "?";
                break;
            }
        }
        vTaskDelay(ADS1113_READ_PERIOD_MS_FAST / portTICK_RATE_MS);
    }
}