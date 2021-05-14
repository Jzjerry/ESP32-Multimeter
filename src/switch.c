
#include "switch.h"
#include "pinconf.h"
#include "esp_log.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
//#include <freeRTOS/queue.h>

/* private functions */
static void _switch_isr_enable(int io_num);
static void _switch_isr_disable();

static const char *SWTAG = "switch";

#define READ(X) gpio_get_level(X)

/* External Global Varibles */
extern uint8_t setting_pin;
extern SemaphoreHandle_t gpio_sem;

/* Local Global Varibles */
volatile state_t setting_state;


void switch_init()
{
    uint8_t read_mask = 0;
    read_mask = (READ(VAC_SETTING)<<3) | 
                (READ(VDC_SETTING)<<2) | 
                (READ(I_SETTING)<<1) | 
                (READ(R_SETTING));
    setting_channel(read_mask);
    setting_state = mask2state(read_mask);
    ESP_LOGI(SWTAG,"switch state set as %d",setting_state);
}

void gpio_task(void *ignore)
{
    while(1)
    {
        if( xSemaphoreTake( gpio_sem, portMAX_DELAY ) == pdTRUE )
        {
            _switch_isr_disable();
            ESP_LOGI(SWTAG,"Taking Semaphore From GPIO ISR, ISR disabled");

            uint8_t read_mask = 0;
            uint8_t sw_mask = 0;

            sw_mask = get_mask(setting_pin);
            vTaskDelay(20 / portTICK_RATE_MS); // debounce Delay
            read_mask = (READ(VAC_SETTING)<<3) | 
                        (READ(VDC_SETTING)<<2) | 
                        (READ(I_SETTING)<<1) | 
                        (READ(R_SETTING));
            if(read_mask == sw_mask)
            {
                setting_state = mask2state(sw_mask);
                ESP_LOGI(SWTAG,"Setting confirm as %d", sw_mask);
                setting_channel(sw_mask);
            }
            else
            {
                ESP_LOGW(SWTAG,"Setting doesn't match: expected %d, read %d", sw_mask, read_mask);
            }
        }
        vTaskDelay(500 / portTICK_RATE_MS);
        _switch_isr_enable(setting_pin); // Lock-up triggered pin isr
        ESP_LOGI(SWTAG,"Leaving Setting, ISR enabled");
    }
}

void setting_channel(const uint8_t mask)
{
    switch (mask)
    {
    case VAC_SETTING_MASK:
        gpio_set_level(MEASURE_SWB,0);
        gpio_set_level(MEASURE_SWA,1);
        ESP_LOGI(SWTAG,"Setting at VAC Measure");
        break;

    case VDC_SETTING_MASK:
        gpio_set_level(MEASURE_SWB,0);
        gpio_set_level(MEASURE_SWA,1);
        ESP_LOGI(SWTAG,"Setting at VDC Measure");
        break;

    case I_SETTING_MASK:
        gpio_set_level(MEASURE_SWB,1);
        gpio_set_level(MEASURE_SWA,1);
        ESP_LOGI(SWTAG,"Setting at I Measure");
        break;

    case R_SETTING_MASK:
        gpio_set_level(MEASURE_SWB,1);
        gpio_set_level(MEASURE_SWA,0);
        ESP_LOGI(SWTAG,"Setting at R Measure");
        break;
    
    default:
        gpio_set_level(MEASURE_SWB,0);
        gpio_set_level(MEASURE_SWA,0);
        ESP_LOGI(SWTAG,"Setting at NULL");
        break;
    }
}

uint8_t get_mask(int pin)
{
    switch (pin)
    {
    case VAC_SETTING:
        return VAC_SETTING_MASK;
        break;
    case VDC_SETTING:
        return VDC_SETTING_MASK;
        break;
    case I_SETTING:
        return I_SETTING_MASK;
        break;
    case R_SETTING:
        return R_SETTING_MASK;
        break;
    default:
        return ERROR_MASK;
        break;
    }
}

static void _switch_isr_enable(int io_num)
{
    switch (io_num)
    {
    case VAC_SETTING:
        gpio_intr_disable(VAC_SETTING);
        gpio_intr_enable(VDC_SETTING);
        gpio_intr_enable(I_SETTING);
        gpio_intr_enable(R_SETTING);
        break;
    case VDC_SETTING:
        gpio_intr_enable(VAC_SETTING);
        gpio_intr_disable(VDC_SETTING);
        gpio_intr_enable(I_SETTING);
        gpio_intr_enable(R_SETTING);
        break;
    case I_SETTING:
        gpio_intr_enable(VAC_SETTING);
        gpio_intr_enable(VDC_SETTING);
        gpio_intr_disable(I_SETTING);
        gpio_intr_enable(R_SETTING);
        break;
    case R_SETTING:
        gpio_intr_enable(VAC_SETTING);
        gpio_intr_enable(VDC_SETTING);
        gpio_intr_enable(I_SETTING);
        gpio_intr_disable(R_SETTING);
        break;
    default:
        gpio_intr_enable(VDC_SETTING);
        gpio_intr_enable(VDC_SETTING);
        gpio_intr_enable(I_SETTING);
        gpio_intr_enable(R_SETTING);
        break;
    }
}
static void _switch_isr_disable()
{
    gpio_intr_disable(VAC_SETTING);
    gpio_intr_disable(VDC_SETTING);
    gpio_intr_disable(I_SETTING);
    gpio_intr_disable(R_SETTING);
}

state_t mask2state(uint8_t mask)
{
    return (mask == VAC_SETTING_MASK) ? VAC_STATE :
        (mask == VDC_SETTING_MASK) ? VDC_STATE :
        (mask == I_SETTING_MASK) ? I_STATE :
        (mask == R_SETTING_MASK) ? R_STATE :
        NULL_STATE;
}