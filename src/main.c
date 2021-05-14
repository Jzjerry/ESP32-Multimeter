/**
 * @file: main.c
 * @author: Jzjerry
 * @brief: Main Task of Multimeter System
 * */

#include "pinconf.h"
#include "switch.h"
#include "ads1113.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "sdkconfig.h"

#include <esp_log.h>
#include <driver/gpio.h>
#include <driver/i2c.h>

#include <u8g2_esp32_hal.h>
#include <u8g2.h>

#include <stdio.h>
#include <string.h>

/* Defines */

/* Pravite Global varibles */
static const char *SSD = "ssd1306";
static const char *MAIN = "main";

static const char *LOAD_FONT[]={"Working.","Working..","Working..."};
static const char *STATE_FONT[]={"AC Set","DC Set","I Set","R Set","No Set"};
static uint8_t load_circle = 0;

/* Local Global varibles */
volatile uint8_t setting_pin = 0;

/* External Global Variables */
extern volatile state_t setting_state;
extern volatile double ex_adc_data;
extern const char* unit_str;



SemaphoreHandle_t  gpio_sem = NULL;

/* Functions */
void u8g2_board_init();
uint8_t polling_test();

/* Intrrput */
void IRAM_ATTR gpio_isr_handler(void* arg);

/* Tasks */
void task_oled(void *ignore);

extern void gpio_task(void *ignore);
extern void ex_adc_task(void *ignore);

/* Setup Task */
void app_main() 
{
  /* Peripherals Init */
  GPIO_Init();
  I2C_Init();
  ADC_Init();

  /* Devices Init */
  u8g2_board_init();
  switch_init();
  ads1113_init();

  //create a Semaphore to handle gpio event from isr
  gpio_sem = xSemaphoreCreateBinary();


  // sub tasks
  xTaskCreate(gpio_task, "gpio task", 2048, NULL, 11, NULL);
  xTaskCreate(task_oled, "U8G2 Task", 4096, NULL, 10, NULL);
  xTaskCreate(ex_adc_task, "ADS1113 Task", 2048, NULL, 11, NULL);
  ESP_LOGI(MAIN,"task created!");
  while(1)
  {
    vTaskDelay(500 / portTICK_RATE_MS);
    // ESP_LOGI(MAIN, "polling read: %d state: %d", polling_test(), setting_state);
    // ESP_LOGI(MAIN,"ADC caclulated as %lf %s", ex_adc_data, unit_str);
    // ESP_LOGI(MAIN, "app_main Heart Beat %d", load_circle % 3);
  } // Wait
}

/* Screen Display Task */
void task_oled(void *ignore)
{
  u8g2_t u8g2;

  u8g2_Setup_ssd1306_i2c_128x64_noname_f(
    &u8g2,
    U8G2_R0,
    u8g2_esp32_i2c_byte_cb,
    u8g2_esp32_gpio_and_delay_cb);
  u8x8_SetI2CAddress(&u8g2.u8x8,0x78);
  ESP_LOGI(SSD, "u8g2 using I2C");


  u8g2_InitDisplay(&u8g2); // send init sequence to the display, display is in sleep mode after this
  ESP_LOGI(SSD, "u8g2 screen init");
  u8g2_SetPowerSave(&u8g2, 0); // wake up display
  u8g2_ClearBuffer(&u8g2);

	ESP_LOGI(SSD, "u8g2_ClearBuffer");
	// ESP_LOGI(SSD, "u8g2_SetFont");
  // u8g2_SetFont(&u8g2, u8g2_font_sticker_mel_tr);

  double test_data = 50.0; // Change it to global resource
  char data[10];
  while(1)
  {
    sprintf(data, "%5.3lf", ex_adc_data);
    data[6] = '\0'; // String cut down
    u8g2_ClearBuffer(&u8g2);
    //ESP_LOGI(SSD, "u8g2_DrawStr");
    u8g2_SetFont(&u8g2, u8g2_font_sticker_mel_tr);
    u8g2_DrawStr(&u8g2, 10, 10, LOAD_FONT[load_circle % 3]);
    u8g2_DrawStr(&u8g2, 100, 44, unit_str);
    u8g2_SetFont(&u8g2, u8g2_font_logisoso24_tn);
    u8g2_DrawStr(&u8g2, 10, 44, data);
    u8g2_SetFont(&u8g2, u8g2_font_tenstamps_mf);
    u8g2_DrawStr(&u8g2, 10, 60, STATE_FONT[setting_state]);
	  //ESP_LOGI(SSD, "u8g2_SendBuffer");
	  u8g2_SendBuffer(&u8g2);
    vTaskDelay(500 / portTICK_RATE_MS);
    load_circle ++ ;
    test_data = (test_data<=200)?test_data+ 0.01:0.0;
  }

	ESP_LOGI(SSD, "All done!");

  vTaskDelete(NULL);
}

void u8g2_board_init()
{
  u8g2_esp32_hal_t u8g2_esp_handle = U8G2_ESP32_HAL_DEFAULT;

  u8g2_esp_handle.sda = LED_I2C_SDA;
  u8g2_esp_handle.scl = LED_I2C_SCL;
  u8g2_esp_handle.dc = LED_I2C_DC;
  u8g2_esp_handle.reset = LED_I2C_RES;
  

  u8g2_esp32_hal_init(u8g2_esp_handle);
}

uint8_t polling_test()
{
    return ((gpio_get_level(VAC_SETTING)<<3) | 
            (gpio_get_level(VDC_SETTING)<<2) | 
            (gpio_get_level(I_SETTING)<<1) | 
            (gpio_get_level(R_SETTING)));
}


void IRAM_ATTR gpio_isr_handler(void* arg)
{
    setting_pin = (uint8_t) arg;
    xSemaphoreGiveFromISR(gpio_sem, pdFALSE);
}