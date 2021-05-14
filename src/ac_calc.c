
#include "ac_calc.h"
#include <fastmath.h>
#include <esp_err.h>
#include <esp_log.h>

#define WIN_SIZE 5

volatile double ac_result;



bool wave_eff_value(double input)
{
    static double last_input;
    static unsigned long long n;
    static double result;
    static bool complete;

    if(last_input <= 0 && input > 0)
    {
        // accumulate start
        result = 0;
        n = 0;
        complete = false;
    }
    else if(last_input > 0 && input <= 0)
    {
        // accumulate end
        complete = true;
    }

    last_input = input;

    if(complete)
    {
        ac_result = sqrt(result/(double)n);
         ESP_LOGI("VAC CALC","AC Voltage Effective Value: %lf", ac_result);
         complete = false;
        return true;
    }
    else
    {
        n++;
        result = result + input*input;
        return false;
    }
}

double avg_filter(double input)
{
    static uint8_t win_now = 1;
    static uint8_t win_max = WIN_SIZE;
    static double win_array[WIN_SIZE+1];
    
    static uint8_t ptr = 0;
    double result = 0;

    win_array[ptr] = input;
    int i = 0;
    for(i=0;i<win_now;i++)
    {
        result += win_array[i];
    }
    result /= win_now;

    if(win_now<win_max)
    {
        win_now++;
    }

    ptr++;

    if(ptr>=win_now)
    {
        ptr = 0;
    }
    return result;
}