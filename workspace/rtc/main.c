#include "nrf.h"
#include "nrf_gpio.h"
#include "nrf_drv_rtc.h"
#include "nrf_drv_clock.h"
#include "nrf_drv_gpiote.h"
#include "nrf_log_ctrl.h"
#include "nrf_log.h"
#include "nrf_log_default_backends.h"
#include "boards.h"
#include "app_error.h"
#include <stdint.h>
#include <stdbool.h>

#define COMPARE_COUNTERTIME  (3UL) /**< Get Compare event COMPARE_TIME seconds after the counter starts from 0. */
#define PIN_IN 19
#define PIN_MOTOR 2

const nrf_drv_rtc_t rtc = NRF_DRV_RTC_INSTANCE(0);

int last_read = 0;

int time_8hz = 0;
int time_sec = 0;
int time_min = 0;

int open_duration_min = 60;
int close_duration_min = 1380;

bool valve_closed = true;
bool valve_should_close = true;

/** @brief: Function for handling the RTC0 interrupts.
 * Triggered on TICK and COMPARE0 match.
 */
static void rtc_handler(nrf_drv_rtc_int_type_t int_type)
{
    if (valve_closed == true && valve_should_close == true) {
        time_8hz++;
        if (time_8hz == 8) {
            time_8hz = 0;
            time_sec++;
        }
        if (time_sec == 60) {
            NRF_LOG_INFO("closed for %d seconds", time_sec);
            time_sec = 0;
            time_min++;
        }
        if (time_min >= close_duration_min) {
            time_min = 0;
            valve_should_close = false;
        }
    }
    
    if (valve_closed == false && valve_should_close == false) {
        time_8hz++;
        if (time_8hz == 8) {
            NRF_LOG_INFO("closed for %d seconds", time_sec);
            time_8hz = 0;
            time_sec++;
        }
        if (time_sec == 60) {
            time_sec = 0;
            time_min++;
        }
        if (time_min >= open_duration_min) {
            time_min = 0;
            valve_should_close = true;
        }
    }
}

/** @brief Function configuring gpio for pin toggling.
 */
static void leds_config(void)
{
    bsp_board_init(BSP_INIT_LEDS);
}

/** @brief Function starting the internal LFCLK XTAL oscillator.
 */
static void lfclk_config(void)
{
    ret_code_t err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_clock_lfclk_request(NULL);
}

/** @brief Function initialization and configuration of RTC driver instance.
 */
static void rtc_config(void)
{
    uint32_t err_code;

    //Initialize RTC instance
    nrf_drv_rtc_config_t config = NRF_DRV_RTC_DEFAULT_CONFIG;
    config.prescaler = 4095;
    err_code = nrfx_rtc_init(&rtc, &config, rtc_handler);
    APP_ERROR_CHECK(err_code);

    //Enable tick event & interrupt
    nrfx_rtc_tick_enable(&rtc,true);

    //Set compare channel to trigger interrupt after COMPARE_COUNTERTIME seconds
    err_code = nrfx_rtc_cc_set(&rtc, 0 ,COMPARE_COUNTERTIME * 8,true);
    APP_ERROR_CHECK(err_code);

    //Power on RTC instance
    nrfx_rtc_enable(&rtc);
}

/**@brief Function for initializing the nrf log module.
 */

uint32_t get_rtc_counter(void)
{
    return NRF_RTC0->COUNTER;
}

static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(get_rtc_counter, 8);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

void gpio_init() 
{
    nrf_gpio_cfg_input(PIN_IN, NRF_GPIO_PIN_PULLDOWN);
    nrf_gpio_cfg_output(PIN_MOTOR);
    // nrf_gpio_cfg_output(BSP_LED_1);
}

/**
 * @brief Function for application main entry.
 */

void update_motor() {
    if (valve_closed == true && valve_should_close == false) 
    {
        // nrf_gpio_pin_write(BSP_LED_1, 0);
		nrf_gpio_pin_write(PIN_MOTOR, 1);
        NRF_LOG_INFO("Valve opening...");
    }
	else if (valve_closed == false && valve_should_close == true) 
    {
        // nrf_gpio_pin_write(BSP_LED_1, 0);
		nrf_gpio_pin_write(PIN_MOTOR, 1);
        NRF_LOG_INFO("Valve closing...");
    }
	else 
    {
        // nrf_gpio_pin_write(BSP_LED_1, 1);
		nrf_gpio_pin_write(PIN_MOTOR, 0);
    }
}

int main(void)
{
    log_init();
    gpio_init();
    leds_config();

    lfclk_config();

    rtc_config();

    valve_closed = !nrf_gpio_pin_read(PIN_IN);

    while (true)
    {
        int val = nrf_gpio_pin_read(PIN_IN);
        if (val != last_read) {
            last_read = val;
            valve_closed = !val;
            NRF_LOG_INFO("changed");
        }
        update_motor();
        if (!NRF_LOG_PROCESS()) {
            __SEV();
            __WFE();
            __WFE();
        }
    }
}


/**  @} */
