#include "nrf_drv_gpiote.h"

#include <stdio.h>
#include "boards.h"
#include "app_util_platform.h"
#include "app_timer.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_drv_clock.h"
#include "bsp.h"
#include "bsp_nfc.h"
#include "app_error.h"
#include "nrf_twi_mngr.h"
#include "hdc1080.h"
#include "compiler_abstraction.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_delay.h"
#include "math.h"
#include "app_scheduler.h"

#define TWI_INSTANCE_ID             0

#define MAX_PENDING_TRANSACTIONS    5

NRF_TWI_MNGR_DEF(m_nrf_twi_mngr, MAX_PENDING_TRANSACTIONS, TWI_INSTANCE_ID);

APP_TIMER_DEF(m_timer_id);

#define SCHED_MAX_EVENT_DATA_SIZE   APP_TIMER_SCHED_EVENT_DATA_SIZE
#define SCHED_QUEUE_SIZE            10

// Pin number for indicating communication with sensors
#define READ_ALL_INDICATOR  BSP_BOARD_LED_3

static uint32_t m_timeout = 20000; // in ms - timer timeout

void * p_context;

static ret_code_t result_mngr_perform;

// temperature and relative humidity related variables
static uint8_t m_temp_and_hr_buffer[4]; // T: bytes 0 and 1; HR: bytes 2 and 3

static nrf_twi_mngr_transfer_t const transfer_write_temp[] =
{
    HDC1080_WRITE_T_AND_HR(&hdc1080_temp_reg_addr)
};

static nrf_twi_mngr_transfer_t const transfer_read_temp[] =
{
    HDC1080_READ_T_AND_HR(&m_temp_and_hr_buffer)
};

float temperature;
float relative_humidity;

#if defined( __GNUC__ ) && (__LINT__ == 0)
    // This is required if one wants to use floating-point values in 'printf'
    // (by default this feature is not linked together with newlib-nano).
    // Please note, however, that this adds about 13 kB code footprint...
    __ASM(".global _printf_float");
#endif

// TWI (with transaction manager) initialization.
static void twi_config_and_init(void)
{
    uint32_t err_code;

    nrf_drv_twi_config_t const config = {
       .scl                = NRF_GPIO_PIN_MAP(0,27), // SCL signal pin
       .sda                = NRF_GPIO_PIN_MAP(0,26), // SDA signal pin
       .frequency          = NRF_DRV_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_LOWEST,
       .clear_bus_init     = false
    };

    err_code = nrf_twi_mngr_init(&m_nrf_twi_mngr, &config);
    APP_ERROR_CHECK(err_code);
} 

static void read_t_and_hr(void)
{


    NRF_LOG_RAW_INFO("\r\nHDC1080 T and HR sensor with timer \r\n");
    NRF_LOG_FLUSH();

    for (int j = 0; j < 3 ; j++)
    {
        bsp_board_leds_on();
        nrf_delay_ms(100);
        bsp_board_leds_off();
        nrf_delay_ms(100);            
    }

    nrf_delay_ms(15); 

    // Initialize sensor
    result_mngr_perform = nrf_twi_mngr_perform(&m_nrf_twi_mngr, 
                                        NULL, hdc1080_init_transfers,
                                        HDC1080_INIT_TRANSFER_COUNT, NULL);
    APP_ERROR_CHECK(result_mngr_perform);

    // nrf_delay_ms(20);

    result_mngr_perform = nrf_twi_mngr_perform(&m_nrf_twi_mngr,
                                        NULL, transfer_write_temp,
                                        1, NULL);
    nrf_delay_ms(15);

    result_mngr_perform = nrf_twi_mngr_perform(&m_nrf_twi_mngr,
                                        NULL, transfer_read_temp,
                                        1, NULL);

    temperature = ((((((int16_t)m_temp_and_hr_buffer[0] << 8) | \
      m_temp_and_hr_buffer[1])) / pow(2.0f, 16.0f)) * 165.0f - 40.0f); // in °C
 
    relative_humidity = ((((((int16_t)m_temp_and_hr_buffer[2] << 8) | \
      m_temp_and_hr_buffer[3])) / pow(2.0f, 16.0f)) * 100.0f); // in %
    

    NRF_LOG_RAW_INFO("Temperature " NRF_LOG_FLOAT_MARKER " C\r\n",
                      NRF_LOG_FLOAT(temperature));
    NRF_LOG_RAW_INFO("Relative Humidity " NRF_LOG_FLOAT_MARKER " %% \r\n",
                      NRF_LOG_FLOAT(relative_humidity) );
    NRF_LOG_FLUSH();

    APP_ERROR_CHECK(result_mngr_perform);


}

static void timer_handler(void * p_context)
{
    //app_sched_pause();
        
    uint32_t err_code = app_timer_stop(m_timer_id);
    APP_ERROR_CHECK(err_code);

    read_t_and_hr();
    
    err_code = app_timer_start(m_timer_id,
                     APP_TIMER_TICKS(m_timeout), NULL);
    APP_ERROR_CHECK(err_code);

    //app_sched_resume();
}


/**@brief Create timers.
 */
static void timer_init()
{
    uint32_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                timer_handler);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function starting the internal LFCLK oscillator.
 *
 * @details This is needed by RTC1 which is used by the Application Timer
 *          (When SoftDevice is enabled the LFCLK is always running and this is not needed).
 *          If a SoftDevice is not enabled (like in this app),
 *          the LFCLK must be requested explicitly. 
 *          One way of doing this is by using the Clock driver.
 */
static void lfclk_request(void)
{
    uint32_t err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);
    nrf_drv_clock_lfclk_request(NULL);
}

/**@brief Function for initializing logging.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/**@brief Function for the Power Management.
 */
static void power_manage(void)
{
    // Use directly __WFE and __SEV macros since the SoftDevice is not available.

    // Wait for event.
    __WFE();
    
    // Clear Event Register.
    __SEV();
    __WFE();
}


int main(void)
{
    lfclk_request();
    log_init();
    bsp_board_init(BSP_INIT_LEDS);
    timer_init();

    NRF_LOG_INFO("Temp and HR Sensor I2C");


    twi_config_and_init();

    // first time read inmediately, do not wait for the first timeout
    read_t_and_hr();

    uint32_t err_code = app_timer_start(m_timer_id,
            APP_TIMER_TICKS(m_timeout), NULL);
    APP_ERROR_CHECK(err_code);

    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);

    while (true)
    {
        app_sched_execute(); // execute all events scheduled since the last time it was called

        power_manage(); // go to sleep
    
    }
}


/** @} */
