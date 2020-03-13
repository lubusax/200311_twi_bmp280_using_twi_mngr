#include "nrf_drv_gpiote.h"

#include <stdio.h>
#include "boards.h"
#include "app_util_platform.h"
#include "app_timer.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_drv_clock.h"
#include "bsp.h"
#include "app_error.h"
#include "nrf_twi_mngr.h"
#include "hdc1080.h"
#include "compiler_abstraction.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_delay.h"
#include "math.h"

#define TWI_INSTANCE_ID             0

#define MAX_PENDING_TRANSACTIONS    5

NRF_TWI_MNGR_DEF(m_nrf_twi_mngr, MAX_PENDING_TRANSACTIONS, TWI_INSTANCE_ID);
// APP_TIMER_DEF(m_timer);
APP_TIMER_DEF(m_single_shot_timer_id);

// Pin number for indicating communication with sensors.
#ifdef BSP_LED_3
    #define READ_ALL_INDICATOR  BSP_BOARD_LED_3
#else
    #error "Please choose an output pin"
#endif

static uint32_t m_timeout = 2000; // in ms

ret_code_t result_mngr_perform;

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
static void twi_config(void)
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

// If a SoftDevice is not enabled (like in this app),
// the LFCLK must be requested explicitly. 
// One way of doing this is by using the Clock driver. 
static void lfclk_request(void)
{
    uint32_t err_code;

    err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_clock_lfclk_request(NULL);
}

static void read_t_and_hr(void)
{
    ret_code_t err_code;

    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();

    
//    NRF_LOG_RAW_INFO("\r\nHDC1080 T and HR sensor with timer \r\n");
//    NRF_LOG_FLUSH();

    nrf_delay_ms(15); 

    // Initialize sensor
    result_mngr_perform = nrf_twi_mngr_perform(&m_nrf_twi_mngr, 
                                        NULL, hdc1080_init_transfers,
                                        HDC1080_INIT_TRANSFER_COUNT, NULL);
//    NRF_LOG_RAW_INFO("\r\nResult Init Sensor: %d \r\n",result_mngr_perform);
//    NRF_LOG_FLUSH();
    APP_ERROR_CHECK(result_mngr_perform);

    nrf_delay_ms(20);

    result_mngr_perform = nrf_twi_mngr_perform(&m_nrf_twi_mngr,
                                        NULL, transfer_write_temp,
                                        1, NULL);
    nrf_delay_ms(20);

    result_mngr_perform = nrf_twi_mngr_perform(&m_nrf_twi_mngr,
                                        NULL, transfer_read_temp,
                                        1, NULL);

    temperature = ((((((int16_t)m_temp_and_hr_buffer[0] << 8) | \
      m_temp_and_hr_buffer[1])) / pow(2.0f, 16.0f)) * 165.0f - 40.0f); // in °C
 
    relative_humidity = ((((((int16_t)m_temp_and_hr_buffer[2] << 8) | \
      m_temp_and_hr_buffer[3])) / pow(2.0f, 16.0f)) * 100.0f); // in %
    

//    NRF_LOG_RAW_INFO("\r\nResult Read T Register once: %d \r\n",
//                    result_mngr_perform);
//    NRF_LOG_RAW_INFO("\r\nT Register 2 bytes: %x %x\r\n",
//                    m_temp_and_hr_buffer[0], m_temp_and_hr_buffer[1]);
//    NRF_LOG_RAW_INFO("\r\nHR Register 2 bytes: %x %x\r\n",
//                    m_temp_and_hr_buffer[2], m_temp_and_hr_buffer[3]);
    NRF_LOG_RAW_INFO("Temperature " NRF_LOG_FLOAT_MARKER " C\r\n",
                      NRF_LOG_FLOAT(temperature));
    NRF_LOG_RAW_INFO("Relative Humidity " NRF_LOG_FLOAT_MARKER " %% \r\n",
                      NRF_LOG_FLOAT(relative_humidity) );


    NRF_LOG_FLUSH();
    APP_ERROR_CHECK(result_mngr_perform);

    err_code = app_timer_start(m_single_shot_timer_id,
                APP_TIMER_TICKS(m_timeout), NULL);
    APP_ERROR_CHECK(err_code);
 
}

static void single_shot_timer_handler(void * p_context)
{
    read_t_and_hr();
}


static void create_timers()
{
    ret_code_t err_code;

    // Create timers
    err_code = app_timer_create(&m_single_shot_timer_id,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                single_shot_timer_handler);
    APP_ERROR_CHECK(err_code);
}

int main(void)
{
    ret_code_t err_code;

    // log_init();
    //bsp_board_init(BSP_INIT_LEDS);

    lfclk_request(); // the LFCLK is requested
    app_timer_init(); //called before any calls to the Timer API
    create_timers();

    //bsp_config();

    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);

    twi_config();


    // Read Temperature Register once and start single shot timer
    // and log_init()
    read_t_and_hr();

    /////////////////////////////////////////


    while (true)
    {
        nrf_pwr_mgmt_run();

        NRF_LOG_FLUSH();
    }
}


/** @} */
