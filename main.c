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
#include "lm75b.h"
#include "mma7660.h"
#include "compiler_abstraction.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define TWI_INSTANCE_ID             0

#define MAX_PENDING_TRANSACTIONS    5

NRF_TWI_MNGR_DEF(m_nrf_twi_mngr,
        MAX_PENDING_TRANSACTIONS, TWI_INSTANCE_ID);
APP_TIMER_DEF(m_timer);

// Pin number for indicating communication with sensors.
#ifdef BSP_LED_3
    #define READ_ALL_INDICATOR  BSP_BOARD_LED_3
#else
    #error "Please choose an output pin"
#endif


// Buffer for data read from sensors.
#define BUFFER_SIZE  6
static uint8_t m_buffer[BUFFER_SIZE];

// Data structures needed for averaging of data read from sensors.
// [max 32, otherwise "int16_t" won't be sufficient to hold the sum
//  of temperature samples]
#define NUMBER_OF_SAMPLES  16

typedef struct
{
    float temp;
    float hum;
} sum_t;

static sum_t m_sum = { 0.0f, 0.0f};

typedef struct
{
    float temp;
    float hum;
} sample_t;

static sample_t m_samples[NUMBER_OF_SAMPLES] = { { 0.0f, 0.0f } };

static uint8_t m_sample_idx = 0;


#if defined( __GNUC__ ) && (__LINT__ == 0)
    // This is required if one wants to use floating-point values in 'printf'
    // (by default this feature is not linked together with newlib-nano).
    // Please note, however, that this adds about 13 kB code footprint...
    __ASM(".global _printf_float");
#endif


////////////////////////////////////////////////////////////////////////////////
// Reading of data from sensors - current temperature and humidity
//
#if (BUFFER_SIZE < 6)
    #error Buffer too small.
#endif

void read_all_cb(ret_code_t result, void * p_user_data)
{
    if (result != NRF_SUCCESS)
    {
        NRF_LOG_WARNING("read_all_cb - error: %d", (int)result);
        return;
    }

    sample_t * p_sample = &m_samples[m_sample_idx];

    m_sum.temp      -= p_sample->temp;
    m_sum.hum       -= p_sample->hum;

    uint8_t temp_hi = m_buffer[2];
    uint8_t temp_lo = m_buffer[3];
    uint8_t hum_hi  = m_buffer[4];
    uint8_t hum_lo  = m_buffer[5];

    p_sample->temp  = HDC1080_GET_TEMP_VALUE(temp_hi, temp_lo);
    p_sample->hum   = HDC1080_GET_HUM_VALUE (hum_hi, hum_lo);

    m_sum.temp      += p_sample->temp;
    m_sum.hum       += p_sample->hum;

    ++m_sample_idx;
    
    if (m_sample_idx >= NUMBER_OF_SAMPLES)
    {
        m_sample_idx = 0;
    }
}
static void read_all(void)
{
    // [these structures have to be "static" - they cannot be placed on stack
    //  since the transaction is scheduled and these structures most likely
    //  will be referred after this function returns]
    static nrf_twi_mngr_transfer_t const transfers[] =
    {
        HDC1080_READ_TEMP(&m_buffer[2])
        ,
        HDC1080_READ_HUM(&m_buffer[4])
    };
    static nrf_twi_mngr_transaction_t NRF_TWI_MNGR_BUFFER_LOC_IND transaction =
    {
        .callback            = read_all_cb,
        .p_user_data         = NULL,
        .p_transfers         = transfers,
        .number_of_transfers = sizeof(transfers) / sizeof(transfers[0])
    };

    APP_ERROR_CHECK(nrf_twi_mngr_schedule(&m_nrf_twi_mngr, &transaction));

    // Signal on LED that something is going on.
    bsp_board_led_invert(READ_ALL_INDICATOR);
}

#if (BUFFER_SIZE < 6)
    #error Buffer too small.
#endif
static void read_hdc1080_registers_cb(ret_code_t result, void * p_user_data)
{
    if (result != NRF_SUCCESS)
    {
        NRF_LOG_WARNING("read_hdc1080_registers_cb - error: %d", (int)result);
        return;
    }

    NRF_LOG_DEBUG("hdc1080: ");
    NRF_LOG_HEXDUMP_DEBUG(m_buffer, 6);
}
static void read_hdc1080_registers(void)
{
    // [these structures have to be "static" - they cannot be placed on stack
    //  since the transaction is scheduled and these structures most likely
    //  will be referred after this function returns]
    static nrf_twi_mngr_transfer_t const transfers[] =
    {
        HDC1080_READ(&hdc1080_config_reg_addr,  &m_buffer[0], 2),
        HDC1080_READ(&hdc1080_temp_reg_addr,    &m_buffer[2], 2),
        HDC1080_READ(&hdc1080_hum_reg_addr,     &m_buffer[4], 2),
    };
    static nrf_twi_mngr_transaction_t NRF_TWI_MNGR_BUFFER_LOC_IND transaction =
    {
        .callback            = read_hdc1080_registers_cb,
        .p_user_data         = NULL,
        .p_transfers         = transfers,
        .number_of_transfers = sizeof(transfers) / sizeof(transfers[0])
    };

    APP_ERROR_CHECK(nrf_twi_mngr_schedule(&m_nrf_twi_mngr, &transaction));
}



////////////////////////////////////////////////////////////////////////////////
// Buttons handling (by means of BSP).
//
static void bsp_event_handler(bsp_event_t event)
{
    // Each time the button 1 or 4 is pushed we start a transaction reading
    // values of all registers from LM75B or MMA7660 respectively.
    switch (event)
    {
    case BSP_EVENT_KEY_0: // Button 1 pushed.
        read_hdc1080_registers();
        break;

    default:
        break;
    }
}
static void bsp_config(void)
{
    uint32_t err_code;

    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    err_code = bsp_init(BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);
}

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

static void lfclk_config(void)
{
    uint32_t err_code;

    err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_clock_lfclk_request(NULL);
}

void timer_handler(void * p_context)
{
    read_all();
}

void read_init(void)
{
    ret_code_t err_code;

    err_code = app_timer_create(&m_timer, APP_TIMER_MODE_REPEATED, timer_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_start(m_timer, APP_TIMER_TICKS(50), NULL);
    APP_ERROR_CHECK(err_code);
}

void log_init(void)
{
    ret_code_t err_code;

    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

int main(void)
{
    ret_code_t err_code;

    log_init();
    bsp_board_init(BSP_INIT_LEDS);

    // Start internal LFCLK XTAL oscillator - it is needed by BSP to handle
    // buttons with the use of APP_TIMER and for "read_all" ticks generation
    // (by RTC).
    lfclk_config();

    bsp_config();

    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);

    NRF_LOG_RAW_INFO("\r\nTWI master example started. \r\n");
    NRF_LOG_FLUSH();
    twi_config();

    read_init();

    // Initialize sensor
    APP_ERROR_CHECK(nrf_twi_mngr_perform(&m_nrf_twi_mngr, 
                                        NULL, hdc1080_init_transfers,
                                        HDC1080_INIT_TRANSFER_COUNT, NULL));

    while (true)
    {
        nrf_pwr_mgmt_run();
        NRF_LOG_FLUSH();
    }
}


/** @} */
