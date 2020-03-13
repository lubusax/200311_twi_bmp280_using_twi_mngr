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
APP_TIMER_DEF(m_timer);

// Pin number for indicating communication with sensors.
#ifdef BSP_LED_3
    #define READ_ALL_INDICATOR  BSP_BOARD_LED_3
#else
    #error "Please choose an output pin"
#endif


// Buffer for data read from sensors.
#define BUFFER_SIZE  10
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

// manufacturer register related variables
static uint8_t m_manufacturer_buffer[2];

static nrf_twi_mngr_transfer_t const transfer_manufacturer[] =
{
    HDC1080_READ_MANUFACTURER(&m_manufacturer_buffer)
};


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
    p_sample->hum   = HDC1080_GET_HUM_VALUE(hum_hi, hum_lo);

    m_sum.temp      += p_sample->temp;
    m_sum.hum       += p_sample->hum;

    ++m_sample_idx;

    if (m_sample_idx >= NUMBER_OF_SAMPLES)
    {
        m_sample_idx = 0;
    }
}
//static void read_all(void)
//{
//    // [these structures have to be "static" - they cannot be placed on stack
//    //  since the transaction is scheduled and these structures most likely
//    //  will be referred after this function returns]
//    static nrf_twi_mngr_transfer_t const transfers[] =
//    {
//        HDC1080_READ_TEMP(&m_buffer[2])
//        ,
//        HDC1080_READ_HUM(&m_buffer[4])
//    };
//    static nrf_twi_mngr_transaction_t NRF_TWI_MNGR_BUFFER_LOC_IND transaction =
//    {
//        .callback            = read_all_cb,
//        .p_user_data         = NULL,
//        .p_transfers         = transfers,
//        .number_of_transfers = sizeof(transfers) / sizeof(transfers[0])
//    };
//
//    APP_ERROR_CHECK(nrf_twi_mngr_schedule(&m_nrf_twi_mngr, &transaction));
//
//    // Signal on LED that something is going on.
//    bsp_board_led_invert(READ_ALL_INDICATOR);
//}

static void read_all(void)
{
    // [these structures have to be "static" - they cannot be placed on stack
    //  since the transaction is scheduled and these structures most likely
    //  will be referred after this function returns]
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
    

    NRF_LOG_RAW_INFO("\r\nResult Read T Register once: %d \r\n",
                    result_mngr_perform);
    NRF_LOG_RAW_INFO("\r\nT Register 2 bytes: %x %x\r\n",
                    m_temp_and_hr_buffer[0], m_temp_and_hr_buffer[1]);
    NRF_LOG_RAW_INFO("\r\nHR Register 2 bytes: %x %x\r\n",
                    m_temp_and_hr_buffer[2], m_temp_and_hr_buffer[3]);
    NRF_LOG_RAW_INFO("Temperature " NRF_LOG_FLOAT_MARKER " C\r\n",
                      NRF_LOG_FLOAT(temperature));
    NRF_LOG_RAW_INFO("Relative Humidity " NRF_LOG_FLOAT_MARKER " %% \r\n",
                      NRF_LOG_FLOAT(relative_humidity) );


    NRF_LOG_FLUSH();
    APP_ERROR_CHECK(result_mngr_perform);   

    // Signal on LED that something is going on.
    bsp_board_led_invert(READ_ALL_INDICATOR);
}

#if (BUFFER_SIZE < 10)
    #error Buffer too small.
#endif

static void read_hdc1080_temp_register_cb(ret_code_t result, void * p_user_data)
{
    if (result != NRF_SUCCESS)
    {
        NRF_LOG_WARNING("read_hdc1080_temp_register_cb - error: %d", (int)result);
        return;
    }

    NRF_LOG_DEBUG("hdc1080: ");
    NRF_LOG_HEXDUMP_DEBUG(m_buffer, 10);
    NRF_LOG_RAW_INFO("\r\nResult Read Temp Register: %d \r\n",result);
    NRF_LOG_FLUSH();
}

static void read_hdc1080_registers_cb(ret_code_t result, void * p_user_data)
{
    if (result != NRF_SUCCESS)
    {
        NRF_LOG_WARNING("read_hdc1080_registers_cb - error: %d", (int)result);
        return;
    }

    NRF_LOG_DEBUG("hdc1080: ");
    NRF_LOG_HEXDUMP_DEBUG(m_buffer, 10);
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
        HDC1080_READ(&hdc1080_man_reg_addr,     &m_buffer[6], 2),
        HDC1080_READ(&hdc1080_dev_reg_addr,     &m_buffer[8], 2),
    };
    static nrf_twi_mngr_transaction_t NRF_TWI_MNGR_BUFFER_LOC_IND transaction =
    {
        .callback            = read_hdc1080_registers_cb,
        .p_user_data         = NULL,
        .p_transfers         = transfers,
        .number_of_transfers = sizeof(transfers) / sizeof(transfers[0])
    };

    APP_ERROR_CHECK(nrf_twi_mngr_schedule(&m_nrf_twi_mngr, &transaction));
    // nrf_delay_ms(50);
}

static void read_hdc1080_temp_register(void)
{
    static nrf_twi_mngr_transfer_t const transfers[] =
    {
        //HDC1080_READ(&hdc1080_config_reg_addr,  &m_buffer[0], 2),
        HDC1080_READ(&hdc1080_temp_reg_addr,    &m_buffer[2], 2),
        //HDC1080_READ(&hdc1080_hum_reg_addr,     &m_buffer[4], 2),
        //HDC1080_READ(&hdc1080_man_reg_addr,     &m_buffer[6], 2),
        //HDC1080_READ(&hdc1080_dev_reg_addr,     &m_buffer[8], 2),
    };
    static nrf_twi_mngr_transaction_t NRF_TWI_MNGR_BUFFER_LOC_IND transaction =
    {
        .callback            = read_hdc1080_temp_register_cb,
        .p_user_data         = NULL,
        .p_transfers         = transfers,
        .number_of_transfers = sizeof(transfers) / sizeof(transfers[0])
    };

    APP_ERROR_CHECK(nrf_twi_mngr_schedule(&m_nrf_twi_mngr, &transaction));
    // nrf_delay_ms(50);
}



////////////////////////////////////////////////////////////////////////////////
// Buttons handling (by means of BSP).
//
static void bsp_event_handler(bsp_event_t event)
{
    // Each time the button 1 is pushed we start a transaction reading
    // values of all registers from HDC1080
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

    err_code = app_timer_start(m_timer, APP_TIMER_TICKS(500), NULL);
    APP_ERROR_CHECK(err_code);
}

void log_init(void)
{
    ret_code_t err_code;

    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

void read_t_and_hr(void)
{
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
    

    NRF_LOG_RAW_INFO("\r\nResult Read T Register once: %d \r\n",
                    result_mngr_perform);
    NRF_LOG_RAW_INFO("\r\nT Register 2 bytes: %x %x\r\n",
                    m_temp_and_hr_buffer[0], m_temp_and_hr_buffer[1]);
    NRF_LOG_RAW_INFO("\r\nHR Register 2 bytes: %x %x\r\n",
                    m_temp_and_hr_buffer[2], m_temp_and_hr_buffer[3]);
    NRF_LOG_RAW_INFO("Temperature " NRF_LOG_FLOAT_MARKER " C\r\n",
                      NRF_LOG_FLOAT(temperature));
    NRF_LOG_RAW_INFO("Relative Humidity " NRF_LOG_FLOAT_MARKER " %% \r\n",
                      NRF_LOG_FLOAT(relative_humidity) );


    NRF_LOG_FLUSH();
    APP_ERROR_CHECK(result_mngr_perform);

    // Signal on LED that something is going on.
    bsp_board_led_invert(READ_ALL_INDICATOR);
 
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

//    nrf_delay_ms(15); 
//
//    // Initialize sensor
//    result_mngr_perform = nrf_twi_mngr_perform(&m_nrf_twi_mngr, 
//                                        NULL, hdc1080_init_transfers,
//                                        HDC1080_INIT_TRANSFER_COUNT, NULL);
//    NRF_LOG_RAW_INFO("\r\nResult Init Sensor: %d \r\n",result_mngr_perform);
//    NRF_LOG_FLUSH();
//    APP_ERROR_CHECK(result_mngr_perform);
//
//    nrf_delay_ms(20);

// Read Temperature Register once
    read_t_and_hr();
    /////////////////////////////////////////


    read_init(); // timer create and start

    while (true)
    {
        nrf_pwr_mgmt_run();

        NRF_LOG_FLUSH();
    }
}


/** @} */
