#include "hdc1080.h"

uint8_t NRF_TWI_MNGR_BUFFER_LOC_IND hdc1080_config_reg_addr = HDC1080_REG_CONFIG;
uint8_t NRF_TWI_MNGR_BUFFER_LOC_IND hdc1080_temp_reg_addr   = HDC1080_REG_TEMP;
uint8_t NRF_TWI_MNGR_BUFFER_LOC_IND hdc1080_hum_reg_addr    = HDC1080_REG_HUM;
uint8_t NRF_TWI_MNGR_BUFFER_LOC_IND hdc1080_man_reg_addr    = HDC1080_REG_MAN_ID;
uint8_t NRF_TWI_MNGR_BUFFER_LOC_IND hdc1080_dev_reg_addr    = HDC1080_REG_DEV_ID;

// Set default configuration of hdc1080 - write x1000 to Conf registers
// bit 12 = 1 (measure both, Temp and Hum)
// bit 10 = 00 - Temp 14 bit resolution
// bit 9:8 = 00 - Humidity 14 bit resolution
static uint8_t NRF_TWI_MNGR_BUFFER_LOC_IND default_config[] = { HDC1080_REG_CONFIG, 0x10, 0x00 };

nrf_twi_mngr_transfer_t const hdc1080_init_transfers[HDC1080_INIT_TRANSFER_COUNT] =
{
    NRF_TWI_MNGR_WRITE(HDC1080_ADDR, default_config, sizeof(default_config), 0)
};
