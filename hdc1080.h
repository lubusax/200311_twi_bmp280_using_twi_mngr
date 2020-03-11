#ifndef HDC1080_H__
#define HDC1080_H__

#include "nrf_twi_mngr.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Default HDC1080 I2C address.
 *  This should be set according to the 
 *  configuration of the hardware address pins. 
 */
#define HDC1080_ADDR        0x40
/** Register Map.
 *  Device ID Register (offset = FFh) [reset = 1050h]
 *  Manufacturer ID Register (offset = FEh) [reset = 5449h]
 */
#define HDC1080_REG_TEMP    0x00 //Temperature measurement output
#define HDC1080_REG_HUM     0x01 //Relative Humidity measurement output
#define HDC1080_REG_CONFIG  0x02 //HDC1080 configuration and status
#define HDC1080_REG_MAN_ID  0xFE //ID of Texas Instruments
#define HDC1080_REG_DEV_ID  0xFF

#define HDC1080_GET_TEMP_VALUE(temp_hi, temp_lo) \
    ((((float)(((int16_t)temp_hi << 8) | temp_lo)) / pow(2.0f, 16.0f)) * 165.0f - 40.0f)

#define HDC1080_GET_HUM_VALUE(hum_hi, hum_lo) \
    (((float)(((int16_t)hum_hi << 8) | hum_lo)) / pow(2.0f, 16.0f)) * 100.0f)

// extern uint8_t NRF_TWI_MNGR_BUFFER_LOC_IND mma7660_xout_reg_addr;

#define HDC1080_READ(p_reg_addr, p_buffer, byte_cnt) \
    NRF_TWI_MNGR_WRITE(HDC1080_ADDR, p_reg_addr, 1,        NRF_TWI_MNGR_NO_STOP), \
    NRF_TWI_MNGR_READ (HDC1080_ADDR, p_buffer,   byte_cnt, 0)


#define HDC1080_INIT_TRANSFER_COUNT 1

extern nrf_twi_mngr_transfer_t const
        hdc1080_init_transfers[HDC1080_INIT_TRANSFER_COUNT];

#ifdef __cplusplus
}
#endif

#endif // HDC1080_H__
