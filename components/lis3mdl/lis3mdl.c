/*
 * lis3mdl.c
 *
 *  Created on: 04.04.2024
 *      Author: Piokli
 */

#include "lis3mdl.h"

#include <math.h>
#include <driver/i2c.h>
#include <esp_err.h>
#include "esp_log.h"

static const char* TAG = "lis3mdl";

esp_err_t lis3mdl_test_connection(void)
{
	uint8_t get_id;

	esp_err_t ret = i2c_helper_read_reg(LIS3MDL_I2C_ADDR, LIS3MDL_WHO_AM_I_ADDR, &get_id, 1);
	if (ret != ESP_OK) {
        return ret;
    }
    if (get_id != LIS3MDL_WHO_ID)
    {
    	ESP_LOGW(TAG, "Failed to connect to LIS3MDL!");
    	return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Connected to LIS3MDL");

    return ESP_OK;
}

esp_err_t lis3mdl_default_setup(void)
{
    // Typical startup sequence from    :
    // "1. Write 40h in CTRL_REG2. Sets full scale ±12 G."
    uint8_t ctrl_reg2_setup = 0x40;
    esp_err_t ret = i2c_helper_write_reg(LIS3MDL_I2C_ADDR, LIS3MDL_CTRL_REG2_ADDR, &ctrl_reg2_setup, 1);
	if (ret != ESP_OK) {
	    return ret;
	}
    // "2. Write FCh in CTRL_REG1. Sets UHP mode on the X/Y axes, ODR at 80 Hz and
    // activates temperature sensor." -> uint8_t ctrl_reg1_setup = 0xFC;
    // Instead I select 155Hz at UHP mode
    uint8_t ctrl_reg1_setup = 0xFE;
    ret = i2c_helper_write_reg(LIS3MDL_I2C_ADDR, LIS3MDL_CTRL_REG1_ADDR, &ctrl_reg1_setup, 1);
	if (ret != ESP_OK) {
	    return ret;
	}
    // "3. Write 0Ch in CTRL_REG4. Sets UHP mode on the Z-axis.
    uint8_t ctrl_reg4_setup = 0x0C;
    ret = i2c_helper_write_reg(LIS3MDL_I2C_ADDR, LIS3MDL_CTRL_REG4_ADDR, &ctrl_reg4_setup, 1);
	if (ret != ESP_OK) {
	    return ret;
	}
    // "4. Write 00h in CTRL_REG3. Sets continuous-measurement mode."
    uint8_t ctrl_reg3_setup = 0x00;
    ret = i2c_helper_write_reg(LIS3MDL_I2C_ADDR, LIS3MDL_CTRL_REG3_ADDR, &ctrl_reg3_setup, 1);
	if (ret != ESP_OK) {
	    return ret;
	}

    // "If the reading of the magnetic data cannot be synchronized with the XYZDA bit in the
    // STATUS_REG register, it is strongly recommended to set the BDU (block data update) bit
    // to 1 in CTRL_REG5 register."
    // BDU quarantees a given pair (OUT_H and OUT_L) is sampled at the same time 
    uint8_t ctrl_reg5_setup = 0x40;
    ret = i2c_helper_write_reg(LIS3MDL_I2C_ADDR, LIS3MDL_CTRL_REG5_ADDR, &ctrl_reg5_setup, 1);
	if (ret != ESP_OK) {
	    return ret;
	}
    
    return ret;
} 

esp_err_t lis3mdl_read_magneto_raw(struct lis3mdl_vector *m)
{
    uint8_t buff_size = 2;
	uint8_t *mx = malloc(sizeof(uint8_t) * buff_size);
	uint8_t *my = malloc(sizeof(uint8_t) * buff_size);
	uint8_t *mz = malloc(sizeof(uint8_t) * buff_size);

    // 1. Read STATUS_REG
    // 2. If STATUS_REG(3) = 0, then go to step 1
    // 3. If STATUS_REG(7) = 1, some data have been overwritten (thus BDU can be enabled)
    
    // 4. Read OUT_X_L
    // 5. Read OUT_X_H
    esp_err_t ret = i2c_helper_read_reg(LIS3MDL_I2C_ADDR, LIS3MDL_OUT_X_L_ADDR, mx, buff_size);
    if (ret != ESP_OK) {
        return ret;
    } 
    // 6. Read OUT_Y_L
    // 7. Read OUT_Y_H
    ret = i2c_helper_read_reg(LIS3MDL_I2C_ADDR, LIS3MDL_OUT_Y_L_ADDR, my, buff_size);
    if (ret != ESP_OK) {
        return ret;
    } 
    // 8. Read OUT_Z_L
    // 9. Read OUT_Z_H
    ret = i2c_helper_read_reg(LIS3MDL_I2C_ADDR, LIS3MDL_OUT_Z_L_ADDR, mz, buff_size);
    if (ret != ESP_OK) {
        return ret;
    } 
    // 10. Data processing
    
    m->x = (int16_t)((mx[1] << 8) | mx[0]);
    m->y = (int16_t)((my[1] << 8) | my[0]);
    m->z = (int16_t)((mz[1] << 8) | mz[0]);
    // data in arbitrary units
    // printf("mag_x = %6.0f, mag_y = %6.0f, mag_z = %6.0f\t", m->x, m->y, m->z);

    // By activating the FAST_READ option we allow the transfer of the high part of the output
    // data only and we can disregard the low part of the output data. 

    free(mx);
    free(my);
    free(mz);

    return ret;
}

// The sensitivity of the magnetic sensor changes when the temperature changes. A
// temperature compensation digital block is introduced to compensate for the effect of
// temperature.