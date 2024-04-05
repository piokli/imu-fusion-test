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
    return ESP_FAIL;
}

esp_err_t lis3mdl_read_magneto_raw(struct lis3mdl_vector *m)
{
    return ESP_FAIL;
}