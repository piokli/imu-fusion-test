/*
 * lis3mdl.h
 *
 *  Created on: 04.04.2024
 *      Author: Piokli
 */

#ifndef COMPONENTS_LIS3MDL_LIS3MDL_H_
#define COMPONENTS_LIS3MDL_LIS3MDL_H_

#include <driver/i2c.h>
#include <esp_err.h>
#include "../i2c_helper/i2c_helper.h"

/** LIS3MDL Register Macros **/
#define LIS3MDL_WHO_AM_I_ADDR   0x0F    // (r)

#define LIS3MDL_CTRL_REG1_ADDR  0x20    // (r/w)
#define LIS3MDL_CTRL_REG2_ADDR  0x21    // (r/w)
#define LIS3MDL_CTRL_REG3_ADDR  0x22    // (r/w)
#define LIS3MDL_CTRL_REG4_ADDR  0x23    // (r/w)
#define LIS3MDL_CTRL_REG5_ADDR  0x24    // (r/w)

#define LIS3MDL_STATUS_REG_ADDR 0x27    // (r)
#define LIS3MDL_OUT_X_L_ADDR    0x28    // (r)
#define LIS3MDL_OUT_X_H_ADDR    0x29    // (r)
#define LIS3MDL_OUT_Y_L_ADDR    0x2A    // (r)
#define LIS3MDL_OUT_Y_H_ADDR    0x2B    // (r)
#define LIS3MDL_OUT_Z_L_ADDR    0x2C    // (r)
#define LIS3MDL_OUT_Z_H_ADDR    0x2D    // (r)
#define LIS3MDL_TEMP_OUT_L_ADDR 0x2E    // (r)
#define LIS3MDL_TEMP_OUT_H_ADDR 0x2F    // (r)
#define LIS3MDL_INT_CFG_ADDR    0x30    // (rw)
#define LIS3MDL_INT_SRC_ADDR    0x31    // (r)
#define LIS3MDL_INT_THS_L_ADDR  0x32    // (r)
#define LIS3MDL_INT_THS_H_ADDR  0x33    // (r)

/** LIS3MDL I2C Address **/
#define LIS3MDL_I2C_ADDR                0x1E // 00111x0b, x=1 when SDO/SA1 connected to VCC - default on AltIMU-10 v5
#define LIS3MDL_I2C_ADDR_SA0_LO         0x1C // 00111x0b, x=0 when SDO/SA1 connected to GND
#define LIS3MDL_I2C_ADDR_AUTOINCR_MASK  0x01 // 1 is autoincrement, 0 by default

/** LIS3MDL WHO_AM_I identifier **/
#define LIS3MDL_WHO_ID 0x3D

/** LIS3MDL CTRL_REG1 Option Masks **/
#define LIS3MDL_CTRL_REG1_TEMP_EN   0x80

#define LIS3MDL_CTRL_REG1_LOW_POWER_MODE
#define LIS3MDL_CTRL_REG1_MED_PERF_MODE
#define LIS3MDL_CTRL_REG1_HI_PERF_MODE
#define LIS3MDL_CTRL_REG1_ULTRA_HI_MODE

// TODO: all other defines and maybe different approach

/** Struct for keeping magnetometer x, y, z values**/
struct lis3mdl_vector {
    float x;
    float y;
    float z;
};

/** LIS3MDL Functions **/

esp_err_t lis3mdl_test_connection(void);

esp_err_t lis3mdl_default_setup(void);

esp_err_t lis3mdl_read_magneto_raw(struct lis3mdl_vector *m);



#endif /* COMPONENTS_LIS3MDL_LIS3MDL_H_ */