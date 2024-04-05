#include <stdbool.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "../components/i2c_helper/i2c_helper.h"
#include "../components/lsm6ds33/lsm6ds33.h"
#include "../components/lis3mdl/lis3mdl.h"
#include "../components/Fusion/Fusion/Fusion.h"

static const char *TAG = "fusion-test";

#define SAMPLE_PERIOD (0.001f) // replace this with actual sample period


void read_sensors_data_task(void *pvParameters)
{

	struct vector acc_data;
	struct vector gyro_data;

	vTaskDelay(1000 / portTICK_PERIOD_MS);

	while(1)
	{
		//ESP_LOGI(TAG_main, "Reading sensors data %d", i++);

		lsm6ds33_read_acc_raw(&acc_data);
		lsm6ds33_read_gyro_raw(&gyro_data);

		lsm6ds33_vector_calculate_acc_raw(&acc_data);
		lsm6ds33_vector_calculate_gyro_raw(&gyro_data);

        ESP_LOGI(TAG, "acc.x: %f, acc.y: %f, acc.z: %f\n", acc_data.x, acc_data.y, acc_data.z);

		vTaskDelay(pdMS_TO_TICKS(20)); // vTaskDelay(pdMS_TO_TICKS(20));  == vTaskDelay(20 / portTICK_PERIOD_MS);
	}
	vTaskDelete(NULL);
}


void blinky(void *pvParameters)
{
	const int blink_gpio = 5;
	esp_rom_gpio_pad_select_gpio(blink_gpio);
	gpio_set_direction(blink_gpio, GPIO_MODE_OUTPUT);

	vTaskDelay(5000 / portTICK_PERIOD_MS);
	while(1)
	{
	    /* Blink off (output low) */;
	    gpio_set_level(blink_gpio, 0);
	    vTaskDelay(1000 / portTICK_PERIOD_MS);
	    /* Blink on (output high) */
	    gpio_set_level(blink_gpio, 1);
	    vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
	vTaskDelete(NULL);
}


void app_main(void)
{
    // Initialize I2C and connect to sensors
    i2c_helper_master_init();
	lsm6ds33_test_connection();
	lis3mdl_test_connection();

	// Delay for safety
	vTaskDelay(50 / portTICK_PERIOD_MS);

    // Set up sensors
	lsm6ds33_default_setup();

	// Create tasks
    xTaskCreate(blinky, "blinky", 1024, NULL, 1, NULL);
    // xTaskCreate(read_sensors_data_task, "read_sensors_data_task", 4096, NULL, 3, NULL);

	struct vector acc_data;
	struct vector gyro_data;

	// Fusion stuff
    FusionAhrs ahrs;
    FusionAhrsInitialise(&ahrs);

	uint32_t millis, millis_last = esp_log_timestamp();
    while (true) {
		lsm6ds33_read_gyro_raw(&gyro_data);
		lsm6ds33_read_acc_raw(&acc_data);

		lsm6ds33_vector_calculate_gyro_raw(&gyro_data);
		lsm6ds33_vector_calculate_acc_raw(&acc_data);

        const FusionVector gyroscope = {{gyro_data.x, gyro_data.y, gyro_data.z}}; // replace this with actual gyroscope data in degrees/s
        const FusionVector accelerometer = {{acc_data.x, acc_data.y, acc_data.z}}; // replace this with actual accelerometer data in g

		millis = esp_log_timestamp();
        FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, (millis - millis_last) * 0.001f);
		millis_last = millis;

        // Print algorithm outputs
        const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
        const FusionVector earth = FusionAhrsGetEarthAcceleration(&ahrs); // <- cos tu nie tak

        printf("{\"Roll\": %0.1f, \"Pitch\": %0.1f, \"Yaw\": %0.1f, \"X\": %0.1f, \"Y\": %0.1f, \"Z\": %0.1f}\n",
               euler.angle.roll, euler.angle.pitch, euler.angle.yaw,
               earth.axis.x, earth.axis.y, earth.axis.z);
		vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}
