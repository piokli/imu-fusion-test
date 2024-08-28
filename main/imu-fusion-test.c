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

#define SAMPLE_PERIOD (0.01f) // replace this with actual sample period
#define SAMPLE_RATE (100)


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
	lis3mdl_default_setup();

	// Create tasks
    xTaskCreate(blinky, "blinky", 1024, NULL, 1, NULL);
    // xTaskCreate(read_sensors_data_task, "read_sensors_data_task", 4096, NULL, 3, NULL);

	struct vector acc_data;
	struct vector gyro_data;
	struct lis3mdl_vector mag_data;

	// Fusion stuff

	// Define calibration (replace with actual calibration data if available)
    const FusionMatrix gyroscopeMisalignment = {{{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}}};
    const FusionVector gyroscopeSensitivity = {{1.0f, 1.0f, 1.0f}};
    const FusionVector gyroscopeOffset = {{4.443f, -5.4975f, -0.2965f}};
    const FusionMatrix accelerometerMisalignment = {{{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}}};
    const FusionVector accelerometerSensitivity = {{0.001f, 0.001f, 0.001f}};
    const FusionVector accelerometerOffset = {{9.164f, -7.1913f, 24.0663f}};
    const FusionMatrix softIronMatrix = {{{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}}}; // bez zmian
    const FusionVector hardIronOffset = {{354.1032f, -831.0851f, 0.3259f}}; // {{0.0f, 0.0f, 0.0f}};

	// Initialise algorithms
	FusionOffset offset;
    FusionAhrs ahrs;

	FusionOffsetInitialise(&offset, SAMPLE_RATE);
    FusionAhrsInitialise(&ahrs);

	// Set AHRS algorithm settings
    const FusionAhrsSettings settings = {
            .convention = FusionConventionEnu,
            .gain = 0.5f,
            .gyroscopeRange = 1000.0f, /* replace this with actual gyroscope range in degrees/s */
            .accelerationRejection = 100.f,//10.0f,
            .magneticRejection = 100.f,//10.0f,
            .recoveryTriggerPeriod = 3 * SAMPLE_RATE, /* 5 seconds */
    };
    FusionAhrsSetSettings(&ahrs, &settings);

	uint32_t millis, millis_last = esp_log_timestamp();
    while (true) {
		lsm6ds33_read_gyro_raw(&gyro_data);
		lsm6ds33_read_acc_raw(&acc_data);
		lis3mdl_read_magneto_raw(&mag_data);

		lsm6ds33_vector_calculate_gyro_raw(&gyro_data);
		lsm6ds33_vector_calculate_acc_raw(&acc_data);

		// printf("g.x %8.1f, g.y %8.1f, g.z %8.1f, ", gyro_data.x, gyro_data.y, gyro_data.z);
		// printf("a.x %8.1f, a.y %8.1f, a.z %8.1f, ", acc_data.x, acc_data.y, acc_data.z);
		// printf("m.x %8.1f, m.y %8.1f, m.z %8.1f\n", mag_data.x, mag_data.y, mag_data.z);

		// printf("%8.0ld, ", esp_log_timestamp());
		// printf("%8.1f, %8.1f, %8.1f, ", acc_data.x, acc_data.y, acc_data.z);
		// printf("%8.1f, %8.1f, %8.1f, ", gyro_data.x, gyro_data.y, gyro_data.z);
		// printf("%8.1f, %8.1f, %8.1f", mag_data.x, mag_data.y, mag_data.z);
		// printf("\n");


		millis = esp_log_timestamp();
        FusionVector gyroscope = {{gyro_data.x, gyro_data.y, gyro_data.z}}; 	// replace this with actual gyroscope data in degrees/s
        FusionVector accelerometer = {{acc_data.x, acc_data.y, acc_data.z}}; 	// replace this with actual accelerometer data in g's !
		FusionVector magnetometer = {{mag_data.x, mag_data.y, mag_data.z}}; 	// replace this with actual magnetometer data in arbitrary units

        // Apply calibration
        gyroscope = FusionCalibrationInertial(gyroscope, gyroscopeMisalignment, gyroscopeSensitivity, gyroscopeOffset);
        accelerometer = FusionCalibrationInertial(accelerometer, accelerometerMisalignment, accelerometerSensitivity, accelerometerOffset);
       	magnetometer = FusionCalibrationMagnetic(magnetometer, softIronMatrix, hardIronOffset);

		// printf("%8.0ld, ", esp_log_timestamp());
		// printf("%2.4f, %2.4f, %2.4f, ", accelerometer.axis.x, accelerometer.axis.y, accelerometer.axis.z);
		// printf("%8.1f, %8.1f, %8.1f, ", gyroscope.axis.x, gyroscope.axis.y, gyroscope.axis.z);
		// printf("%4.4f, %4.4f, %4.4f, ", magnetometer.axis.x, magnetometer.axis.y, magnetometer.axis.z);
		// printf("\n");

		// Update gyroscope offset correction algorithm
        gyroscope = FusionOffsetUpdate(&offset, gyroscope);

		FusionAhrsUpdate(&ahrs, gyroscope, accelerometer, magnetometer, (millis - millis_last) * 0.001f);
		// FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, (millis - millis_last) * 0.001f);
		millis_last = millis;

        // Print algorithm outputs
		const FusionQuaternion quat = FusionAhrsGetQuaternion(&ahrs);
        const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
        const FusionVector earth = FusionAhrsGetEarthAcceleration(&ahrs);

        printf("{\"Roll\": %0.1f, \"Pitch\": %0.1f, \"Yaw\": %0.1f, \"X\": %0.4f, \"Y\": %0.4f, \"Z\": %0.4f, \"w\": %0.4f, \"x\": %0.4f, \"y\": %0.4f, \"z\": %0.4f}\n",
               euler.angle.roll, euler.angle.pitch, euler.angle.yaw,
               earth.axis.x, earth.axis.y, earth.axis.z,
			   quat.element.w, quat.element.x, quat.element.y, quat.element.z);

		vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}
	