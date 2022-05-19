
/*
 * BME280 - temperature, pressure and humidity programming code
 */


#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include "driver/i2c.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "BME280.h"

void hal_i2c_init();

void app_main(void)
{
	BMP280_calib_t		NVM_coef;
	float			temperature, pressure, humidity;
	int			sensor;

	sensor = 0;		// 1 if BME280; 0 if BMP280

	hal_i2c_init();
	BMP280_init(&NVM_coef, sensor);

	vTaskDelay(100/portTICK_RATE_MS);

	while(1) {
		BMP280_final_data(&temperature, &pressure, &humidity, sensor, &NVM_coef);
		printf("Pressure: %f mBar\nTemperature: %f Celsius\n\n", pressure, temperature);

		vTaskDelay(2000/portTICK_RATE_MS);
	}

}
