# BME280 sensor API
## Author
Name:			Francisco Martín Villegas

Email:			f.martinvillegas00@gmail.com

Colleague:		University of Almería (Spain)

## Introduction
This package contains the BME/P280 temperature, humidity and pressure sensor.

The sensor driver package includes BME280.c and BME280.h files.


## Integration details
* Integrate BME280.c and BME280.h file in to the project.
* Include the BME280.h file in your code like below.
``` c
#include "BME280.h"
```

## File information
* BME280.h : This header file has the constants, function declarations, macros and datatype declarations.
* BME280.c : This source file contains the definitions of the sensor driver APIs.

## Supported sensor interfaces
* SPI 4-wire	:	SDI, SDO and CSB pins
* I2C			:	SDA and SDC pins

SPI 3-wire is currently not supported in the API.
## Usage guide
### Initializing the master
First you need to initialize the I2C master with:
```c
hal_i2c_init();
```
### Initializing the sensor
First create a BMP280_calib_t NVM_coef struct variable.

Then initialize the sensor using:
```c
BMP280_init(&NVM_coef);
```

#### Example for I2C
Create some FLOAT variables to save final pressure, humidity and temperature, and the call the function:
```c
BMP280_final_data();
```

### Sensor data units
> The temperature data is read in Celsius. 
> The pressure data is read in hPa.
> The humidity data is read in %RH.


### Templates for function
``` c
/**
 * @brief	Master initialition
 *
 */
void hal_i2c_init();

/*
 * @brief Data lecture on BMP280
 *
 * @param[in]	BMP280_command	:	(uint8_t)	command that we want to execute
 * @param[in]	size			:	(unsigned)	size of the number of bytes that we want to read
 *
 * @param[out]	*buffer_out		:	(uint8_t)	pointer to array where the functions save the lecture of BMP280_command
 *
 * @return		Response of esp_err_t function type
 *
 */
esp_err_t BMP280_read(uint8_t * buffer_out, uint8_t BMP280_command, unsigned size)

/*
 * @brief	Data writting on BMP280
 *
 * @param[in]	BMP280_command	:	(uint8_t)	command that we want to execute
 *
 * @return		Response of esp_err_t function type
 *
 */
esp_err_t BMP280_write_command(uint8_t BMP280_command)

/*
 * @brief	Data writting on BMP280
 *
 * @param[in]	BMP280_register			:	(uint8_t)	address of the register where we want to write
 * @param[in]	BMP280_register_value	:	(uint8_t)	value of the register that we want to write
 *
 * @return		Response of esp_err_t function type
 *
 */
esp_err_t BMP280_write_register(uint8_t BMP280_register, uint8_t BMP280_register_value)

/**
 * @brief	Calibration coefficients of temperature, humidity and pressure
 *
 * @param[ou]	*NMV_coefs	:	(BMP280_calib_t)	struct where we are going to save the calibration parameters for compensate readings
 *
 * @return		Response of esp_err_t function type
 *
 */
esp_err_t BMP280_read_calib_coeff(BMP280_calib_t *NVM_coefs)

/**
 * @brief 		RESET register
 *
 * @return		Response of esp_err_t function type
 *
 */
esp_err_t BMP280_reset_function()

/**
 *	@brief	Slave initialation
 *
 *	@param[in]	*NVM_coef	:	(BMP280_calib_t)	struct where we are going to save the calibration parameters for compensate readings
 *
 */
int BMP280_init(BMP280_calib_t *NVM_coef, int sensor)

/**
 * @brief	Temperature compensation in int-variable for BMP280
 *
 * @param[in]	adc_tempe	:	(int32_t)			raw adc temperature data
 * @param[in]	*NVM_coef	:	(BMP280_calib_t)	struct where we are going to save the calibration parameters for compensate readings
 *
 * @return	t_fine	:	(int32_t)	compensate t_fine temperature variable
 *
 */
int32_t BMP280_int_temperature_compensation(int32_t adc_tempe, BMP280_calib_t *NVM_coef)

/**
 * @brief	Pressure compensation for BMP280
 *
 * @param[in]	adc_pres	:	(int32_t)			raw adc pressure data
 * @param[in]	*NVM_coef	:	(BMP280_calib_t)	struct where we are going to save the calibration parameters for compensate readings
 * @param[in]	t_fine		:	(int32_t)			compensate t_fine temperature variable
 *
 * @return		p			:	(uint32_t)			compensate pressure
 *
 */
uint32_t BMP280_int_pressure_compensation(int32_t adc_pres, BMP280_calib_t *NVM_coef, int32_t t_fine)

/**
 * @brief	Compensate humidity for BME280
 *
 * @param[in]	adc_hum		:	(int32_t)			raw adc humidity data
 * @param[in]	*NVM_coef	:	(BMP280_calib_t)	struct where we are going to save the calibration parameters for compensate readings
 * @param[in]	t_fine		:	(int32_t)			compensate t_fine temperature variable
 *
 * @return		p			:	(uint32_t)			compensate humidity
 *
 */
uint32_t BMP280_int_humidity_compensation(int32_t adc_hum, BMP280_calib_t *NVM_coef, int32_t t_fine)

/**
 * @brief	Get final data for BME/P 280
 *
 * @param[in]	*temperature	:	(float)				pointer to temperature float variable
 * @param[in]	*pressure		:	(float)				pointer to pressure float variable
 * @param[in]	*humidity		:	(float)				pointer to humidity float variable
 * @param[in]	sensor			:	(int)				type of sensor that we are using. Choose 1 for BME280 or 0 for BMP280
 * @param[in]	*NVM_coef		:	(BMP280_calib_t)	struct where we are going to save the calibration parameters for compensate readings
 *
 */
void BMP280_final_data(float *temperature, float *pressure, float *humidity, int sensor, BMP280_calib_t *NVM_coef)




```
