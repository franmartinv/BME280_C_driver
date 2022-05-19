/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
*	BME280 / BMP280 sensor DRIVER
*	-------------------------------------------
*
*	***************************************************
*	*		 FAST USER GUIDE 		  *
*	***************************************************
*  * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
*
*			1. Master initialition with function "	hal_i2c_init();	".
*			2. BME680 slave initialition with function "	BMP280_init(&NVM_coef);	".
*			3. Create a "BMP280_calib_t" struct, where we are going to safe the calibration coefficients
*			4. Create some "float" variables for save the final temperature, humidity and pressure values
*			5. Direct read of the variables calling "BMP280_final_data" function.
*
*  * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
*  * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
*
*
*	Step by step to use BME680
*	--------------------------
*		1. First I2C read to ID-REGISTER (it always fail...).
*		2. Do a soft-reset.
*		3. Read hardware-ID. It will be 0x61.
*		4. Read calibration-data.
*		5. Configuring type of oversampling for the sensors. In this driver is x2 by default. Search for it if you want to change it !!
*		6. Selecting Forced-mode, where we can read temperature, pressure and humidity ones.
*		7. Read pressure ADC data and then compensate it.
*		8. Read temperature ADC data and then compensate it.
*		9. If (sensor == 1) -> BME280 -> read humidity ADC data and then compensate it.
*
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */


#include "bme280.h"
#include "driver/gpio.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "sdkconfig.h"

#ifdef __cplusplus
extern "C" {
#endif


/*
 * @brief 		Data lecture on BMP280
 *
 * @param[in]		BMP280_command		:	(uint8_t)	command that we want to execute
 * @param[in]		size			:	(unsigned)	size of the number of bytes that we want to read
 *
 * @param[out]		*buffer_out		:	(uint8_t)	pointer to array where the functions save the lecture of BMP280_command
 *
 * @return		Response of esp_err_t function type
 *
 */
esp_err_t BMP280_read(uint8_t * buffer_out, uint8_t BMP280_command, unsigned size)
{
	int 		i;
	esp_err_t 	ret;

	i2c_cmd_handle_t i2c_cmd = i2c_cmd_link_create();
	ESP_ERROR_CHECK(i2c_master_start(i2c_cmd));
	ESP_ERROR_CHECK(i2c_master_write_byte(i2c_cmd, (BMP280_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, I2C_MASTER_ACK));
	ESP_ERROR_CHECK(i2c_master_write_byte(i2c_cmd, BMP280_command, I2C_MASTER_ACK));
	ESP_ERROR_CHECK(i2c_master_start(i2c_cmd));
	ESP_ERROR_CHECK(i2c_master_write_byte(i2c_cmd, (BMP280_I2C_ADDRESS << 1) | I2C_MASTER_READ, I2C_MASTER_ACK));

	for(i = 0; i < (size - 1); i++) {
		ESP_ERROR_CHECK(i2c_master_read_byte(i2c_cmd, &buffer_out[i], I2C_MASTER_ACK));
	}

	ESP_ERROR_CHECK(i2c_master_read_byte(i2c_cmd, &buffer_out[size-1], I2C_MASTER_NACK));

	ESP_ERROR_CHECK(i2c_master_stop(i2c_cmd));
	ret = i2c_master_cmd_begin(I2C_NUM_1, i2c_cmd, 1000/portTICK_PERIOD_MS);
	i2c_cmd_link_delete(i2c_cmd);

	return ret;
}


/*
 * @brief		Data writting on BMP280
 *
 * @param[in]		BMP280_command	:	(uint8_t)	command that we want to execute
 *
 * @return		Response of esp_err_t function type
 *
 */
esp_err_t BMP280_write_command(uint8_t BMP280_command)
{
    esp_err_t	ret;

	i2c_cmd_handle_t i2c_cmd = i2c_cmd_link_create();
    i2c_master_start(i2c_cmd);
    i2c_master_write_byte(i2c_cmd, (BMP280_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, I2C_MASTER_ACK);
    i2c_master_write_byte(i2c_cmd, BMP280_command, I2C_MASTER_ACK);
    i2c_master_stop(i2c_cmd);
    ret = i2c_master_cmd_begin(I2C_NUM_1, i2c_cmd, 1000/portTICK_RATE_MS);
    i2c_cmd_link_delete(i2c_cmd);

    return ret;
}


/*
 * @brief		Data writting on BMP280
 *
 * @param[in]		BMP280_register			:	(uint8_t)	address of the register where we want to write
 * @param[in]		BMP280_register_value		:	(uint8_t)	value of the register that we want to write
 *
 * @return		Response of esp_err_t function type
 *
 */
esp_err_t BMP280_write_register(uint8_t BMP280_register, uint8_t BMP280_register_value)
{
	esp_err_t 	ret;

	i2c_cmd_handle_t i2c_cmd = i2c_cmd_link_create();
	ESP_ERROR_CHECK(i2c_master_start(i2c_cmd));
	ESP_ERROR_CHECK(i2c_master_write_byte(i2c_cmd, (BMP280_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, I2C_MASTER_ACK));
	ESP_ERROR_CHECK(i2c_master_write_byte(i2c_cmd, BMP280_register, I2C_MASTER_ACK));
	ESP_ERROR_CHECK(i2c_master_write_byte(i2c_cmd, BMP280_register_value, I2C_MASTER_ACK));
	ESP_ERROR_CHECK(i2c_master_stop(i2c_cmd));
	ret = i2c_master_cmd_begin(I2C_NUM_1, i2c_cmd, 1000/portTICK_RATE_MS);
	i2c_cmd_link_delete(i2c_cmd);

	return ret;
}


/**
 * @brief		Calibration coefficients of temperature, humidity and pressure
 *
 * @param[ou]		*NMV_coefs	:	(BMP280_calib_t)	struct where we are going to save the calibration parameters for compensate readings
 *
 * @return		Response of esp_err_t function type
 *
 */
esp_err_t BMP280_read_calib_coeff(BMP280_calib_t *NVM_coefs)
{
	int				i;
	uint16_t		E4_read, E5_read, E6_read;
	uint8_t			buffer_out[2];
	uint16_t		coefs_t_p[12];
	esp_err_t		ret;

	for(i = 0; i < 12; i++) {
		ret = BMP280_read(buffer_out, BMP280_REG_CALIB + (2*i), 2);		// The temperature and pressure registers are separated
											// by 2 directions, so, i will read 2 by 2
		if(ret != ESP_OK) {
			printf("ERROR writing calibration coefficient number %d\n", i);
		}

		if(i == 0 || i == 3) {
			coefs_t_p[i] = (buffer_out[1] << 8) | buffer_out[0];
		}
		else coefs_t_p[i] = ((int16_t)buffer_out[1] << 8) | (int16_t)buffer_out[0];
	}

	(NVM_coefs -> BMP280_dig_T1) = coefs_t_p[0];
	(NVM_coefs -> BMP280_dig_T2) = coefs_t_p[1];
	(NVM_coefs -> BMP280_dig_T3) = coefs_t_p[2];
	(NVM_coefs -> BMP280_dig_P1) = coefs_t_p[3];
	(NVM_coefs -> BMP280_dig_P2) = coefs_t_p[4];
	(NVM_coefs -> BMP280_dig_P3) = coefs_t_p[5];
	(NVM_coefs -> BMP280_dig_P4) = coefs_t_p[6];
	(NVM_coefs -> BMP280_dig_P5) = coefs_t_p[7];
	(NVM_coefs -> BMP280_dig_P6) = coefs_t_p[8];
	(NVM_coefs -> BMP280_dig_P7) = coefs_t_p[9];
	(NVM_coefs -> BMP280_dig_P8) = coefs_t_p[10];
	(NVM_coefs -> BMP280_dig_P9) = coefs_t_p[11];

	// Humidity calibrate coefficiente begin here
		// dig_H1 coefficient lecture
		ret = BMP280_read(buffer_out, BMP280_REG_dig_H1, 1);
		if(ret != ESP_OK) {
			printf("ERROR reading calibration coefficient dig_H1: %x\n", ret);
		}
		else (NVM_coefs -> BMP280_dig_H1) = buffer_out[0];

		// dig_H2 coefficient lecture
		ret = BMP280_read(buffer_out, BMP280_REG_dig_H2, 2);
		if(ret != ESP_OK) {
			printf("ERROR reading calibration coefficient dig_H2: %x\n", ret);
		}
		else (NVM_coefs -> BMP280_dig_H2) = ((int16_t)buffer_out[1] << 8) | (int16_t)buffer_out[0];

		// dig_H3 coefficient lecture
		ret = BMP280_read(buffer_out, BMP280_REG_dig_H3, 1);
		if(ret != ESP_OK) {
			printf("ERROR reading calibration coefficient dig_H3: %x\n", ret);
		}
		else (NVM_coefs -> BMP280_dig_H3) = buffer_out[0];

		// dig_H6 coefficient lecture
		ret = BMP280_read(buffer_out, BMP280_REG_dig_H6, 1);
		if(ret != ESP_OK) {
			printf("ERROR reading calibration coefficient dig_H6: %x\n", ret);
		}
		else (NVM_coefs -> BMP280_dig_H6) = buffer_out[0];

		// dig_H4 and dig_H5 coefficient lecture, the use the same register for some differents bits
		ret = BMP280_read(buffer_out, BMP280_REG_dig_H4, 1);
		if(ret != ESP_OK) {
			printf("ERROR reading calibration coefficient dig_H4: %x\n", ret);
		}
		E4_read = (int16_t)buffer_out[0];	// saving content of 0xE4 register in another variable

		ret = BMP280_read(buffer_out, BMP280_REG_dig_H5, 1);
		if(ret != ESP_OK) {
			printf("ERROR reading calibration coefficient dig_H5: %x\n", ret);
		}
		E5_read = (int16_t)buffer_out[0];	// saving content of 0xE5 register in another variable

		ret = BMP280_read(buffer_out, BMP280_REG_dig_H6, 1);
		if(ret != ESP_OK) {
			printf("ERROR reading calibration coefficient dig_H6: %x\n", ret);
		}
		E6_read = (int16_t)buffer_out[0];	// saving content of 0xE6 register in another variable

		(NVM_coefs -> BMP280_dig_H4) = ((int16_t)E4_read << 4) | ((int16_t)(E5_read & 0x0F));
		(NVM_coefs -> BMP280_dig_H5) = ((int16_t)(E6_read << 4) | ((int16_t)E5_read >> 4));

		printf("Coeficientes NVM: \n");
		printf("T1: %d\n", NVM_coefs -> BMP280_dig_T1);
		printf("T2: %d\n", NVM_coefs -> BMP280_dig_T2);
		printf("T3: %d\n", NVM_coefs -> BMP280_dig_T3);
		printf("P1: %d\n", NVM_coefs -> BMP280_dig_P1);
		printf("P2: %d\n", NVM_coefs -> BMP280_dig_P2);
		printf("P3: %d\n", NVM_coefs -> BMP280_dig_P3);
		printf("P4: %d\n", NVM_coefs -> BMP280_dig_P4);
		printf("P5: %d\n", NVM_coefs -> BMP280_dig_P5);
		printf("P6: %d\n", NVM_coefs -> BMP280_dig_P6);
		printf("P7: %d\n", NVM_coefs -> BMP280_dig_P7);
		printf("P8: %d\n", NVM_coefs -> BMP280_dig_P8);
		printf("P9: %d\n", NVM_coefs -> BMP280_dig_P9);
		printf("H1: %d\n", NVM_coefs -> BMP280_dig_H1);
		printf("H2: %d\n", NVM_coefs -> BMP280_dig_H2);
		printf("H3: %d\n", NVM_coefs -> BMP280_dig_H3);
		printf("H4: %d\n", NVM_coefs -> BMP280_dig_H4);
		printf("H5: %d\n", NVM_coefs -> BMP280_dig_H5);
		printf("H6: %d\n", NVM_coefs -> BMP280_dig_H6);

	return ret;
}


/**
 * @brief 		RESET register
 *
 * @return		Response of esp_err_t function type
 *
 */
esp_err_t BMP280_reset_function()
{
	uint8_t		buffer_out[1];
	esp_err_t	ret;

	ret = BMP280_write_register(BMP280_REG_RESET, BMP280_RESET_VALUE);
	if(ret != ESP_OK) {
		printf("ERROR writing RESET: %x\n", ret);
	}

	ret = BMP280_read(buffer_out, BMP280_REG_RESET, 1);
	if(ret != ESP_OK) {
		printf("ERROR reading RESET: %x\n", ret);
	}
	else {
		printf("Software-reset response: %x\n", buffer_out[0]);
	}

	return ret;
}


/**
 * @brief	Master initialation
 *
 */
void hal_i2c_init()
{
	int i2c_master_port = I2C_MASTER_NUM;
	i2c_config_t conf;
		conf.mode = I2C_MODE_MASTER;
	    conf.sda_io_num = I2C_MASTER_SDA_IO;
	    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	    conf.scl_io_num = I2C_MASTER_SCL_IO;
	    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
	    conf.clk_flags = 0;
	i2c_param_config(i2c_master_port, &conf);
	i2c_driver_install(i2c_master_port, conf.mode,
			I2C_MASTER_RX_BUF_DISABLE,
			I2C_MASTER_TX_BUF_DISABLE, 0);

	vTaskDelay(100/portTICK_RATE_MS);
}


/**
 *	@brief		Slave initialation
 *
 *	@param[in]	*NVM_coef	:	(BMP280_calib_t)	struct where we are going to save the calibration parameters for compensate readings
 *
 */
int BMP280_init(BMP280_calib_t *NVM_coef, int sensor)
{
	uint8_t						data, aux;
	uint8_t						buffer_out[1];
	esp_err_t 					ret;

	int i2c_slave_port = I2C_SLAVE_NUM;
	i2c_config_t conf_slave;
	memset(&conf_slave, 0, sizeof(i2c_config_t));
		conf_slave.sda_io_num = I2C_MASTER_SDA_IO;
		conf_slave.sda_pullup_en = GPIO_PULLUP_ENABLE;
		conf_slave.scl_io_num = I2C_MASTER_SCL_IO;
		conf_slave.scl_pullup_en = GPIO_PULLUP_ENABLE;
		conf_slave.mode = I2C_MODE_SLAVE;
		conf_slave.slave.addr_10bit_en = 0;
		conf_slave.slave.slave_addr = BMP280_I2C_ADDRESS;
	i2c_param_config(i2c_slave_port, &conf_slave);
	i2c_driver_install(i2c_slave_port, I2C_MODE_SLAVE,
						I2C_SLAVE_RX_BUF_LEN,
						I2C_SLAVE_TX_BUF_LEN, 0);

	vTaskDelay(1000/portTICK_RATE_MS);

	ret = BMP280_write_command(BMP280_REG_ID);
	if(ret != ESP_OK) {
		printf("ERROR at first iteration.\n");
	}

	ret = BMP280_read(buffer_out, BMP280_REG_ID, 1);
	if(ret != ESP_OK) {
		printf("ERROR reading hardware ID: %x\n", ret);
		return -1;
	}
	else printf("Sensor hardware-id: %x\n", buffer_out[0]);

	vTaskDelay(100/portTICK_RATE_MS);	// without this delay the sensor cant do the next instruction!!


	// RESET operation, you choose if you want to use it... i recommend to execute it
	BMP280_reset_function();

	vTaskDelay(100/portTICK_RATE_MS);


	// Status register comprobation. NVM coefs copied to registers if -> bit 0 = 0
	while(1) {
		ret = BMP280_read(buffer_out, BMP280_REG_STATUS, 1);
		if(ret != ESP_OK) {
			printf("ERROR reding status-register: %x\n", ret);
		}
		//else printf("Lectura de registro de estado: %x\n",buffer_out[0]);

		aux = buffer_out[0] & 0x01;		// bit-mask for bit 0 of the register
		if(aux == 0) {
			break;
		}
	}


	// Calibration coefficients lecture - temperature, humidity and pressure
	BMP280_read_calib_coeff(NVM_coef);

	vTaskDelay(100/portTICK_RATE_MS);


	// Configuration selection of temperature and pressure
	// oversamp_temperature - oversamp_pressure - sensor_mode
	data = ((BMP280_oversampling_x2 << 5) & 0xE0) | ((BMP280_oversampling_x2 << 2) & 0xFC) | (BMP280_set_sleep_mode);
	// AND (&) because using the bit-mask you write the valor completely
	ret = BMP280_write_register(BMP280_REG_CTRL_MEAS, data);
	if(ret != ESP_OK) {
		printf("ERROR writing at %x (temperature and pressure): %x\n", BMP280_REG_CTRL_MEAS,ret);
	}
	ret = BMP280_read(buffer_out, BMP280_REG_CTRL_MEAS, 1);
	if(ret != ESP_OK) {
		printf("ERROR reading at %x (temperature and pressure): %x\n", BMP280_REG_CTRL_MEAS, ret);
		return -1;
	}
	else printf("%x valor (temperature and pressure): %x\n", BMP280_REG_CTRL_MEAS, buffer_out[0]);


	// Options to select the data adquire of humidity -> uncomment if you have BME280 !!! comment it if you have BMP280

	if(sensor == 1) {
		ret = BMP280_write_register(BMP280_REG_CTRL_HUM, BMP280_oversampling_x16);
		if(ret != ESP_OK) {
			printf("ERROR writing oversampling for humidity: %x\n", ret);
		}
		ret = BMP280_read(buffer_out, BMP280_REG_CTRL_HUM, 1);
		if(ret != ESP_OK) {
			printf("ERROR reading at %x: %x\n", BMP280_REG_CTRL_HUM, ret);
			return -1;
		}
		else printf("Humidity oversampling valor: %x\n", buffer_out[0]);
	}

	vTaskDelay(100/portTICK_RATE_MS);

	return 0;

}


/**
 * @brief		Temperature compensation in int-variable for BMP280
 *
 * @param[in]		adc_tempe	:	(int32_t)			raw adc temperature data
 * @param[in]		*NVM_coef	:	(BMP280_calib_t)	struct where we are going to save the calibration parameters for compensate readings
 *
 * @return		t_fine		:	(int32_t)	compensate t_fine temperature variable
 *
 */
int32_t BMP280_int_temperature_compensation(int32_t adc_tempe, BMP280_calib_t *NVM_coef)
{
	int32_t		t_fine;
    int32_t 	var1, var2;

    var1 = ((((adc_tempe / 8) - ((int32_t)NVM_coef->BMP280_dig_T1 << 1))) * ((int32_t)NVM_coef->BMP280_dig_T2)) / 2048;
    var2 = (((((adc_tempe / 16) - ((int32_t)NVM_coef->BMP280_dig_T1)) * ((adc_tempe / 16) - ((int32_t)NVM_coef->BMP280_dig_T1))) / 4096) * ((int32_t)NVM_coef->BMP280_dig_T3)) / 16384;

    t_fine = (var1 + var2);

    return t_fine;
}


/**
 * @brief		Pressure compensation for BMP280
 *
 * @param[in]		adc_pres		:	(int32_t)			raw adc pressure data
 * @param[in]		*NVM_coef		:	(BMP280_calib_t)		struct where we are going to save the calibration parameters for compensate readings
 * @param[in]		t_fine			:	(int32_t)			compensate t_fine temperature variable
 *
 * @return		p			:	(uint32_t)			compensate pressure
 *
 */
uint32_t BMP280_int_pressure_compensation(int32_t adc_pres, BMP280_calib_t *NVM_coef, int32_t t_fine)
{
    int64_t var1, var2, p;

    var1 = (int64_t)t_fine - 128000;
    var2 = var1 * var1 * (NVM_coef->BMP280_dig_P6);
    var2 = var2 + (((int64_t)var1*(NVM_coef->BMP280_dig_P5)) << 17);
    var2 = var2 + ((int64_t)(NVM_coef->BMP280_dig_P4) << 35);
    var1 = ((var1 * var1 * (NVM_coef->BMP280_dig_P3)) >> 8) + ((var1 * (NVM_coef->BMP280_dig_P2)) << 12);
    var1 = (((int64_t)1 << 47) + var1) * (NVM_coef->BMP280_dig_P1) >> 33;

    if (var1 == 0) {
        return 0;
    }

    p = 1048576 - adc_pres;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = ((NVM_coef->BMP280_dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = ((NVM_coef->BMP280_dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + ((NVM_coef->BMP280_dig_P7) << 4);

    return (uint32_t)p;
}


/**
 * @brief		Compensate humidity for BME280
 *
 * @param[in]		adc_hum			:	(int32_t)			raw adc humidity data
 * @param[in]		*NVM_coef		:	(BMP280_calib_t)		struct where we are going to save the calibration parameters for compensate readings
 * @param[in]		t_fine			:	(int32_t)			compensate t_fine temperature variable
 *
 * @return		p			:	(uint32_t)			compensate humidity
 *
 */
uint32_t BMP280_int_humidity_compensation(int32_t adc_hum, BMP280_calib_t *NVM_coef, int32_t t_fine)
{
    int32_t res;

    res = (t_fine) - 76800;
    res = ((((adc_hum << 14) - ((int32_t)(NVM_coef->BMP280_dig_H4) << 20) - ((NVM_coef->BMP280_dig_H5) * res))
                + 16384) >> 15) * (((((((res * (NVM_coef->BMP280_dig_H6)) >> 10) *
                (((res * (NVM_coef->BMP280_dig_H3)) >> 11) + 32768)) >> 10) + 2097152) * (NVM_coef->BMP280_dig_H2) +
                8192) >> 14);
    res = res - (((((res >> 15) * (res >> 15)) >> 7) * (NVM_coef->BMP280_dig_H1)) >> 4);
    res = res < 0 ? 0 : res;
    res = res > 419430400 ? 419430400 : res;

    return (uint32_t)(res >> 12);
}


/**
 * @brief		Get final data for BME/P 280
 *
 * @param[in]		*temperature		:	(float)					pointer to temperature float variable
 * @param[in]		*pressure		:	(float)				pointer to pressure float variable
 * @param[in]		*humidity		:	(float)				pointer to humidity float variable
 * @param[in]		sensor			:	(int)				type of sensor that we are using. Choose 1 for BME280 or 0 for BMP280
 * @param[in]		*NVM_coef		:	(BMP280_calib_t)		struct where we are going to save the calibration parameters for compensate readings
 *
 */
void BMP280_final_data(float *temperature, float *pressure, float *humidity, int sensor, BMP280_calib_t *NVM_coef)
{
	uint8_t			measuring_flag, buffer_in;
	uint8_t			buffer_out[8];
	uint32_t		adc_tempe, adc_pres, adc_hum;
	int32_t			t_fine;
	esp_err_t		ret;

	adc_tempe = 0;
	adc_pres = 0;
	adc_hum = 0;
	measuring_flag = 1;

	// We need to set forced-mode if we want to read ADC values... is because in forced-mode when we read ones, automatically sets to sleep-mode...
	ret = BMP280_read(buffer_out, BMP280_REG_CTRL_MEAS, 1);
	if(ret != ESP_OK) {
		printf("ERROR reading ctrl-meas register: %x\n", ret);
	}

	buffer_in = (buffer_out[0] & 0xFC) | BMP280_set_forced_mode;

	ret = BMP280_write_register(BMP280_REG_CTRL_MEAS, buffer_in);
	if(ret != ESP_OK) {
		printf("ERROR selecting forced mode: %x\n", ret);
	}

	while (measuring_flag == 1) {
		ret = BMP280_read(buffer_out, BMP280_REG_STATUS, 1);
		if(ret != ESP_OK) {
			printf("ERROR reading status register: %x\n", ret);
		}

		measuring_flag = (buffer_out[0] >> 7) & 0x01;
	}

	// Pressure reading
	ret = BMP280_read(buffer_out, BMP280_REG_PRESS_MSB, 3);
	if(ret != ESP_OK) {
		printf("ERROR reading pressure data registers: %x\n", ret);
	}
	else adc_pres = (uint32_t) (((uint32_t) buffer_out[0] << 12) | ((uint32_t) buffer_out[1] << 4) | ((uint32_t) buffer_out[2] >> 4));

	vTaskDelay(100/portTICK_RATE_MS);

	// Temperature reading
	ret = BMP280_read(buffer_out, BMP280_REG_TEMP_MSB, 3);
	if(ret != ESP_OK) {
		printf("ERROR reading temperature data registers: %x\n", ret);
	}
	else adc_tempe = (uint32_t) (((uint32_t) buffer_out[0] << 12) | ((uint32_t) buffer_out[1] << 4) | ((uint32_t) buffer_out[2] >> 4));

	vTaskDelay(100/portTICK_RATE_MS);

	t_fine = BMP280_int_temperature_compensation(adc_tempe, NVM_coef);
	*temperature = (float) ((t_fine*5 + 128) / 256) / 100;

	// Humidity reading
	if(sensor == 1) {
		ret = BMP280_read(buffer_out, BMP280_REG_HUM_MSB, 2);
		if(ret != ESP_OK) {
			printf("ERROR reading humidity data registers: %x\n", ret);
		}
		else adc_hum = (uint32_t) (((uint32_t) buffer_out[0] << 8) | (uint32_t) buffer_out[1]);

		*humidity = (float) BMP280_int_humidity_compensation(adc_hum, NVM_coef, t_fine) / 1024.0;
	}

	*pressure = (float) BMP280_int_pressure_compensation(adc_pres, NVM_coef, t_fine) / 25600.0;

}




#ifdef __cplusplus
}
#endif
