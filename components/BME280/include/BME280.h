#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

// ***************************************************

esp_err_t 	BMP280_read(uint8_t * buffer_out, uint8_t BMP280_command, unsigned size);
esp_err_t 	BMP280_write_command(uint8_t BMP280_command);
esp_err_t 	BMP280_write_register(uint8_t BMP280_register, uint8_t BMP280_register_value);
esp_err_t 	BMP280_read_calib_coeff(BMP280_calib_t *NVM_coefs);
esp_err_t 	BMP280_reset_function();
int 		BMP280_init(BMP280_calib_t *NVM_coef, int sensor);
void 		BMP280_final_data(float *temperature, float *pressure, float *humidity, int sensor, BMP280_calib_t *NVM_coef);
uint32_t 	BMP280_int_humidity_compensation(int32_t adc_hum, BMP280_calib_t *NVM_coef, int32_t t_fine);
uint32_t 	BMP280_int_pressure_compensation(int32_t adc_pres, BMP280_calib_t *NVM_coef, int32_t t_fine);
int32_t		BMP280_int_temperature_compensation(int32_t adc_tempe, BMP280_calib_t *NVM_coef);

// ***************************************************


// Parametros para conexion I2C
#define 	I2C_MASTER_FREQ_HZ          	100000
#define 	I2C_MASTER_TX_BUF_DISABLE   	0                          /*!< I2C master doesn't need buffer */
#define 	I2C_MASTER_RX_BUF_DISABLE   	0                          /*!< I2C master doesn't need buffer */
#define 	I2C_MASTER_TIMEOUT_MS       	1000
#define 	I2C_MASTER_SDA_IO 				21
#define 	I2C_MASTER_SCL_IO 				22
#define 	I2C_MASTER_NUM	 				I2C_NUM_1

#define 	DATA_LENGTH	  	        		512
#define 	I2C_SLAVE_NUM 					I2C_NUM_0    /*!<I2C port number for slave dev */
#define 	I2C_SLAVE_TX_BUF_LEN	  		(2*DATA_LENGTH) /*!<I2C slave tx buffer size */
#define 	I2C_SLAVE_RX_BUF_LEN  			(2*DATA_LENGTH) /*!<I2C slave rx buffer size */

// Direccion I2C del dispositivo
#define		BMP280_I2C_ADDRESS				0x76
#define		BMP280_I2C_ADDRESS_2			0x77

// Registros del BMP280
#define		BMP280_REG_HUM_MSB				0xFD
#define		BMP280_REG_HUM_LSB				0xFE
#define 	BMP280_REG_TEMP_XLSB   			0xFC /* bits: 7-4 */
#define 	BMP280_REG_TEMP_LSB    			0xFB
#define 	BMP280_REG_TEMP_MSB    			0xFA
#define 	BMP280_REG_TEMP        			(BMP280_REG_TEMP_MSB)
#define 	BMP280_REG_PRESS_XLSB  			0xF9 /* bits: 7-4 */
#define 	BMP280_REG_PRESS_LSB   			0xF8
#define	 	BMP280_REG_PRESS_MSB   			0xF7
#define 	BMP280_REG_PRESSURE    			(BMP280_REG_PRESS_MSB)
#define 	BMP280_REG_CONFIG      			0xF5 /* bits: 7-5 t_sb; 4-2 filter; 0 spi3w_en */
#define 	BMP280_REG_CTRL_MEAS      		0xF4 /* bits: 7-5 osrs_t; 4-2 osrs_p; 1-0 mode */
#define 	BMP280_REG_STATUS      			0xF3 /* bits: 3 measuring; 0 im_update */
#define 	BMP280_REG_CTRL_HUM    			0xF2 /* bits: 2-0 osrs_h; */
#define 	BMP280_REG_RESET       			0xE0
#define 	BMP280_REG_ID          			0xD0
#define 	BMP280_REG_CALIB       			0x88
#define 	BMP280_REG_HUM_CALIB   			0x88
#define 	BMP280_RESET_VALUE     			0xB6
#define		BMP280_REG_dig_H1				0xA1
#define		BMP280_REG_dig_H2				0xE1
#define		BMP280_REG_dig_H3				0xE3
#define		BMP280_REG_dig_H4				0xE4
#define		BMP280_REG_dig_H5				0xE5
#define		BMP280_REG_dig_H6				0xE7

// Configuracion de oversampling
#define		BMP280_oversampling_skip		0x00	// (000)
#define		BMP280_oversampling_x1			0x01	// (001)
#define		BMP280_oversampling_x2			0x02	// (010)
#define		BMP280_oversampling_x4			0x03	// (011)
#define		BMP280_oversampling_x8			0x04	// (100)
#define		BMP280_oversampling_x16			0x07	// (111)

//	Configuracion de modo del sensor
#define		BMP280_set_sleep_mode			0x00	// (00)
#define		BMP280_set_forced_mode			0x01	// (01)
#define		BMP280_set_normal_mode			0x03	// (11)

// Tiempos de standby
#define		BMP280_standby_05ms				0x00	// (000)
#define		BMP280_standby_62_5ms			0x01	// (001)
#define		BMP280_standby_125ms			0x02	// (010)
#define		BMP280_standby_250ms			0x03	// (011)
#define		BMP280_standby_500ms			0x04	// (100)
#define		BMP280_standby_1000ms			0x05	// (101)
#define		BMP280_standby_10ms				0x06	// (110)
#define		BMP280_standby_20ms				0x07	// (111)

// Configuracion del filtro
#define		BMP280_set_filter_off			0x00	// (000)
#define		BMP280_set_filter_coef_2		0x01	// (001)
#define		BMP280_set_filter_coef_4		0x02	// (010)
#define		BMP280_set_filter_coef_8		0x03	// (011)
#define		BMP280_set_filter_coef_16		0x04	// (100)


typedef struct {
	// coeficientes calibracion temperatura
	uint16_t		BMP280_dig_T1;
	int16_t			BMP280_dig_T2;
	int16_t			BMP280_dig_T3;

	// coeficientes calibracion presion
	uint16_t		BMP280_dig_P1;
	int16_t			BMP280_dig_P2;
	int16_t			BMP280_dig_P3;
	int16_t			BMP280_dig_P4;
	int16_t			BMP280_dig_P5;
	int16_t			BMP280_dig_P6;
	int16_t			BMP280_dig_P7;
	int16_t			BMP280_dig_P8;
	int16_t			BMP280_dig_P9;

	// coeficientes calibracion humedad
	uint8_t			BMP280_dig_H1;
	int16_t			BMP280_dig_H2;
	uint8_t			BMP280_dig_H3;
	int16_t			BMP280_dig_H4;
	int16_t			BMP280_dig_H5;
	int8_t			BMP280_dig_H6;
} BMP280_calib_t;



#ifdef __cplusplus
}
#endif
