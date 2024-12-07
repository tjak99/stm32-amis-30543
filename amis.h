/*
 * amis.h
 *
 *  Created on: Oct 31, 2024
 *      Author: tomek
 *
 *      Status registers have parity check
 *      Control registers do not have parity check
 */

#ifndef INC_AMIS_H_
#define INC_AMIS_H_

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <string.h>
#include <math.h>
// SPI CONTROL REGISTERS
#define WR 		0x00U
#define CR0		0x01U
#define CR1		0x02U
#define CR2 	0x03U
#define CR3 	0x09U

// SPI STATUS REGISTERS
#define SR0 	0x04U
#define SR1 	0x05U
#define SR2		0x06U
#define SR3 	0x07U
#define SR4 	0x0AU

#define MOTOR_TIMEOUT 	  100.0
#define MAX_MOTOR_TIMEOUT 250.0

typedef uint8_t (*amis_read_ptr)(void *conf, uint8_t reg,  uint8_t *data, uint16_t len);
typedef uint8_t (*amis_write_ptr)(void *conf, uint8_t reg,  uint8_t *data, uint16_t len);
typedef void     (*amis_timeout_ptr)(uint32_t millis);

typedef struct {
	struct{
		void *interface_handle; // SPI HAL handle
		void *gpio_handle; // GPIO handle
		uint16_t gpio_pin;
	} spi_gpio;
	amis_write_ptr write_reg; // for writing data
	amis_read_ptr read_reg;   // for reading data
	amis_timeout_ptr timeout; // wait function
	TIM_HandleTypeDef *tim;
} amis_base_st;

typedef enum {
	WATCHDOG_ENABLE = 0x01,
	WATCHDOG_DISABLE = 0x00,
} amis_watchdog_start_en;

typedef enum {
	WATCHDOG_TIMEOUT_32_MILLIS = 0x00,
	WATCHDOG_TIMEOUT_64_MILLIS = 0x01,
	WATCHDOG_TIMEOUT_96_MILLIS = 0x02,
	WATCHDOG_TIMEOUT_128_MILLIS = 0x03,
	WATCHDOG_TIMEOUT_160_MILLIS = 0x04,
	WATCHDOG_TIMEOUT_192_MILLIS = 0x05,
	WATCHDOG_TIMEOUT_224_MILLIS = 0x06,
	WATCHDOG_TIMEOUT_256_MILLIS = 0x07,
} amis_watchdog_timeout_en;

typedef struct {
	amis_watchdog_start_en start;
	amis_watchdog_timeout_en timeout;
} amis_watchodog_st;

typedef enum {
	STEP_MODE_32_MICRO_STEP = 0x00,
	STEP_MODE_16_MICRO_STEP = 0x01,
	STEP_MODE_8_MICRO_STEP = 0x02,
	STEP_MODE_4_MICRO_STEP = 0x03,
	STEP_MODE_COMPENSATED_HALF = 0x04,
	STEP_MODE_UNCOMPENSATED_HALF = 0x05,
	STEP_MODE_UNCOMPENSATED_FULL = 0x06, // ESM is always set to 0x00
} amis_stepmode_en;

typedef enum {
	CURRENT_RANGE_0_132_millis = 0x00,
	CURRENT_RANGE_0_245_millis = 0x01,
	CURRENT_RANGE_0_355_millis = 0x02,
	CURRENT_RANGE_1_395_millis = 0x03,
	CURRENT_RANGE_1_445_millis = 0x04,
	CURRENT_RANGE_1_485_millis = 0x05,
	CURRENT_RANGE_1_540_millis = 0x06,
	CURRENT_RANGE_1_586_millis = 0x07,
	CURRENT_RANGE_1_640_millis = 0x08,
	CURRENT_RANGE_1_715_millis = 0x09,
	CURRENT_RANGE_2_780_millis = 0x0A,
	CURRENT_RANGE_2_870_millis = 0x0B,
	CURRENT_RANGE_2_955_millis = 0x0C,
	CURRENT_RANGE_2_1060_millis = 0x0D,
	CURRENT_RANGE_2_1150_millis = 0x0E,
	CURRENT_RANGE_2_1260_millis = 0x0F,
	CURRENT_RANGE_3_1405_millis = 0x10,
	CURRENT_RANGE_3_1520_millis = 0x11,
	CURRENT_RANGE_3_1695_millis = 0x12,
} amis_current_en;

typedef enum {
	ENABLE_AMIS = 0x01,
	DISABLE_AMIS = 0x00,
} amis_driver_enable_en;

typedef struct{
	float motor_angle_resolution; // degree
} motor_params_t;

typedef struct {
	amis_watchodog_st     watchdog;
	amis_current_en       current;
	amis_stepmode_en      stepmode;
	amis_driver_enable_en start;
} amis_config_st;

typedef enum {
	AMIS_OK,
	AMIS_ERROR_SPI,
	AMIS_OVERCURRENT,
	AMIS_ERROR,
} AMIS_Status;

typedef struct {
	GPIO_TypeDef *GPIO_NXT;
	uint16_t PIN_NXT;
	GPIO_TypeDef *GPIO_DIR;
	uint16_t PIN_DIR;
	GPIO_TypeDef *GPIO_CLR;
	uint16_t PIN_CLR;
} amis_pins_st;


// Auxiliary functions
void delay_us(TIM_HandleTypeDef *tim, uint16_t us);

// Base functions
AMIS_Status AMIS_Init(amis_base_st *base, amis_config_st *config);
void Amis_Move(amis_config_st *config, motor_params_t *motor_param, float angle_deg, amis_pins_st* amis_pins, amis_base_st *base);
AMIS_Status AMIS_GetCurrentPosition(amis_base_st *base, uint16_t *pos, motor_params_t *motor_param);
AMIS_Status Amis_Reset(amis_pins_st *amis_pins);
AMIS_Status Amis_ChangeDir(amis_pins_st *amis_pins);

// Low level functions
uint8_t writeRegister(void *conf, uint8_t reg,  uint8_t *data, uint16_t len);
uint8_t readRegister(void *conf, uint8_t reg,  uint8_t *data, uint16_t len);
void setCsPin(void *conf, uint8_t val);


// Medium level functions
// Init function
AMIS_Status Amis_step_set(amis_base_st *base, amis_config_st *config);
AMIS_Status Amis_watchdog_set(amis_base_st *base, amis_config_st *config);
AMIS_Status Amis_current_set(amis_base_st *base, amis_config_st *config);
AMIS_Status Amis_start_set(amis_base_st *base, amis_config_st *config);

// Move function
float Amis_calculate_steps(amis_config_st *config, motor_params_t *motor_param, float angle_deg);
void pwm_impulse_gen(amis_pins_st *amis_pins, uint16_t us, amis_base_st *base);
void setPin(amis_pins_st *amis_pins, uint8_t val, uint8_t which);
void move_gently(amis_base_st *base, amis_pins_st *amis_pins,int steps);

// Diagnostic functions
uint8_t Amis_status_parity_check(uint8_t status);
uint8_t Amis_status_get(amis_base_st *base, uint8_t reg);

#endif
