/*
 * amis_config.c
 *
 *  Created on: Nov 27, 2024
 *      Author: tomek
 */


#include "amis_config.h"

void AMIS_Base_Init(amis_base_st *base, SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin, TIM_HandleTypeDef *htim) {
    base->spi_gpio.interface_handle = hspi;
    base->spi_gpio.gpio_handle = cs_port;
    base->spi_gpio.gpio_pin = cs_pin;
    base->write_reg = &writeRegister;  // Set default write function
    base->read_reg = &readRegister;    // Set default read function
    base->timeout = NULL;             // Not used; set to NULL or default
    base->tim = htim;
}

void AMIS_Config_Init(amis_config_st *config) {
    config->start = ENABLE_AMIS;
    config->current = CURRENT_RANGE_2_1260_millis;
    config->stepmode = STEP_MODE_32_MICRO_STEP;
    config->watchdog.start = WATCHDOG_DISABLE;
    config->watchdog.timeout = WATCHDOG_TIMEOUT_128_MILLIS;
}

void AMIS_Pins_Init(amis_pins_st *pins, GPIO_TypeDef *gpio_port_nxt, GPIO_TypeDef *gpio_port_dir, GPIO_TypeDef *gpio_port_clr, uint16_t nxt_pin, uint16_t dir_pin, uint16_t clr_pin) {
    pins->GPIO_NXT = gpio_port_nxt;
    pins->PIN_NXT = nxt_pin;
    pins->GPIO_DIR = gpio_port_dir;
    pins->PIN_DIR = dir_pin;
    pins->GPIO_CLR = gpio_port_clr;
    pins->PIN_CLR = clr_pin;
}
