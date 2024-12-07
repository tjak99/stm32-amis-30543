/*
 * amis_config.h
 *
 *  Created on: Nov 27, 2024
 *      Author: tomek
 */

#ifndef INC_AMIS_CONFIG_H_
#define INC_AMIS_CONFIG_H_

#include "amis.h"

void AMIS_Base_Init(amis_base_st *base, SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin, TIM_HandleTypeDef *htim);

void AMIS_Config_Init(amis_config_st *config);

void AMIS_Pins_Init(amis_pins_st *pins, GPIO_TypeDef *gpio_port_nxt, GPIO_TypeDef *gpio_port_dir, GPIO_TypeDef *gpio_port_clr, uint16_t nxt_pin, uint16_t dir_pin, uint16_t clr_pin);

#endif /* INC_AMIS_CONFIG_H_ */
