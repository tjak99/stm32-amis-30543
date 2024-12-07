/*
 * amis.c
 *
 *  Created on: Nov 4, 2024
 *      Author: tomek
 */

#include "amis.h"
#include "stm32f1xx_hal.h"


/* Data from AMIS are transsferred using MSB first
 * Uses handle to the hal spi to transfer data into specified register
 *
 */
uint8_t writeRegister(void *conf, uint8_t reg, uint8_t *data, uint16_t len) {
    amis_base_st *base_conf = (amis_base_st *)conf;
    SPI_HandleTypeDef *spi = (SPI_HandleTypeDef *)base_conf->spi_gpio.interface_handle;

    setCsPin(base_conf, 0); // CS pin down

    // Check if TX buffer is empty before sending data
    if (SPI_CHECK_FLAG(spi->Instance->SR, SPI_SR_TXE)) {
        uint8_t buff[len + 1];             // Data + command/address byte
        memset(buff, 0, sizeof(buff));      // Clear memory

        // Set the command and address byte
        buff[0] = (reg & 0x1FU) | 0x80U;

        memcpy(&buff[1], data, len);

        if (HAL_SPI_Transmit(spi, buff, len + 1, HAL_MAX_DELAY) == HAL_OK) {
        	setCsPin(base_conf, 1);
            return HAL_OK;
        } else{
            setCsPin(base_conf, 1);
            return HAL_ERROR;
        }
    } else {
    	setCsPin(base_conf, 1);
    }
    return HAL_BUSY;
}

/*
 * @ brief
 * @ data is where read bytes will be placed
 * @ conf is a handle to SPI or other interface
 * @ len is length of data that should be read
 * @ reg reg from where data should be read
 */
uint8_t readRegister(void *conf, uint8_t reg, uint8_t *data, uint16_t len) { // rejestr ,tablica jednoelementowa, dlugosc 1
    amis_base_st *base_conf = (amis_base_st *)conf;
    SPI_HandleTypeDef *spi = (SPI_HandleTypeDef *)base_conf->spi_gpio.interface_handle;

    setCsPin(base_conf, 0); // setting CS down

    uint8_t txbuff[len+1];
    uint8_t rxbuff[len+1];   // Buffer to receive data
    memset(txbuff, 0, sizeof(txbuff));
    memset(rxbuff, 0, sizeof(rxbuff));

    txbuff[0] = (reg & 0x1FU) & (~0x80U); // Prepare the READ command

    // Perform the SPI transmit-receive operation
    if (HAL_SPI_TransmitReceive(spi, txbuff, rxbuff, len+1, HAL_MAX_DELAY) == HAL_OK) {
        memcpy(data, &rxbuff[1], len); // copies only data from rxbuff to data
        setCsPin(base_conf, 1);
        return HAL_OK;
    }
    setCsPin(base_conf, 1);
    return HAL_ERROR;
}


/*
 * @ brief pass configuration of the amis driver and value
 * @ val can be 0 to set low dedicated pin otherwise pin will be set high
 * @ conf has to be driver structure that has access to gpio
 */
void setCsPin(void *conf, uint8_t val){
    amis_base_st *base_conf = (amis_base_st *)conf;
    GPIO_TypeDef *gpio = (GPIO_TypeDef*)base_conf->spi_gpio.gpio_handle;
    if(val != 0){
    	gpio->BSRR = base_conf->spi_gpio.gpio_pin;
    } else {
    	gpio->BSRR = (uint32_t)base_conf->spi_gpio.gpio_pin << 16u;
    }
}

void delay_us(TIM_HandleTypeDef *tim, uint16_t us){
	{
		__HAL_TIM_SET_COUNTER(tim,0);  // set the counter value a 0
		while (__HAL_TIM_GET_COUNTER(tim) < us);  // wait for the counter to reach the us input in the parameter
	}
}
void setPin(amis_pins_st *amis_pins, uint8_t val, uint8_t which){
	if(which==0){
		if(val==1){
			//amis_pins->GPIO_NXT->BSRR = amis_pins->PIN_NXT;
			HAL_GPIO_WritePin(amis_pins->GPIO_NXT, amis_pins->PIN_NXT, GPIO_PIN_SET);
		}
		else{
			//amis_pins->GPIO_NXT->BSRR = amis_pins->PIN_NXT << 16u;
			HAL_GPIO_WritePin(amis_pins->GPIO_NXT, amis_pins->PIN_NXT, GPIO_PIN_RESET);
		}
	}else{
		if(val==1){
			amis_pins->GPIO_DIR->BSRR = amis_pins->PIN_DIR;
		}
		else{
			amis_pins->GPIO_DIR->BSRR = amis_pins->PIN_DIR << 16u;
		}
	}
}
void pwm_impulse_gen(amis_pins_st *amis_pins, uint16_t us, amis_base_st *base)
{
	setPin(amis_pins, 1, 0);
	delay_us(base->tim, us);
	setPin(amis_pins, 0, 0);
	delay_us(base->tim, us);
}

//AMIS_Status Amis_step_set(amis_base_st *base, amis_config_st *config){
//	// set zeros in ESM
//	// set value provided in config
//	uint8_t buff[1] = {192};
//	if(base->write_reg((void*)base, CR3, buff, 1) != HAL_OK){
//		return AMIS_ERROR;
//	}
//	uint8_t check1[1] = {0};
//	if(base->read_reg((void*)base, CR3, check1, 1) != HAL_OK){
//		return AMIS_ERROR;
//	}
//	uint8_t data_recv[1] = {0}; // one data byte, after read it should be data in the register
//	if(base->read_reg((void*)base, CR0, data_recv, 1) != HAL_OK){ // length 1, because one data byte
//		return AMIS_ERROR;
//	}
//	// Clear bits 7, 6, and 5
//	data_recv[0] &= ~(0b111 << 5);
//	data_recv[0] |= (config->stepmode & 0b111) << 5;
//
//	if(base->write_reg((void*)base, CR0, data_recv, 1) != HAL_OK){
//		return AMIS_ERROR;
//	}
//	uint8_t check[1] = {0};
//	if(base->read_reg((void*)base, CR0, check, 1) != HAL_OK){
//		return AMIS_ERROR;
//	}
//	return AMIS_OK;
//}
AMIS_Status Amis_step_set(amis_base_st *base, amis_config_st *config) {
    uint8_t data_recv[1];

    // Clear ESM[2:0] bits in CR3 (bits 0-2)
    if (base->read_reg((void *)base, CR3, data_recv, 1) != HAL_OK) {
        return AMIS_ERROR;
    }
    data_recv[0] &= ~0x07; // Clear bits 0-2 (ESM[2:0])
    // Set ESM[2:0] to 0x00 (standard step modes)
    if (base->write_reg((void *)base, CR3, data_recv, 1) != HAL_OK) {
        return AMIS_ERROR;
    }

    // Configure SM[2:0] in CR0 (bits 5-7)
    if (base->read_reg((void *)base, CR0, data_recv, 1) != HAL_OK) {
        return AMIS_ERROR;
    }
    data_recv[0] &= ~(0x07 << 5);                      // Clear bits 5-7 (SM[2:0])
    data_recv[0] |= (config->stepmode & 0x07) << 5;    // Set new step mode
    if (base->write_reg((void *)base, CR0, data_recv, 1) != HAL_OK) {
        return AMIS_ERROR;
    }

    return AMIS_OK;
}

// 00000101
AMIS_Status Amis_watchdog_set(amis_base_st *base, amis_config_st *config){
	uint8_t data_recv[1] = {0};

	if(base->read_reg((void*)base, WR, data_recv, 1) != HAL_OK){
		return AMIS_ERROR;
	}
	data_recv[0] = data_recv[0] | (config->watchdog.start << 7) | (config->watchdog.timeout << 3);

	if(base->write_reg((void*)base, WR, data_recv, 1) != HAL_OK){
		return AMIS_ERROR;
	}
	return AMIS_OK;
}

AMIS_Status Amis_current_set(amis_base_st *base, amis_config_st *config){
	uint8_t data_recv[1] = {0};
	if(base->read_reg((void*)base, CR0, data_recv, 1) != HAL_OK){
		return AMIS_ERROR;
	}
	//data_recv[0] = data_recv[0] | config->current; // set current keeping the rest
	data_recv[0] = (data_recv[0] & ~(uint8_t)0x1F) | config->current;
	if(base->write_reg((void*)base, CR0, data_recv, 1) != HAL_OK){
		return AMIS_ERROR;
	}
	uint8_t ch[1] = {0};
	if(base->read_reg((void*)base, CR0, ch, 1) != HAL_OK){
		return AMIS_ERROR;
	}
	return AMIS_OK;
}



AMIS_Status Amis_start_set(amis_base_st *base, amis_config_st *config) {
    uint8_t data_recv[1];

    // Read the current value of CR2
    if (base->read_reg((void *)base, CR2, data_recv, 1) != HAL_OK) {
        return AMIS_ERROR;
    }

    // clear 4 MSB bits MOTEN/SLP/SLAG/SLAT
    data_recv[0] &= ~0x0F;

    // Set MOTEN
    data_recv[0] |= (config->start << 7);
    // Write the updated value back to CR2
    if (base->write_reg((void *)base, CR2, data_recv, 1) != HAL_OK) {
        return AMIS_ERROR;
    }

    return AMIS_OK;
}

float Amis_calculate_steps(amis_config_st *config, motor_params_t *motor_param, float angle_deg){
	float steps_res = 0;

	switch(config->stepmode){
		case STEP_MODE_COMPENSATED_HALF:
			steps_res = 0.5;
			break;
		case STEP_MODE_UNCOMPENSATED_FULL:
			steps_res = 1.0;
			break;
		case STEP_MODE_UNCOMPENSATED_HALF:
			steps_res = 2.0;
			break;
		case STEP_MODE_4_MICRO_STEP:
			steps_res = 4.0;
			break;
		case STEP_MODE_8_MICRO_STEP:
			steps_res = 8.0;
			break;
		case STEP_MODE_16_MICRO_STEP:
			steps_res = 16.0;
			break;
		case STEP_MODE_32_MICRO_STEP:
			steps_res = 32.0;
			break;
		default:
			steps_res = 1.0;
		}
	float steps = (angle_deg*steps_res) / (motor_param->motor_angle_resolution);

	return steps;
}

void Amis_Move(amis_config_st *config, motor_params_t *motor_param, float angle_deg, amis_pins_st* amis_pins, amis_base_st *base){

	float steps = Amis_calculate_steps(config, motor_param, angle_deg);
	int steps_r = round(steps);

	move_gently(base, amis_pins, steps_r);
//	for(int s=0; s<steps; s++){
//		pwm_impulse_gen(amis_pins, MOTOR_TIMEOUT, base);
//	}
}

void move_gently(amis_base_st *base, amis_pins_st *amis_pins,int steps){
	int slow = steps/5;
	int normal = steps - slow - slow;
	float acc = (MAX_MOTOR_TIMEOUT - MOTOR_TIMEOUT)/slow;
	float temp_acc = 0;
	int acc_set = 0;

	for(int s = 0; s < slow; s++){
	    temp_acc = s * acc;
	    acc_set = round(temp_acc);
	    if(acc_set >= (MAX_MOTOR_TIMEOUT - MOTOR_TIMEOUT - 5)){
	        acc_set = MAX_MOTOR_TIMEOUT - MOTOR_TIMEOUT;
	    }
	    pwm_impulse_gen(amis_pins, MAX_MOTOR_TIMEOUT - acc_set, base); // conversion from double to uint16_t so thats max what can be set
	}
	for(int n = 1; n<normal; n++){
		pwm_impulse_gen(amis_pins, MOTOR_TIMEOUT, base);
	}
    temp_acc = 0; acc_set = 0;
	for(int k = 0; k<slow; k++){
        temp_acc = k*acc;
        acc_set = round(temp_acc);
        if((acc_set) >(MAX_MOTOR_TIMEOUT - MOTOR_TIMEOUT-5)){
            acc_set = MAX_MOTOR_TIMEOUT - MOTOR_TIMEOUT;
        }
        pwm_impulse_gen(amis_pins, MOTOR_TIMEOUT + acc_set, base);
	}

}
// Status functions
/*
 * @brief returns 0 or 1. Checks in number of ones in the last 7 bits is odd or even.
 * @param status unsigned char aka uint8_t
 * @ret returns 0 if numbers are the same, 1 if numbers are different
 */

uint8_t Amis_status_parity_check(uint8_t status){
	// 0001 1101  jesli parzysta to 0, jesli nieparzysta to 1
	// 1010 1010
	uint8_t c = 0;
	for(uint8_t n=0; n<7; n++){
		if(status & ((uint8_t)0x01 << n)){
			c++;
		}
	}
    if(c%2 == (status & (uint8_t)0x80) >> 7) return 0;
    else return 1;
}

uint8_t Amis_status_get(amis_base_st *base, uint8_t reg){  // returning AMIS ERROR can be misleading, in would be better to return -1
	uint8_t data_recv[1] = {0};
	if(base->read_reg((void*)base, (uint8_t)reg, data_recv, 1) != HAL_OK){
		return AMIS_ERROR;
	}
	uint8_t res = Amis_status_parity_check(data_recv[0]);
	if(res != 0){
		memset(data_recv, 0, 1);
		if(base->read_reg((void*)base, (uint8_t)reg, data_recv, 1) != HAL_OK){
			return AMIS_ERROR;
		}
		uint8_t res = Amis_status_parity_check(data_recv[0]);
		if(res!=0) return AMIS_ERROR;
	}
	return data_recv[0];
}

/*
 * @brief This returns value that describes the position of the motor. 0 is start, 511 is almost full step.
 *  It can be between 0 and 511. Position is returned regarding that one microstep i 1/128.
 *
 *
 */
AMIS_Status AMIS_GetCurrentPosition(amis_base_st *base, uint16_t *pos, motor_params_t *motor_param){
	uint8_t msph = Amis_status_get(base, SR3); // MSP 8:2
	uint8_t mspl = Amis_status_get(base, SR4); // MSP 6:0

	if(msph == AMIS_ERROR || mspl == AMIS_ERROR) return AMIS_ERROR;

	uint16_t temp = (msph << 2) | (mspl & 0x03);

	*pos = temp;

	return AMIS_OK;
}



AMIS_Status AMIS_Init(amis_base_st *base, amis_config_st *config){
	HAL_TIM_Base_Start(base->tim);

	AMIS_Status step_set = Amis_step_set(base, config);
	if(step_set != AMIS_OK) return AMIS_ERROR;

	AMIS_Status watchdog_set = Amis_watchdog_set(base, config);
	if(watchdog_set != AMIS_OK) return AMIS_ERROR;

	AMIS_Status current_set = Amis_current_set(base, config);
	if(current_set != AMIS_OK) return AMIS_ERROR;

	AMIS_Status start_set = Amis_start_set(base, config);
	if(start_set != AMIS_OK) return AMIS_ERROR;


	return AMIS_OK;
}
// Cant use custom delay function, because reset can be invoked before init function thus timer is not available yet.
// Im using HAL_Delay instead to not complicate things.
AMIS_Status Amis_Reset(amis_pins_st *amis_pins){
	GPIO_TypeDef *gpio_dir = (GPIO_TypeDef*)amis_pins->GPIO_DIR;
	GPIO_TypeDef *gpio_nxt = (GPIO_TypeDef*)amis_pins->GPIO_NXT;
	GPIO_TypeDef *gpio_clr = (GPIO_TypeDef*)amis_pins->GPIO_CLR;

	if((gpio_nxt == NULL) || (gpio_dir == NULL) || (gpio_clr == NULL)){
		return AMIS_ERROR;
	}
	gpio_dir->BSRR = amis_pins->PIN_DIR << 16u; // reset dir
	gpio_nxt->BSRR = amis_pins->PIN_NXT << 16u; // reset nxt

	// reset amis
	gpio_clr->BSRR = amis_pins->PIN_CLR;
	HAL_Delay(10);
	gpio_clr->BSRR = amis_pins->PIN_CLR << 16u;
	HAL_Delay(10);

	return AMIS_OK;
}

AMIS_Status Amis_ChangeDir(amis_pins_st *amis_pins){
	GPIO_TypeDef *gpio_dir = (GPIO_TypeDef*)amis_pins->GPIO_DIR;
	if(gpio_dir == NULL){
		return AMIS_ERROR;
	}
	HAL_GPIO_TogglePin(gpio_dir, amis_pins->PIN_DIR);
	return AMIS_OK;
}







