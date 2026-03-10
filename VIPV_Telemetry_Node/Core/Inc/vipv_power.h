/*
 * vipv_power.h
 *
 *  Created on: Mar 10, 2026
 *      Author: omarf
 */

#ifndef INC_VIPV_POWER_H_
#define INC_VIPV_POWER_H_

#include "main.h"

//prototipos funciones
void VIPV_Power_Init(I2C_HandleTypeDef *hi2c, UART_HandleTypeDef *huart);
void VIPV_Power_Process(uint8_t *power_buffer, UART_HandleTypeDef *huart, float *out_voltaje, float *out_corriente, float *out_potencia);

#endif /* INC_VIPV_POWER_H_ */
