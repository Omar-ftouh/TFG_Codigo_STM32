/*
 * vipv_accel.h
 *
 *  Created on: Mar 1, 2026
 *      Author: omarf
 */

#ifndef INC_VIPV_ACCEL_H_
#define INC_VIPV_ACCEL_H_

#include "main.h"


//prototipos funciones
void VIPV_Accel_Init(I2C_HandleTypeDef *hi2c, UART_HandleTypeDef *huart);
void VIPV_Accel_Process(uint8_t *inercia_buffer, UART_HandleTypeDef *huart, float *out_x, float *out_y, float *out_z);


#endif /* INC_VIPV_ACCEL_H_ */
