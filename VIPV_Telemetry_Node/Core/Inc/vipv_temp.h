/*
 * vipv_temp.h
 *
 *  Created on: Mar 1, 2026
 *      Author: omarf
 */

#ifndef INC_VIPV_TEMP_H_
#define INC_VIPV_TEMP_H_


#include "main.h"


//prototipos funciones
void VIPV_Temp_Init(I2C_HandleTypeDef *hi2c);
float VIPV_Temp_Process(uint8_t *temp_buffer, UART_HandleTypeDef *huart);


#endif /* INC_VIPV_TEMP_H_ */
