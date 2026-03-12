/*
 * vipv_can.h
 *
 *  Created on: Mar 1, 2026
 *      Author: omarf
 */

#ifndef INC_VIPV_CAN_H_
#define INC_VIPV_CAN_H_


#include "main.h"


//prototipos funciones
void VIPV_CAN_Init(FDCAN_HandleTypeDef *hfdcan, UART_HandleTypeDef *huart);
//void VIPV_CAN_SendTest(FDCAN_HandleTypeDef *hfdcan, UART_HandleTypeDef *huart);
void VIPV_CAN_Send_Entorno(FDCAN_HandleTypeDef *hfdcan, UART_HandleTypeDef *huart, float temperatura_leida);
void VIPV_CAN_Send_Dinamica(FDCAN_HandleTypeDef *hfdcan, UART_HandleTypeDef *huart, float ax, float ay, float az);
void VIPV_CAN_Send_Potencia(FDCAN_HandleTypeDef *hfdcan, UART_HandleTypeDef *huart, float voltaje, float corriente, float potencia);
void VIPV_CAN_Send_Irradiancia(FDCAN_HandleTypeDef *hfdcan, UART_HandleTypeDef *huart, float irradiancia);


#endif /* INC_VIPV_CAN_H_ */
