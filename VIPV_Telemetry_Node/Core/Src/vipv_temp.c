/*
 * vipv_temp.c
 *
 *  Created on: Mar 1, 2026
 *      Author: omarf
 */

#include "vipv_temp.h"
#include <stdio.h>
#include <string.h>


//DATOS CLAVE PARA EL SENSOR STTS22H:
  	  // Direccion I2C obtenida en Datasheet del STTS22H: 0x3F . Al desplazar a la izqda (<< 1) para adpatar a STM32 = 0x7E
  	  // Registro de temperatura según Datasheet del STTS22H: LSB = 0x06 (TEMP_L_OUT); MSB = 0x07 (TEMP_H_OUT)
  	  // Registro de control según Datasheet del STTS22H: 0x04. Valor para salir del modo sleep = 0x0C


    // Lectura continua STTS22H
    //HAL_I2C_Mem_Read_IT(&hi2c1, 0x7E, 0x06, I2C_MEMADD_SIZE_8BIT, temp_buffer, 2);
    //argumentos: hi2c, DevAddress, MemAddress, MemAddSize, pData, Size


void VIPV_Temp_Init(I2C_HandleTypeDef *hi2c) {

	// Setup inicial STTS22H
	uint8_t registro_power_temp = 0x0C;
    HAL_I2C_Mem_Write(hi2c, 0x7E, 0x04, I2C_MEMADD_SIZE_8BIT, &registro_power_temp, 1, 100); //timeout = 100
}




float VIPV_Temp_Process(uint8_t *temp_buffer, UART_HandleTypeDef *huart) {


    // OBTENCIÓN DEL VALOR DE LA TEMPERATURA DEL SENSOR
    int16_t temp_raw = (int16_t)((temp_buffer[1] << 8) | temp_buffer[0]);
    float temp_real = (float)temp_raw / 100.0f; // Conversión según datasheet (resolución de 0.01ºC)


    //------------------------- LECTURA DEL VALOR EN EL PUERTO SERIE (PUTTY)------------------------------------

    int temp_entera = (int)temp_real;
    int temp_decimal = (int)((temp_real - temp_entera) * 100);

    if (temp_decimal < 0) { temp_decimal = -temp_decimal; } // Por si hay negativos

    char mensaje[50];

    sprintf(mensaje, "Temperatura VIPV: %d.%02d C\r\n", temp_entera, temp_decimal);

    HAL_UART_Transmit(huart, (uint8_t*)mensaje, strlen(mensaje), 100);


    return temp_real;
}
