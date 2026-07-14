/*
 * vipv_power.c
 *
 *  Created on: Mar 10, 2026
 *      Author: omarf
 */

#include "vipv_power.h"
#include <stdio.h>
#include <string.h>



//DATOS CLAVE PARA EL SENSOR PAC1934:
  	  // Direccion I2C obtenida en Datasheet del PAC1934: 0x10 . Al desplazar a la izqda (<< 1) para adpatar a STM32 = 0x20
	  // Registro de identidad según Datasheet del PAC1934: 0xFD siempre devuelve el valor 0x5B



void VIPV_Power_Init(I2C_HandleTypeDef *hi2c, UART_HandleTypeDef *huart) {

	uint8_t product_id = 0;

    // Lectura continua PAC1934
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(hi2c, 0x20, 0xFD, I2C_MEMADD_SIZE_8BIT, &product_id, 1, 100);
    //argumentos: hi2c, DevAddress, MemAddress, MemAddSize, productID, pData, Size


    char msg[100];

    if (status == HAL_OK && product_id == 0x5B) {
        sprintf(msg, "-> EXITO: PAC1934 detectado (ID: 0x%02X).\r\n", product_id);
    }
    else {
        sprintf(msg, "-> ERROR: PAC1934 no responde o ID incorrecto (Leido: 0x%02X).\r\n", product_id);
    }

    HAL_UART_Transmit(huart, (uint8_t*)msg, strlen(msg), 100);
}



void VIPV_Power_Process(uint8_t *power_buffer, UART_HandleTypeDef *huart, float *out_voltaje, float *out_corriente, float *out_potencia) {

    // El buffer tiene 10 bytes porque se lee en bloque desde 0x07 (VBUS1) hasta 0x0B (VSENSE1)
    // Bytes 0 y 1: VBUS del Canal 1 (tensión real del panel solar)
    // Bytes 8 y 9: VSENSE del Canal 1 (caída de tensión al atravesar la resistencia shunt del sensor)


    uint16_t vbus_raw = (uint16_t)((power_buffer[0] << 8) | power_buffer[1]);
    uint16_t vsense_raw = (uint16_t)((power_buffer[8] << 8) | power_buffer[9]);


    // CONVERSIONES (según Datasheet)

    // 1. Voltaje (Para el pin VBUS, el Voltaje Máximo (FSR) = 32V)
    float voltaje_bus = 32.0f * ((float)vbus_raw / 65535.0f); //16 bits

    // 2. Corriente (Para el pin VSENSE, el Voltaje Máximo (FSR) = 0.1V)
    float r_sense = 0.1f; // Resistencia Shunt de la placa Click (0.1 Ohm)
    float voltaje_sense = 0.1f * ((float)vsense_raw / 65535.0f); //16 bits
    float corriente = voltaje_sense / r_sense; // Ley Ohm

    // 3. Potencia
    float potencia = voltaje_bus * corriente;


    // Devolver valores al main
    *out_voltaje = voltaje_bus;
    *out_corriente = corriente;
    *out_potencia = potencia;


    // Verfificación opcional por el puerto serie (PuTTY)
    char msg[100];
    sprintf(msg, "Estado: PAC1934 | V: %.2f V | I: %.3f A | P: %.2f W\r\n", voltaje_bus, corriente, potencia);
    HAL_UART_Transmit(huart, (uint8_t*)msg, strlen(msg), 100);
}
