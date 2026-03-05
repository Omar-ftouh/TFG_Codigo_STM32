/*
 * vipv_accel.c
 *
 *  Created on: Mar 1, 2026
 *      Author: omarf
 */


#include "vipv_accel.h"
#include <stdio.h>
#include <string.h>
#include <math.h>



	//DATOS CLAVE PARA EL SENSOR ADXL345:
    	  // Direccion I2C obtenida en Datasheet del ADXL345: 0x1D. Al desplazar a la izqda (<< 1) para adpatar a STM32 = 0x3A
    	  // Registro de los datos de posición según Datasheet del ADXL345: Registros del 0x32 al 0x37 (6 bytes)
    	  // Registro de control según Datasheet del ADXL345: 0x2D. Valor para salir del modo sleep = 0x08


      // Lectura continua ADXL345
      //HAL_I2C_Mem_Read_IT(&hi2c1, 0x3A, 0x32, I2C_MEMADD_SIZE_8BIT, inercia_buffer, 6);
      //argumentos: hi2c, DevAddress, MemAddress, MemAddSize, pData, Size


void VIPV_Accel_Init(I2C_HandleTypeDef *hi2c, UART_HandleTypeDef *huart) {

    uint8_t registro_power_inercia = 0x08;

    // Setup inicial ADXL345: HAL_I2C_Mem_Write(&hi2c, 0x3A, 0x2D, I2C_MEMADD_SIZE_8BIT, &0x08, 1, 100); //timeout = 100

    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(hi2c, 0x3A, 0x2D, I2C_MEMADD_SIZE_8BIT, &registro_power_inercia, 1, 100);

    if (status == HAL_OK) {
        char msg[] = "-> EXITO: ADXL345 en funcionamiento.\r\n";
        HAL_UART_Transmit(huart, (uint8_t*)msg, strlen(msg), 100);
    }
    else {
        char msg[] = "-> ERROR: ADXL345 no responde.\r\n";
        HAL_UART_Transmit(huart, (uint8_t*)msg, strlen(msg), 100);
    }
}


void VIPV_Accel_Process(uint8_t *inercia_buffer, UART_HandleTypeDef *huart, float *out_x, float *out_y, float *out_z) {


	// OBTENCIÓN DEL VALOR DE LA ACELERACIÓN DEL SENSOR
	int16_t x_raw = (int16_t)((inercia_buffer[1] << 8) | inercia_buffer[0]);
	int16_t y_raw = (int16_t)((inercia_buffer[3] << 8) | inercia_buffer[2]);
	int16_t z_raw = (int16_t)((inercia_buffer[5] << 8) | inercia_buffer[4]);

	// Por defecto, el ADXL345 tiene una resolución de 3.9 mg/LSB
	float x_g = x_raw * 0.0039f;
	float y_g = y_raw * 0.0039f;
	float z_g = z_raw * 0.0039f;

	// Devolver los valores de los 3 ejes de vuelta al main
	*out_x = x_g;
	*out_y = y_g;
	*out_z = z_g;


	// Calcular variables estáticas para guardar la lectura anterior de cada eje
    static float x_ant = 0.0f, y_ant = 0.0f, z_ant = 0.0f;
    static int contador_quieto = 0;

    // Delta entre magnitud actual y la anterior (hace 1 segundo por ej.) para cada eje
    float delta_x = fabsf(x_g - x_ant);
    float delta_y = fabsf(y_g - y_ant);
    float delta_z = fabsf(z_g - z_ant);

    // guardar valores actuales para el siguiente ciclo
    x_ant = x_g; y_ant = y_g; z_ant = z_g;




    // Establecer umbral de movimiento (0.05G por ej.)
    if (delta_x < 0.05f && delta_y < 0.05f && delta_z < 0.05f) {
        contador_quieto++;
    }
    else {
        contador_quieto = 0;
    }



    //------------------------- LECTURA DEL VALOR EN EL PUERTO SERIE (PUTTY)------------------------------------
    char msg[150];

    if (contador_quieto >= 5) {   //SE ESTABLECEN 5 SEG. EN PARADO PARA ENTRAR EN MODO DE BAJO CONSUMO

        sprintf(msg, "Estado: PARADO | Deltas [X:%d Y:%d Z:%d] mg\r\n", (int)(delta_x*1000), (int)(delta_y*1000), (int)(delta_z*1000));
    }
    else {

        sprintf(msg, "Estado: MOVIMIENTO | Deltas [X:%d Y:%d Z:%d] mg\r\n", (int)(delta_x*1000), (int)(delta_y*1000), (int)(delta_z*1000));
    }

    HAL_UART_Transmit(huart, (uint8_t*)msg, strlen(msg), 100);
    //HAL_UART_Transmit(&hlpuart1, (uint8_t*)msg, strlen(msg), 100);

}

