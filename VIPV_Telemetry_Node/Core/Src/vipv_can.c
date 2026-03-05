/*
 * vipv_can.c
 *
 *  Created on: Mar 1, 2026
 *      Author: omarf
 */


#include "vipv_can.h"
#include <string.h>


static FDCAN_TxHeaderTypeDef TxHeader; // Oculto del main



void VIPV_CAN_Init(FDCAN_HandleTypeDef *hfdcan, UART_HandleTypeDef *huart) {

	//INICIAR LA COMUNICACIÓN CAN:

	 // Rellenar configuración del mensaje
	 TxHeader.Identifier = 0x123;                  // El ID del mensaje (arbitrario en este caso)
	 TxHeader.IdType = FDCAN_STANDARD_ID;          // ID de 11 bits (Estándar)
	 TxHeader.TxFrameType = FDCAN_DATA_FRAME;      // Marco de datos (no Remote Request)
	 TxHeader.DataLength = FDCAN_DLC_BYTES_8;      // Se van a mandar 8 bytes
	 TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	 TxHeader.BitRateSwitch = FDCAN_BRS_OFF;       // Apagado porque se usará CAN Clásico, no FD
	 TxHeader.FDFormat = FDCAN_CLASSIC_CAN;        // Formato CAN 2.0 Clásico
	 TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	 TxHeader.MessageMarker = 0;


	 // Arrancar periférico CAN
	 if (HAL_FDCAN_Start(hfdcan) != HAL_OK) {

         char err[] = "X -> Error al arrancar el CAN\r\n";
	     HAL_UART_Transmit(huart, (uint8_t*)err, strlen(err), 100);
	 }
	 else{

		 char ok[] = "-> EXITO: CAN arrancado y listo\r\n";
		 HAL_UART_Transmit(huart, (uint8_t*)ok, strlen(ok), 100);
	 }



}


/*
void VIPV_CAN_SendTest(FDCAN_HandleTypeDef *hfdcan, UART_HandleTypeDef *huart) {

	static uint8_t contador = 0; // static para que la variable no se borre cada segundo

    // Relleno de los 8 bytes con datos de prueba fácilmente reconocibles
    uint8_t TxData[8];
    TxData[0] = 0x56; // Letra 'V'
    TxData[1] = 0x49; // Letra 'I'
    TxData[2] = 0x50; // Letra 'P'
    TxData[3] = 0x56; // Letra 'V'
    TxData[4] = 0x00; // Espacio vacío
    TxData[5] = 0x00; // Espacio vacío
    TxData[6] = 0x00; // Espacio vacío
    TxData[7] = contador; // El byte 7 es el contador incremental


    if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, TxData) != HAL_OK) { //argumentos: &hfdcan1, &Cabecera, Puntero a datos

    	char err_tx[] = "X -> Error al intentar enviar el mensaje CAN\r\n";
    	HAL_UART_Transmit(huart, (uint8_t*)err_tx, strlen(err_tx), 100);
    }
    else {

    	char ok[50];
    	sprintf(ok, "-> Mensaje CAN [VIPV] enviado | Seq: %d\r\n", contador);
    	HAL_UART_Transmit(huart, (uint8_t*)ok, strlen(ok), 100);

    	contador++; // Sumamos 1 para el siguiente segundo
    }
}
*/



void VIPV_CAN_Send_Entorno(FDCAN_HandleTypeDef *hfdcan, UART_HandleTypeDef *huart, float temperatura_leida) {

	static uint8_t contador_entorno = 0;
    uint8_t TxData[8] = {0};

    int16_t temp_entero = (int16_t)(temperatura_leida * 100);

    TxData[0] = (temp_entero >> 8) & 0xFF;
    TxData[1] = temp_entero & 0xFF;
    // [2,3,4,5,6] libres para el futuro (Potencia, etc.)
    TxData[7] = contador_entorno;

    TxHeader.Identifier = 0x100; // ID PARA ENTORNO

    if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, TxData) == HAL_OK) {
        contador_entorno++;
    }
}



void VIPV_CAN_Send_Dinamica(FDCAN_HandleTypeDef *hfdcan, UART_HandleTypeDef *huart, float ax, float ay, float az) {

	static uint8_t contador_dinamica = 0;
    uint8_t TxData[8] = {0};

    int16_t x_entero = (int16_t)(ax * 100);
    int16_t y_entero = (int16_t)(ay * 100);
    int16_t z_entero = (int16_t)(az * 100);

    TxData[0] = (x_entero >> 8) & 0xFF;
    TxData[1] = x_entero & 0xFF;
    TxData[2] = (y_entero >> 8) & 0xFF;
    TxData[3] = y_entero & 0xFF;
    TxData[4] = (z_entero >> 8) & 0xFF;
    TxData[5] = z_entero & 0xFF;
    TxData[7] = contador_dinamica;

    TxHeader.Identifier = 0x101; // ID PARA DINÁMICA

    if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, TxData) == HAL_OK) {
        contador_dinamica++;
    }
}
