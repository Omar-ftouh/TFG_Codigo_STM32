/*
 * vipv_can.c
 *
 *  Created on: Mar 1, 2026
 *      Author: omarf
 */


#include "vipv_can.h"
#include <string.h>


static FDCAN_TxHeaderTypeDef TxHeader; // Cabecera global



void VIPV_CAN_Init(FDCAN_HandleTypeDef *hfdcan, UART_HandleTypeDef *huart) {

	//INICIAR LA COMUNICACIÓN CAN:

	// Configuración base de la cabecera (Compartida para todos los mensajes)
	TxHeader.Identifier = 0x000;                  // ID por defecto (Se cambiará antes de enviar)
	TxHeader.IdType = FDCAN_STANDARD_ID;          // ID de 11 bits (Estándar)
	TxHeader.TxFrameType = FDCAN_DATA_FRAME;      // Marco de datos
	TxHeader.DataLength = FDCAN_DLC_BYTES_8;      // 8 bytes de carga útil
	TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	TxHeader.BitRateSwitch = FDCAN_BRS_OFF;       // CAN Clásico (No FD)
	TxHeader.FDFormat = FDCAN_CLASSIC_CAN;        // Formato CAN 2.0 Clásico
	TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	TxHeader.MessageMarker = 0;



	// -------------------------------------------------------------------------
	// CONFIGURACIÓN DEL FILTRO DE RECEPCIÓN (Para el OBD)
	FDCAN_FilterTypeDef sFilterConfig;
	sFilterConfig.IdType = FDCAN_STANDARD_ID;
	sFilterConfig.FilterIndex = 0;
	sFilterConfig.FilterType = FDCAN_FILTER_MASK;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	sFilterConfig.FilterID1 = 0x7E8; // ID correspondiente a respuesta de peticiones de diagnóstico a la ECU del coche (ISO 15765-4)
	sFilterConfig.FilterID2 = 0x7FF; // Máscara de mensaje CAN (corresponde a 11 bits en hexadecimal, longitud exacta deseada)

	if (HAL_FDCAN_ConfigFilter(hfdcan, &sFilterConfig) != HAL_OK) {
		char err_filt[] = "X -> Error al configurar el Filtro CAN\r\n";
		HAL_UART_Transmit(huart, (uint8_t*)err_filt, strlen(err_filt), 100);
	}
	// -------------------------------------------------------------------------




	 // Arrancar periférico CAN
	 if (HAL_FDCAN_Start(hfdcan) != HAL_OK) {

         char err[] = "X -> Error al arrancar el CAN\r\n";
	     HAL_UART_Transmit(huart, (uint8_t*)err, strlen(err), 100);
	 }
	 else{

		 char ok[] = "-> EXITO: CAN arrancado y listo\r\n";
		 HAL_UART_Transmit(huart, (uint8_t*)ok, strlen(ok), 100);
	 }


	 // Activar interrupción para cuando llegue un mensaje por OBD (la velocidad)
	 HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
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


    // ESCALADO PARA EL ENVÍO
    int16_t temp_entero = (int16_t)(temperatura_leida * 100);


    // EMPAQUETADO
    TxData[0] = (temp_entero >> 8) & 0xFF;
    TxData[1] = temp_entero & 0xFF;
    // [2,3,4,5,6] libres para el futuro
    TxData[7] = contador_entorno;

    TxHeader.Identifier = 0x100; // ID PARA ENTORNO

    if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, TxData) == HAL_OK) {
        contador_entorno++;
    }
}



void VIPV_CAN_Send_Dinamica(FDCAN_HandleTypeDef *hfdcan, UART_HandleTypeDef *huart, float ax, float ay, float az) {

	static uint8_t contador_dinamica = 0;
    uint8_t TxData[8] = {0};


    // ESCALADO PARA EL ENVÍO
    int16_t x_entero = (int16_t)(ax * 100);
    int16_t y_entero = (int16_t)(ay * 100);
    int16_t z_entero = (int16_t)(az * 100);


    // EMPAQUETADO
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



void VIPV_CAN_Send_Potencia(FDCAN_HandleTypeDef *hfdcan, UART_HandleTypeDef *huart, float voltaje, float corriente, float potencia) {

    static uint8_t contador_potencia = 0;
    uint8_t TxData[8] = {0};


    // ESCALADO PARA EL ENVÍO (De float a int16)
    int16_t v_int = (int16_t)(voltaje * 100.0f);     // Ej: 3.31V -> 331
    int16_t i_int = (int16_t)(corriente * 1000.0f);  // Ej: 0.5A -> 500 mA
    int16_t p_int = (int16_t)(potencia * 1000.0f);   // Ej: 1.6W -> 1600 mW


    // EMPAQUETADO (Se divide cada int16 en 2 bytes: High y Low)
    TxData[0] = (uint8_t)(v_int >> 8);   // Voltaje HIGH
    TxData[1] = (uint8_t)(v_int & 0xFF); // Voltaje LOW

    TxData[2] = (uint8_t)(i_int >> 8);   // Corriente HIGH
    TxData[3] = (uint8_t)(i_int & 0xFF); // Corriente LOW

    TxData[4] = (uint8_t)(p_int >> 8);   // Potencia HIGH
    TxData[5] = (uint8_t)(p_int & 0xFF); // Potencia LOW

    TxData[6] = 0x00; // Byte libre
    TxData[7] = contador_potencia; // Secuencia


    TxHeader.Identifier = 0x102; // ID PARA POTENCIA


    if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, TxData) == HAL_OK) {
        contador_potencia++;
    }
}



void VIPV_CAN_Send_Irradiancia(FDCAN_HandleTypeDef *hfdcan, UART_HandleTypeDef *huart, float irradiancia) {

    static uint8_t contador_luz = 0;
    uint8_t TxData[8] = {0};


    // ESCALADO PARA EL ENVÍO (De float a int16, un decimal de precisión)
    int16_t irr_int = (int16_t)(irradiancia * 10.0f);


    // EMPAQUETADO
    TxData[0] = (uint8_t)(irr_int >> 8);
    TxData[1] = (uint8_t)(irr_int & 0xFF);
    TxData[7] = contador_luz;


    TxHeader.Identifier = 0x103; // ID PARA IRRADIANCIA


    if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, TxData) == HAL_OK) {
        contador_luz++;
    }
}


void VIPV_CAN_Pedir_Velocidad(FDCAN_HandleTypeDef *hfdcan) {

    // Los 8 bytes del estándar OBD-II para pedir la velocidad (PID 0x0D)
    uint8_t TxData_Req[8] = {0x02, 0x01, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x00};

    // Cambiamos temporalmente el ID de la cabecera al de peticiones de diagnóstico
    TxHeader.Identifier = 0x7DF;

    // Lanzar la pregunta al bus CAN (y por tanto, al coche)
    HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, TxData_Req);
}


void VIPV_CAN_Pedir_RPM(FDCAN_HandleTypeDef *hfdcan) {

    // Los 8 bytes del estándar OBD-II para pedir las RPM (PID 0x0C)
    uint8_t TxData_Req[8] = {0x02, 0x01, 0x0C, 0x00, 0x00, 0x00, 0x00, 0x00};

    // Cambiamos temporalmente el ID de la cabecera al de peticiones de diagnóstico
    TxHeader.Identifier = 0x7DF;

    // Lanzar la pregunta al bus CAN
    HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, TxData_Req);
}
