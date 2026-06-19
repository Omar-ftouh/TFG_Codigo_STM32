/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "vipv_accel.h"
#include "vipv_temp.h"
#include "vipv_can.h"
//#include "vipv_power.h"
#include "vipv_mppt.h"
#include <stdio.h>
#include <string.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

FDCAN_HandleTypeDef hfdcan1;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef hlpuart1;

/* USER CODE BEGIN PV */

//_________________________________________________________________________________________________________________________

volatile uint8_t flag_temp_ready = 0;    // Flag temperatura
volatile uint8_t flag_inercia_ready = 0; // Flag acelerómetro
volatile uint8_t flag_power_ready = 0;   // Flag sensor potencia
volatile uint8_t flag_adc_ready = 0;     // Flag ADC

uint8_t temp_buffer[2];                  // Buffer para almacenamiento de datos I2C del sensor temperatura
uint8_t inercia_buffer[6]; 				 // Buffer para almacenamiento datos I2C del acelerómetro (6 bytes (X, Y, Z))
uint8_t power_buffer[10];                // Buffer para almacenamiento de datos I2C del sensor potencia

float temperatura_real = 0.0;
volatile uint32_t adc_valor_bruto = 0;   // Para almacenar el dato de la interrupción ADC
uint32_t tiempo_anterior = 0;


//MÁQ. ESTADOS PARA MANEJO DEL CALLBACK DE LAS INTERRUPCIONES
typedef enum {
    BUS_LIBRE,
    LEYENDO_TEMP,
    LEYENDO_INERCIA,
	LEYENDO_POTENCIA,
	LEYENDO_IRRADIANCIA
} Estado_I2C;


volatile Estado_I2C sensor_actual = BUS_LIBRE; //variable que guarda qué sensor está usando el bus I2C


FDCAN_TxHeaderTypeDef TxHeader; // Estructura de la cabecera del mensaje FDCAN
uint8_t TxData[8]; // Array de 8 bytes para guardar los datos a transmitir

volatile uint8_t velocidad_coche = 0; // Para guardar la velocidad real capturada por OBD (en km/h)


//----------------------- ALGORITMO MPPT -----------------------------

// --- ARRAYS DE LAS CURVAS I-V --- (Promedio de todas las curvas muestreadas en excel)
const float v_vector[50] = {0.0f, 0.548f, 1.097f, 1.645f, 2.194f, 2.742f, 3.291f, 3.839f, 4.388f, 4.936f, 5.485f, 6.033f, 6.582f,
		7.13f, 7.679f, 8.228f, 8.776f, 9.324f, 9.873f, 10.422f, 10.97f, 11.519f, 12.067f, 12.615f, 13.164f, 13.712f, 14.261f,
		14.809f, 15.358f, 15.907f, 16.455f, 17.004f, 17.552f, 18.101f, 18.649f, 19.198f, 19.746f, 20.295f, 20.844f, 21.392f,
		21.941f, 22.489f, 23.037f, 23.587f, 24.134f, 24.683f, 25.231f, 25.78f, 26.329f, 26.877f };

const float i_sol[50] = {0.96f, 0.95f, 0.94f, 0.93f, 0.92f, 0.91f, 0.9f, 0.89f, 0.88f, 0.87f, 0.86f, 0.85f, 0.84f, 0.83f,
		0.83f, 0.82f, 0.81f, 0.8f, 0.8f, 0.79f, 0.78f, 0.77f, 0.76f, 0.76f, 0.75f, 0.74f, 0.73f, 0.72f, 0.72f, 0.71f, 0.7f,
		0.69f, 0.68f, 0.68f, 0.67f, 0.66f, 0.65f, 0.64f, 0.63f, 0.63f, 0.61f, 0.6f, 0.58f, 0.56f, 0.54f, 0.51f, 0.45f, 0.34f,
		0.15f, -0.14f };

const float i_sombra[50] = {0.87f, 0.85f, 0.83f, 0.82f, 0.8f, 0.79f, 0.77f, 0.75f, 0.74f, 0.73f, 0.71f, 0.69f, 0.68f, 0.67f,
		0.66f, 0.64f, 0.63f, 0.62f, 0.61f, 0.6f, 0.59f, 0.58f, 0.57f, 0.56f, 0.55f, 0.54f, 0.53f, 0.52f, 0.51f, 0.5f, 0.49f,
		0.47f, 0.46f, 0.45f, 0.43f, 0.42f, 0.4f, 0.39f, 0.37f, 0.36f, 0.34f, 0.32f, 0.3f, 0.28f, 0.26f, 0.23f, 0.19f, 0.13f,
		0.02f, -0.24f };


//Arrays de potencia precalculada
float p_sol[50];
float p_sombra[50];

//Umbrales de irradiancia calibrados
const float IRR_MAX = 1000.0f;
const float IRR_MIN = 10.0f;

// --- VARIABLES DE ESTADO MPPT ---
float mppt_v_ant = 18.0f;
float mppt_p_ant = 0.0f;
float mppt_v_act = 19.0f;
float mppt_paso = 0.1f; // Paso del algoritmo


//______________________________________________________________________________________________mppt_v_act___________________________

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_I2C1_Init(void);
static void MX_LPUART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_FDCAN1_Init();
  MX_I2C1_Init();
  MX_LPUART1_UART_Init();

  /* USER CODE BEGIN 2 */

  //_________________________________________________________________________________________________________________________

  VIPV_Temp_Init(&hi2c1);
  VIPV_Accel_Init(&hi2c1, &hlpuart1);
 // VIPV_Power_Init(&hi2c1, &hlpuart1);
  VIPV_CAN_Init(&hfdcan1, &hlpuart1);

  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);

  // Inicialización del MPPT (asumiendo que arranca al sol)
  //mppt_p_ant = obtener_potencia_panel(mppt_v_ant, v_vector, i_sol, 50);


   // --- PRECALCULAR ARRAYS DE POTENCIA ---
   for(int i = 0; i < 50; i++) {
       p_sol[i] = v_vector[i] * i_sol[i];
       p_sombra[i] = v_vector[i] * i_sombra[i];
   }

  //_________________________________________________________________________________________________________________________


  // --- PUTTY: ESCÁNER I2C PARA DETECCIÓN DE ERRORES ---

  char msg[64];
  sprintf(msg, "\r\n=== INICIANDO DIAGNÓSTICO I2C ===\r\n");
  HAL_UART_Transmit(&hlpuart1, (uint8_t*)msg, strlen(msg), 100);

  int dispositivos_encontrados = 0;

  for(uint8_t i = 1; i < 128; i++) {
      // Mandar ping a la dirección 'i' desplazada 1 bit a la izquierda
      HAL_StatusTypeDef result = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(i<<1), 2, 10);

      if (result == HAL_OK) {
          sprintf(msg, "-> EXITO: Sensor detectado en dir: 0x%02X\r\n", i);
          HAL_UART_Transmit(&hlpuart1, (uint8_t*)msg, strlen(msg), 100);
          dispositivos_encontrados++;
      }
  }

  if(dispositivos_encontrados == 0){
      sprintf(msg, "-> ERROR CRÍTICO: El bus I2C NO RESPONDE\r\n");
      HAL_UART_Transmit(&hlpuart1, (uint8_t*)msg, strlen(msg), 100);
  }
  sprintf(msg, "=== FIN DEL DIAGNÓSTICO ===\r\n\r\n");
  HAL_UART_Transmit(&hlpuart1, (uint8_t*)msg, strlen(msg), 100);



  /* USER CODE END 2 */


  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

   //_________________________________________________________________________________________________________________________

	  //------------------------- LECTURA DE LOS SENSORES (CADA 1 SEGUNDO) ------------------------------------------------

	  if (HAL_GetTick() - tiempo_anterior >= 1000) {

	            tiempo_anterior = HAL_GetTick(); //resetear


	       // ARRANQUE DE SENSORES Y PETICIONES POR OBD (Nota: el acelerómetro se inicializa en vipv_accel.c)

	            // Disparo sensor potencia
	            //uint8_t cmd_refresh = 0x1F; // Comando REFRESH_V
	            //HAL_I2C_Master_Transmit(&hi2c1, 0x20, &cmd_refresh, 1, 10);



	            // Petición de Velocidad y de RPMs al coche por CAN (puerto OBD-II)
	            VIPV_CAN_Pedir_Velocidad(&hfdcan1); // (Petición de diagnóstico: 0x7DF)
	            VIPV_CAN_Pedir_RPM(&hfdcan1); // (Petición de diagnóstico: 0x7DF)



	            // Disparo inicial sensor temperatura
	            if(sensor_actual == BUS_LIBRE){
	                sensor_actual = LEYENDO_TEMP; //establecer estado de lectura de temperatura

	                HAL_I2C_Mem_Read_IT(&hi2c1, 0x7E, 0x06, I2C_MEMADD_SIZE_8BIT, temp_buffer, 2); // Lectura continua STTS22H
	      	  	  	//argumentos: hi2c, DevAddress, MemAddress, MemAddSize, pData, Size
	            }

	   }


	  //----------------------------------------- GESTIÓN DE RESPUESTAS ------------------------------------------------

	  if (flag_temp_ready == 1) {

	        flag_temp_ready = 0; // reseteo flag


	        // Extracción del float real cada vez que llega un valor de temperatura
	        float temperatura_actual = VIPV_Temp_Process(temp_buffer, &hlpuart1);

	        // Inyección del valor de temperatura en la red CAN con el ID 0x100
	        VIPV_CAN_Send_Entorno(&hfdcan1, &hlpuart1, temperatura_actual);



	        // Una vez el BUS QUEDA LIBRE, pasar el relevo al acelerómetro
	        sensor_actual = LEYENDO_INERCIA; //establecer estado de lectura del acelerómetro

	        HAL_I2C_Mem_Read_IT(&hi2c1, 0x3A, 0x32, I2C_MEMADD_SIZE_8BIT, inercia_buffer, 6); // Lectura continua ADXL345
	        //argumentos: hi2c, DevAddress, MemAddress, MemAddSize, pData, Size
	  }



	  if (flag_inercia_ready == 1) {

	        flag_inercia_ready = 0; // reseteo flag


	        // Variables para guardar lo que devuelva el sensor
            float eje_x, eje_y, eje_z;

            // Procesar acelerómetro pasando las direcciones de memoria
	        VIPV_Accel_Process(inercia_buffer, &hlpuart1, &eje_x, &eje_y, &eje_z);

            // Inyectar en el CAN con el ID 0x101
            VIPV_CAN_Send_Dinamica(&hfdcan1, &hlpuart1, eje_x, eje_y, eje_z);


            /*
            // Una vez el BUS QUEDA LIBRE, pasar el relevo al sensor de potencia
            sensor_actual = LEYENDO_POTENCIA; //establecer estado de lectura del sensor potencia

            // Leer 10 bytes seguidos empezando en la dirección 0x07 (VBUS1)
            HAL_I2C_Mem_Read_IT(&hi2c1, 0x20, 0x07, I2C_MEMADD_SIZE_8BIT, power_buffer, 10);
            //argumentos: hi2c, DevAddress, MemAddress, MemAddSize, pData, Size
            */

            // Una vez el BUS QUEDA LIBRE, pasar el relevo al ADC para leer irradiancia
            sensor_actual = LEYENDO_IRRADIANCIA; //establecer estado de lectura de irradiancia

            HAL_ADC_Start_IT(&hadc1);
	  }


	  /*
	  if (flag_power_ready == 1) {

	  	    flag_power_ready = 0; // reseteo flag


	  	    // Variables para guardar lo que devuelva el sensor
	        float v_bus, i_sense, p_calc;

	        // Procesar sensor potencia pasando las direcciones de memoria
	  	    VIPV_Power_Process(power_buffer, &hlpuart1, &v_bus, &i_sense, &p_calc);

	  	    // Inyectar en el CAN con el ID 0x102
	  	    VIPV_CAN_Send_Potencia(&hfdcan1, &hlpuart1, v_bus, i_sense, p_calc);


            // Una vez el BUS QUEDA LIBRE, pasar el relevo al ADC para leer irradiancia
            sensor_actual = LEYENDO_IRRADIANCIA; //establecer estado de lectura de irradiancia

            HAL_ADC_Start_IT(&hadc1);
	  }
 	 */

	  if (flag_adc_ready == 1) {

		    flag_adc_ready = 0; // reseteo flag


		    // 1. LECTURA DE LA IRRADIANCIA

		    // Convertir a Voltios reales (El ADC de la placa tiene una resolución de 12 bits --> Arroja valores de 0 a 4095)
		    float voltaje_adc = ((float)adc_valor_bruto / 4095.0f) * 3.3f; //Un valor del ADC de 4095 equivale a 3.3V

		    //Constante de Calibración (OBTENIDA EMPÍRICAMENTE EN LABORATORIO PARA UNA RESISTENCIA DE 10.7 ohm)
		    float constante_calibracion = 934.58f;
		    float irradiancia = voltaje_adc * constante_calibracion;

		    // Enviar por CAN los datos recibidos
		    VIPV_CAN_Send_Irradiancia(&hfdcan1, &hlpuart1, irradiancia);



		    // 2. EJECUCIÓN DEL ALGORITMO MPPT ADAPTATIVO
		    /*
		    const float *i_vector_actual;

		    if (irradiancia > 150.0f) { // Se elige 150W/m^2 como el umbral de Sol/sombra
		        i_vector_actual = i_sol;
		    }
		    else {
		        i_vector_actual = i_sombra;
		    }
		    */

		    /*
		    float *p_array_actual; // Puntero al array de potencia que toque

		    if (irradiancia > 150.0f) { // Se elige 150W/m^2 como el umbral de Sol/sombra
		        p_array_actual = p_sol;
		    } else {
		        p_array_actual = p_sombra;
		    }
		    */

		    // Potencia base para este milisegundo
		    float mppt_p_act = obtener_potencia_dinamica(mppt_v_act, irradiancia);

		    // FILTRO ADAPTATIVO
		    float mppt_v_sig = ejecutar_mppt_adaptativo(mppt_v_act, irradiancia, mppt_v_ant, mppt_p_ant, mppt_paso, mppt_p_act);


		    if (mppt_v_sig == 19.5f) { // ESTADO: CONGELADO
		    // Clavamos las variables en el punto de seguridad para no perder el norte.
		    	mppt_v_act = 19.5f;
		        mppt_v_ant = 19.5f;
		        mppt_p_ant = obtener_potencia_dinamica(19.5f, irradiancia);
		    }
		    else {  // ESTADO: FUNCIONAMIENTO NORMAL


			    for (int j = 0; j < 10; j++) { // Aumento de la velocidad de seguimiento del MPPT en 10 pasos

			    	//float mppt_p_act = obtener_potencia_directa(mppt_v_act, v_vector, p_array_actual, 50);
			    	//float mppt_v_sig = mppt_po(mppt_v_act, mppt_p_act, mppt_v_ant, mppt_p_ant, mppt_paso);

			    	float mppt_p_act = obtener_potencia_dinamica(mppt_v_act, irradiancia);
			    	float mppt_v_sig = mppt_po(mppt_v_act, mppt_p_act, mppt_v_ant, mppt_p_ant, mppt_paso);

			        // Actualizar datos para la siguiente iteración del for
			        mppt_v_ant = mppt_v_act;
			        mppt_p_ant = mppt_p_act;
			        mppt_v_act = mppt_v_sig;
			    }
		    }


		    // Empaquetar el resultado en CAN. Se envían: Voltaje actual, 0.0 en Corriente y Potencia resultante
		    VIPV_CAN_Send_Potencia(&hfdcan1, &hlpuart1, mppt_v_act, 0.0f, mppt_p_ant);

		    // Apagar ADC hasta próxima lectura
		    HAL_ADC_Stop_IT(&hadc1);
	  }


   //_________________________________________________________________________________________________________________________

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 10;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 14;
  hfdcan1.Init.NominalTimeSeg2 = 2;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.StdFiltersNbr = 0;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x20510E2D;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


//_________________________________________________________________________________________________________________________

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c){

    if (hi2c->Instance == I2C1) {

    	if (sensor_actual == LEYENDO_TEMP) {  // El I2C ha terminado de descargar los datos en temp_buffer

    	    flag_temp_ready = 1; // Avisar al main
    	}
    	else if (sensor_actual == LEYENDO_INERCIA) {  // El I2C ha terminado de descargar los datos en inercia_buffer

    	    flag_inercia_ready = 1; // Avisar al main
    	}
    	/*
    	else if (sensor_actual == LEYENDO_POTENCIA) { // El I2C ha terminado de descargar los datos en power_buffer

    	    flag_power_ready = 1; // Avisar al main
    	}
    	*/

        // Liberar bus al terminar
        sensor_actual = BUS_LIBRE;
    }
}


//Callback para lectura de la irradiancia
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {

    if (hadc->Instance == ADC1) {

        if (sensor_actual == LEYENDO_IRRADIANCIA) {

            adc_valor_bruto = HAL_ADC_GetValue(hadc);
            flag_adc_ready = 1;

            sensor_actual = BUS_LIBRE; // Liberar bus al terminar
        }

    }
}


// Callback para recepción de mensajes CAN (Respuestas del coche por OBD)
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
    FDCAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];

    // Leer el mensaje entrante
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {

        // Comprobar que es la ECU del motor (0x7E8) y que responde al PID de Velocidad (0x0D)
        if (RxHeader.Identifier == 0x7E8 && RxData[2] == 0x0D) {

            // Extraer el byte 3, que contiene la velocidad directamente en km/h
            velocidad_coche = RxData[3];
        }
    }
}

//_________________________________________________________________________________________________________________________



/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
