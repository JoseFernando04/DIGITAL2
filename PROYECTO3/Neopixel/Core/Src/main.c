/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "Neopixel.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UMBRAL_LDR 2000
#define STM32_SLAVE_ADDRESS 0x04  // Segunda Nucleo esclava (espacios 5-8)
#define ESP32_SLAVE_ADDRESS 0x28  // ESP32 para interfaz web
#define CMD_DISPLAY_UPDATE 0xA1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
DMA_HandleTypeDef hdma_tim1_ch1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
float brilloled = 50.0f;

// LDRs y estado LOCAL (esta Nucleo - espacios 1-4)
uint32_t ldrValues[4];
uint8_t espaciosLibresLocal = 0;

// Datos recibidos de la Nucleo ESCLAVA (espacios 5-8)
uint8_t slave_data[9];  // 1 byte libres + 4 uint16_t LDR
uint16_t ldr_values_slave[4];
uint8_t espaciosLibresRemoto = 0;

// Estado TOTAL del parqueo (8 espacios)
uint8_t espaciosLibresTotal = 0;
uint8_t estadoParqueos[8] = {0};  // 0=ocupado, 1=libre

// Buffer para enviar al ESP32
uint8_t esp32_data[9];  // 1 byte total + 8 bytes estado

char buffer[200];
unsigned long lastI2CRead = 0;

// Tabla display 7 segmentos
uint8_t digitosDisplay[10] = {
    0b00111111,  // 0
    0b00000110,  // 1
    0b01011011,  // 2
    0b01001111,  // 3
    0b01100110,  // 4
    0b01101101,  // 5
    0b01111101,  // 6
    0b00000111,  // 7
    0b01111111,  // 8
    0b01101111   // 9
};

bool slave_error = false;
bool esp32_error = false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */

void LeerLDRsLocal(void);
void ActualizarNeoPixelsLocal(void);
void MostrarDigito(uint8_t numero);
void LeerDatosEsclavo(void);
void ConsolidarDatos(void);
void EnviarDatosESP32(void);
int _write(int file, char *ptr, int len);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Función para redirigir printf al UART
int _write(int file, char *ptr, int len) {
    HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
}

// Función auxiliar para leer LDRs locales
void LeerLDRsLocal(void) {
    HAL_ADC_Start(&hadc1);

    for (int i = 0; i < 4; i++) {
        HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
        ldrValues[i] = HAL_ADC_GetValue(&hadc1);
    }

    HAL_ADC_Stop(&hadc1);

    // Calcular espacios libres locales
    espaciosLibresLocal = 0;
    for (int i = 0; i < 4; i++) {
        if (ldrValues[i] > UMBRAL_LDR) {
            espaciosLibresLocal++;
            estadoParqueos[i] = 1;  // Libre
        } else {
            estadoParqueos[i] = 0;  // Ocupado
        }
    }
}

// Función para actualizar NeoPixels locales
void ActualizarNeoPixelsLocal(void) {
    for (int i = 0; i < 4; i++) {
        if (ldrValues[i] > UMBRAL_LDR) {
            setPixelColor(i, 0, 255, 0);  // Verde = libre
        } else {
            setPixelColor(i, 255, 0, 0);  // Rojo = ocupado
        }
    }
    pixelShow();
}

// Función para mostrar número en display 7 segmentos
void MostrarDigito(uint8_t numero) {
    if (numero > 9) numero = 9;

    uint8_t segmentos = digitosDisplay[numero];

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, (segmentos & 0b00000001) ? GPIO_PIN_SET : GPIO_PIN_RESET); // a
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, (segmentos & 0b00000010) ? GPIO_PIN_SET : GPIO_PIN_RESET); // b
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, (segmentos & 0b00000100) ? GPIO_PIN_SET : GPIO_PIN_RESET); // c
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, (segmentos & 0b00001000) ? GPIO_PIN_SET : GPIO_PIN_RESET); // d
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, (segmentos & 0b00010000) ? GPIO_PIN_SET : GPIO_PIN_RESET); // e
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, (segmentos & 0b00100000) ? GPIO_PIN_SET : GPIO_PIN_RESET); // f
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, (segmentos & 0b01000000) ? GPIO_PIN_SET : GPIO_PIN_RESET); // g
}
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
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  printf("\r\n[I2C SCAN] Escaneando bus I2C...\r\n");
  for (uint8_t addr = 1; addr < 128; addr++) {
      if (HAL_I2C_IsDeviceReady(&hi2c2, (addr << 1), 1, 10) == HAL_OK) {
          printf("  ✓ Dispositivo encontrado en: 0x%02X\r\n", addr);
      }
  }
  printf("[I2C SCAN] Escaneo completo\r\n\r\n");

  // ⭐ VERIFICAR que el esclavo responde antes de continuar
  printf("\r\n[VERIFICACIÓN] Probando comunicación con esclavo 0x%02X...\r\n", STM32_SLAVE_ADDRESS);
  HAL_Delay(100);

  if (HAL_I2C_IsDeviceReady(&hi2c2, (STM32_SLAVE_ADDRESS << 1), 3, 100) == HAL_OK) {
      printf("  ✓ Nucleo esclava (0x%02X) responde correctamente\r\n\r\n", STM32_SLAVE_ADDRESS);
  } else {
      printf("  ✗ ADVERTENCIA: Nucleo esclava (0x%02X) NO responde\r\n", STM32_SLAVE_ADDRESS);
      printf("    Verifica:\r\n");
      printf("    - Esclavo programado y funcionando\r\n");
      printf("    - HAL_I2C_EnableListen_IT() llamado en el esclavo\r\n");
      printf("    - Conexiones I2C y GND\r\n");
      printf("    - Resistencias pull-up\r\n\r\n");
  }

  printf("\r\n========================================\r\n");
  printf("   Nucleo MAESTRO - Parqueo JJ\r\n");
  printf("   Control de espacios 1-4\r\n");
  printf("========================================\r\n\r\n");

  MostrarDigito(0);
  printf("Display 7 segmentos OK\r\n");

  setBrightness(50);
  pixelClear();
  pixelShow();
  printf("NeoPixels OK\r\n");

  printf("Umbral LDR: %d\r\n\r\n", UMBRAL_LDR);

  printf("I2C Maestro inicializado\r\n");
  printf("  Nucleo Esclava: 0x%02X (Espacios 5-8)\r\n", STM32_SLAVE_ADDRESS);
  printf("  ESP32 Display:  0x%02X\r\n\r\n", ESP32_SLAVE_ADDRESS);

  printf("Iniciando monitoreo de 8 espacios...\r\n");
  printf("========================================\r\n\r\n");

  memset(slave_data, 0, sizeof(slave_data));
  memset(esp32_data, 0, sizeof(esp32_data));

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // 1. Leer sensores LOCALES (espacios 1-4)
	      LeerLDRsLocal();

	      // 2. Actualizar NeoPixels LOCALES
	      ActualizarNeoPixelsLocal();

	      // 3. Leer datos de la Nucleo ESCLAVA cada 300ms
	      if (HAL_GetTick() - lastI2CRead >= 300) {
	          LeerDatosEsclavo();
	          HAL_Delay(10);

	          ConsolidarDatos();
	          HAL_Delay(10);

	          EnviarDatosESP32();
	          HAL_Delay(10);

	          EnviarDatosNucleoEsclava();

	          // Debug completo
	          printf("[MAESTRO] Total: %d/8 | Local(1-4): %d/4 | Remoto(5-8): %d/4\r\n",
	                 espaciosLibresTotal, espaciosLibresLocal, espaciosLibresRemoto);

	          printf("  LDR Local:  [%4lu, %4lu, %4lu, %4lu]\r\n",
	                 ldrValues[0], ldrValues[1], ldrValues[2], ldrValues[3]);

	          printf("  LDR Remoto: [%4u, %4u, %4u, %4u]\r\n",
	                 ldr_values_slave[0], ldr_values_slave[1],
	                 ldr_values_slave[2], ldr_values_slave[3]);

	          printf("  Estado: [%d,%d,%d,%d,%d,%d,%d,%d]\r\n\r\n",
	                 estadoParqueos[0], estadoParqueos[1], estadoParqueos[2], estadoParqueos[3],
	                 estadoParqueos[4], estadoParqueos[5], estadoParqueos[6], estadoParqueos[7]);

	          lastI2CRead = HAL_GetTick();
	      }

	      HAL_Delay(200);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 105-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, SEG_C_Pin|SEG_G_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SEG_A_Pin|SEG_B_Pin|SEG_D_Pin|SEG_E_Pin
                          |SEG_F_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SEG_C_Pin SEG_G_Pin */
  GPIO_InitStruct.Pin = SEG_C_Pin|SEG_G_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SEG_A_Pin SEG_B_Pin SEG_D_Pin SEG_E_Pin
                           SEG_F_Pin */
  GPIO_InitStruct.Pin = SEG_A_Pin|SEG_B_Pin|SEG_D_Pin|SEG_E_Pin
                          |SEG_F_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
 * Leer datos de la Nucleo ESCLAVA (espacios 5-8)
 * Formato esperado: 9 bytes
 * - Byte 0: espacios libres (0-4)
 * - Bytes 1-8: 4 valores uint16_t en little-endian (LDR values)
 */
/**
 * Leer datos de la Nucleo ESCLAVA (espacios 5-8)
 * Formato esperado: 9 bytes
 * - Byte 0: espacios libres (0-4)
 * - Bytes 1-8: 4 valores uint16_t en little-endian (LDR values)
 */
void LeerDatosEsclavo(void) {
    HAL_StatusTypeDef status;

    // ⭐ AGREGAR: Pequeño delay antes de leer
    HAL_Delay(5);

    // Solicitar 9 bytes del esclavo
    status = HAL_I2C_Master_Receive(&hi2c2, (STM32_SLAVE_ADDRESS << 1),
                                     slave_data, 9, 200);  // ⚠️ Aumentar timeout a 200ms

    if (status == HAL_OK) {
        // Parsear datos recibidos
        espaciosLibresRemoto = slave_data[0];

        // Extraer valores LDR (little-endian: LSB primero, MSB después)
        ldr_values_slave[0] = slave_data[1] | (slave_data[2] << 8);
        ldr_values_slave[1] = slave_data[3] | (slave_data[4] << 8);
        ldr_values_slave[2] = slave_data[5] | (slave_data[6] << 8);
        ldr_values_slave[3] = slave_data[7] | (slave_data[8] << 8);

        // ⭐ CORREGIR: Actualizar estado según convención del esclavo
        // El esclavo envía: parking_status donde 0=libre, 1=ocupado
        // Pero necesitamos: estadoParqueos donde 1=libre, 0=ocupado
        for (int i = 0; i < 4; i++) {
            if (ldr_values_slave[i] > UMBRAL_LDR) {
                estadoParqueos[4 + i] = 1;  // Libre (LDR alto = sin carro)
            } else {
                estadoParqueos[4 + i] = 0;  // Ocupado (LDR bajo = con carro)
            }
        }

        // ⭐ AGREGAR: Recalcular espaciosLibresRemoto basado en LDR
        espaciosLibresRemoto = 0;
        for (int i = 4; i < 8; i++) {
            if (estadoParqueos[i] == 1) espaciosLibresRemoto++;
        }

        if (slave_error) {
            printf("[OK] Comunicación con Nucleo esclava restaurada\r\n");
            slave_error = false;
        }

    } else {
        // Error en comunicación
        if (!slave_error) {
            printf("[ERROR] No se pudo leer Nucleo esclava (0x%02X) - Status: %d\r\n",
                   STM32_SLAVE_ADDRESS, status);

            // ⭐ AGREGAR: Diagnóstico adicional
            if (status == HAL_TIMEOUT) {
                printf("  -> Timeout: El esclavo no responde\r\n");
                printf("  -> Verifica que el esclavo tenga HAL_I2C_EnableListen_IT()\r\n");
            } else if (status == HAL_ERROR) {
                printf("  -> Error I2C: Code 0x%08lX\r\n", hi2c2.ErrorCode);
            }

            slave_error = true;
        }

        // Marcar espacios remotos como desconocidos (ocupados por seguridad)
        espaciosLibresRemoto = 0;
        for (int i = 4; i < 8; i++) {
            estadoParqueos[i] = 0;
        }
    }
}

/**
 * Consolidar datos de ambas Nucleos (local + remota)
 */
void ConsolidarDatos(void) {
    // Calcular total de espacios libres
    espaciosLibresTotal = espaciosLibresLocal + espaciosLibresRemoto;

    // Actualizar display local con el total
    MostrarDigito(espaciosLibresTotal);

    // Preparar buffer para enviar al ESP32
    // Byte 0: Total de espacios libres
    esp32_data[0] = espaciosLibresTotal;

    // Bytes 1-8: Estado individual de cada parqueo (0=ocupado, 1=libre)
    for (int i = 0; i < 8; i++) {
        esp32_data[i + 1] = estadoParqueos[i];
    }
}

/**
 * Enviar datos consolidados al ESP32 por I2C
 * Formato: 9 bytes (1 byte total + 8 bytes estado)
 */
void EnviarDatosESP32(void) {
    HAL_StatusTypeDef status;

    // Enviar 9 bytes al ESP32
    status = HAL_I2C_Master_Transmit(&hi2c2, (ESP32_SLAVE_ADDRESS << 1),
                                      esp32_data, 9, 100);

    if (status == HAL_OK) {
        esp32_error = false;
    } else {
        if (!esp32_error) {
            printf("[ERROR] No se pudo enviar datos al ESP32 (0x%02X)\r\n",
                   ESP32_SLAVE_ADDRESS);
            esp32_error = true;
        }
    }
}

void EnviarDatosNucleoEsclava(void)
{
    uint8_t packet[10];
    packet[0] = CMD_DISPLAY_UPDATE;  // 0xA1
    packet[1] = espaciosLibresTotal;

    for (int i = 0; i < 8; i++) {
        packet[2 + i] = estadoParqueos[i]; // 1=libre, 0=ocupado
    }

    // ⭐ AGREGAR: Delay antes de enviar
    HAL_Delay(5);

    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hi2c2,
                                                        (STM32_SLAVE_ADDRESS << 1),
                                                        packet,
                                                        sizeof(packet),
                                                        200);  // ⚠️ Timeout 200ms

    if (status != HAL_OK) {
        if (!slave_error) {
            printf("[ERROR] No se pudo enviar CMD_DISPLAY_UPDATE a Nucleo esclava (0x%02X)\r\n",
                   STM32_SLAVE_ADDRESS);
            printf("  -> Status: %d, ErrorCode: 0x%08lX\r\n", status, hi2c2.ErrorCode);
            slave_error = true;
        }
    } else {
        if (slave_error) {
            printf("[OK] Envío de comandos restaurado\r\n");
            slave_error = false;
        }
    }
}
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
