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
#include <stdarg.h>
#include "Neopixel.h"
#include "ili9341.h"
#include "bitmaps.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LDR_THRESHOLD 2000  // Umbral para detectar presencia (ajustar según calibración)
#define NUM_PARKING_SPOTS 4
#define ADC_CHANNELS 4

// Colores Neopixel
#define COLOR_GREEN_R 0
#define COLOR_GREEN_G 255
#define COLOR_GREEN_B 0

#define COLOR_RED_R 255
#define COLOR_RED_G 0
#define COLOR_RED_B 0

#define NEOPIXEL_BRIGHTNESS 50.0f  // Brillo (0.0 - 100.0)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc2;

I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
DMA_HandleTypeDef hdma_tim1_ch1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

#define I2C_TX_SIZE      9   // byte0 = libres (0..4), bytes1..8 = 4 x uint16 LSB/MSB
#define I2C_RX_CMD_SIZE 10   // CMD (1) + total_libres (1) + 8 estados = 10

uint8_t i2c_tx_buf[I2C_TX_SIZE];
uint8_t i2c_rx_buf[I2C_RX_CMD_SIZE];

volatile bool i2c_tx_busy = false;
volatile bool i2c_rx_ready = false;
volatile uint8_t i2c_rx_len = 0;

/* Datos enviados por el maestro (cuando llegue CMD_DISPLAY_UPDATE) */
uint8_t master_states[8];
uint8_t master_total_libres = 0;

extern const uint8_t pantalla_inicio[];

extern unsigned char carro_blancoSprite[];
extern unsigned char carro_negroSprite[];

extern unsigned char carro_rojoSprite[];
extern unsigned char carro_frenteSprite[];

extern unsigned char display[];

// Variables para el sistema de parqueo
uint32_t adc_values[ADC_CHANNELS] = {0};  // Valores ADC de las 4 LDR
uint8_t parking_status[NUM_PARKING_SPOTS] = {0};  // 0 = libre, 1 = ocupado

// Canales ADC correspondientes a cada LDR
uint32_t adc_channels[ADC_CHANNELS] = {
    ADC_CHANNEL_12,  // PC2
    ADC_CHANNEL_13,  // PC3
    ADC_CHANNEL_14,  // PC4
    ADC_CHANNEL_15   // PC5
};

// Buffer para UART debugging
char uart_buffer[200];

float brilloled = 50.0f;  // Variable para brillo de Neopixels (0.0 - 100.0)

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC2_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */
void Read_LDR_Sensors(void);
void Update_Parking_Status(void);
void Update_Neopixels(void);
uint32_t Read_ADC_Channel(uint32_t channel);
void Debug_Print_Status(void);
void UART_Print(const char* format, ...);
void Update_LCD_Parking_Display(void);
void Update_LCD_Available_Counter(void);
void Update_LCD_Full_Display(uint8_t *states8, uint8_t total_libres);

void Update_LCD_Parking_Display(void);      // ← NUEVA
void Update_LCD_Available_Counter(void);     // ← NUEVA
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief  Función auxiliar para imprimir por UART con formato
  * @param  format: String con formato printf
  * @retval None
  */
void UART_Print(const char* format, ...)
{
    va_list args;
    va_start(args, format);
    vsnprintf(uart_buffer, sizeof(uart_buffer), format, args);
    va_end(args);
    HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, strlen(uart_buffer), 1000);
}

/**
  * @brief  Lee un canal específico del ADC
  * @param  channel: Canal ADC a leer
  * @retval Valor ADC leído
  */
uint32_t Read_ADC_Channel(uint32_t channel)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    sConfig.Channel = channel;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;

    if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_ADC_Start(&hadc2);
    HAL_ADC_PollForConversion(&hadc2, HAL_MAX_DELAY);
    uint32_t adc_value = HAL_ADC_GetValue(&hadc2);
    HAL_ADC_Stop(&hadc2);

    return adc_value;
}

/**
  * @brief  Lee todos los sensores LDR
  * @param  None
  * @retval None
  */
void Read_LDR_Sensors(void)
{
    for(int i = 0; i < ADC_CHANNELS; i++)
    {
        adc_values[i] = Read_ADC_Channel(adc_channels[i]);
    }
}

/**
  * @brief  Actualiza el estado de los espacios de parqueo basado en las LDR
  * @param  None
  * @retval None
  */
void Update_Parking_Status(void)
{
    for(int i = 0; i < NUM_PARKING_SPOTS; i++)
    {
        // Si el valor ADC es menor al umbral, hay un carro (LDR recibe menos luz)
        if(adc_values[i] < LDR_THRESHOLD)
        {
            parking_status[i] = 1;  // Ocupado
        }
        else
        {
            parking_status[i] = 0;  // Libre
        }
    }
}

/**
  * @brief  Actualiza los Neopixels según el estado de cada parqueo
  * @param  None
  * @retval None
  */
void Update_Neopixels(void)
{
    for(int i = 0; i < NUM_PARKING_SPOTS; i++)
    {
        if(parking_status[i] == 0)  // Libre
        {
            setPixelColor(i, COLOR_GREEN_R, COLOR_GREEN_G, COLOR_GREEN_B);
        }
        else  // Ocupado
        {
            setPixelColor(i, COLOR_RED_R, COLOR_RED_G, COLOR_RED_B);
        }
    }
    pixelShow();  // Actualizar los LEDs
}


/**
  * @brief  Actualiza la visualización de los parqueos en la pantalla LCD
  *         Espacios de la parte alta (controlados por esta Nucleo)
  * @param  None
  * @retval None
  */
void Update_LCD_Parking_Display(void)
{
    // Coordenadas X de cada espacio de parqueo (parte alta)
    uint16_t parking_x_positions[NUM_PARKING_SPOTS] = {58, 90, 122, 154};
    uint16_t parking_y = 165;  // Coordenada Y para la parte alta
    uint16_t parking_width = 29;
    uint16_t parking_height = 56;

    // Actualizar cada espacio de parqueo
    for(int i = 0; i < NUM_PARKING_SPOTS; i++)
    {
        if(parking_status[i] == 0)  // Libre - mostrar carretera (index 1)
        {
            // Alternar entre sprites para variedad visual
            if(i % 2 == 0)
            {
                LCD_Sprite(parking_x_positions[i], parking_y, parking_width, parking_height,
                          carro_blancoSprite, 2, 1, 0, 0);
            }
            else
            {
                LCD_Sprite(parking_x_positions[i], parking_y, parking_width, parking_height,
                          carro_negroSprite, 2, 1, 0, 0);
            }
        }
        else  // Ocupado - mostrar carro (index 0)
        {
            // Alternar entre sprites para variedad visual
            if(i % 2 == 0)
            {
                LCD_Sprite(parking_x_positions[i], parking_y, parking_width, parking_height,
                          carro_blancoSprite, 2, 0, 0, 0);
            }
            else
            {
                LCD_Sprite(parking_x_positions[i], parking_y, parking_width, parking_height,
                          carro_negroSprite, 2, 0, 0, 0);
            }
        }
    }
}

/**
  * @brief  Actualiza el display de 7 segmentos con el número de espacios disponibles
  * @param  None
  * @retval None
  */
void Update_LCD_Available_Counter(void)
{
    // Contar espacios disponibles (libres) de esta Nucleo
    int available_count = 0;
    for(int i = 0; i < NUM_PARKING_SPOTS; i++)
    {
        if(parking_status[i] == 0)  // Libre
        {
            available_count++;
        }
    }

    // Coordenadas del display de 7 segmentos
    uint16_t display_x = 258;
    uint16_t display_y = 83;
    uint16_t display_width = 44;
    uint16_t display_height = 75;

    // Actualizar display con el número de espacios disponibles
    // El sprite tiene 9 columnas (dígitos 0-8)
    LCD_Sprite(display_x, display_y, display_width, display_height,
              display, 9, available_count, 0, 0);
}




/**
  * @brief  Imprime el estado del sistema por UART para debugging
  * @param  None
  * @retval None
  */
void Debug_Print_Status(void)
{
    // Título
    UART_Print("\r\n===== ESTADO DEL PARQUEO =====\r\n");

    // Valores ADC de cada sensor
    UART_Print("Valores ADC (Umbral=%d):\r\n", LDR_THRESHOLD);

    for(int i = 0; i < ADC_CHANNELS; i++)
    {
        UART_Print("  LDR[%d] (PC%d): %4lu %s\r\n",
                i, i+2, adc_values[i],
                (adc_values[i] < LDR_THRESHOLD) ? "[OCUPADO]" : "[LIBRE]");
    }

    // Estado de cada espacio
    UART_Print("\nEstado de Parqueos:\r\n");

    for(int i = 0; i < NUM_PARKING_SPOTS; i++)
    {
        UART_Print("  Espacio %d: %s (LED: %s)\r\n",
                i+1,
                parking_status[i] ? "OCUPADO" : "LIBRE",
                parking_status[i] ? "ROJO" : "VERDE");
    }

    // Conteo total
    int ocupados = 0;
    for(int i = 0; i < NUM_PARKING_SPOTS; i++)
    {
        if(parking_status[i] == 1) ocupados++;
    }

    UART_Print("\nResumen: %d/%d ocupados, %d/%d libres\r\n",
            ocupados, NUM_PARKING_SPOTS,
            NUM_PARKING_SPOTS - ocupados, NUM_PARKING_SPOTS);

    UART_Print("===============================\r\n\n");
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
  MX_ADC2_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */

  // Mensaje de inicio por UART primero
  HAL_Delay(100);  // Pequeño delay para estabilizar
  HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n\n*** SISTEMA INICIADO ***\r\n", 29, 1000);

  // Inicializar Neopixels
  UART_Print("Iniciando PWM para Neopixels...\r\n");
  HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, (uint32_t*)&htim1.Instance->CCR1, 1);

  UART_Print("Configurando Neopixels...\r\n");
  setBrightness(NEOPIXEL_BRIGHTNESS);
  pixelClear();  // Apagar todos los LEDs al inicio
  pixelShow();   // Enviar comando de actualización
  HAL_Delay(10);

  // Test de Neopixels - todos en rojo por 1 segundo
  UART_Print("Test Neopixels - ROJO...\r\n");
  for(int i = 0; i < NUM_PARKING_SPOTS; i++)
  {
      setPixelColor(i, 255, 0, 0);  // Rojo
  }
  pixelShow();
  HAL_Delay(1000);

  // Test de Neopixels - todos en verde por 1 segundo
  UART_Print("Test Neopixels - VERDE...\r\n");
  for(int i = 0; i < NUM_PARKING_SPOTS; i++)
  {
      setPixelColor(i, 0, 255, 0);  // Verde
  }
  pixelShow();
  HAL_Delay(1000);

  // Apagar todos
  UART_Print("Apagando Neopixels...\r\n");
  pixelClear();
  pixelShow();
  HAL_Delay(500);

  // Inicializar LCD
    UART_Print("[3/4] Iniciando pantalla LCD ILI9341...\r\n");
    LCD_Init();
    LCD_Clear(0x00);

    // Cargar pantalla inicial
    LCD_Bitmap(0, 0, 320, 240, pantalla_inicio);

    // Espacios en parte baja (estos permanecen estáticos por ahora)
    LCD_Sprite(58, 165, 29, 56, carro_blancoSprite, 2, 1, 0, 0);
    LCD_Sprite(90, 166, 29, 56, carro_negroSprite, 2, 1, 0, 0);
    LCD_Sprite(122, 165, 29, 56, carro_blancoSprite, 2, 1, 0, 0);
    LCD_Sprite(154, 166, 29, 56, carro_negroSprite, 2, 1, 0, 0);

    // Espacios en parte alta (controlados por esta Nucleo - inicialmente todos libres)
    UART_Print("Configurando espacios de parqueo en pantalla...\r\n");
    LCD_Sprite(58, 19, 29, 56, carro_frenteSprite, 2, 1, 0, 0);  // Espacio 1 - Libre
    LCD_Sprite(90, 19, 29, 56, carro_rojoSprite, 2, 1, 1, 0);    // Espacio 2 - Libre
    LCD_Sprite(122, 19, 29, 56, carro_frenteSprite, 2, 1, 0, 0); // Espacio 3 - Libre
    LCD_Sprite(154, 19, 29, 56, carro_rojoSprite, 2, 1, 0, 0);   // Espacio 4 - Libre

    // Display de 7 segmentos - inicialmente mostrando 4 (todos disponibles)
    UART_Print("Configurando display de espacios disponibles...\r\n");
    LCD_Sprite(258, 83, 44, 75, display, 9, 4, 0, 0);  // Mostrar "4"

    UART_Print("    LCD inicializado correctamente\r\n");

  // Mensaje inicial por UART - Prueba simple
  HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n\n*** SISTEMA INICIADO ***\r\n", 29, 1000);

  UART_Print("Umbral LDR: %d\r\n", LDR_THRESHOLD);
  UART_Print("Brillo Neopixels: %d/255\r\n\n", NEOPIXEL_BRIGHTNESS);

  /* ============ Inicializar I2C SLAVE ============ */
  UART_Print("[I2C SLAVE] Inicializando como esclavo (0x%02X)...\r\n", hi2c2.Init.OwnAddress1);

  /* ⭐ HABILITAR MODO LISTEN - CRÍTICO */
  if (HAL_I2C_EnableListen_IT(&hi2c2) != HAL_OK) {
      UART_Print("[I2C SLAVE] ERROR: No se pudo habilitar Listen mode\r\n");
      Error_Handler();
  } else {
      UART_Print("[I2C SLAVE] Modo Listen habilitado correctamente\r\n");
  }

  /* Preparar primer paquete TX (inicialmente todo libre) */
  uint8_t libres_inicial = 4;  // Todos libres al inicio
  i2c_tx_buf[0] = libres_inicial;
  for (int i = 0; i < 4; i++) {
      i2c_tx_buf[1 + i*2] = 0xFF;      // LDR alto (libre) LSB
      i2c_tx_buf[1 + i*2 + 1] = 0x0F;  // MSB
  }

  UART_Print("[I2C SLAVE] Buffer inicial preparado\r\n\r\n");

  /* Preparar primer paquete TX para que el maestro pueda leer (inicial) */
  /* Calcular libres locales (convención local: parking_status 0=libre,1=ocupado) */
  uint8_t libres_local = 0;
  for (int i = 0; i < NUM_PARKING_SPOTS; i++) {
      if (parking_status[i] == 0) libres_local++;
  }
  i2c_tx_buf[0] = libres_local;
  /* Escribir LDRs raw (uint16 little-endian) */
  for (int i = 0; i < NUM_PARKING_SPOTS; i++) {
      uint16_t v = (uint16_t)adc_values[i];
      i2c_tx_buf[1 + i*2]     = (uint8_t)(v & 0xFF);       // LSB
      i2c_tx_buf[1 + i*2 + 1] = (uint8_t)((v >> 8) & 0xFF); // MSB
  }

  if (HAL_I2C_Slave_Transmit_IT(&hi2c2, i2c_tx_buf, I2C_TX_SIZE) != HAL_OK) {
      UART_Print("[I2C SLAVE] Error iniciando Slave Transmit IT\r\n");
  } else {
      i2c_tx_busy = true;
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
   uint32_t last_debug_time = 0;
   uint8_t prev_parking_status[NUM_PARKING_SPOTS] = {0};  // Para detectar cambios

   // Inicializar estado anterior
   for(int i = 0; i < NUM_PARKING_SPOTS; i++)
   {
       prev_parking_status[i] = parking_status[i];
   }

   while (1)
   {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

     // Leer sensores LDR
     Read_LDR_Sensors();

     // Actualizar estado de los parqueos
     Update_Parking_Status();

     /* ===== Preparar paquete I2C TX (que el maestro leerá) ===== */
     {
         uint8_t libres_local = 0;
         for (int i = 0; i < NUM_PARKING_SPOTS; i++) {
             // ⭐ INVERTIR: parking_status 0=libre, 1=ocupado
             // Pero enviamos en formato del maestro: 1=libre, 0=ocupado
             if (parking_status[i] == 0) libres_local++;  // 0 = libre
         }

         i2c_tx_buf[0] = libres_local;

         for (int i = 0; i < NUM_PARKING_SPOTS; i++) {
             uint16_t v = (uint16_t)adc_values[i];
             i2c_tx_buf[1 + i*2]     = (uint8_t)(v & 0xFF);       // LSB
             i2c_tx_buf[1 + i*2 + 1] = (uint8_t)((v >> 8) & 0xFF); // MSB
         }

         /* Re-arm transmit IT solo si no hay una transmisión en curso */
         if (!i2c_tx_busy) {
             if (HAL_I2C_Slave_Transmit_IT(&hi2c2, i2c_tx_buf, I2C_TX_SIZE) == HAL_OK) {
                 i2c_tx_busy = true;
             } else {
                 UART_Print("[I2C SLAVE] Falló rearm Transmit_IT\r\n");
             }
         }
     }

     // Detectar si hubo cambios en el estado
     uint8_t status_changed = 0;
     for(int i = 0; i < NUM_PARKING_SPOTS; i++)
     {
         if(prev_parking_status[i] != parking_status[i])
         {
             status_changed = 1;
             prev_parking_status[i] = parking_status[i];
         }
     }

     // Si hubo cambios, actualizar la pantalla LCD
     if(status_changed)
     {
         UART_Print("[LCD] Actualizando visualización...\r\n");
         Update_LCD_Parking_Display();
         Update_LCD_Available_Counter();
     }

     // Actualizar Neopixels (siempre, son muy rápidos)
     Update_Neopixels();

     // Imprimir debug cada 2 segundos
     if((HAL_GetTick() - last_debug_time) >= 2000)
     {
         Debug_Print_Status();
         last_debug_time = HAL_GetTick();
     }

     /* ===== Procesar comandos recibidos desde el maestro (fuera del ISR) ===== */
     if (i2c_rx_ready) {
         i2c_rx_ready = false;

         // CMD_DISPLAY_UPDATE (0xA1)
         if (i2c_rx_buf[0] == 0xA1) {
             master_total_libres = i2c_rx_buf[1];
             for (int i = 0; i < 8; i++) {
                 master_states[i] = i2c_rx_buf[2 + i]; // convención del maestro: 1=LIBRE, 0=OCUPADO
             }
             UART_Print("[I2C SLAVE] CMD_DISPLAY_UPDATE recibido. Total libres: %d\r\n", master_total_libres);

             // Actualizar la pantalla LCD para mostrar los 8 espacios tal como mandó el maestro
             Update_LCD_Full_Display(master_states, master_total_libres);
         } else {
             UART_Print("[I2C SLAVE] Comando desconocido 0x%02X\r\n", i2c_rx_buf[0]);
         }
     }

     // Delay entre lecturas
     HAL_Delay(100);

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
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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
  hi2c2.Init.OwnAddress1 = 0x04;
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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  HAL_GPIO_WritePin(LCD_RESET_GPIO_Port, LCD_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_RESET_Pin */
  GPIO_InitStruct.Pin = LCD_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(LCD_RESET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_DC_Pin */
  GPIO_InitStruct.Pin = LCD_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(LCD_DC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_CS_Pin */
  GPIO_InitStruct.Pin = LCD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(LCD_CS_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
 * @brief Actualiza TODA la pantalla LCD con los 8 espacios
 * @param states8: Array de 8 bytes (convención maestro: 1=LIBRE, 0=OCUPADO)
 * @param total_libres: Total de espacios libres (0-8)
 */
void Update_LCD_Full_Display(uint8_t *states8, uint8_t total_libres) {
    uint16_t x_positions[4] = {58, 90, 122, 154};
    uint16_t top_y = 19;      // Parte alta (espacios 1-4 del maestro)
    uint16_t bottom_y = 165;  // Parte baja (espacios 5-8 de este esclavo)
    uint16_t w = 29, h = 56;

    UART_Print("[LCD UPDATE] Total: %d | Estados: [%d%d%d%d|%d%d%d%d]\r\n",
               total_libres, states8[0], states8[1], states8[2], states8[3],
               states8[4], states8[5], states8[6], states8[7]);

    // ===== PARTE ALTA: Espacios 1-4 (del maestro) =====
    for (int i = 0; i < 4; i++) {
        uint8_t estado = states8[i];  // 1=LIBRE, 0=OCUPADO

        if (estado == 1) {  // Libre
            if (i % 2 == 0)
                LCD_Sprite(x_positions[i], top_y, w, h, carro_frenteSprite, 2, 1, 0, 0);
            else
                LCD_Sprite(x_positions[i], top_y, w, h, carro_rojoSprite, 2, 1, 0, 0);
        } else {  // Ocupado
            if (i % 2 == 0)
                LCD_Sprite(x_positions[i], top_y, w, h, carro_frenteSprite, 2, 0, 0, 0);
            else
                LCD_Sprite(x_positions[i], top_y, w, h, carro_rojoSprite, 2, 0, 0, 0);
        }
    }

    // ===== PARTE BAJA: Espacios 5-8 (de este esclavo) =====
    for (int j = 0; j < 4; j++) {
        uint8_t estado = states8[4 + j];  // 1=LIBRE, 0=OCUPADO

        if (estado == 1) {  // Libre
            if (j % 2 == 0)
                LCD_Sprite(x_positions[j], bottom_y, w, h, carro_blancoSprite, 2, 1, 0, 0);
            else
                LCD_Sprite(x_positions[j], bottom_y + 1, w, h, carro_negroSprite, 2, 1, 0, 0);
        } else {  // Ocupado
            if (j % 2 == 0)
                LCD_Sprite(x_positions[j], bottom_y, w, h, carro_blancoSprite, 2, 0, 0, 0);
            else
                LCD_Sprite(x_positions[j], bottom_y + 1, w, h, carro_negroSprite, 2, 0, 0, 0);
        }
    }

    // ===== ACTUALIZAR DISPLAY 7 SEGMENTOS =====
    if (total_libres > 8) total_libres = 8;
    LCD_Sprite(258, 83, 44, 75, display, 9, total_libres, 0, 0);

    UART_Print("[LCD UPDATE] Pantalla actualizada correctamente\r\n");
}
//* ===== I2C Callbacks Corregidos ===== */

/* Callback cuando el maestro direcciona al esclavo */
void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
{
    if (hi2c->Instance != I2C2) return;

    if (TransferDirection == I2C_DIRECTION_RECEIVE) {
        // Maestro quiere LEER (Slave TX) - Enviar 9 bytes
        if (HAL_I2C_Slave_Seq_Transmit_IT(&hi2c2, i2c_tx_buf, I2C_TX_SIZE,
                                           I2C_FIRST_AND_LAST_FRAME) == HAL_OK) {
            i2c_tx_busy = true;
        }
    }
    else if (TransferDirection == I2C_DIRECTION_TRANSMIT) {
        // Maestro quiere ESCRIBIR (Slave RX) - Recibir 10 bytes
        HAL_I2C_Slave_Seq_Receive_IT(&hi2c2, i2c_rx_buf, I2C_RX_CMD_SIZE,
                                       I2C_FIRST_AND_LAST_FRAME);
    }
}

/* Callback: Transmisión completada (maestro leyó los 9 bytes) */
void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance != I2C2) return;
    i2c_tx_busy = false;
    // UART_Print("[I2C] TX Complete\r\n");  // Debug opcional
}

/* Callback: Recepción completada (maestro escribió comando) */
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance != I2C2) return;

    i2c_rx_ready = true;
    i2c_rx_len = I2C_RX_CMD_SIZE;

    // Debug: mostrar comando recibido
    UART_Print("[I2C RX] Comando: 0x%02X, Total: %d\r\n", i2c_rx_buf[0], i2c_rx_buf[1]);
}

/* Callback: Comunicación completada, re-habilitar Listen */
void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance != I2C2) return;
    HAL_I2C_EnableListen_IT(&hi2c2);
}

/* Callback de error */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance != I2C2) return;

    UART_Print("[I2C ERROR] Code: 0x%08lX\r\n", hi2c->ErrorCode);

    // Limpiar flags y re-habilitar
    i2c_tx_busy = false;
    HAL_Delay(5);
    HAL_I2C_EnableListen_IT(&hi2c2);
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
