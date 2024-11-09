/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
#include "usbd_cdc_if.h"
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
unsigned char IMU1array[50] = {0};
unsigned char IMU2array[50] = {0};
unsigned char IMU3array[50] = {0};
char *pos1 = NULL;
char *pos2 = NULL;
double roll1 = 0.0, pitch1 = 0.0, yaw1 = 0.0;
double gyro_x1 = 0.0, gyro_y1 = 0.0, gyro_z1 = 0.0;
double accel_x1 = 0.0, accel_y1 = 0.0, accel_z1 = 0.0;

double roll2 = 0.0, pitch2 = 0.0, yaw2 = 0.0;
double gyro_x2 = 0.0, gyro_y2 = 0.0, gyro_z2 = 0.0;
double accel_x2 = 0.0, accel_y2 = 0.0, accel_z2 = 0.0;

double roll3 = 0.0, pitch3 = 0.0, yaw3 = 0.0;
char alpha1[10] = {0}, beta1[10] = {0}, gamm1[10] = {0};
char gyro_x1_str[10] = {0}, gyro_y1_str[10] = {0}, gyro_z1_str[10] = {0};
char accel_x1_str[10] = {0}, accel_y1_str[10] = {0}, accel_z1_str[10] = {0};

char alpha2[10] = {0}, beta2[10] = {0}, gamm2[10] = {0};
char gyro_x2_str[10] = {0}, gyro_y2_str[10] = {0}, gyro_z2_str[10] = {0};
char accel_x2_str[10] = {0}, accel_y2_str[10] = {0}, accel_z2_str[10] = {0};

uint8_t data_buffer[146];
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim9;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart6_rx;
DMA_HandleTypeDef hdma_usart6_tx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM9_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void get_IMU1();
void get_IMU2();
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
  MX_TIM9_Init();
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();
  MX_USART6_UART_Init();
  MX_USART2_UART_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      get_IMU1(); // IMU 데이터를 업데이트
      get_IMU2();

      // Roll, Pitch, Yaw 데이터를 전송 버퍼에 넣기
      data_buffer[0] = 0xAA;
      memcpy(data_buffer + 1, &roll1, sizeof(roll1));
      memcpy(data_buffer + 9, &pitch1, sizeof(pitch1));
      memcpy(data_buffer + 17, &yaw1, sizeof(yaw1));
      memcpy(data_buffer + 25, &gyro_x1, sizeof(gyro_x1));
      memcpy(data_buffer + 33, &gyro_y1, sizeof(gyro_y1));
      memcpy(data_buffer + 41, &gyro_z1, sizeof(gyro_z1));
      memcpy(data_buffer + 49, &accel_x1, sizeof(accel_x1));
      memcpy(data_buffer + 57, &accel_y1, sizeof(accel_y1));
      memcpy(data_buffer + 65, &accel_z1, sizeof(accel_z1));

      memcpy(data_buffer + 73, &roll2, sizeof(roll2));
      memcpy(data_buffer + 81, &pitch2, sizeof(pitch2));
      memcpy(data_buffer + 89, &yaw2, sizeof(yaw2));
      memcpy(data_buffer + 97, &gyro_x2, sizeof(gyro_x2));
      memcpy(data_buffer + 105, &gyro_y2, sizeof(gyro_y2));
      memcpy(data_buffer + 113, &gyro_z2, sizeof(gyro_z2));
      memcpy(data_buffer + 121, &accel_x2, sizeof(accel_x2));
      memcpy(data_buffer + 129, &accel_y2, sizeof(accel_y2));
      memcpy(data_buffer + 137, &accel_z2, sizeof(accel_z2));

      data_buffer[145] = 0xBB;

      // USB CDC를 통해 데이터 전송
      if (CDC_Transmit_FS(data_buffer, sizeof(data_buffer)) == USBD_OK) {
         HAL_Delay(500); // 100ms 대기 (전송 주기)
      } else {
         HAL_Delay(10); // 실패 시 잠시 대기 후 재시도
      }
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* OTG_FS_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(OTG_FS_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(OTG_FS_IRQn);
  /* USART1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USART2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 8399;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 99;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream5_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void get_IMU1()
{
    HAL_UART_Receive_DMA(&huart1, IMU1array, 50);  // 수신 데이터 크기 확장
    int cnt0 = 0, pos = 0;
    for (int i = 0; i < 50; i++) {
        if (IMU1array[i] == ',') {
            switch (cnt0++) {
                case 0: strncpy(alpha1, (char *)IMU1array + pos, i - pos); alpha1[i - pos] = '\0'; break;
                case 1: strncpy(beta1, (char *)IMU1array + pos, i - pos); beta1[i - pos] = '\0'; break;
                case 2: strncpy(gamm1, (char *)IMU1array + pos, i - pos); gamm1[i - pos] = '\0'; break;
                case 3: strncpy(gyro_x1_str, (char *)IMU1array + pos, i - pos); gyro_x1_str[i - pos] = '\0'; break;
                case 4: strncpy(gyro_y1_str, (char *)IMU1array + pos, i - pos); gyro_y1_str[i - pos] = '\0'; break;
                case 5: strncpy(gyro_z1_str, (char *)IMU1array + pos, i - pos); gyro_z1_str[i - pos] = '\0'; break;
                case 6: strncpy(accel_x1_str, (char *)IMU1array + pos, i - pos); accel_x1_str[i - pos] = '\0'; break;
                case 7: strncpy(accel_y1_str, (char *)IMU1array + pos, i - pos); accel_y1_str[i - pos] = '\0'; break;
                case 8: strncpy(accel_z1_str, (char *)IMU1array + pos, i - pos); accel_z1_str[i - pos] = '\0'; break;
            }
            pos = i + 1;
        }
    }
    roll1 = strtod(alpha1, NULL);
    pitch1 = strtod(beta1, NULL);
    yaw1 = strtod(gamm1, NULL);
    gyro_x1 = strtod(gyro_x1_str, NULL);
    gyro_y1 = strtod(gyro_y1_str, NULL);
    gyro_z1 = strtod(gyro_z1_str, NULL);
    accel_x1 = strtod(accel_x1_str, NULL);
    accel_y1 = strtod(accel_y1_str, NULL);
    accel_z1 = strtod(accel_z1_str, NULL);
}

void get_IMU2()
{
    HAL_UART_Receive_DMA(&huart6, IMU2array, 50);
    int cnt0 = 0, pos = 0;
    for (int i = 0; i < 50; i++) {
        if (IMU2array[i] == ',') {
            switch (cnt0++) {
                case 0: strncpy(alpha2, (char *)IMU2array + pos, i - pos); alpha2[i - pos] = '\0'; break;
                case 1: strncpy(beta2, (char *)IMU2array + pos, i - pos); beta2[i - pos] = '\0'; break;
                case 2: strncpy(gamm2, (char *)IMU2array + pos, i - pos); gamm2[i - pos] = '\0'; break;
                case 3: strncpy(gyro_x2_str, (char *)IMU2array + pos, i - pos); gyro_x2_str[i - pos] = '\0'; break;
                case 4: strncpy(gyro_y2_str, (char *)IMU2array + pos, i - pos); gyro_y2_str[i - pos] = '\0'; break;
                case 5: strncpy(gyro_z2_str, (char *)(char *)IMU2array + pos, i - pos); gyro_z2_str[i - pos] = '\0'; break;
                case 6: strncpy(accel_x2_str, (char *)IMU2array + pos, i - pos); accel_x2_str[i - pos] = '\0'; break;
                case 7: strncpy(accel_y2_str, (char *)IMU2array + pos, i - pos); accel_y2_str[i - pos] = '\0'; break;
                case 8: strncpy(accel_z2_str, (char *)IMU2array + pos, i - pos); accel_z2_str[i - pos] = '\0'; break;
            }
            pos = i + 1;
        }
    }
    roll2 = strtod(alpha2, NULL);
    pitch2 = strtod(beta2, NULL);
    yaw2 = strtod(gamm2, NULL);
    gyro_x2 = strtod(gyro_x2_str, NULL);
    gyro_y2 = strtod(gyro_y2_str, NULL);
    gyro_z2 = strtod(gyro_z2_str, NULL);
    accel_x2 = strtod(accel_x2_str, NULL);
    accel_y2 = strtod(accel_y2_str, NULL);
    accel_z2 = strtod(accel_z2_str, NULL);
}
/*void get_IMU1(double *roll1, double *pitch1, double *yaw1){
	int cnt0 = 0, cnt1 = 0, cnt2 = 0, cnt3 = 0, cnt4 = 0;
	HAL_UART_Receive_DMA(&huart1, &IMU1array[0], 31);
    for(int i = 1; i < 31; i++){
      if(IMU1array[i] == ','){
         if(cnt0 == 0){
            cnt1 = i + 1;
         }
         else if (cnt0 == 1){
            cnt2 = i + 1;
         }
         cnt0++;
      }
      else{
         switch (cnt0){
         case 0:{
            alpha1[i - 1] = IMU1array[i];
            break;
         }
         case 1:{
            beta1[i - cnt1] = IMU1array[i];
            break;
         }
         case 2:{
            gamm1[i - cnt2] = IMU1array[i];
            break;
         }
         }
      }
      if(IMU1array[i] == '\r'){
         cnt3 = i + 1;
         break;
      }
   }
   for (int j = cnt1 - 2; j < 7; j++){
      alpha1[j] = '0';
   }
   for (int k = cnt2 - (cnt1); k < 8; k++){
      beta1[k - 1] = '0';
   }
   for (int l = cnt3 - (cnt2); l < 8; l++){
      gamm1[l - 1] = '0';
   }
   cnt0 = 0;

   *roll1 = strtod(alpha1, &pos1);
   *pitch1 = strtod(beta1, &pos1);
   *yaw1 = strtod(gamm1, &pos1);
}

void get_IMU2(double *roll2, double *pitch2, double *yaw2){
	int cnt0 = 0, cnt1 = 0, cnt2 = 0, cnt3 = 0, cnt4 = 0;
   HAL_UART_Receive_DMA(&huart2, &IMU2array[0], 31);
   for(int i = 1; i < 31; i++){
      if(IMU2array[i] == ','){
         if(cnt0 == 0){
            cnt1 = i + 1;
         }
         else if (cnt0 == 1){
            cnt2 = i + 1;
         }
         cnt0++;
      }
      else{
         switch (cnt0){
         case 0:{
            alpha2[i - 1] = IMU2array[i];
            break;
         }
         case 1:{
            beta2[i - cnt1] = IMU2array[i];
            break;
         }
         case 2:{
            gamm2[i - cnt2] = IMU2array[i];
            break;
         }
         }
      }
      if(IMU2array[i] == '\r'){
         cnt3 = i + 1;
         break;
      }
   }
   for (int j = cnt1 - 2; j < 7; j++){
      alpha2[j] = '0';
   }
   for (int k = cnt2 - (cnt1); k < 8; k++){
      beta2[k - 1] = '0';
   }
   for (int l = cnt3 - (cnt2); l < 8; l++){
      gamm2[l - 1] = '0';
   }
   cnt0 = 0;

   *roll2 = strtod(alpha2, &pos2);
   *pitch2 = strtod(beta2, &pos2);
   *yaw2 = strtod(gamm2, &pos2);
}
*/
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

#ifdef  USE_FULL_ASSERT
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
