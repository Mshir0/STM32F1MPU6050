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
#include "cmsis_os.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>

#include "command.h"
#include "kalman.h"
#include "oled.h"
#include "mpu6050.h"
#include "kalman.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define  QUEUE_LEN    4   /* 队列的长度，最大可包含多少个消息 */
#define  QUEUE_SIZE   4   /* 队列中每个消息大小（字节） */

static TaskHandle_t APPTaskCreate_Handel = NULL;
static TaskHandle_t UART_Task_Handel = NULL;
static TaskHandle_t KEY_Task_Handel = NULL;
static TaskHandle_t LED_Task_Handel = NULL;
static TaskHandle_t OLED_Task_Handel = NULL;
static TaskHandle_t MPU6050_Task_Handel = NULL;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint8_t readBuffer[128];
QueueHandle_t Queue = NULL;
char buf[25]={0};
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void APPCreate_Task(void);
static void UART_Task(void *pvParameters);
static void KEY_Task(void *pvParameters);
static void LED_Task(void *pvParameters);
static void OLED_Task(void *pvParameters);
static void MPU6050_Task(void *pvParameters);

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
  if (huart == &huart2) {
    Command_Write(readBuffer , Size);
    HAL_UARTEx_ReceiveToIdle_IT(&huart2, readBuffer, sizeof(readBuffer));
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  int sum = 10;
  if (GPIO_Pin == GPIO_PIN_10) {
    sum = sum + 1;
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_9);
    if (xQueueSendFromISR(Queue, &sum, &xHigherPriorityTaskWoken) == pdPASS) {
      HAL_UART_Transmit(&huart2, (uint8_t*) "key exti\r\n", sizeof("key exti\r\n"), 100);
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}




/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  BaseType_t xReturn = pdPASS;
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  xReturn = xTaskCreate((TaskFunction_t ) APPCreate_Task,
               (const char*    ) "APPCreate_Task",
               (uint32_t       ) 512,
               (void*          ) NULL,
               (UBaseType_t    ) 1,
               (TaskHandle_t* ) &APPTaskCreate_Handel);
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

/* USER CODE BEGIN 4 */
static void APPCreate_Task(void) {
  BaseType_t xReturn = pdPASS;
  taskENTER_CRITICAL();

  Queue = xQueueCreate( (UBaseType_t ) QUEUE_LEN,
                        (UBaseType_t ) QUEUE_SIZE);
  if (Queue != NULL)
    HAL_UART_Transmit(&huart2, (uint8_t*) "Queue init\r\n", sizeof("Queue init\r\n"), 100);

  xReturn = xTaskCreate((TaskFunction_t ) UART_Task,
                    (const char*    ) "UART_Task",
                    (uint32_t       ) 64,
                    (void*          ) NULL,
                    (UBaseType_t    ) 16,
                    (TaskHandle_t* ) &UART_Task_Handel);
  if(pdPASS == xReturn)
    HAL_UART_Transmit(&huart2, (uint8_t*) "uart init\r\n", sizeof("uart init\r\n"), 100);

  xReturn = xTaskCreate((TaskFunction_t ) OLED_Task,
                (const char*    ) "OLED_Task",
                (uint32_t       ) 64,
                (void*          ) NULL,
                (UBaseType_t    ) 4,
                (TaskHandle_t* ) &OLED_Task_Handel);
  if(pdPASS == xReturn)
    HAL_UART_Transmit(&huart2, (uint8_t*) "oled init\r\n", sizeof("oled init\r\n"), 100);

  xReturn = xTaskCreate((TaskFunction_t ) KEY_Task,
                (const char*    ) "KEY_Task",
                (uint32_t       ) 64,
                (void*          ) NULL,
                (UBaseType_t    ) 5,
                (TaskHandle_t* ) &KEY_Task_Handel);
  if(pdPASS == xReturn)
    HAL_UART_Transmit(&huart2, (uint8_t*) "key init\r\n", sizeof("key init\r\n"), 100);

  xReturn = xTaskCreate((TaskFunction_t ) LED_Task,
                  (const char*    ) "LED_Task",
                  (uint32_t       ) 64,
                  (void*          ) NULL,
                  (UBaseType_t    ) 10,
                  (TaskHandle_t* ) &LED_Task_Handel);
  if(pdPASS == xReturn)
    HAL_UART_Transmit(&huart2, (uint8_t*) "led init\r\n", sizeof("led init\r\n"), 100);

  xReturn = xTaskCreate((TaskFunction_t ) MPU6050_Task,
                (const char*    ) "MPU6050_Task",
                (uint32_t       ) 128,
                (void*          ) NULL,
                (UBaseType_t    ) 20,
                (TaskHandle_t* ) &MPU6050_Task_Handel);
  if(pdPASS == xReturn)
    HAL_UART_Transmit(&huart2, (uint8_t*) "MPU6050 init\r\n", sizeof("MPU6050 init\r\n"), 100);

  vTaskDelete(APPTaskCreate_Handel);
  taskEXIT_CRITICAL();
}

static void UART_Task(void *pvParameters) {
  HAL_UARTEx_ReceiveToIdle_IT(&huart2, readBuffer, sizeof(readBuffer));
  static uint8_t command[50];
  int commandLength = 0;

  while (1)
  {
    vTaskDelay(50);
    commandLength = Command_GetCommand(command);
    if (commandLength != 0){
      HAL_UART_Transmit(&huart2, command, commandLength, HAL_MAX_DELAY);
      }
    }
}

static void KEY_Task(void *pvParameters) {
  int sum = 0;
  while (1) {
    if (xQueueReceive(Queue, &sum, portMAX_DELAY) == pdTRUE) {
      HAL_UART_Transmit(&huart2, (uint8_t*) "KEY DETECT\r\n", sizeof("KEY DETECT\r\n"), 100);
    }
  }
}

static void LED_Task(void *pvParameters) {
  while (1) {
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_9);
    vTaskDelay(500);
    //HAL_UART_Transmit(&huart2, (uint8_t*) "LED TOGGLE\r\n", sizeof("LED TOGGLE\r\n"), 100);
  }
}

static void OLED_Task(void *pvParameters) {
  vTaskDelay(200);
  OLED_Init();
  OLED_NewFrame();
  OLED_PrintString(0,0,"Hello World!",&font16x16,OLED_COLOR_NORMAL);
  OLED_ShowFrame();


  vTaskDelete(OLED_Task_Handel);
}

static void MPU6050_Task(void *pvParameters) {
  static portTickType lastWakeTime;
  const TickType_t xPeriod = pdMS_TO_TICKS(20);
  lastWakeTime = xTaskGetTickCount();

  const float dt = 0.02f; // 20ms采样周期
  float roll = 0,  pitch = 0,  yaw = 0;
  MPU6050_DATA mpu_data = {};//传感器信息读取
  IMU_DATA imu_data = {};
  vTaskDelay(200);
  MPU6050_Init(0X07,0X06,0X00,0X00);
  vTaskDelay(200);
  MPU6050_TEST_WHO_AM_I();
  MPU6050_Calibrate();
  while (1) {

    vTaskDelayUntil(&lastWakeTime,xPeriod);
    mpu_data = MPU6050_ReadCalibratedData(); //这里的加速度和角速度为 m/s² * 100来进行积分计算
    imu_data = Quaternion_Update(mpu_data,dt);
    QuaternionToEuler(imu_data,&roll,&pitch,&yaw);

    roll = roll * 180.0f / M_PI;
    pitch = pitch * 180.0f / M_PI;
    yaw = yaw * 180.0f / M_PI;

    ANODT_Send((int16_t)(-roll*100),(int16_t)(pitch*100),(int16_t)(yaw*100));

    //OLED_NewFrame();

    //sprintf(buf,"ROLL :%d",(int)roll);
    //OLED_PrintString(0,0,buf,&font16x16,OLED_COLOR_NORMAL);

    //sprintf(buf,"PITCH :%d",(int)pitch);
    //OLED_PrintString(0,20,buf,&font16x16,OLED_COLOR_NORMAL);

    //sprintf(buf,"YAW :%d",(int)yaw);
    //OLED_PrintString(0,40,buf,&font16x16,OLED_COLOR_NORMAL);


    //OLED_ShowFrame();
  }
  vTaskDelete(MPU6050_Task_Handel);
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
