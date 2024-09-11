/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
# include "FreeRTOS.h"
#include "stm32f4xx_hal_cortex.h"
#include "stm32f4xx_hal_gpio.h"
# include "task.h"
# include "queue.h"
# include "semphr.h"
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

/* USER CODE BEGIN PV */
xTaskHandle xTaskHandle1, xTaskHandle2, xTaskHandle3, xTaskHandle4;
xSemaphoreHandle xBinarySemaphore;
uint8_t buttonPressed = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
void vTask1(void* pvParameters);
void vTask2(void* pvParameters);
void vTask3(void* pvParameters);
void vTask4(void* pvParameters);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void vTask1(void* pvParameters) {
  uint32_t tasksCount = 1;
  uint32_t led_state = 1;

  while (1) {
    vTaskDelay(500 / portTICK_RATE_MS);
    gpio_led_state(1, led_state);
    led_state = (led_state == 1)? 0 : 1;

    xSemaphoreTake(xBinarySemaphore, portMAX_DELAY);
    if (buttonPressed) {
      buttonPressed = 0;
      xSemaphoreGive(xBinarySemaphore);

      tasksCount++;
      if (tasksCount > 4) {
        vTaskDelete(xTaskHandle2);
        vTaskDelete(xTaskHandle3);
        vTaskDelete(xTaskHandle4);

        gpio_led_state(2, 0);
        gpio_led_state(3, 0);
        gpio_led_state(4, 0);

        tasksCount = 1;

      } else {
        switch (tasksCount) {
          case 2:
            xTaskCreate(
              vTask2,
              "TASK2",
              configMINIMAL_STACK_SIZE,
              NULL,
              1,
              &xTaskHandle2
            );
            break;

          case 3:
            xTaskCreate(
              vTask3,
              "TASK3",
              configMINIMAL_STACK_SIZE,
              NULL,
              1,
              &xTaskHandle3
            );
            break;

          case 4:
            xTaskCreate(
              vTask4,
              "TASK4",
              configMINIMAL_STACK_SIZE,
              NULL,
              1,
              &xTaskHandle4
            );
            break;
        }
      }
      
    } else {
      xSemaphoreGive(xBinarySemaphore);
    }
  }
}

void vTask2(void* pvParameters) {
  uint32_t led_state = 1;

  while (1) {
    vTaskDelay(500 / portTICK_RATE_MS);
    gpio_led_state(2, led_state);
    led_state = (led_state == 1)? 0 : 1;
  }
}

void vTask3(void* pvParameters) {
  uint32_t led_state = 1;

  while (1) {
    vTaskDelay(500 / portTICK_RATE_MS);
    gpio_led_state(3, led_state);
    led_state = (led_state == 1)? 0 : 1;
  }
}

void vTask4(void* pvParameters) {
  uint32_t led_state = 1;

  while (1) {
    vTaskDelay(500 / portTICK_RATE_MS);
    gpio_led_state(4, led_state);
    led_state = (led_state == 1)? 0 : 1;
  }
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();

  xBinarySemaphore = xSemaphoreCreateBinary();
  xSemaphoreGive(xBinarySemaphore);
  
  xTaskCreate(
    vTask1,
    "TASK1",
    configMINIMAL_STACK_SIZE,
    NULL,
    1,
    NULL
  );

  vTaskStartScheduler();

  while (1);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == USER_BUTTON_PIN) {
    HAL_NVIC_DisableIRQ(EXTI0_IRQn);
    xSemaphoreTakeFromISR(xBinarySemaphore, NULL);
    buttonPressed = 1;
    xSemaphoreGiveFromISR(xBinarySemaphore, NULL);
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);
  } else {
    __NOP();
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
