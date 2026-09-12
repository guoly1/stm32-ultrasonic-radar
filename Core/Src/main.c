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
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
#include "oled.h"
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  printf("\r\n=== boot ===\r\n");

  /* --- I2C 总线扫描：确认 OLED(0x3D) 与 AHT20(0x38) 在位 --- */
  printf("=== I2C Bus Scan ===\r\n");
  uint8_t found = 0;
  for (uint8_t addr = 1; addr < 128; addr++)
  {
      /* HAL 需要 8 位地址，7 位地址必须左移 1 位 */
      if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(addr << 1), 3, 10) == HAL_OK)
      {
          printf("  found 7-bit: 0x%02X   (8-bit: 0x%02X)\r\n", addr, addr << 1);
          found++;
      }
  }
  if (found == 0) printf("  no device found! check I2C config\r\n");
  printf("=== scan done, %d device(s) ===\r\n\r\n", found);

  /* --- OLED 初始化 --- */
  HAL_Delay(20);   /* STM32 比 OLED 启动快，不延时会导致初始化失败（表现为屏幕偏暗） */
  OLED_Init();

  /* --- 主循环用的状态变量 --- */
  uint8_t  page         = 0;      /* 当前页面 0/1/2 */
  uint8_t  pageDrawn    = 0xFF;   /* 已画过的页面；0xFF 表示还没画，强制首帧绘制 */
  uint32_t lastKeyTick  = 0;
  uint32_t lastBeatTick = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /* ---------- 1. 按键扫描：每 20ms 一次，非阻塞 ---------- */
    if (HAL_GetTick() - lastKeyTick >= 20)
    {
      lastKeyTick = HAL_GetTick();

      static uint8_t last1 = 1, last2 = 1;
      uint8_t now1 = HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin);
      uint8_t now2 = HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin);

      if (last1 == 1 && now1 == 0)        /* KEY1 按下（检测下降沿） */
      {
        page = (page + 1) % 3;
        printf("page -> %d\r\n", page);
      }
      if (last2 == 1 && now2 == 0)        /* KEY2 按下 */
      {
        page = (page + 2) % 3;            /* +2 等价于上一页 */
        printf("page -> %d\r\n", page);
      }
      last1 = now1;
      last2 = now2;
    }

    /* ---------- 2. 只在页面变化时重绘 ---------- */
    /* OLED 推一整帧要 20~30ms，绝不能每圈循环都刷，否则界面会卡死 */
    if (page != pageDrawn)
    {
      pageDrawn = page;

      OLED_NewFrame();
      switch (page)
      {
        case 0:   /* 首页 */
          OLED_DrawRectangle(0, 0, 128, 64, OLED_COLOR_NORMAL);
          OLED_PrintString(6, 6,  "Radar v0.1", &font16x16, OLED_COLOR_NORMAL);
          OLED_PrintString(6, 26, "Day 1 OK",   &font16x16, OLED_COLOR_NORMAL);
          break;

        case 1:   /* 绘图测试页 */
          OLED_DrawLine(0, 0, 127, 63, OLED_COLOR_NORMAL);
          OLED_DrawLine(127, 0, 0, 63, OLED_COLOR_NORMAL);
          OLED_DrawCircle(64, 32, 24, OLED_COLOR_NORMAL);
          OLED_DrawFilledCircle(64, 32, 3, OLED_COLOR_NORMAL);
          break;

        default:  /* 按键说明页 */
          OLED_PrintString(6, 6,  "KEY1: next", &font16x16, OLED_COLOR_NORMAL);
          OLED_PrintString(6, 26, "KEY2: prev", &font16x16, OLED_COLOR_NORMAL);
          break;
      }
      OLED_ShowFrame();
    }

    /* ---------- 3. 心跳：每秒一次 ---------- */
    if (HAL_GetTick() - lastBeatTick >= 1000)
    {
      lastBeatTick = HAL_GetTick();
      HAL_GPIO_TogglePin(LED_B_GPIO_Port, LED_B_Pin);
      printf("alive, tick = %u ms, page = %d\r\n", (unsigned)HAL_GetTick(), page);
    }
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
/* printf 重定向到 USART2 */
int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return len;
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
