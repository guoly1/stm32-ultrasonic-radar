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
#include "tim.h"
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
/* ===== 超声波测距用变量（TIM1_CH3 输入捕获）=====
 *
 * ⚠️ 为什么必须加 volatile？
 * 这几个变量在【中断服务函数】里被改写，在【主循环】里被读取。
 * 如果没有 volatile，编译器看到主循环里没人修改它，
 * 就会把它缓存进 CPU 寄存器、只读一次 —— 于是主循环永远读到旧值，
 * 表现为"中断明明进了，主循环却看不到新数据"。
 * volatile 就是告诉编译器：每次都老老实实从内存重新读。
 */
volatile uint32_t us_riseTime   = 0;   /* 上升沿时间戳（µs） */
volatile uint32_t us_pulseWidth = 0;   /* ECHO 高电平宽度（µs） */
volatile uint8_t  us_state      = 0;   /* 0 = 等上升沿，1 = 等下降沿 */
volatile uint8_t  us_done       = 0;   /* 1 = 本次测量完成，有新数据 */
volatile uint8_t  us_busy       = 0;   /* 1 = 已触发，正在等回波 */

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
  MX_TIM4_Init();
  MX_TIM1_Init();
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

  /* --- 启动舵机 PWM 输出（TIM4_CH3 = PB8，50Hz） --- */
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);

  /* --- 启动超声波输入捕获（TIM1_CH3 = PA10） --- */
  HAL_TIM_Base_Start(&htim1);                    /* 启动 TIM1 计数器 */
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_3);    /* 启动 CH3 捕获 + 打开捕获中断 */

  /* --- 主循环用的状态变量 --- */
  uint8_t  page         = 0;      /* 当前页面 0/1/2 */
  uint8_t  pageDrawn    = 0xFF;   /* 已画过的页面；0xFF 表示还没画，强制首帧绘制 */
  uint32_t lastKeyTick  = 0;
  uint32_t lastBeatTick = 0;
  uint32_t lastTrigTick = 0;      /* 上一次触发超声波测距的时间 */

  /* --- 舵机测试用变量（Day 2 Part A 验收用，验完可删） --- */
  /* 注意：脉宽范围收窄到 70~230（而不是理论的 50~250）。
     便宜舵机的实际机械行程往往够不到两端，指令超出后会顶在限位上持续堵转，
     表现为大声嗡嗡响、外壳发热、电流飙升。 */
  const uint16_t servoPulse[3] = {70, 150, 230};
  uint8_t  servoStep     = 0;
  uint8_t  servoSweeps   = 0;      /* 已走步数，走满 6 步（2 轮）自动停机 */
  uint32_t lastServoTick = 0;

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

    /* ---------- 3. 舵机测试：走 2 轮后自动停机（Day 2 Part A 验收用） ---------- */
    if (servoSweeps < 6 && HAL_GetTick() - lastServoTick >= 800)
    {
      lastServoTick = HAL_GetTick();

      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, servoPulse[servoStep]);
      printf("servo -> pulse=%u  (%u/6)\r\n", servoPulse[servoStep], servoSweeps + 1);

      servoStep = (servoStep + 1) % 3;
      servoSweeps++;

      if (servoSweeps >= 6)
      {
        /* 测试结束：停掉 PWM 输出，舵机内部电机断电、彻底不响 */
        HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
        printf("servo test done, PWM stopped\r\n");
      }
    }

    /* ---------- 4. 超声波：每 100ms 触发一次测距（完全非阻塞） ---------- */
    if (!us_busy && (HAL_GetTick() - lastTrigTick >= 100))
    {
      lastTrigTick = HAL_GetTick();

      us_busy  = 1;      /* 标记"测量中"，防止重复触发 */
      us_state = 0;      /* 状态机复位：从"等上升沿"开始 */
      us_done  = 0;

      __HAL_TIM_SET_COUNTER(&htim1, 0);   /* 计数器清零，让时间戳从 0 开始数 */
      __HAL_TIM_SET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_RISING);

      /* TRIG：拉高 ≥10µs 再拉低。HC-SR04 靠这个上升沿启动一次测距。
         @72MHz 下这个空循环约 20µs，__NOP() 是为了防止被编译器优化掉 */
      HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_SET);
      for (volatile uint16_t i = 0; i < 100; i++) { __NOP(); }
      HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);
    }

    /* 超时保护：触发后 40ms 还没等到下降沿，判定无回波。
       没有这一段，一次失败的测量会让 us_busy 永远卡在 1，程序再也不会测距了 */
    if (us_busy && (HAL_GetTick() - lastTrigTick >= 40))
    {
      us_busy  = 0;
      us_state = 0;
      __HAL_TIM_SET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_RISING);
      printf("us: no echo (timeout)\r\n");
    }

    /* 有新数据就换算并打印：距离 = 时间 × 声速 ÷ 2（÷2 是因为声波是往返的） */
    if (us_done)
    {
      us_done = 0;
      float cm = (float)us_pulseWidth * 0.034f / 2.0f;
      printf("us: pulse = %lu us,  distance = %.2f cm\r\n",
             (unsigned long)us_pulseWidth, cm);
    }

    /* ---------- 5. 心跳：每秒一次 ---------- */
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

/* ============================================================
 *  超声波输入捕获中断回调
 *
 *  硬件链路：PA10 出现边沿 → TIM1 硬件把当前计数值锁存进 CCR3
 *            → 触发 TIM1_CC 中断 → HAL 调用下面这个函数
 *
 *  单通道测脉宽的核心技巧：在中断里【切换捕获边沿极性】
 *    第 1 次中断（当前极性=上升）→ 记下时间戳，把极性改成"下降"
 *    第 2 次中断（当前极性=下降）→ 算出宽度，把极性改回"上升"
 *
 *  ⚠️ 中断函数里只做"读寄存器 + 置标志"，绝不做 printf 或耗时运算 ——
 *     中断里待太久会拖慢整个系统，导致丢事件。
 * ============================================================ */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  /* 只处理 TIM1 的 CH3，其他定时器/通道的中断直接返回 */
  if (htim->Instance != TIM1 || htim->Channel != HAL_TIM_ACTIVE_CHANNEL_3)
  {
    return;
  }

  uint32_t cap = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);

  if (us_state == 0)          /* ---------- 抓到上升沿 ---------- */
  {
    us_riseTime = cap;        /* 记下回波开始的时间戳 */
    us_state    = 1;
    __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_FALLING);
  }
  else                        /* ---------- 抓到下降沿 ---------- */
  {
    us_pulseWidth = cap - us_riseTime;   /* 结束时间 - 开始时间 = 高电平宽度 */
    us_done  = 1;             /* 通知主循环：有新数据了 */
    us_busy  = 0;             /* 测量结束，可以开始下一次 */
    us_state = 0;
    __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_RISING);
  }
}

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
