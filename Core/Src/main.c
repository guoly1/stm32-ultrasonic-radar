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

/* ===================== 雷达扫描参数 ===================== */
#define SCAN_STEP_DEG    10     /* 角度步进（度） */
#define SCAN_POINTS      19     /* 0°~180°，共 19 个采样点 */
#define SCAN_SETTLE_MS   100    /* 舵机到位后等它机械停稳的时间 */
#define SCAN_TRIG_GAP_MS 60     /* 超声波两次触发的最小间隔（HC-SR04 要求 ≥60ms） */
#define US_TIMEOUT_MS    40     /* 单次测距超时 */
#define US_MEDIAN_N      3      /* 每个角度的采样次数，取中值 */

#define SERVO_PULSE_MIN  70     /* 0°   对应的舵机脉宽（收窄过的安全范围） */
#define SERVO_PULSE_MAX  230    /* 180° 对应的舵机脉宽 */

/* 角度(0~180) → 舵机脉宽(70~230) */
static uint16_t angleToPulse(uint16_t deg)
{
  return (uint16_t)(SERVO_PULSE_MIN +
                    (uint32_t)deg * (SERVO_PULSE_MAX - SERVO_PULSE_MIN) / 180);
}

/* 三个数取中值 —— 中值滤波的核心，野值会被自动扔掉
 *
 * 为什么用中值不用平均值？
 *   三次测得 30 / 56 / 31，平均 = 39（❌ 现实里不存在 39cm 的东西）
 *                       中值 = 31（✅ 56 是野值，被剔除且不污染结果）
 * 只要野值不超过一半，中值滤波就能完全剔除它。
 *
 * 3 次比较完成三个数排序，比调 qsort 快得多，也没有函数调用开销。
 */
static uint16_t median3(uint16_t a, uint16_t b, uint16_t c)
{
  uint16_t t;
  if (a > b) { t = a; a = b; b = t; }   /* 让 a ≤ b */
  if (b > c) { t = b; b = c; c = t; }   /* 让 b ≤ c，此时 c 是最大值 */
  if (a > b) { t = a; a = b; b = t; }   /* 再排一次 a、b */
  return b;                              /* 中间那个就是中值 */
}

/* ===================== 雷达图绘制 ===================== */

#define RADAR_CX      64      /* 圆心 x：屏幕水平中心 */
#define RADAR_CY      63      /* 圆心 y：屏幕底边（雷达站在屏幕底部往上看） */
#define RADAR_RADIUS  58      /* 最大显示半径（像素） */
#define RADAR_MAX_CM  100     /* 量程：100cm 映射到 RADAR_RADIUS 像素 */

/* 正弦表 ×1000，索引 = 角度 / 10（0°,10°,...,180°）
 *
 * 为什么不用 sinf()/cosf()？
 *   1. F103C8T6 没有 FPU，浮点三角函数是纯软件实现，一次几十微秒
 *   2. 我们只有 19 个固定角度，查表是 O(1)，而且完全确定
 *   3. 省掉 math.h 和 libm，代码更小
 */
static const int16_t sinTab[SCAN_POINTS] = {
      0,  174,  342,  500,  643,  766,  866,  940,  985, 1000,
    985,  940,  866,  766,  643,  500,  342,  174,    0
};

/* 余弦表：cos(θ) = sin(90° - θ) */
static const int16_t cosTab[SCAN_POINTS] = {
   1000,  985,  940,  866,  766,  643,  500,  342,  174,    0,
   -174, -342, -500, -643, -766, -866, -940, -985,-1000
};

/* 带裁剪的画点 —— 坐标必须先判断再画，否则越界会写坏 OLED 显存 */
static void radarPixel(int x, int y)
{
  if (x < 0 || x > 127 || y < 0 || y > 63) return;
  OLED_SetPixel((uint8_t)x, (uint8_t)y, OLED_COLOR_NORMAL);
}

/* Bresenham 画线（整数运算，带裁剪） */
static void radarLine(int x0, int y0, int x1, int y1)
{
  int dx = (x1 > x0) ? (x1 - x0) : (x0 - x1);
  int dy = (y1 > y0) ? (y1 - y0) : (y0 - y1);
  int sx = (x0 < x1) ? 1 : -1;
  int sy = (y0 < y1) ? 1 : -1;
  int err = dx - dy;
  int e2;

  for (;;)
  {
    radarPixel(x0, y0);
    if (x0 == x1 && y0 == y1) break;
    e2 = 2 * err;
    if (e2 > -dy) { err -= dy; x0 += sx; }
    if (e2 <  dy) { err += dx; y0 += sy; }
  }
}

/* 极坐标 → 屏幕坐标
 *   rad = 半径(像素)，idx = 角度索引(0~18)
 *   θ=90° 永远画在正上方；y 用减号是因为屏幕 y 轴朝下
 *
 * RADAR_FLIP_X = 水平翻转开关（屏幕是俯视图，要和舵机实际指向对上）
 *    1 → 角度索引 0 画在屏幕【左边】
 *   -1 → 角度索引 0 画在屏幕【右边】  ← 舵机安装方向相反时用这个
 *
 * ⚠️ 如果发现雷达图左右和实物相反，只改这一个数字，其他代码不用动。
 */
#define RADAR_FLIP_X  (-1)

static void radarPoint(int rad, int idx, int *px, int *py)
{
  *px = RADAR_CX - (RADAR_FLIP_X * rad * cosTab[idx]) / 1000;
  *py = RADAR_CY - (rad * sinTab[idx]) / 1000;
}

/* 用 19 段短直线拼出一整条弧（OLED 驱动只有画圆，画不了半圆弧） */
static void radarArc(int rad)
{
  int i, px, py, x, y;
  radarPoint(rad, 0, &px, &py);
  for (i = 1; i < SCAN_POINTS; i++)
  {
    radarPoint(rad, i, &x, &y);
    radarLine(px, py, x, y);
    px = x;
    py = y;
  }
}

/* 画一整帧雷达图。sweepIdx = 当前扫描到的角度索引 */
static void radarDraw(const uint16_t *dist, uint8_t sweepIdx)
{
  int i, x, y, r;

  OLED_NewFrame();

  /* 1) 三条距离刻度弧：1/3、2/3、满量程 */
  radarArc(RADAR_RADIUS / 3);
  radarArc(RADAR_RADIUS * 2 / 3);
  radarArc(RADAR_RADIUS);

  /* 2) 两条边界线（0° 和 180°） */
  radarPoint(RADAR_RADIUS, 0, &x, &y);
  radarLine(RADAR_CX, RADAR_CY, x, y);
  radarPoint(RADAR_RADIUS, SCAN_POINTS - 1, &x, &y);
  radarLine(RADAR_CX, RADAR_CY, x, y);

  /* 3) 目标点：距离 → 半径，再转成屏幕坐标 */
  for (i = 0; i < SCAN_POINTS; i++)
  {
    uint16_t d = dist[i];
    if (d == 0 || d > RADAR_MAX_CM) continue;   /* 0 = 无效，超量程不显示 */

    r = (int)d * RADAR_RADIUS / RADAR_MAX_CM;
    radarPoint(r, i, &x, &y);

    /* 画 2×2 的小方块，比单个像素显眼得多 */
    radarPixel(x,     y);
    radarPixel(x + 1, y);
    radarPixel(x,     y + 1);
    radarPixel(x + 1, y + 1);
  }

  /* 4) 扫描线 + 两条余晖（长度递减，形成"拖尾"效果） */
  radarPoint(RADAR_RADIUS, sweepIdx, &x, &y);
  radarLine(RADAR_CX, RADAR_CY, x, y);

  if (sweepIdx >= 2)
  {
    radarPoint(RADAR_RADIUS * 2 / 3, sweepIdx - 1, &x, &y);
    radarLine(RADAR_CX, RADAR_CY, x, y);
    radarPoint(RADAR_RADIUS / 3, sweepIdx - 2, &x, &y);
    radarLine(RADAR_CX, RADAR_CY, x, y);
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
  uint8_t  page         = 0;      /* 当前页面 0=雷达图 1=绘图测试 2=按键说明 */
  uint8_t  pageDrawn    = 0xFF;   /* 已画过的页面；0xFF 表示还没画，强制首帧绘制 */
  uint8_t  radarDirty   = 1;      /* 1 = 雷达图需要重绘（有新数据或刚切到该页） */
  uint32_t lastKeyTick  = 0;
  uint32_t lastBeatTick = 0;
  uint32_t lastTrigTick = 0;      /* 上一次触发超声波测距的时间 */

  /* --- 雷达扫描状态变量 --- */
  uint16_t scanDist[SCAN_POINTS];   /* 各角度的距离(cm)，0 = 无效 */
  uint8_t  scanIdx     = 0;         /* 当前角度索引 0 ~ 18 */
  int8_t   scanDir     = 1;         /* +1 递增 / -1 递减 */
  uint8_t  scanPhase   = 0;         /* 0 = 等舵机稳定，1 = 测距中 */
  uint32_t scanTick    = 0;         /* 进入当前阶段的时间戳 */
  uint16_t usSamples[US_MEDIAN_N];  /* 本角度的 3 次采样值 */
  uint8_t  usSampleCnt = 0;

  /* 上电先把舵机摆到起始角度，并清空数据表 */
  memset(scanDist, 0, sizeof(scanDist));
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, angleToPulse(0));
  scanTick = HAL_GetTick();

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

    /* ---------- 2. 页面切换 ----------
     * 0 号页是雷达图，由后面的"雷达刷新"负责（它要随数据变化持续重画）；
     * 1/2 号页是静态页，画一次就不用再管了。
     */
    if (page != pageDrawn)
    {
      pageDrawn = page;

      if (page == 0)
      {
        radarDirty = 1;              /* 切回雷达页，立刻重画一帧 */
      }
      else
      {
        OLED_NewFrame();

        if (page == 1)               /* 绘图测试页 */
        {
          OLED_DrawLine(0, 0, 127, 63, OLED_COLOR_NORMAL);
          OLED_DrawLine(127, 0, 0, 63, OLED_COLOR_NORMAL);
          OLED_DrawCircle(64, 32, 24, OLED_COLOR_NORMAL);
          OLED_DrawFilledCircle(64, 32, 3, OLED_COLOR_NORMAL);
        }
        else                         /* 按键说明页 */
        {
          OLED_PrintString(6, 6,  "KEY1: next", &font16x16, OLED_COLOR_NORMAL);
          OLED_PrintString(6, 26, "KEY2: prev", &font16x16, OLED_COLOR_NORMAL);
        }

        OLED_ShowFrame();
      }
    }

    /* ---------- 3. 雷达扫描状态机（舵机 + 超声波联动，全程非阻塞） ----------
     *
     *  为什么用状态机而不是 for 循环 + HAL_Delay？
     *  for 循环写法：转舵机 → 死等100ms → 死等测距 → 死等推屏 → 下一个角度
     *                每一圈 CPU 有几秒钟纯属空转，按键不响应、心跳不闪。
     *  状态机写法：每圈循环只花几微秒判断"到时间了没"，
     *               没到就直接跳过继续往下跑，CPU 99% 时间空闲待命。
     */
    switch (scanPhase)
    {
      /* ===== 阶段 0：舵机已转向目标角度，等它机械停稳 ===== */
      case 0:
        if (HAL_GetTick() - scanTick >= SCAN_SETTLE_MS)
        {
          usSampleCnt  = 0;
          /* 把"上次触发时间"往回推，让第一次触发能立刻发生 */
          lastTrigTick = HAL_GetTick() - SCAN_TRIG_GAP_MS;
          scanPhase    = 1;
        }
        break;

      /* ===== 阶段 1：在同一角度连测 3 次，取中值 ===== */
      case 1:
      {
        uint8_t sampleReady = 0;

        /* --- 触发一次测距（两次触发之间必须隔够 60ms，否则会收到上一次的余波）--- */
        if (!us_busy && (HAL_GetTick() - lastTrigTick >= SCAN_TRIG_GAP_MS))
        {
          lastTrigTick = HAL_GetTick();
          us_busy  = 1;      /* 标记"测量中"，防止重复触发 */
          us_state = 0;      /* 状态机复位：从"等上升沿"开始 */
          us_done  = 0;

          __HAL_TIM_SET_COUNTER(&htim1, 0);
          __HAL_TIM_SET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_RISING);

          /* TRIG：拉高 ≥10µs 再拉低。@72MHz 下这个空循环约 20µs */
          HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_SET);
          for (volatile uint16_t i = 0; i < 100; i++) { __NOP(); }
          HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);
        }

        /* --- 收集本次结果（成功 或 超时，两种都算"这一次采样结束"）--- */
        if (us_done)
        {
          us_done = 0;
          /* 距离(cm) = 脉宽(µs) × 0.034 ÷ 2 = 脉宽 × 17 ÷ 1000，用整数避免浮点 */
          usSamples[usSampleCnt] = (uint16_t)(us_pulseWidth * 17UL / 1000UL);
          sampleReady = 1;
        }
        else if (us_busy && (HAL_GetTick() - lastTrigTick >= US_TIMEOUT_MS))
        {
          us_busy  = 0;
          us_state = 0;
          __HAL_TIM_SET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_RISING);
          usSamples[usSampleCnt] = 0;      /* 0 表示这次没测到 */
          sampleReady = 1;
        }

        /* --- 3 次都测完了：取中值 → 存表 → 换下一个角度 --- */
        if (sampleReady)
        {
          usSampleCnt++;

          if (usSampleCnt >= US_MEDIAN_N)
          {
            /* 中值滤波：野值在这里被扔掉 */
            scanDist[scanIdx] = median3(usSamples[0], usSamples[1], usSamples[2]);

            printf("angle=%3u  d=%3u cm\r\n", scanIdx * SCAN_STEP_DEG, scanDist[scanIdx]);

            radarDirty = 1;      /* 有新数据了，通知下面的雷达刷新重画一帧 */

            /* 推进角度：走到两端就反向，实现来回扫描 */
            if (scanDir > 0 && scanIdx >= SCAN_POINTS - 1)   scanDir = -1;
            else if (scanDir < 0 && scanIdx == 0)            scanDir =  1;
            else                                             scanIdx = (uint8_t)(scanIdx + scanDir);

            /* 舵机转向新角度，回到"等稳定"阶段，开始下一轮 */
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, angleToPulse(scanIdx * SCAN_STEP_DEG));
            scanTick  = HAL_GetTick();
            scanPhase = 0;
          }
        }
        break;
      }
    }

    /* ---------- 4. 雷达刷新：只在需要时重画 ----------
     * 每拿到一个新的距离值才重画一帧（约 280ms 一次），而不是每圈循环都画。
     * 整帧推屏要 20~30ms，每圈都刷会把主循环拖垮。
     */
    if (page == 0 && radarDirty)
    {
      radarDirty = 0;
      radarDraw(scanDist, scanIdx);
      OLED_ShowFrame();
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
