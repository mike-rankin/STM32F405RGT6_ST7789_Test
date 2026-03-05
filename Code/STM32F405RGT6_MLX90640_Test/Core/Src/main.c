/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : MLX90640 Thermal Camera - STM32F4 with ST7789V Display
  *
  * Reads 32x24 thermal image from MLX90640 over I2C and renders it on a
  * 280x240 ST7789V LCD using a rainbow colormap. Displays min, max, and
  * centre-point temperatures below the image with a colour gradient bar.
  *
  * Display layout (280 x 240 px):
  *   [ 5px top margin                          ]
  *   [   224 x 168 px thermal image (7x7 px    ]
  *   [   per sensor pixel), centred            ]
  *   [ 4px gap                                 ]
  *   [ 224 x 8px colour gradient bar           ]
  *   [ 4px gap                                 ]
  *   [ Min / Max / Centre temperature labels   ]
  *   [ 4px gap                                 ]
  *   [ Emissivity & refresh rate label         ]
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "st7789v.h"
#include "fonts.h"
#include "MLX90640_API.h"
#include "MLX90640_I2C_Driver.h"

/* ─── MLX90640 configuration ─────────────────────────────────────────── */
#define MLX90640_ADDR       0x33    /* Default 7-bit I2C address           */
#define EMISSIVITY          0.95f   /* Emissivity (0.95 suits most objects) */

/*
 * Refresh rate register values (write to MLX90640_SetRefreshRate):
 *   0x00 = 0.5 Hz   0x01 = 1 Hz   0x02 = 2 Hz   0x03 = 4 Hz
 *   0x04 = 8 Hz     0x05 = 16 Hz  0x06 = 32 Hz  0x07 = 64 Hz
 * At 400 kHz I2C, 4 Hz is reliable on STM32F4.
 */
#define MLX90640_REFRESH    0x03    /* 4 Hz                                */

/* ─── Display geometry ───────────────────────────────────────────────── */
#define BLOCK_SIZE          7       /* Pixels on screen per thermal pixel  */
#define IMG_W               (32 * BLOCK_SIZE)       /* 224 px              */
#define IMG_H               (24 * BLOCK_SIZE)       /* 168 px              */
#define IMG_X               ((ST7789V_WIDTH - IMG_W) / 2)  /* 28 px        */
#define IMG_Y               5

#define CBAR_Y              (IMG_Y + IMG_H + 4)     /* Colour bar top: 177 */
#define CBAR_H              8                        /* Colour bar height   */

#define TEXT_Y              (CBAR_Y + CBAR_H + 4)   /* Labels top: 189     */

/* ─── HAL handles ────────────────────────────────────────────────────── */
I2C_HandleTypeDef hi2c1;
SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;

/* ─── MLX90640 data buffers ──────────────────────────────────────────── */
static uint16_t     eeData[832];
static paramsMLX90640 mlx90640Params;
static uint16_t     frameData[834];
static float        tempValues[768];    /* 32 * 24 temperature array       */

/* ─── Private function prototypes ────────────────────────────────────── */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);

static int          thermal_init(void);
static void         thermal_read_full_frame(void);
static uint16_t     temp_to_color565(float temp, float minT, float maxT);
static void         draw_thermal_image(float minT, float maxT);
static void         draw_color_bar(float minT, float maxT);
static void         draw_temperature_labels(float minT, float maxT);
static void         find_min_max(float *arr, int len, float *minOut, float *maxOut);


/* ═══════════════════════════════════════════════════════════════════════
 *  MAIN
 * ═══════════════════════════════════════════════════════════════════════ */
int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_DMA_Init();
    MX_I2C1_Init();
    MX_SPI1_Init();

    /* ── Display init ─────────────────────────────────────────────────── */
    ST7789V_Init();
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);   /* Backlight ON  */
    ST7789V_Fill_Color(BLACK);

    /* ── Startup splash ───────────────────────────────────────────────── */
    ST7789V_WriteString(60, 90,  "MLX90640",        Font_16x26, WHITE,  BLACK);
    ST7789V_WriteString(40, 120, "Thermal Camera",  Font_11x18, CYAN,   BLACK);
    ST7789V_WriteString(25, 148, "Initialising...", Font_11x18, YELLOW, BLACK);

    /* ── Sensor init ──────────────────────────────────────────────────── */
    if (thermal_init() != 0)
    {
        /* Show error and halt */
        ST7789V_Fill_Color(BLACK);
        ST7789V_WriteString(20, 100, "SENSOR ERROR",    Font_16x26, RED,   BLACK);
        ST7789V_WriteString(10, 135, "Check I2C wiring",Font_11x18, WHITE, BLACK);
        while (1) { /* halt */ }
    }

    ST7789V_Fill_Color(BLACK);

    /* ── Static UI chrome (drawn once) ───────────────────────────────── */
    /* Thin border around the image area */
    ST7789V_DrawRectangle(IMG_X - 1, IMG_Y - 1,
                          IMG_X + IMG_W, IMG_Y + IMG_H, GRAY);


    /* ── Main loop ────────────────────────────────────────────────────── */
    while (1)
    {
        /* Read a complete interleaved frame (both subpages) */
        thermal_read_full_frame();

        /* Find dynamic temperature range */
        float minT, maxT;
        find_min_max(tempValues, 768, &minT, &maxT);

        /* Render */
        draw_thermal_image(minT, maxT);
        draw_color_bar(minT, maxT);
        draw_temperature_labels(minT, maxT);
    }
}


/* ═══════════════════════════════════════════════════════════════════════
 *  THERMAL FUNCTIONS
 * ═══════════════════════════════════════════════════════════════════════ */

/**
 * @brief  Initialise the MLX90640: dump EEPROM, extract calibration params,
 *         set chess mode and desired refresh rate.
 * @retval 0 on success, non-zero on failure.
 */
static int thermal_init(void)
{
    /* Read EEPROM calibration data */
    if (MLX90640_DumpEE(MLX90640_ADDR, eeData) != 0)
        return -1;

    /* Extract calibration parameters from EEPROM */
    if (MLX90640_ExtractParameters(eeData, &mlx90640Params) != 0)
        return -2;

    /* Chess mode: each subpage covers a checkerboard pattern of pixels.
     * This gives a more uniform image than interleaved mode when displaying
     * single subpages.                                                     */
    MLX90640_SetChessMode(MLX90640_ADDR);

    /* Set the sensor refresh rate */
    MLX90640_SetRefreshRate(MLX90640_ADDR, MLX90640_REFRESH);

    return 0;
}

/**
 * @brief  Read two consecutive subpages from the MLX90640 and compute
 *         the full 32x24 temperature array in @p tempValues[].
 *
 * The MLX90640 alternates subpage 0 and subpage 1 on each measurement.
 * MLX90640_CalculateTo() only writes pixels belonging to the current
 * subpage, so after two calls the entire 768-pixel array is filled.
 */
static void thermal_read_full_frame(void)
{
    for (int subpage = 0; subpage < 2; subpage++)
    {
        int status = MLX90640_GetFrameData(MLX90640_ADDR, frameData);
        if (status < 0)
            continue;   /* Skip bad reads; previous subpage data persists */

        /* Reflected temperature: ambient - 8 °C is a common approximation */
        float tr = MLX90640_GetTa(frameData, &mlx90640Params) - 8.0f;

        /* Calculate object temperatures for every pixel in this subpage */
        MLX90640_CalculateTo(frameData, &mlx90640Params,
                             EMISSIVITY, tr, tempValues);
    }
}

/**
 * @brief  Compute the display temperature range using mean ± 2.5 × stddev.
 *
 * Using the raw min/max means a single hot or cold outlier pixel
 * (e.g. a specular reflection or sensor noise) compresses everything else
 * into a narrow band of the colour map, washing out fine detail.
 * Clamping to ±2.5σ keeps ~99% of pixels within the colour range while
 * ignoring extremes, so a hand at 34 °C against a 22 °C desk uses most of
 * the blue→red gradient rather than just one corner of it.
 */
static void find_min_max(float *arr, int len, float *minOut, float *maxOut)
{
    /* Pass 1: mean */
    double sum = 0.0;
    for (int i = 0; i < len; i++) sum += arr[i];
    double mean = sum / len;

    /* Pass 2: variance */
    double var = 0.0;
    for (int i = 0; i < len; i++)
    {
        double d = arr[i] - mean;
        var += d * d;
    }
    double stddev = sqrt(var / len);

    /* Clamp range to mean ± 2.5 σ */
    *minOut = (float)(mean - 2.5 * stddev);
    *maxOut = (float)(mean + 2.5 * stddev);
}


/* ═══════════════════════════════════════════════════════════════════════
 *  COLOUR MAPPING
 * ═══════════════════════════════════════════════════════════════════════ */

/**
 * @brief  Map a temperature value to an RGB565 colour using a rainbow
 *         gradient:  blue → cyan → green → yellow → red.
 *
 * @param  temp  Temperature to convert.
 * @param  minT  Temperature that maps to blue (cold).
 * @param  maxT  Temperature that maps to red  (hot).
 * @retval RGB565 colour word.
 */
static uint16_t temp_to_color565(float temp, float minT, float maxT)
{
    /* Clamp */
    if (temp <= minT) temp = minT;
    if (temp >= maxT) temp = maxT;

    /* Normalise to 0..255 */
    float ratio = (temp - minT) / (maxT - minT);
    uint8_t idx  = (uint8_t)(ratio * 255.0f);

    uint8_t r, g, b;

    if (idx < 64)
    {
        /* Blue → Cyan */
        r = 0;
        g = (uint8_t)(idx * 4);
        b = 255;
    }
    else if (idx < 128)
    {
        /* Cyan → Green */
        r = 0;
        g = 255;
        b = (uint8_t)(255 - (idx - 64) * 4);
    }
    else if (idx < 192)
    {
        /* Green → Yellow */
        r = (uint8_t)((idx - 128) * 4);
        g = 255;
        b = 0;
    }
    else
    {
        /* Yellow → Red */
        r = 255;
        g = (uint8_t)(255 - (idx - 192) * 4);
        b = 0;
    }

    /* Pack into RGB565: RRRRRGGGGGGBBBBB */
    return (uint16_t)(((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3));
}


/* ═══════════════════════════════════════════════════════════════════════
 *  DISPLAY RENDERING
 * ═══════════════════════════════════════════════════════════════════════ */

/**
 * @brief  Render the 32x24 thermal image with bilinear interpolation.
 *
 * Rather than painting one solid 7x7 block per sensor pixel (nearest-
 * neighbour), we iterate over every screen pixel within the image area and
 * compute a weighted blend of the four surrounding sensor pixels. This
 * eliminates the "grid of coloured squares" look and produces smooth
 * transitions between pixels so that shapes such as a hand become clearly
 * recognisable.
 *
 * Screen pixel (px, py) maps to sensor-space float coordinates:
 *   sx = px * 31 / (IMG_W-1)   (0.0 .. 31.0)
 *   sy = py * 23 / (IMG_H-1)   (0.0 .. 23.0)
 *
 * If your image appears flipped vertically, change sy to:
 *   sy = (IMG_H - 1 - py) * 23.0f / (IMG_H - 1)
 */
static void draw_thermal_image(float minT, float maxT)
{
    const float sx_scale = 31.0f / (float)(IMG_W - 1);
    const float sy_scale = 23.0f / (float)(IMG_H - 1);

    for (int py = 0; py < IMG_H; py++)
    {
        float sy  = (float)py * sy_scale;
        int   y0  = (int)sy;
        int   y1  = (y0 < 23) ? y0 + 1 : 23;
        float fy  = sy - (float)y0;
        float ify = 1.0f - fy;

        for (int px = 0; px < IMG_W; px++)
        {
            float sx  = (float)px * sx_scale;
            int   x0  = (int)sx;
            int   x1  = (x0 < 31) ? x0 + 1 : 31;
            float fx  = sx - (float)x0;
            float ifx = 1.0f - fx;

            /* Bilinear blend of the four surrounding sensor pixels */
            float temp =
                ifx * ify * tempValues[y0 * 32 + x0] +
                 fx * ify * tempValues[y0 * 32 + x1] +
                ifx *  fy * tempValues[y1 * 32 + x0] +
                 fx *  fy * tempValues[y1 * 32 + x1];

            uint16_t color = temp_to_color565(temp, minT, maxT);
            ST7789V_DrawPixel((uint16_t)(IMG_X + px),
                              (uint16_t)(IMG_Y + py),
                              color);
        }
    }
}

/**
 * @brief  Draw a horizontal colour gradient bar (cold → hot) spanning the
 *         same width as the thermal image.  Labels minT and maxT are
 *         printed just above the temperature text row.
 */
static void draw_color_bar(float minT, float maxT)
{
    for (int x = 0; x < IMG_W; x++)
    {
        /* Compute the temperature this x position represents */
        float t     = minT + (maxT - minT) * ((float)x / (float)(IMG_W - 1));
        uint16_t c  = temp_to_color565(t, minT, maxT);

        /* Draw a vertical slice of the gradient bar */
        ST7789V_Fill((uint16_t)(IMG_X + x),  CBAR_Y,
                     (uint16_t)(IMG_X + x),  CBAR_Y + CBAR_H - 1,
                     c);
    }
}

/**
 * @brief  Print min, max, and centre-pixel temperature values below the
 *         colour bar.
 */
static void draw_temperature_labels(float minT, float maxT)
{
    char buf[48];

    /* Centre pixel of the sensor array (row 11, col 15  →  index 367) */
    float centreT = tempValues[11 * 32 + 15];

    /* ── Row 1: Min and Max ─────────────────────────────────────────── */
    /* Clear the label area first to avoid ghost characters */
    ST7789V_Fill(0, TEXT_Y, ST7789V_WIDTH - 1, TEXT_Y + 18, BLACK);

    snprintf(buf, sizeof(buf), "Mn:%.1fC  Mx:%.1fC", (double)minT, (double)maxT);
    /* Centre the string: Font_11x18 is 11 px wide per char             */
    uint16_t str_w = (uint16_t)(strlen(buf) * 11);
    uint16_t str_x = (ST7789V_WIDTH > str_w) ?
                     (ST7789V_WIDTH - str_w) / 2 : 0;
    ST7789V_WriteString(str_x, TEXT_Y, buf, Font_11x18, WHITE, BLACK);

    /* ── Row 2: Centre temperature ───────────────────────────────────── */
    ST7789V_Fill(0, TEXT_Y + 20, ST7789V_WIDTH - 1, TEXT_Y + 38, BLACK);

    snprintf(buf, sizeof(buf), "Centre: %.1f C", (double)centreT);
    str_w = (uint16_t)(strlen(buf) * 11);
    str_x = (ST7789V_WIDTH > str_w) ? (ST7789V_WIDTH - str_w) / 2 : 0;
    ST7789V_WriteString(str_x, TEXT_Y + 20, buf, Font_11x18, CYAN, BLACK);
}



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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 10;
  RCC_OscInitStruct.PLL.PLLN = 168;
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
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
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
     hi2c1.Init.ClockSpeed = 100000;
     //  hi2c1.Init.ClockSpeed = 400000;

  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  hspi1.Init.Direction = SPI_DIRECTION_1LINE;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LCD_BL_Pin|SPI_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, TFT_CS_Pin|DC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LCD_BL_Pin SPI_CS_Pin */
  GPIO_InitStruct.Pin = LCD_BL_Pin|SPI_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : RST_Pin */
  GPIO_InitStruct.Pin = RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TFT_CS_Pin DC_Pin */
  GPIO_InitStruct.Pin = TFT_CS_Pin|DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
