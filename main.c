/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */
#define LCD_ADDR   (0x27 << 1)   //
const float LOW_T  = 30.0f;      //
const float HIGH_T = 70.0f;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);

/* USER CODE BEGIN 0 */

static void leds_off(void){
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2, GPIO_PIN_RESET);
}
static void led_y_on(void){ HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET); }
static void led_g_on(void){ HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET); }
static void led_r_on(void){ HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET); }


static void sensor_leds_off(void){
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);
}
static void sensor_led_set_one(uint8_t idx){

  sensor_leds_off();
  switch(idx){
    case 0: HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET); break;
    case 1: HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET); break;
    case 2: HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET); break;
    default: break;
  }
}

/**************************** ADC helper *************************************/
static uint32_t adc_read_one(void){
  HAL_ADC_PollForConversion(&hadc1, 10);
  return HAL_ADC_GetValue(&hadc1);   // 0..4095 (12-bit)
}

/**************************** LCD I2C (PCF8574) ******************************/
static void lcd_i2c_send(uint8_t b){ HAL_I2C_Master_Transmit(&hi2c1, LCD_ADDR, &b, 1, 50); }
static void lcd_send_nibble(uint8_t nib, uint8_t rs){
  uint8_t BL=0x08, EN=0x04, RS=rs?0x01:0x00, d=(nib&0x0F)<<4;
  lcd_i2c_send(d|BL|RS|EN); lcd_i2c_send(d|BL|RS);
}
static void lcd_send_byte(uint8_t b,uint8_t rs){ lcd_send_nibble(b>>4,rs); lcd_send_nibble(b&0x0F,rs); }
static void lcd_cmd(uint8_t c){ lcd_send_byte(c,0); HAL_Delay(2); }
static void lcd_data(uint8_t d){ lcd_send_byte(d,1); }
static void lcd_init(void){
  HAL_Delay(50);
  lcd_send_nibble(0x03,0); HAL_Delay(5);
  lcd_send_nibble(0x03,0); HAL_Delay(5);
  lcd_send_nibble(0x03,0); HAL_Delay(1);
  lcd_send_nibble(0x02,0); HAL_Delay(1);
  lcd_cmd(0x28); lcd_cmd(0x0C); lcd_cmd(0x06); lcd_cmd(0x01); HAL_Delay(2);
}
static void lcd_set_cursor(uint8_t col,uint8_t row){ static const uint8_t base[]={0x80,0xC0}; lcd_cmd(base[row]+col); }
static void lcd_print(const char* s){ while(*s) lcd_data((uint8_t)*s++); }
static void lcd_clear_lines(void){ lcd_set_cursor(0,0); lcd_print("                "); lcd_set_cursor(0,1); lcd_print("                "); }

static void lcd_show_msg(float pct){
  lcd_clear_lines();
  if(pct<LOW_T){ lcd_set_cursor(0,0); lcd_print("Lumina SLABA");      lcd_set_cursor(0,1); lcd_print("LED: Galben"); }
  else if(pct<=HIGH_T){ lcd_set_cursor(0,0); lcd_print("Lumina FAVORABILA"); lcd_set_cursor(0,1); lcd_print("LED: Verde"); }
  else { lcd_set_cursor(0,0); lcd_print("Lumina PUTERNICA"); lcd_set_cursor(0,1); lcd_print("LED: Rosu"); }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();

  /* USER CODE BEGIN 2 */
  HAL_ADC_Start(&hadc1);
  leds_off();
  sensor_leds_off();

  lcd_init();
  lcd_set_cursor(0,0); lcd_print("Sistem lumina");
  lcd_set_cursor(0,1); lcd_print("Pornit");
  HAL_Delay(600);
  /* USER CODE END 2 */

  while (1)
  {

    uint32_t v0 = adc_read_one();
    uint32_t v1 = adc_read_one();
    uint32_t v2 = adc_read_one();


    uint32_t avg = (v0 + v1 + v2) / 3;
    float pct = (avg * 100.0f) / 4095.0f;
    pct = 100.0f - pct;


    leds_off();
    if (pct < LOW_T)        led_y_on();
    else if (pct <= HIGH_T) led_g_on();
    else                    led_r_on();


    uint32_t best_val = v0;
    uint8_t  best_idx = 0;
    if (v1 < best_val) { best_val = v1; best_idx = 1; }
    if (v2 < best_val) { best_val = v2; best_idx = 2; }
    sensor_led_set_one(best_idx);


    lcd_show_msg(pct);

    HAL_Delay(200);
  }
}

/* System / Peripherals init (generate de CubeMX) ---------------------------*/
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) { Error_Handler(); }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) { Error_Handler(); }
}

static void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  if (HAL_ADC_Init(&hadc1) != HAL_OK) { Error_Handler(); }

  sConfig.Channel = ADC_CHANNEL_0;                // PA0
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { Error_Handler(); }

  sConfig.Channel = ADC_CHANNEL_1;                // PA1
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { Error_Handler(); }

  sConfig.Channel = ADC_CHANNEL_2;                // PA2
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { Error_Handler(); }
}

static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) { Error_Handler(); }
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();


  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);


  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}
#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) { (void)file; (void)line; }
#endif
