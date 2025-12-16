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
#include "lcd_16x2.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
int mode = 0;
int ldr_val = 0;
int mq135_val = 0;
int ledMatrix[8];
int ldr_matrix[8][8] = {
        {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80},
        {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0xC0},
        {0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x60, 0xE0},
        {0x00, 0x00, 0x00, 0x00, 0x10, 0x30, 0x70, 0xF0},
        {0x00, 0x00, 0x00, 0x08, 0x18, 0x38, 0x78, 0xF8},
        {0x00, 0x00, 0x04, 0x0C, 0x1C, 0x3C, 0x7C, 0xFC},
        {0x00, 0x02, 0x06, 0x0E, 0x1E, 0x3E, 0x7E, 0xFE},
        {0x01, 0x03, 0x07, 0x0F, 0x1F, 0x3F, 0x7F, 0xFF}
};
int maled[10] = {
        0b00111111, // 0
        0b00000110, // 1
        0b01011011, // 2
        0b01001111, // 3
        0b01100110, // 4
        0b01101101, // 5
        0b01111101, // 6
        0b00000111, // 7
        0b01111111, // 8
        0b01101111  // 9
};
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
ADC_HandleTypeDef hadc1;
TIM_HandleTypeDef htim2;
/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//Khởi tạo delay
void Delay_us(uint16_t us) {
    __HAL_TIM_SET_COUNTER(&htim2, 0);  // Reset bộ đếm
    HAL_TIM_Base_Start(&htim2);        // Bắt đầu Timer
    while (__HAL_TIM_GET_COUNTER(&htim2) < us);
    HAL_TIM_Base_Stop(&htim2);         // Dừng Timer nếu muốn
}

void setinput() {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void setouput() {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void sendstart() {
    setouput();
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, RESET);
    HAL_Delay(18);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, SET);
    Delay_us(20);

    //Doi phan hoi tu dht11
    setinput();
    while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == 1) {}; //Doi xuong 0
    while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == 0) {}; //Doi len 1
    while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == 1) {}; //Doi xuong 0
    //Khi da xuong 0 thi 4 bit bat dau chuyen
}

//Doc moi lan 8 bit (1byte)
int DHT11_read_data() {
    int res = 0;
    for (int i = 0; i < 8; i++) {
            while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == 0) {}; //Doi len 1
            Delay_us(50);
            if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == 1) {
                    //Neu da het 50us no van con o muc 2 thi la bit1
                    res = (res << 1) | (1 << 0);
            } else {
                    res = (res << 1) & ~(1 << 0);
            }
            while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == 1) {}; //Doi xuong 0
    }
    return res;
}

//Đọc giá trị kênh ADC
uint16_t readADC(uint32_t channel) {
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = channel;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;

    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
    uint16_t value = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);

    return value;
}

int int_temp = 0, int_hump = 0;
float float_temp = 0, float_hump = 0;
uint8_t check = 0;
int nhietdo_canhbao = 40;

uint32_t lastUpdate = 0;

//Đọc dữ liệu
void read_full_data() {
    sendstart();
    int_hump = DHT11_read_data();
    float_temp = DHT11_read_data();
    int_temp = DHT11_read_data();
    float_hump = DHT11_read_data();
    check = DHT11_read_data();
}

// Chân column (PA3 -> PA10)
#define ROW1_Pin GPIO_PIN_0
#define ROW1_GPIO_Port GPIOB
#define ROW2_Pin GPIO_PIN_1
#define ROW2_GPIO_Port GPIOB
#define ROW3_Pin GPIO_PIN_2
#define ROW3_GPIO_Port GPIOB
#define ROW4_Pin GPIO_PIN_3
#define ROW4_GPIO_Port GPIOB
#define ROW5_Pin GPIO_PIN_4
#define ROW5_GPIO_Port GPIOB
#define ROW6_Pin GPIO_PIN_5
#define ROW6_GPIO_Port GPIOB
#define ROW7_Pin GPIO_PIN_6
#define ROW7_GPIO_Port GPIOB
#define ROW8_Pin GPIO_PIN_7
#define ROW8_GPIO_Port GPIOB

#define COL1_Pin GPIO_PIN_3
#define COL1_GPIO_Port GPIOA
#define COL2_Pin GPIO_PIN_4
#define COL2_GPIO_Port GPIOA
#define COL3_Pin GPIO_PIN_5
#define COL3_GPIO_Port GPIOA
#define COL4_Pin GPIO_PIN_6
#define COL4_GPIO_Port GPIOA
#define COL5_Pin GPIO_PIN_7
#define COL5_GPIO_Port GPIOA
#define COL6_Pin GPIO_PIN_8
#define COL6_GPIO_Port GPIOA
#define COL7_Pin GPIO_PIN_9
#define COL7_GPIO_Port GPIOA
#define COL8_Pin GPIO_PIN_10
#define COL8_GPIO_Port GPIOA

//Mảng lưu trữ các chân hàng
GPIO_TypeDef* rowPorts[8] = {
    ROW1_GPIO_Port, ROW2_GPIO_Port, ROW3_GPIO_Port, ROW4_GPIO_Port,
    ROW5_GPIO_Port, ROW6_GPIO_Port, ROW7_GPIO_Port, ROW8_GPIO_Port
};
uint16_t rowPins[8] = {
    ROW1_Pin, ROW2_Pin, ROW3_Pin, ROW4_Pin,
    ROW5_Pin, ROW6_Pin, ROW7_Pin, ROW8_Pin
};

// Mảng lưu trữ các chân cột
GPIO_TypeDef* colPorts[8] = {
    COL1_GPIO_Port, COL2_GPIO_Port, COL3_GPIO_Port, COL4_GPIO_Port,
    COL5_GPIO_Port, COL6_GPIO_Port, COL7_GPIO_Port, COL8_GPIO_Port
};
uint16_t colPins[8] = {
    COL1_Pin, COL2_Pin, COL3_Pin, COL4_Pin,
    COL5_Pin, COL6_Pin, COL7_Pin, COL8_Pin
};

// Hàm quét LED matrix
void scanLedMatrix() {
    for (int i = 0; i < 24; i++) {
        for (int row = 0; row < 8; row++) {
            // Bật hàng
            HAL_GPIO_WritePin(rowPorts[row], rowPins[row], 1);

            // Dặt dữ liệu cột
            for (int col = 0; col < 8; col++) {
                if ((ledMatrix[row] >> (7 - col)) & 0x01) { //7-col để đảo theo trục Y
                    HAL_GPIO_WritePin(colPorts[col], colPins[col], 0); // Âm chung: Bật LED
                } else {
                    HAL_GPIO_WritePin(colPorts[col], colPins[col], 1); // Âm chung: Tắt LED
                }
            }

            // Tắt hàng và cột
            HAL_GPIO_WritePin(rowPorts[row], rowPins[row], 0);
            for (int col = 0; col < 8; col++) {
                HAL_GPIO_WritePin(colPorts[col], colPins[col], 1);
            }
        }
    }
}

//Led 7 doan
void hienthi(int x) {
    uint8_t chuc = 0;
    uint8_t donvi = 0;
    chuc = x / 10;
    donvi = x % 10;

    //Hang don vi
    HAL_GPIO_WritePin(GPIOB, maled[donvi], 1);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, 1);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, 0);
    HAL_Delay(0);
    HAL_GPIO_WritePin(GPIOB, maled[donvi], 0);

    //Hang chuc
    HAL_GPIO_WritePin(GPIOB, maled[chuc], 1);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, 0);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, 1);
    HAL_Delay(0);
    HAL_GPIO_WritePin(GPIOB, maled[chuc], 0);
}

void scan_Matrix_7(int i) {
    for (int j = 0; j < 6; j++) {
    	// Hiển thị LED 7 đoạn trước
		hienthi(i);  // Hiển thị số i

		// Tạm tắt LED 7 đoạn để không chồng tín hiệu
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, 1);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, 1);

		// Quét LED ma trận 1 lần
		scanLedMatrix();
    }
}

// Menu
void mode_0() {
    Lcd_clear_display();
    Lcd_gotoxy(0, 0);
    Lcd_write_string("1. Khong khi");
    Lcd_gotoxy(0, 1);
    Lcd_write_string("2. Nhiet do");
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    switch (GPIO_Pin) {
        case GPIO_PIN_13: // Nút 1
            if (mode == 0)
                mode = 1;
            else if (mode == 2)
                nhietdo_canhbao++; // Tăng ngưỡng
            break;

        case GPIO_PIN_14: // Nút 2
            if (mode == 0)
                mode = 2;
            else if (mode == 2 && nhietdo_canhbao > 0)
                nhietdo_canhbao--; // Giảm ngưỡng
            break;

        case GPIO_PIN_15: // Nút 3
            mode = 0;
            break;
    }
}

void update_ledMatrix_from_ldr() {
    int index = (ldr_val - 500) * 7 / (3500 - 500);
    index = 7 - index;

    if (index < 0) index = 0;
    if (index > 7) index = 7;

    for (int i = 0; i < 8; i++) {
        ledMatrix[i] = ldr_matrix[index][i];
    }
}
/* USER CODE END 0 */

/**
    * @brief  The application entry point.
    * @retval int
    */
int main(void) {
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
    MX_ADC1_Init();
    MX_TIM2_Init();
    HAL_TIM_Base_Start(&htim2);
    /* USER CODE BEGIN 2 */
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    Lcd_Init();
    Lcd_clear_display();

    read_full_data(); // Đọc DHT11 lần đầu
    while (1) {
        mq135_val = readADC(ADC_CHANNEL_1);
        ldr_val = readADC(ADC_CHANNEL_2);

        scan_Matrix_7(int_hump);
        update_ledMatrix_from_ldr();
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 1);

        if (int_temp >= nhietdo_canhbao || mq135_val >= 4000) {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 0);
        }

        if (mode == 0) {
            Lcd_gotoxy(0, 0);
            Lcd_write_string("1. Khong khi      ");
            Lcd_gotoxy(0, 1);
            Lcd_write_string("2. Nhiet do       ");

        } else if (mode == 1) {
            Lcd_gotoxy(0, 0);
            Lcd_write_string("Khong khi:        ");

            if (mq135_val <= 4000) {
                Lcd_gotoxy(0, 1);
                //Lcd_write_int(mq135_val);
                Lcd_write_string("Tot            ");
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 1);
                if (int_temp >= nhietdo_canhbao) {
                	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 0);
                }
            } else {
                Lcd_gotoxy(0, 1);
                //Lcd_write_int(mq135_val);
                Lcd_write_string("Te            ");
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 0);
            }
        } else if (mode == 2) {
            read_full_data();
            Lcd_gotoxy(0, 0);
            Lcd_write_string("Nhiet do:       ");
            Lcd_gotoxy(10, 0);
            Lcd_write_int(int_temp);
            Lcd_gotoxy(13, 0);
            Lcd_write_string("C     ");
            Lcd_gotoxy(0, 1);
            Lcd_write_string("Canh bao:    ");
            Lcd_gotoxy(10, 1);
            Lcd_write_int(nhietdo_canhbao);
            Lcd_gotoxy(13, 1);
            Lcd_write_string("C       ");

            if (int_temp >= nhietdo_canhbao || mq135_val >= 4000) {
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 0);
            } else {
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 1);
            }
        }
    }
    /* USER CODE END 3 */
}

/**
    * @brief System Clock Configuration
    * @retval None
    */
void SystemClock_Config(void) {
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
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
        | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
        Error_Handler();
    }
}

/**
    * @brief ADC1 Initialization Function
    * @param None
    * @retval None
    */
static void MX_ADC1_Init(void) {
    /* USER CODE BEGIN ADC1_Init 0 */
    /* USER CODE END ADC1_Init 0 */

    ADC_ChannelConfTypeDef sConfig = {0};

    /* USER CODE BEGIN ADC1_Init 1 */
    /* USER CODE END ADC1_Init 1 */

    /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
    */
    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode = DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 1;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) {
        Error_Handler();
    }

    /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
    */
    sConfig.Channel = ADC_CHANNEL_0;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN ADC1_Init 2 */
    /* USER CODE END ADC1_Init 2 */
}

/**
    * @brief TIM2 Initialization Function
    * @param None
    * @retval None
    */
static void MX_TIM2_Init(void) {
    /* USER CODE BEGIN TIM2_Init 0 */
    /* USER CODE END TIM2_Init 0 */

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    /* USER CODE BEGIN TIM2_Init 1 */
    /* USER CODE END TIM2_Init 1 */
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 15;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 4294967295;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM2_Init 2 */
    /* USER CODE END TIM2_Init 2 */
}

/**
    * @brief GPIO Initialization Function
    * @param None
    * @retval None
    */
static void MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    /* USER CODE BEGIN MX_GPIO_Init_1 */
    /* USER CODE END MX_GPIO_Init_1 */

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6
        | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10
        | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_15, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_10
        | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15
        | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6
        | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_RESET);

    /*Configure GPIO pins : PC13 PC14 PC15 */
    GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /*Configure GPIO pins : PA3 PA4 PA5 PA6
    PA7 PA8 PA9 PA10
    PA11 PA12 PA15 */
    GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6
        | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10
        | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pins : PB0 PB1 PB2 PB10
    PB12 PB13 PB14 PB15
    PB3 PB4 PB5 PB6
    PB7 PB8 PB9 */
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_10
        | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15
        | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6
        | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* EXTI interrupt init*/
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

    /* USER CODE BEGIN MX_GPIO_Init_2 */
    /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
    * @brief  This function is executed in case of error occurrence.
    * @retval None
    */
void Error_Handler(void) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1) {
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
void assert_failed(uint8_t *file, uint32_t line) {
        /* USER CODE BEGIN 6 */
        /* User can add his own implementation to report the file name and line number,
             ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
        /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
