/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : ads1220.c
  * @brief          : ADS1220 ADC Driver
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "ads1220.h"

/* External variables --------------------------------------------------------*/
extern SPI_HandleTypeDef hspi2;

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Read a register from ADS1220
  * @param  reg_addr: Register address (0-3)
  * @retval Register value
  */
uint8_t ADS1220_ReadRegister(uint8_t reg_addr) {
    uint8_t command = ADS1220_CMD_RREG | (reg_addr << 2);
    uint8_t reg_value = 0;

    // 1. CSをLowにしてデバイスをアクティブにする
    HAL_GPIO_WritePin(ADC_CS_GPIO_Port, ADC_CS_Pin, GPIO_PIN_RESET);

    // 2. RREGコマンドを送信
    HAL_SPI_Transmit(&hspi2, &command, 1, HAL_MAX_DELAY);

    // 3. レジスタの内容を受信（マイコンからクロックを送る必要があるためReceiveを使用）
    HAL_SPI_Receive(&hspi2, &reg_value, 1, HAL_MAX_DELAY);

    // 4. CSをHighにして通信終了
    HAL_GPIO_WritePin(ADC_CS_GPIO_Port, ADC_CS_Pin, GPIO_PIN_SET);

    return reg_value;
}

/**
  * @brief  Write a register to ADS1220
  * @param  reg_addr: Register address (0-3)
  * @param  value: Value to write
  * @retval None
  */
void ADS1220_WriteRegister(uint8_t reg_addr, uint8_t value) {
    uint8_t command = ADS1220_CMD_WREG | (reg_addr << 2);
    HAL_GPIO_WritePin(ADC_CS_GPIO_Port, ADC_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, &command, 1, HAL_MAX_DELAY);
    HAL_SPI_Transmit(&hspi2, &value, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(ADC_CS_GPIO_Port, ADC_CS_Pin, GPIO_PIN_SET);
}

/**
  * @brief  Read ADC conversion data from ADS1220
  * @retval 24-bit signed conversion result
  */
int32_t ADS1220_ReadData(void) {
    uint8_t data[3] = {0};

    // CSをLowにする
    HAL_GPIO_WritePin(ADC_CS_GPIO_Port, ADC_CS_Pin, GPIO_PIN_RESET);

    // 24ビット（3バイト）分を受信
    // ADS1220はDRDYがLowの時にクロックを送ればデータを出力します
    HAL_SPI_Receive(&hspi2, data, 3, HAL_MAX_DELAY);

    // CSをHighにする
    HAL_GPIO_WritePin(ADC_CS_GPIO_Port, ADC_CS_Pin, GPIO_PIN_SET);

    // 3バイトを1つの32ビット変数に結合
    int32_t result = ((int32_t)data[0] << 16) | ((int32_t)data[1] << 8) | (int32_t)data[2];

    // 24ビットの符号拡張（マイナス値を正しく扱う処理）
    if (result & 0x00800000) {
        result |= 0xFF000000;
    }

    return result;
}

/**
  * @brief  Initialize ADS1220 with default configuration
  * @retval None
  */
void ADS1220_Init(void) {
    // レジスタ設定
    ADS1220_WriteRegister(0, 0x01); // 00h: MUX=AIN0/AIN1, Gain=1, PGA=On
    ADS1220_WriteRegister(1, 0xA4); // 01h: 600SPS, Normal, Continuous Mode
    ADS1220_WriteRegister(2, 0x10); // 02h: Int VREF, Filter On, LSS=Open
    ADS1220_WriteRegister(3, 0x00); // 03h: IDAC Off
}

/**
  * @brief  Send START/SYNC command to ADS1220
  * @retval None
  */
void ADS1220_StartConversion(void) {
    uint8_t start_cmd = ADS1220_CMD_START;
    HAL_GPIO_WritePin(ADC_CS_GPIO_Port, ADC_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, &start_cmd, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(ADC_CS_GPIO_Port, ADC_CS_Pin, GPIO_PIN_SET);
}
