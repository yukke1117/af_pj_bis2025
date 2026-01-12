/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : ads1220.h
  * @brief          : ADS1220 ADC Driver Header
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __ADS1220_H
#define __ADS1220_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* ADS1220 Commands ----------------------------------------------------------*/
#define ADS1220_CMD_RESET       0x06
#define ADS1220_CMD_START       0x08
#define ADS1220_CMD_POWERDOWN   0x02
#define ADS1220_CMD_RDATA       0x10
#define ADS1220_CMD_RREG        0x20
#define ADS1220_CMD_WREG        0x40

/* ADS1220 Register Addresses ------------------------------------------------*/
#define ADS1220_REG0            0
#define ADS1220_REG1            1
#define ADS1220_REG2            2
#define ADS1220_REG3            3

/* ADS1220 Constants ---------------------------------------------------------*/
#define ADS1220_VREF_MV         2048      // 内部基準電圧 (mV)
#define ADS1220_VREF_V          2.048f    // 内部基準電圧 (V)
#define ADS1220_RESOLUTION      8388608   // 2^23 (24bit ADC)
#define ADS1220_GAIN            1         // ゲイン設定

/* Current Calculation Constants ---------------------------------------------*/
#define ADS1220_VB_V            0.5f      // バイアス電圧 (V)
#define ADS1220_R_OHM           50000.0f  // 抵抗値 (Ω)

/* Exported functions prototypes ---------------------------------------------*/
uint8_t ADS1220_ReadRegister(uint8_t reg_addr);
void ADS1220_WriteRegister(uint8_t reg_addr, uint8_t value);
int32_t ADS1220_ReadData(void);
float ADS1220_ConvertToVoltage(int32_t adc_value);
float ADS1220_ConvertToCurrent(float voltage);
void ADS1220_Init(void);
void ADS1220_StartConversion(void);

#ifdef __cplusplus
}
#endif

#endif /* __ADS1220_H */
