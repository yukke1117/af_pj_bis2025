/* lcd.c ---------------------------------------------------------- */
#include "lcd.h"
#include <string.h>

#define HDR4BIT   0x24

/* 静的なLCD設定（LCD_Initで初期化される） */
static LCD_Config_t lcd_cfg;

/* 内部ヘルパ */
static inline void LCD_SELECT  (void){ HAL_GPIO_WritePin(lcd_cfg.cs_port, lcd_cfg.cs_pin, GPIO_PIN_SET); }
static inline void LCD_UNSELECT(void){ HAL_GPIO_WritePin(lcd_cfg.cs_port, lcd_cfg.cs_pin, GPIO_PIN_RESET); }
static inline void lcd_tx(const uint8_t *p, size_t len){
    HAL_SPI_Transmit(lcd_cfg.hspi, (uint8_t*)p, len, HAL_MAX_DELAY);
}

/* --- Public API ------------------------------------------------- */
const LCD_Config_t* LCD_GetConfig(void)
{
    return &lcd_cfg;
}

void LCD_Init(const LCD_Config_t *config)
{
    /* 設定を保存 */
    lcd_cfg = *config;

    /* ハードリセット相当 */
    HAL_GPIO_WritePin(lcd_cfg.disp_port, lcd_cfg.disp_pin, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(lcd_cfg.disp_port, lcd_cfg.disp_pin, GPIO_PIN_SET);
    HAL_Delay(10);

    LCD_AllClear();                      /* メモリ全消去 (CMD 0x20) */

    /* 画面全体を白で塗りつぶし */
    LCD_FillWhite();
}

void LCD_AllClear(void)
{
    const uint8_t cmd = 0x20;            /* ALL CLEAR */
    LCD_SELECT();
    lcd_tx(&cmd, 1);
    LCD_UNSELECT();
}

void LCD_SendLine4bit(uint16_t y, const uint8_t *buf)
{
    uint16_t hdr = (HDR4BIT << 10) | (y & 0x03FF);   /* 16‑bit ヘッダ */
    uint8_t  tx[2 + 88 + 2];                         /* hdr + data + 2B dummy */

    tx[0] = hdr >> 8;         /* MSB first */
    tx[1] = hdr & 0xFF;
    memcpy(&tx[2], buf, 88);
    tx[90] = tx[91] = 0x00;   /* 16clk transfer period */

    LCD_SELECT();             /* ★ CS = High (SCS=H がアクティブ) */
    lcd_tx(tx, sizeof tx);    /* SPI 8‑bit, ≒20 MHz, MSB first */
    LCD_UNSELECT();           /* CS = Low */
}

/* 画面全体を白で塗りつぶす */
void LCD_FillWhite(void)
{
    uint16_t bytes_per_line = lcd_cfg.width / 2;  /* 4bit/pixel なので 2px = 1byte */
    uint8_t white_line[128];  /* 最大256px幅まで対応 */

    /* 全ピクセルを白(0b1110)で初期化 */
    for (uint16_t i = 0; i < bytes_per_line && i < sizeof(white_line); i++) {
        white_line[i] = 0xEE;  /* 0b11101110 = 白白 */
    }

    /* 全ラインを白で塗りつぶし */
    for (uint16_t y = 0; y < lcd_cfg.height; y++) {
        LCD_SendLine4bit(y, white_line);
    }
}