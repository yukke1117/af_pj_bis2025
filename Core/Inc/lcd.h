/* lcd.h ---------------------------------------------------------- */
#ifndef LPM013_LCD_H
#define LPM013_LCD_H
#include "main.h"

/* LCD設定構造体 */
typedef struct {
    SPI_HandleTypeDef *hspi;      /* SPIハンドル */
    GPIO_TypeDef *cs_port;        /* CSピンのポート */
    uint16_t cs_pin;              /* CSピン番号 */
    GPIO_TypeDef *disp_port;      /* DISPピンのポート */
    uint16_t disp_pin;            /* DISPピン番号 */
    uint16_t width;               /* 画面幅（ピクセル） */
    uint16_t height;              /* 画面高さ（ピクセル） */
} LCD_Config_t;

void LCD_Init(const LCD_Config_t *config);
const LCD_Config_t* LCD_GetConfig(void);  /* 設定取得用 */
void LCD_AllClear(void);
void LCD_FillWhite(void);
void LCD_SendLine4bit(uint16_t y, const uint8_t *buf);
void LCD_DrawString4bit(uint16_t y0, const char *str);
void LCD_DrawString4bitScaled(uint16_t y0, const char *str, uint8_t scale);
void LCD_DrawImage(void);

/* アニメーション関数 */
void LCD_ScrollText(uint16_t y0, const char *str, uint16_t scroll_offset);
void LCD_BounceText(const char *str, uint16_t bounce_y);
void LCD_BlinkText(uint16_t y0, const char *str, uint8_t visible);

#endif /* LPM013_LCD_H */