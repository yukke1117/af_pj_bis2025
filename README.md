# af_pj_bis2025

STM32U385マイコンを使用した高精度電流測定システム

## 概要

NUCLEO-U385RG-Q開発ボードをベースに、ADS1220（24ビット高精度ADC）を用いて微小電流を測定するシステムです。測定結果はシリアル通信（UART）で出力され、オプションでLCDディスプレイにも表示可能です。

## ハードウェア構成

- **MCU**: STM32U385RGT6Q (Cortex-M33)
- **開発ボード**: NUCLEO-U385RG-Q
- **ADC**: ADS1220 (24ビット高精度ADC、SPI接続)
- **LCD**: 176x176 メモリ液晶ディスプレイ (SPI接続、オプション)

## ピン配置

| 機能 | ピン | 説明 |
|------|------|------|
| ADC_CS | PC2 | ADS1220 チップセレクト |
| ADC_DRDY | PC0 | ADS1220 データレディ |
| LCD_CS | PC8 | LCD チップセレクト |
| LCD_DISP | PC9 | LCD 表示制御 |
| LCD_EXTCOMIN | PC7 | LCD EXTCOMIN |
| SPI1 (LCD) | PA5/PA7 | SCK/MOSI |
| SPI2 (ADC) | PB10/PB15 | SCK/MOSI |
| UART1 | PA9/PA10 | TX/RX (VCP) |

## 機能

- ADS1220による高精度電圧測定
- 電圧から電流への変換
- シリアル出力（115200bps）で測定値を表示
- LCD表示機能（開発中）

## 開発環境

- STM32CubeMX 6.16.1
- STM32CubeIDE または IAR EWARM
- CMake対応

## ビルド方法

### CMakeを使用する場合

```bash
mkdir build && cd build
cmake ..
make
```

### STM32CubeIDEを使用する場合

1. STM32CubeIDEでプロジェクトをインポート
2. ビルドして書き込み

## 出力例

```
V: 0.001234 V, I: 12.340 uA
V: 0.001235 V, I: 12.350 uA
...
```

## ファイル構成

```
af_pj_bis2025/
├── Core/
│   ├── Inc/          # ヘッダファイル
│   │   ├── ads1220.h
│   │   ├── lcd.h
│   │   └── ...
│   └── Src/          # ソースファイル
│       ├── main.c
│       ├── ads1220.c
│       ├── lcd.c
│       └── ...
├── Drivers/          # STM32 HALドライバ
├── EWARM/            # IAR EWARM プロジェクト
├── af-bis2025.ioc    # CubeMX設定ファイル
└── CMakeLists.txt    # CMake設定
```

## ライセンス

STMicroelectronicsのソフトウェアライセンスに準拠
