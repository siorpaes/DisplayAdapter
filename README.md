# DisplayAdapter
This project aims at extending Yamaha CRX-140 display with TM1637
seven segments display or SSD1406 OLED display.

Implementation uses this cute board hosting a TSSOP20 STM32F0:
https://hackaday.io/project/4277-stm32f030f4p6-breakout-board

The idea is to decode the M66003 display driver SPI protocol and re-route only
the digits representing the song ID to secondary display.
See docs folder for all components' specifications

Pinout

|Signal        | STM32 IO | Notes                             |
|--------------|:--------:|:----------------------------------:
| SPI_CLK      |  PA5     |                                   |
| SPI_MOSI     |  PA7     |                                   |
| SPI_CS       |  PA0/PA4 | Connect both pins to CS           |
| TM1637  SCL  |  PA1     | Bitbanged, not defined in CubeMX  |
| TM1637  SDA  |  PA2     | Bitbanged, not defined in CubeMX  |
| SSD1306 SCL  |  PA9     |                                   |
| SSD1306 SDA  |  PA10    |                                   |


