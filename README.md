# DisplayAdapter
This project aims at mirroring Yamaha CRX-140 display with some more visible ones, such as e-paper, TM1637 seven segments display or SSD1306 OLED display.

Implementation uses this very simple board hosting a TSSOP20 STM32F0:
https://hackaday.io/project/4277-stm32f030f4p6-breakout-board
or a STMicroelectronics DiscoveryL0 board which features an e-ink display.

The idea is to decode the M66003 display driver SPI protocol and
re-target the contents to a secondary display.
SPI display signals are available on internal Yamaha 15 pin connector:

| Signal | Connector pin |
|--------|---------------|
|  DGND  |       4       |
|  MOSI  |       7       |
|  +3V3  |       8       |
|   CS   |       9       |
|  CLK   |      11       |

Signals can be easily wired out on a 6 pin flat cable connector, e.g.: 
```
NC       - CS(9)   - MOSI(7)
CLK(11)  - 3V3(8)  - GND(4)
```

SPI parameters: 200kHz clock, MSB First, CPOL=1, CPHA=1. Check salae-dumps directory for actual snooped SPI data. Data is transmitted as plain ASCII codes and is composed of 15 characters with possible padding spaces (0x20) with heading 0xFA, 0x00 bytes and 0xF9, 0xFF trailing bytes.

See docs folder for all components' specifications.


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
