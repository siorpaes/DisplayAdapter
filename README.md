# DisplayAdapter
This project aims at mirroring Yamaha CRX-140 display with some more visible ones, such as e-paper, TM1637 seven segments display or SSD1306 OLED display.

The e-paper display is driven by a STMicroelectronics Nucleo-F401RE board, whereas the other two can be driven by a simpler low pin count STM32F0.

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

## Nucleo-F401 implementation ##
Pinout for Electronics Paper WaveShare display and Yamaha SPI display tapped signals.

|Signal               | STM32 IO | FTDI Cable (*)        |
|---------------------|:--------:|:----------------------:
| EP SD    (Black)    |  PC6     |                       |
| EP BUSY  (Brown)    |  PB15    |                       |
| EP RST   (Red)      |  PB14    |                       |
| EP CS    (Orange)   |  PB13    |                       |
| EP CLK   (Yellow)   |  PB10    |                       |
| EP MOSI  (Green)    |  PC3     |                       |
| Yamaha CS           |  PA4/PB0 | Brown                 |
| Yamaha SCK          |  PC10    | Orange                |
| Yamaha MOSI         |  PC12    | Yellow                |

(*)This is for testing purposes if no Yamaha device is available. Note that FTDI MPSSE cable only supports SPI modes 0 and 2 so you cannot use Yahama mode 3 for such tests. Windows test application is available in this same repository.

## STM32F0 implementation ##
Implementation uses this very simple board hosting a TSSOP20 STM32F0: https://hackaday.io/project/4277-stm32f030f4p6-breakout-board

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


test

