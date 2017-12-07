# DisplayAdapter
This project aims at extending Yamaha CRX-140 display with TM1637 seven segment one.

Implementation uses this cute board hosting a TSSOP20 STM32F0:
https://hackaday.io/project/4277-stm32f030f4p6-breakout-board

The idea is to decode the M66003 display driver SPI protocol and re-route only the digits representing the song ID to TM1637 display.
See docs folder for all components' specifications
