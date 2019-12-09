# DDS (Direct Digital Synthesis)
This is a project control of ad9851 with stm32f103c The project is not finished yet, but the frequency generator part is already running.

Hardvare config:

- AD9851 DDS module or chip (paralell mode, can modify in Src/App/ddsdrv.h)
  - DDS_D0 : PA0
  - DDS_D1 : PA1
  - DDS_D2 : PA2
  - DDS_D3 : PA3
  - DDS_D4 : PA4
  - DDS_D5 : PA5
  - DDS_D6 : PA6
  - DDS_D7 : PA7
  - DDS_UD : PA8
  - DDS_WCK : PB9
  - DDS_RST : PB10

- Encoder with include button (the button pin can modify in Src/App/control.h)
  - Encoder A : PB6
  - Encoder B : PB7
  - Encoder button : PB8
  (Common GND, using internal pull-up resistors)

- Lcd (ST7735 128x160, SPI mode, can modify in Src/Lcd/lcd_io_spi.h)
  - LCD_RST (RESET) : PB11
  - LCD_RS (A0) : PB14
  - LCD_CS (CS) : PB12
  - LCD_SCK (SCK) : PB13
  - LCD_MOSI (SDA) : PB15
  (if modify: change the LCD_SPI define from 2 to 0, or see the controller datasheet for SPI pins)
