# u8g2 renderer subset for CH32X035

This directory is intentionally a **small renderer port**, not a copy of every
u8g2 display driver.

The firmware owns SSD1306 transport in `BSP/SSD1306` and executes I2C transfers
through the asynchronous DMA state machine in `Peripheral/I2C`. u8g2 only
renders into a 1024-byte 128x64 framebuffer.

## u8g2 core

The project-local compatibility/rendering core is:

- `csrc/u8x8.h`
- `csrc/u8x8_compat.c`
- `csrc/u8g2_ch32x035_core.c`
- `csrc/u8g2_ch32x035_hvline.c`

The upstream files still required are `u8g2.h` and `u8g2_font.c`.

The generic upstream table `u8g2_fonts.c` (~37 MB) is not kept in this
repository: it was excluded from the MRS build and no stock u8g2 font symbol is
referenced by the firmware. If the full stock collection is ever needed,
regenerate `u8g2_fonts.c` from an upstream u8g2 release and add back only the
fonts actually referenced.

## Active font

The display uses MiaoUI's `font_menu_main_h12w6` from
`BSP/SSD1306/fonts.c`. It is a 6x12 u8g2-compatible font imported from
`TKWTL/Oscilloscope-STM32F103C8T6/Application/MiaoUI/fonts/fonts.c`.

Do not add the generated multi-display setup code or synchronous SSD1306 byte
transport back into the runtime path. Do not call `u8g2_SendBuffer()`; full
screen transfers go through `SSD1306_U8G2_BeginFlush()` and I2C DMA.
