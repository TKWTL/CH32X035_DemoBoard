# USART 异步 API

`usart_async.c/.h` 将 USART1（PB10 TX / PB11 RX）实现为 DMA 支撑的全双工
流，TX 与 RX 各 256 字节存储。

- TX：DMA1 Channel4，从软件环形缓冲取普通模式分块；TC 中断自动推进
  环形缓冲并启动下一段连续分块。
- RX：DMA1 Channel5，256 字节循环 DMA；TC 中断统计完整圈数，前台读取
  通过 DMA `CNTR` 推算生产者位置。
- `printf()` 通过 `Project/Debug/debug.c` 重定向到这里；SDI 不参与编译。
- TX 背压有界。DMA 路径异常不会让 coroOS 永久停顿。
