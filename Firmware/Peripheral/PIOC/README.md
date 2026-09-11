# PIOC 运行时 / WS2812 内存说明

当前内嵌的 WCH RGB1W 指令映像为 1142 字节；但 CH32X035 的 PIOC 程序存储是
一个固定的 4 KiB 系统 SRAM 窗口——只要 PIOC 时钟门控开启，该窗口就复用给
PIOC。因此 RGB-only 的 PIOC 映像可以减小固件 Flash 占用与启动拷贝时间，
**但无法改变** CPU/PIOC 的 SRAM 划分。

使用 PIOC 期间，`Project/Ld/Link.ld` 必须保持 16 KiB CPU RAM。

本工程只暴露 IO0 短 SFR RGB 码流命令（PC7）。DS18B20、IO1 与长 RAM 码流
操作不属于 C API。EVT 中的厂商 PIOC 汇编器是 Windows 可执行程序；本版本
不会用未经验证的手改二进制替换已知可用的映像。
