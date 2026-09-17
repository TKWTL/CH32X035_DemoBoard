# CH32X035 DemoBoard

## 简介

基于 CH32X035C8T6 的自制开发板固件与硬件仓库，围绕 USB Power Delivery 受电端（Sink）实验，包括 PD 3.1 EPR 28 V 协商、异步 I2C/DMA 遥测、SSD1306 状态显示，以及 PIOC 驱动的 WS2812 灯效。

## 当前固件特性

- CH32X035C8T6 USB-PD 受电端（Sink）。
- SPR 协商最高至 20 V；当供电源（Source）支持时，进入 PD 3.1 EPR 并请求最高可用的固定 EPR 电压（默认目标 28 V；上限宏允许提升至 36 V）。
- 在已验证的 28 V 路径上每 375 ms 发送一次 EPR KeepAlive。
- 通过共享的 I2C1（400 kHz）读取 INA226 总线/分流电压电流。
- 中断驱动 I2C 事务引擎：TX 使用 DMA1 CH6、RX 使用 DMA1 CH7；I2C1 事件/错误中断推进状态机，前台仅保留看门狗做超时/恢复。
- SSD1306 128x64 仪表盘，支持热插拔检测，使用 MiaoUI 6x12 字体数据。
- 供电源 PDO 仪表盘；显示的 PDO 数值由收到的 Source 数据解码，与 Sink 请求电流策略相互独立。
- 四颗 WS2812 LED 由 CH32X035 PIOC 驱动：4 ms 刷新、Q8.8 内部颜色值，以及用于平滑渐变的时域误差扩散。
- coroOS 无栈协作式任务调度。
- USART1 PB10/PB11 控制台，921600 波特率，DMA TX/RX。

## 板级接口与引脚映射

以下映射按 `Hardware/Preview/Netlist_CH32X035_DemoBoard.tel` 网表整理（主控 U9 = CH32X035C8T6，LQFP48）。

**MCU 功能连接**

- **USB-PD**：CC1/CC2（PC14/PC15）经 Q2（双 N-MOS）接 USB-C 的 CC 触点，`CCEN`（PB9）控制接通；5.1 kΩ Rd 网络（R12/R13）经 Q1（双 P-MOS）由 `#CCPD`（PB2，低有效）控制。
- **USB 数据**：PC16 → DM（D-）、PC17 → DP（D+），经 H20 跳线、RN1（22 Ω 阵列）接 Type-C 触点；D3（SRV05-4）提供 ESD 保护。
- **I2C1**：PA10（SCL）/ PA11（SDA），经 H16 跳线接 I2C 总线（INA226、H19 插座、R15/R16 上拉）。
- **USART1**：PB10 TX / PB11 RX。
- **WS2812**：PC7（PIOC IO0）经 H15 跳线接 LED 链数据输入；LED 供电同样经 H15 跳线取自 3V3。
- **调试/复位**：H9 = GND / DCK（SWCLK）/ DIO（SWDIO）/ RST；SW1 为复位按键；PA21 需在 Option Byte 中配置为 RST（详见下文「PA21 / RST」）。

**按键与指示**

- SW1：复位按键（RST）。
- SW2：用户按键（接 PC17，与 USB D+ 跳线共用该引脚）。
- LED5：电源指示灯（VBUS 经 R38 驱动）。

**电源与保护**

- 输入路径：USB-C VBUS（协商后最高 28 V）→ D2（SMAJ28A）TVS → R44（10 mΩ 分流电阻）→ U19（LGS5148）降压为 3V3（L1 = 22 µH）。
- 监测：INA226（U20，I2C 地址 0x40）跨接 R44；`ALERT` 引出至 H19 第 5 脚。
- 保护：D3（SRV05-4）保护 CC/USB 数据线；D4（SMF3.3）保护 3V3。
- 电源引出：H13（VBUS/GND ×3）、H4（3V3/GND ×3）、P1（KF301-5.0 端子，VBUS/GND）。

**插座、跳线、引出排针**

- H19：I2C 5P 插座（GND / 3V3 / SCL / SDA / INA226 ALERT），用于接 SSD1306 OLED 等 I2C 模块。
- H16：I2C 跳线（PA10↔SCL、PA11↔SDA）。
- H15：WS2812 跳线（3V3↔LED 供电、PC7↔LED 数据）。
- H20：功能跳线（PC16↔DM、PC17↔DP、PB9↔CCEN、PB2↔#CCPD）；使用对应外设时需短接。
- H7：2P（PC7 ↔ DM）——预留扩展口：计划用 PIOC 在 DM（USB D-）上处理通信协议的实验，固件尚未实现、未验证。
- H17、H18（各 1×25）引出其余 GPIO 与电源：
  - H17（1→25）：`PA15 PA16 PA17 PA18 PA19 PA20 RST PA22 PA23 PA0 PA1 PA2 PA3 PA4 PA5 PA6 PA7 PC6 PC7 PB0 PB1 PB2 PB3 PB4 GND`
  - H18（1→25）：`3V3 GND PA14 PA13 PA12 PA11 PA10 PA9 PA8 PC15 PC14 DCK PB13 PB12 DIO PC17 PC16 PB11 PB10 PB9 PB8 PB7 PB6 PB5 GND`

> 跳线断开即可隔离对应外设（例如把 PC16/PC17 移作他用，或断开 LED 链）。

板级别名集中在 `APP/main.h`；修改宏并不能绕过 CH32X035 复用功能或 USB-PD 引脚限制。

## 工程结构

```text
Firmware/
├─ APP/                  main、coroOS 任务注册、中断向量包装
├─ BSP/
│  ├─ board.c/.h
│  ├─ INA226/             仅使用 Peripheral/I2C
│  ├─ WS2812/             无状态 RGB 码流驱动，仅使用 Peripheral/PIOC
│  └─ SSD1306/            u8g2 渲染器 + 异步 I2C/DMA SSD1306 传输
├─ Peripheral/
│  ├─ PD/                 Sink 策略/协议 + CH32X035 原子 PHY 端口 + 调试笔记
│  ├─ I2C/                I2C1 400 kHz 中断驱动事务引擎 + DMA1 CH6/CH7 + 恢复
│  ├─ PIOC/               PC7 RGB 码流运行时
│  ├─ Time/               SysTick 自由运行µs计数器 + 1 ms 节拍中断
│  └─ USART/              USART1 DMA 异步 TX/RX API，各 256 B
├─ Project/              底层 MCU/工具链支持
│  ├─ Core/               青稞内核 + WCH 标准外设库
│  │  ├─ inc/
│  │  └─ src/
│  ├─ Debug/              仅延时兼容 + printf 重定向；无 SDI
│  ├─ Ld/                 链接脚本；PIOC 下 CPU RAM 保持 16 KiB
│  ├─ Scheduler/          无栈 coroOS / Protothreads 风格调度器
│  ├─ Startup/
│  └─ obj/                生成的构建输出；已被 Git 忽略
├─ .project               MounRiver Studio II / Eclipse 工程模型所需
├─ .cproject              必需的构建元数据，已纳入 Git 跟踪
└─ CH32X035_DemoBoard.wvproj
```

## 调度

前台现在采用协作式 coroOS。它沿用与 Protothreads 相同的无栈设计：各任务共享主 C 栈，并显式让出/延时。

注册顺序：

1. PD 协议任务 —— 每次调度轮次运行一次。
2. I2C 看门狗 —— 每 10 ms 检查一次，仅在超时/总线故障时执行总线恢复；事务全程
   由中断推进，不再占用调度线程。
3. INA226 —— 异步寄存器读取；VBUS 每 50 ms 更新一次，忙时让出。
4. SSD1306 —— 热插拔探测/初始化/全缓冲 DMA 刷新，独立于 PD 状态运行。
5. WS2812 灯效 —— 4 ms 物理刷新并带时域抖动（dithering），仅在 PD 合同稳定后启用。

`THRD_DELAY()` 使用自由运行 SysTick API 的 `TIME_Millis()`；毫秒计数由 1 ms SysTick 节拍中断维护，主循环空闲时进入 WFI 休眠。跨越让出点后必须保留的值，必须声明为 `static` 或存放在线程函数之外。

## USART1 异步流

- PB10 TX / PB11 RX，921600 8N1。
- TX：DMA1 Channel4，256 字节软件环形缓冲，普通模式 DMA 分块。
- RX：DMA1 Channel5，256 字节循环 DMA 与回绕计数器。
- `printf()` 只输出到这条 UART 路径。所有 SDI mailbox 代码已移除。
- TX 环形缓冲满时的等待被限制在 5 ms 内；DMA 路径异常时记录丢弃的字节数，
  而不是让协作式调度器无限期卡死。

## PIOC / WS2812 内存

面向 C 的 WS2812 驱动现在是无状态的：直接发送 RGB/GRB 码流，不在 CPU RAM 中保留常驻像素帧缓冲。

内部仍然使用已验证的 WCH RGB1W 指令映像。它只有 1142 B，但 PIOC 运行时，CH32X035 的 PIOC 程序存储会占用固定的 4 KiB 高地址 SRAM 窗口。因此，将来用 RGB-only 方案重组可以节省 Flash 与启动拷贝时间，但无法把这 4 KiB 的任何部分安全地还给 CPU。`Project/Ld/Link.ld` 保持 16 KiB CPU RAM。

## INA226 遥测

`BSP/INA226/ina226.c` 通过共享的异步 I2C1 读取 INA226（地址 0x40，10 mΩ 分流电阻）：VBUS 每 50 ms 读取并立即喂给 PD 策略（`PD_SetVbusMillivolts()`，用于掉电检测与合同拆除），分流电压/电流每 200 ms 读取一次，约每秒输出一条测量心跳；离线时按 1 s 周期重试探测。所有等待都经 coroOS 让出，不阻塞 PD 与显示任务。

INA226 与 SSD1306 共用同一条 I2C1 总线（PA10/PA11，400 kHz）。总线只有一个事务槽，由 `Peripheral/I2C` 按持有者（owner）仲裁：客户端以 `THRD_UNTIL(I2C_API_Try*())` 竞争所有权；单个设备 NACK 不影响另一设备的后续访问，总线级故障由独立看门狗线程处理（75 ms 无进展则中止并恢复）。

## SSD1306 / u8g2 状态显示

`BSP/SSD1306/ssd1306_u8g2.c` 将 u8g2 用作 128x64 全缓冲渲染器，所有物理传输均走异步 I2C API。仪表盘使用 MiaoUI `font_menu_main_h12w6` 字体。刻意不使用 `u8g2_SendBuffer()`；BSP 通过 DMA 分 8 个 128 字节页刷新，以便 coroOS 继续调度 PD 与其他任务。它与 INA226 共用 I2C1 总线（见「INA226 遥测」），同一时刻只有一个设备的事务在总线上执行。

显示监视器支持 0x3C/0x3D 热插拔，且独立于 PD 状态运行，因此由外部供电的板子在复位后会主动覆盖 OLED 中残留的 GDDRAM。初始化前要求连续两次探测成功；收到 NACK 后将在线的显示屏标记为离线，并在 I2C 总线恢复后重新探测。

渲染器兼容核心位于 `ThirdParty/u8g2/csrc/`。本工程刻意采用精简的 u8g2 兼容层与异步 SSD1306 传输；保留的 API 范围与字体集成方式见 `BSP/SSD1306/README.md`。

## USB-PD 受电端分层

`Peripheral/PD/pd.c` 负责 Sink 协议/策略状态：SPR/EPR 固定挡位选择（EPR 上限宏允许提升至 36 V）、自适应 Enter PDP 的 EPR 进入、KeepAlive、超时/恢复与合同状态。它不直接接触 CH32X035 USBPD 寄存器。

`Peripheral/PD/pd_port.c` 是 MCU 相关的 PHY/CC 后端。唯一刻意保持原子的普通消息操作是 `PD_Port_TransactSOP()`：前台 SOP 发送、立即 RX 换向与 GoodCRC 轮询都留在同一函数内，因为此前拆开它们曾让日志/调度延迟破坏 USB-PD 时序。自动 GoodCRC 在 USBPD ISR 内完成，之后收到的报文才交给策略层。Hard Reset 有独立的端口 API。

禁止在 PD 发送-响应关键路径中加入 `printf`、UART 刷写、I2C、LED 更新、调度让出或毫秒级延时。详见 `Peripheral/PD/pd_snk_debug.md`。

## PA21 / RST

固件不进行任何 Option Byte 编程。请在 WCH 烧录工具的 Option Byte 设置中将 PA21 配置为 RST。

## 构建

在 MRS II 中从本 `Firmware/` 目录打开 `CH32X035_DemoBoard.wvproj`，然后执行 Clean Project -> Rebuild Project。`.project` 与 `.cproject` 是必需的工程元数据，必须与 `.wvproj` 文件放在一起。构建输出生成在 `Project/obj/` 下（MRS II 的输出目录跟随构建配置名，当前配置名即 `Project/obj`）。
