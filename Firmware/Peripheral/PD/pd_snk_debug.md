# CH32X035 USB-PD Sink 调试记录

## 当前稳定基线

- MCU：CH32X035C8T6，USB-PD Sink。
- 已验证路径：SPR 20 V / 5 A -> EPR Mode Enter -> EPR Source Capabilities -> Fixed PDO 28 V / 5 A。
- 已连续多次插拔稳定进入 28 V；INA226 实测 VBUS 约 28.00 V。
- EPR KeepAlive 周期：375 ms。
- 板卡由 VBUS 供电，因此本地协议恢复默认不主动发送 Hard Reset，避免 Source 关闭 VBUS 导致 MCU 自己掉电。

## 最终确认的关键问题

### 1. 自动 GoodCRC 正在发送时被重新初始化 RX

早期代码可能在 USBPD ISR 已启动自动 GoodCRC、但 TX_END 尚未完成时再次调用 RX 初始化。
`PD_ALL_CLR` 会清状态并可能截断正在发送的 GoodCRC。典型诊断是：

```text
auto-GoodCRC started/completed=6/5
```

修复后必须满足：收到普通 SOP 消息 -> ISR 自动发 GoodCRC -> GoodCRC TX_END -> 才把消息交给策略层。
`PD_Port_RxStart()` 不允许在 `auto_ack_inflight` 时重置 PHY。

### 2. 普通 SOP 发送事务不能被拆开

已验证稳定的发送顺序是：

```text
mask USBPD IRQ
  -> blocking SOP TX
  -> TX_END
  -> immediately switch PHY to RX
  -> poll GoodCRC in the WCH/C140 timing window
```

因此 `PD_Port_TransactSOP()` 是有意保留的原子 PHY 操作。
不要把 TX、RX turnaround、GoodCRC wait 拆成多个高层 API。

### 3. 调试输出改变了 USB-PD 时序

曾经在 Source_Capabilities 的 GoodCRC 与 Sink Request 之间输出 PDO、Request 内容并等待 UART，
导致 Source 对 Sink 响应超时。现象是 Request 内容本身正确，但 Source 连 GoodCRC 都不返回，随后 VBUS 降低并触发板卡 POR/PDR。

移除关键路径中的日志后，实测：

```text
Source_Capabilities GoodCRC 完成 -> 第一次 Request TX ≈ 5.1 ms
Request attempts = 1
```

随后可稳定协商 20 V，再进入 EPR 28 V。

## 已验证的 SPR Request

20 V / 5 A、PDO5、EPR-capable RDO 的首个 Request：

```text
82 10 F4 D1 47 51
```

- Header：PD3.x Request，1 Data Object，Sink，Message ID 0。
- RDO（little-endian）：`0x5147D1F4`。
- Object Position = 5。
- No USB Suspend = 1。
- EPR Mode Capable = 1。
- Operating / Max current = 5 A。

这个 RDO 已通过真实适配器验证，不应再作为首要怀疑对象。

## 健康协商日志特征

正常路径应接近：

```text
Source_Capabilities
SPR Request PDO5: 20000 mV, 5000 mA
SPR contract ready
EPR Mode: Enter Acknowledged
EPR Mode: Enter Succeeded
EPR_Source_Capabilities
EPR Request PDO8: 28000 mV, 5000 mA
EPR contract ready
EPR KeepAlive period=375 ms
INA226 VBUS ~= 28000 mV
```

## 代码分层约束

### `pd.c`：协议 / 策略层

负责：

- Type-C attach 后的 Sink 状态推进。
- SPR PDO 解析与 Fixed PDO 选择。
- Request / Accept / PS_RDY 状态。
- EPR Mode Enter、EPR Source Capabilities、28 V Request。
- EPR KeepAlive。
- VBUS detach、Soft Reset、超时和恢复策略。

禁止直接访问 `USBPD->...`、GPIO、RCC、NVIC。

### `pd_port.c`：CH32X035 PHY / CC 层

负责：

- USBPD / CC / GPIO / RCC / NVIC 寄存器。
- RX DMA。
- USBPD ISR。
- 自动 GoodCRC。
- `PD_Port_TransactSOP()` 原子 SOP 事务。
- Hard Reset 物理发送。

除必须保证微秒级连续时序的 PHY 操作外，尽量保持小函数和清晰接口。

## 永久规则

1. **PD sender-response 关键路径禁止 `printf()` / `Debug_Flush()`。**
2. **关键路径禁止 I2C、INA226、WS2812、scheduler yield、毫秒延时。**
3. 自动 GoodCRC 尚未 TX_END 时，不得 `PD_ALL_CLR` 或重新启动 RX。
4. 普通 SOP TX 必须通过 `PD_Port_TransactSOP()`；不要重新引入“Send + WaitGoodCRC”分裂接口。
5. 收到 Source Hard Reset 要先软件锁存，再清硬件 flag，避免下一次 `PD_ALL_CLR` 抹掉证据。
6. Source-originated Hard Reset 会使 VBUS 掉向 vSafe0V；VBUS 供电板会因此 POR/PDR，不能简单当作 MCU 软件崩溃。
7. 正常运行只打印阶段性状态；raw frame、GoodCRC 计数、ack->TX 等详细信息只在 TX 失败时输出。
8. 修改 PD PHY 后先验证 SPR 5/20 V，再验证 EPR 28 V 和 KeepAlive，最后做多次热插拔。

## 回归测试

每次修改 `Peripheral/PD/` 后至少验证：

- 冷启动后插入适配器可到 28 V。
- 连续拔插至少 10 次，均能重新建立合同。
- 28 V 保持运行至少数分钟，不因 KeepAlive 超时退出 EPR。
- INA226 周期读取与 WS2812 动画开启时不影响 PD。
- 拔出后能由 VBUS 下降正确清合同并重新进入 attach detection。
- TX 失败诊断中 `auto-GoodCRC started/completed` 不应长期出现 started > completed。

## 备注

这份记录只保留已被当前硬件和稳定日志验证过的结论。后续如果再次出现 PD 问题，优先检查时序和 PHY 事件生命周期，不要先在 EPR RDO 或应用任务上大范围改动。
