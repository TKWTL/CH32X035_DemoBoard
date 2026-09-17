# CH32X035 I2C1 中断驱动异步 DMA 后端

> 职责:为多属主提供一条共享硬件 I2C1 总线,整个事务由中断推进。

## 硬件映射

```text
PA10  I2C1_SCL
PA11  I2C1_SDA
DMA1 Channel6  I2C_TX
DMA1 Channel7  I2C_RX
时钟            400 kHz
```

## 事务引擎(全部中断驱动)

| 中断 | 处理内容 |
|---|---|
| `I2C1_EV` | `SB` → 发地址;`ADDR` → 启动 TX DMA / 重复 START / 单字节读 |
| `I2C1_ER` | `AF`(NACK)、`BERR`、`ARLO`、`OVR` |
| `DMA1_Channel6` | TX 负载发完 → 等待 `BTF` → 重复 START 或 STOP |
| `DMA1_Channel7` | RX 负载收完 → STOP |

前台只剩**看门狗**(10 ms 周期),它负责:

1. 事务超过 75 ms 无进展 → 中止并请求总线恢复;
2. 恢复序列(关 I2C → GPIO 接管 → 最多 9 个 SCL → 类 STOP 释放 → 重新初始化);
3. 补启动:请求发起时若总线仍忙(`BUSY=1`),事务停在 `WAIT_BUS_IDLE`,
   由看门狗在总线空闲时补发 START(发起时已先做 250 µs 短暂等待;
   正常 STOP 之后不会进入这个状态)。

## 事件中断风暴保护(2026-09-12 修复实装)

`I2C1_EV` 是**电平触发**的:只要 `SB`/`ADDR`/`BTF`/`STOPF` 中有一个没被清掉, `IT_EVT`
一使能就会立刻重入。实测这种风暴可达 **~10^6 次/秒**,而 `I2C1_EV` 的抢占优先级
高于 SysTick → 1 ms 节拍、IWDG 喂狗全停摆 → 表现成“停在某页 → IWDG 复位”。
现在的处理分四层:

0. **开事务前先清场** `i2c_prepare_start()`(每次发 START 前必调):`STAR1→STAR2`
   读序列清掉 `ADDR`/`STOPF`;若还残留 `SB`/`BTF`(这两个只能靠读/写 DR 清,
   会往总线发真数据)且总线不忙,则先 `i2c_peripheral_reset()`(RCC 复位 +
   重新初始化)再发 START——这是唯一安全且彻底的清法。
1. **正常路径一个标志都不留**:
   - `finish()` 在 `BUSY` 或 `BTF` 置位时补一个 STOP(STOP 顺带清 `BTF`);
   - TX-DMA 等 `BTF` 期间先关 `IT_EVT`,避免“预期中的 BTF 事件”被当成异常;
   - ISR 在“事务已结束”时只清 `ADDR`/`STOPF` + 关 `IT_EVT`,**不请求恢复**
     （否则每次正常收尾的 TX-DMA 都会把 I2C 复位一次）;
   - 非预期状态的 `SB`/`ADDR` 各有明确处置(读 `STAR2` 清掉 / 熔断恢复)。
2. **风暴熔断**: ISR 记录“同一状态被连续打断”的次数,超过 `I2C_API_EV_STORM_LIMIT`
   (64) 就关掉全部 I2C 中断源 + 拉 STOP + `s_recovery_requested=1`,把现场
   (`STAR1`/`STAR2`/`state`)交给下一轮看门狗恢复;
3. **看门狗恢复**: 下一轮 10 ms 看门狗执行 `bus_recover()`(外设复位 + GPIO 位翻转
   + 重新初始化)把总线拉回干净状态。

标志位对照:`STAR1` bit0=SB、bit1=ADDR、bit2=BTF、bit4=STOPF。

## 客户端用法(与重构前一致)

```c
THRD_UNTIL(I2C_API_TryWriteRead(owner, addr, tx, tx_len, rx, rx_len));
THRD_UNTIL(I2C_API_GetResult(owner) != I2C_API_RESULT_ACTIVE);
result = I2C_API_TakeResult(owner);
```

客户端任务中不允许 `while(flag)` 轮询;缓冲区零拷贝,必须保持有效直到
`TakeResult()`。

## DMA 策略

- TX 负载:DMA1 CH6;
- RX ≥ 2 字节:DMA1 CH7 + `I2C LAST`(硬件对最后一字节回 NACK);
- RX == 1 字节:ACK=0 → 清 ADDR → STOP → 等 `RXNE` 后读字节
  (中断内有界自旋,避免 ITBUFEN 引发的 TXE 中断风暴);
- 中断里等待 `BTF`/`BSY` 都是有界自旋(最长约一个字节时间);
  USBPD 中断优先级更高,不会被它阻塞。

## 看门狗与恢复

`I2C_API_WatchdogService()` 由独立 coroOS 线程每 10 ms 调用一次。
事务超过 75 ms 未完成会被中止并请求恢复。BERR、ARLO、OVR 同样触发恢复;
普通器件 NACK 不触发。

恢复序列:

1. 关闭 I2C DMA 与 I2C1;
2. 复位 I2C1 外设;
3. 释放 SCL/SDA 为浮空输入;
4. 若 SDA 仍为低,以“输出低/释放输入”方式最多发 9 个 SCL 脉冲;
5. 在 SCL 释放状态下做类 STOP 的 SDA 释放;
6. 恢复 PA10/PA11 为 I2C 复用功能,以 400 kHz 重新初始化 I2C1。

GPIO 恢复过程从不主动拉高 SCL/SDA,依靠板上的 I2C 上拉电阻完成拉高。
