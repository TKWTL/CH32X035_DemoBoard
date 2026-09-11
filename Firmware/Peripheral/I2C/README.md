# CH32X035 I2C1 异步 DMA 后端

硬件映射：

```text
PA10  I2C1_SCL
PA11  I2C1_SDA
DMA1 Channel6  I2C_TX
DMA1 Channel7  I2C_RX
时钟           400 kHz
```

CH32X035 参考手册将 I2C TX/RX 映射到 DMA1 CH6/CH7。驱动将 START/地址/重复
START/STOP 与错误处理保留在 `i2c_api.c` 中；DMA 只搬运负载字节。

## 协作式所有权

只有一条物理总线和一个事务槽位。客户端调用 `I2C_API_Try*()`。
若已有其他持有者处于活动状态，该调用会立即返回 0。coroOS 任务应这样写：

```c
THRD_UNTIL(I2C_API_TryWrite(...));
THRD_UNTIL(I2C_API_GetResult(owner) != I2C_API_RESULT_ACTIVE);
result = I2C_API_TakeResult(owner);
```

客户端任务中不允许出现 `while(flag)` 循环。`I2C_API_Service()` 在每次
调度轮次推进一个小状态机步骤。

缓冲区是零拷贝的，因此必须保持有效直到 `TakeResult()`。对于跨越让出点的
缓冲区，Protothread 调用方应使用 static/持久存储。

## DMA 策略

- TX 负载：DMA1 CH6。
- RX 负载 >= 2 字节：使用 DMA1 CH7，并处理 I2C LAST。
- RX 负载 == 1 字节：ACK/STOP 边界刻意由外设状态机原子处理，而不是强行
  让 DMA 执行脆弱的一字节接收序列。
- DMA TC 不算 I2C-TX 结束：状态机在 STOP 或重复 START 前仍会等待 BTF。

## 看门狗与恢复

`I2C_API_WatchdogService()` 由独立的 coroOS 线程每 10 ms 调用一次。
超过 75 ms 的事务会被中止并请求恢复。BERR、ARLO 与 OVR 也会请求恢复；
正常的器件 NACK 不会。

恢复流程：

1. 关闭 I2C DMA 与 I2C1；
2. 复位 I2C1 外设；
3. 将 SCL/SDA 释放为浮空输入；
4. 若 SDA 为低，以「输出低 / 释放为输入」的方式最多发送 9 个 SCL 脉冲；
5. 在 SCL 释放的状态下，做出类似 STOP 的 SDA 释放；
6. 将 PA10/PA11 恢复为 I2C 复用功能，并以 400 kHz 重新初始化 I2C1。

GPIO 恢复过程从不会主动把 SCL/SDA 拉高；它依赖板上的 I2C 上拉电阻。
