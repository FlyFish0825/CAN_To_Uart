# 亚博 IMU 驱动（USART1）后端接口与集成说明

> 对标文档：《ms5837.md》（深度计）。本文件对应 IMU 侧：`Core/Inc/imu_sensor.h`、
> `Core/Src/imu_sensor.c`（1977+ 行级复杂度更低：纯 C、无 HAL 依赖、可在主机直接编译）。
> 上位机协议见《imu-aa5b-host-protocol.md》（TARGET=1）。
> 原生协议依据：亚博 IMU《通信协议.xlsx》（路径见 §7），2026-10-06 逐行核实。

## 1. 硬件与串口约定

| 项 | 值 |
| --- | --- |
| 串口 | USART1，PA9=TX、PA10=RX，**115200 8N1** |
| 传感器 | 亚博 10 轴 IMU（内部 ICM42670P），上电即自动上报，默认 25 Hz |
| 每周期 4 帧 | 0x04 原始 23B / 0x16 四元数 21B / 0x26 欧拉 17B / 0x32 气压 21B ≈ 2 KB/s |

**集成硬性要求**：主控分支原 USART1 波特率为 921600（历史用途），接入 IMU 必须改为
115200（`Core/Src/usart.c` + `CAN_To_Uart.ioc` 同步）——本分支未代改，属集成方动作。

**关键硬件事实（2026-10-06 实测确立）**：WCH-Link 插着时其自带串口占用 PA9/PA10，
IMU TX 无法拉低电平 → PA10 必然 0 字节；**任何 IMU 测试/联调都必须拔掉烧写器**。

## 2. 后端架构（纯 C、无 HAL、传输无关）

- **数据路径**：集成层负责 UART 接收（推荐 DMA 循环 + USART1 空闲中断，中断里只
  快照游标/时间戳并置标志，主循环搬运——参考 imu-uart1-debug 分支 `25138b9` 的
  `ImuUartDebug_Uart1IrqHook()` 实现，已实机验证），把新到字节经
  `ImuSensor_Feed(data, len, now_us)` 交入后端。
- **有界**：内部 1024 字节环形缓冲 + 16 个分块头，放不下时整块原子丢弃并计数，
  永不阻塞、永不增长；**解析全部发生在 `ImuSensor_Process()`（主循环），不在中断里**。
- **时间体系**：模块自身无时钟，全部时间由调用方显式传入（`Feed` 的 now_us = 该批
  字节接收完成时刻）；各数据组（raw/quat/euler/baro）持有独立时间戳与序号，组间
  互不刷新新鲜度。
- **安全默认**：`pins_blocked=1`（模块自身永不解锁引脚）、无自发帧、无自动校准；
  解锁需集成层在用户确认后调 `ImuSensor_SetPinsBlocked(0)`。
- **浮点安全**：显式按字节拼小端再 memcpy 到对齐临时变量；浮点帧全部 `isfinite`
  检查，非有限整帧拒收并计数。

## 3. imu_sensor.h / imu_sensor.c（后端 API）

### 3.1 生命周期与接收

| 函数 | 说明 |
| --- | --- |
| `ImuSensor_Init(const ImuSensor_Config*)` | 绑定 TX 回调（可为 NULL=只收不发）与请求超时；pins_blocked 恒置 1 |
| `ImuSensor_Feed(data, len, now_us)` | 喂入一个到达分块（非 ISR 安全，与 Process 同上下文）；整块原子接收 |
| `ImuSensor_Process(now_us)` | 主循环：解析环形缓冲 + 处理请求超时 |
| `ImuSensor_SetPinsBlocked(b)` / `IsPinsBlocked()` | 引脚占用门：为 1 时一切请求回 PIN_BLOCKED |

### 3.2 数据与状态

| 函数 | 说明 |
| --- | --- |
| `ImuSensor_GetSample(out, now_us)` | 快照：四组各自独立的原始 int16/物理量/时间戳/序号 + 版本 + 全部统计；status 位实时评估 |
| 数据 | raw（0x04，9×int16，accel g=16/32767、gyro rad/s=2000/32767·π/180、mag=800/32767 单位未证实）、quat（0x16，f32 wxyz）、euler（0x26，f32 rad）、baro（0x32，4×f32） |

### 3.3 参数（无原生读回）

`ImuSensor_GetParameter(id, &value)`：有缓存返回 1（仍属 UNCONFIRMED）、无缓存 0
（对应 NOT_READY）、未知 ID -1（对应 UNSUPPORTED）。缓存 = 最后成功下发的值。

### 3.4 原生命令请求

`ImuSensor_Request(op, arg, now_us, &host_seq)`：同步拒绝（PIN_BLOCKED/BAD_VALUE/
BUSY/UNSUPPORTED）或受理（异步结果经 `ImuSensor_PopResult` 弹出）。

- **无回复命令**（0x60 设频率 / 0x61 设模式 / 0xA0 重置用户数据）：发出即回
  `UNCONFIRMED`，绝不等待任何回包假装成功。
- **待确认命令**（0x80 版本 → 0x01 回包；0x70/0x71/0x73 校准 → 0x81 [原命令,0失败|1成功]
  回包）：单一在飞，超时回 TIMEOUT。默认超时（Config.request_timeout_us=0 时）：
  版本 1 s、校准 30 s；显式非 0 值统一覆盖。
- 在飞期间一切新请求（含无回复命令）保守回 BUSY。
- TX 经 `Config.tx` 回调交集成层；回调非 0 = IO_ERROR，不进待确认、不写缓存。

### 3.5 假数据防护（评审要点）

- 校验和（SUM8）错误、长度与功能字不符、浮点 NaN/Inf、d1/d2 无效码字一律拒收并计数；
- 型号/模式无读回 → `MODEL_CONFIRMED` 恒 0、GET 恒 UNCONFIRMED，绝不伪造确认；
- MAG 单位未证实，字段命名保持中性（`mag_units`），文档禁标 uT。

## 4. 原生 7E23 协议核对记录（对照《通信协议.xlsx》原文，2026-10-06）

帧格式：`7E 23 LEN FUNC payload... SUM8`；LEN=整帧总长；SUM8=从 7E 到校验前累加和
低 8 位（由文档字面量 `7E 23 07 80 01 00 29` 逐字节锚定，测试断言）。多字节小端。

**MCU→IMU 命令全集（7 条，全部已实现）：**

| 功能字 | 帧格式 | 返回 |
| --- | --- | --- |
| 0x80 版本查询 | `7E 23 07 80 01 00 29` | 0x01（8B，主/副/补丁版本） |
| 0x60 输出频率 | `7E 23 07 60 <Hz单字节> 5F SUM` | **无** |
| 0x61 算法类型 | `7E 23 07 61 <6\|9> 5F SUM` | **无** |
| 0x70 校准陀螺仪+加速度计 | `7E 23 07 70 <0\|1> 5F SUM` | 0x81 `[0x70, 0失败\|1成功]` |
| 0x71 校准磁力计 | `7E 23 07 71 <0\|1> 5F SUM` | 0x81 `[0x71, 状态]` |
| 0x73 校准温度 | `7E 23 08 73 <T100低> <T100高> 5F SUM`（表格"长度07"与下标行矛盾，按下标行取 8 字节） | 0x81 `[0x73, 状态]` |
| 0xA0 重置用户数据 | `7E 23 07 A0 01 5F A8`（表格字面量） | **无** |

数据帧（IMU→MCU，自动上报）：0x04 原始 23B / 0x16 四元数 21B / 0x26 欧拉 17B /
0x32 气压 21B（仅十轴）/ 0x01 版本（应答）/ 0x81 状态（应答）。

## 5. 集成方需要做的修改（本分支不做）

1. **波特率**：usart.c + .ioc 改 115200（见 §1，硬性）。
2. **CMake**：`Core/Src/imu_sensor.c` 加入构建（同 ms5837 的处理方式）。
3. **AA5B TARGET=1 接线**：参考集成分支 `sensor_service.c`——GET/SET_PARAMETER 路由、
   GET_INFO、CALIBRATE 映射、流 0x80/0x81/0x83 均已有实现。
4. **已知缺口（必须修）**：① 集成分支的 imu_sensor 停在 82c590a，需更新到本版
   （0x60 单字节修正——真机实测旧 8 字节帧被设备静默忽略；0x73/0xA0 新增）；
   ② `sensor_service.c` CALIBRATE type=3 未把 payload 里的 reference_temperature_centiC
   传入 `ImuSensor_Request` 的 arg（当前传 0 会发 0°C）。
5. **接收模式建议**：USART1 IDLE 中断 + 主循环搬运（参考 25138b9），替代轮询。

## 6. 已知限制与假设

- 原生命令除 0x80 外均未真机发送（0x70/0x71/0x73 会改设备校准状态，需单独授权）；
  0x60 生效验证方法 = 改后统计 0x80 实际帧率（旧帧已实测无效）。
- 0x73 的 8 字节帧长解释与 T_100 有符号性未实机确认（表格自相矛盾，已按更自洽侧实现）。
- CONFIG_UNKNOWN 恒语义、0x81 合并语义等解释性约定见《imu-aa5b-host-protocol.md》。

## 7. 验证证据、构建/测试方法

| 项 | 结果 |
| --- | --- |
| 主机测试 | `tests/imu_sensor_host_test.c` 22 用例 PASS（碎包/粘包/噪声/校验和/长度/负数/NaN/参数边界/无ACK/有ACK/超时/引脚占用等） |
| 原生流实机 | CDC 重组法：25.07 Hz 四帧齐全、1503 帧仅 1 坏帧、样本物理自洽（2026-10-06） |
| IDLE 中断接收 | 空闲事件 25.9/s（avg 38.7ms）、0 错帧（commit 25138b9） |
| AA5B 命令 | 9/9 回复符合协议（GET_INFO/GET_STATUS/参数读写/边界；版本 1.0.0 端到端） |
| 0x60 旧帧 | 真机静默无效（速率不变）——8367a93 修正的实证依据 |

构建与测试命令（主机测试不经 CMake，直接 GCC）：

```
gcc -std=c11 -Wall -Wextra -Werror -O0 -ICore/Inc tests/imu_sensor_host_test.c -o build/imu_sensor_host_test.exe
gcc -std=c11 -Wall -Wextra -Werror -O0 -ICore/Inc tests/imu_bin_analyze.c -o build/imu_bin_analyze.exe
```

目标侧交叉编译检查（-Wall -Wextra -Werror 零告警）：

```
arm-none-eabi-gcc -c -std=c11 -Wall -Wextra -Werror -Og -g \
  -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard \
  -ICore/Inc Core/Src/imu_sensor.c -o build/arm_check/imu_sensor.o
```

工具链：w64devkit GCC 15.2 / Arm GNU 14.3.Rel1；串口采集与重组工具、AA5B 测试脚本、
实测报告见 `worktrees/imu-uart1-debug`（tests/ 与 build/hardware-test/）。
