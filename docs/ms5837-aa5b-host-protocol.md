# 深度计 AA5B 上位机协议（TARGET = 2）

> 用途：直接交给上位机（Qt）开发照着实现。
> 来源：**全部字段与样例均为真机实测**（本次固件 = 集成分支 `b19a70e` 的接线 + 驱动 `9abf64e`，
> 经 USB CDC 在 `COM11` 上抓取；抓取日期 2026-10-06）。凡未实测的内容都在文中显式标注。
> 深度计相关代码：`Core/Inc/ms5837.h`、`Core/Src/ms5837.c`；协议编解码：`Core/Src/sensor_protocol.c`；
> 业务层：`Core/Src/sensor_service.c`。

---

## 1. 物理链路与帧族共存

| 项 | 值 |
| --- | --- |
| 链路 | USB CDC 虚拟串口（设备名 `CAN_To_Uart`，实测枚举为 `COM11`） |
| 串口参数 | 115200 8N1（CDC 下波特率不参与实际速率，按此设置即可） |
| 同一条串口上共存的帧族 | `AA 55` CAN 帧、`AA 58` 心跳、`AA 59` 固件流/日志、**`AA 5B` 传感器协议（本文）** |
| 上行复用方式 | 所有帧**走同一个 USB 发送队列**（255 槽 × 78 字节），按入队顺序串行发出；上位机按 `AA` + 家族字节路由即可 |

深度计只使用 `AA 5B` 帧族，`TARGET` 字段恒为 **2**（`TARGET=1` 是同一条协议上的 IMU，不要混用）。

## 2. 通用帧格式（所有 AA5B 帧一致）

| 偏移 | 长度 | 字段 | 说明 |
| --- | --- | --- | --- |
| 0 | 1 | 帧头 | 固定 `0xAA` |
| 1 | 1 | 家族 | 固定 `0x5B` |
| 2 | 1 | 版本 | 固定 `0x01` |
| 3 | 1 | `CMD` | 命令码，见 §4 |
| 4 | 1 | `FLAGS` | 见 §3 |
| 5 | 1 | `TARGET` | 2 = 深度计 |
| 6 | 4 | `SEQ` | u32 **小端**；回复原样回显请求的 SEQ |
| 10 | 2 | `LEN` | u16 **小端**，负载字节数（不含帧头与 CRC） |
| 12 | 4 | `TIMESTAMP_US` | u32 **小端**，微秒；语义见 §11 |
| 16 | `LEN` | `PAYLOAD` | 负载，见各命令 |
| 16+`LEN` | 2 | `CRC16` | u16 **小端**，见下 |
| 18+`LEN` | 1 | 帧尾 | 固定 `0x5B` |
| 19+`LEN` | 1 | 帧尾 | 固定 `0xAA` |

* 整帧长度 = `20 + LEN`（最大 `20 + 58 = 78` 字节）。
* **CRC16**：CRC-16/CCITT-FALSE —— 多项式 `0x1021`、初值 `0xFFFF`、不反射、无结束异或；
  计算范围 **偏移 1 ~ 15+LEN**（即从第 2 个字节 `0x5B` 起，到负载最后一个字节），结果**小端**写入。
* 解码校验（不全过就丢帧）：`[0]=0xAA`、`[1]=0x5B`、`[2]=0x01`、`LEN ≤ 58`、
  `[18+LEN]=0x5B`、`[19+LEN]=0xAA`、CRC 一致。

## 3. `FLAGS` 与回复约定

| 值 | 含义 | 实测出现场景 |
| --- | --- | --- |
| `0x01` | 请求（上位机 → 设备） | 上位机发出的所有命令 |
| `0x02` | 成功回复 | `GET_INFO/GET_STATUS/GET_PARAMETER/SET_PARAMETER/START/STOP_STREAM/ZERO_DEPTH` 成功 |
| `0x06` | 错误/未支持回复 | 实测出现在 `SAVE_CONFIG(0x45)`、`RESTORE_DEFAULTS(0x46)`、`CALIBRATE(0x49)`、`SELF_TEST(0x4A)`，均为 `RESULT=1` |
| `0x08` | 遥测（设备主动上报） | `0x82`、`0x83` |

* **回复命令码 = 请求命令码 + 0x40**（例：请求 `0x02` → 回复 `0x42`）。
* 所有回复的**负载第 0 字节固定为 `RESULT`**（§4）。
* 遥测帧没有 `RESULT`，负载第 0 字节就是数据（例 `0x82` 的 `status`）。遥测帧的 `SEQ` 是按 TARGET 各自累加的**流序号**，与请求 SEQ 无关。

### `RESULT` 码表

| 值 | 名称 | 含义 | 实测例子 |
| --- | --- | --- | --- |
| 0 | OK | 成功 | 绝大多数回复 |
| 1 | UNSUPPORTED | 该命令/参数未实现 | `SAVE_CONFIG(05)`、`RESTORE_DEFAULTS(06)`、`CALIBRATE(09)`、`SELF_TEST(0A)` |
| 2 | BAD_VALUE | 参数非法 | 例如型号填 7、密度填 5000 |
| 3 | BUSY | 设备忙 | （未实测） |
| 4 | TIMEOUT | 超时 | 例如深度计未绑定时 |
| 5 | OFFLINE | 传感器离线 | （未实测） |
| 6 | IO_ERROR | 总线错误 | （未实测） |
| 7 | UNCONFIRMED | 未确认 | （未实测，IMU 用） |
| 8 | NOT_READY | 数据尚未就绪 | **`GET_PARAMETER 0103` 实测返回 `08`**（尚未采集水面零点） |
| 9 | PIN_BLOCKED | 被管脚占用 | IMU 专用 |

## 4. TARGET=2（深度计）命令总表（全部实测）

| `CMD` | 名称 | 请求负载 | 回复 `CMD` | 回复负载 | 实测结果 |
| --- | --- | --- | --- | --- | --- |
| `0x01` | `GET_INFO` | 空 | `0x41` | `RESULT + 30B` 描述体（§6） | `RESULT=0`，`LEN=31` |
| `0x02` | `GET_STATUS` | 空 | `0x42` | `RESULT + 20B` 统计体（§7） | `RESULT=0`，`LEN=21` |
| `0x03` | `GET_PARAMETER` | `ID:u16` | `0x43` | `RESULT + ID:u16 + TYPE:u8 + LEN:u8 + VALUE` | 见 §5 |
| `0x04` | `SET_PARAMETER` | `ID:u16 + TYPE:u8 + LEN:u8 + VALUE` | `0x44` | 回显 `ID/TYPE/LEN/VALUE` | `RESULT=0` |
| `0x07` | `START_STREAM` | 空 | `0x47` | `RESULT` | `RESULT=0`（开始推 `0x82`/`0x83`） |
| `0x08` | `STOP_STREAM` | 空 | `0x48` | `RESULT` | `RESULT=0`（停止推流） |
| `0x0C` | `ZERO_DEPTH` | 空 | `0x4C` | `RESULT` | `RESULT=0`（需在水面执行） |
| `0x05` | `SAVE_CONFIG` | 空 | `0x45` | `RESULT` | **`RESULT=1` UNSUPPORTED**（无持久化） |
| `0x06` | `RESTORE_DEFAULTS` | 空 | `0x46` | `RESULT` | **`RESULT=1` UNSUPPORTED**（当前固件未开放） |
| `0x09` | `CALIBRATE` | —— | `0x49` | `RESULT` | **`RESULT=1` UNSUPPORTED**（MS5837 无校准寄存器） |
| `0x0A` | `SELF_TEST` | 空 | `0x4A` | `RESULT` | **`RESULT=1` UNSUPPORTED** |
| `0x0B` | `REBOOT` | 空 | `0x4B` | `RESULT` | 文档标为 UNSUPPORTED；**未实测**（避免设备复位） |

## 5. 参数表（`GET/SET_PARAMETER`）

类型码（实测）：`02` = u8，`04` = u16，`07` = f32（其余类型码未实测）。

| 参数 ID | 名称 | 类型 | 取值 | 默认 | 实测回读 |
| --- | --- | --- | --- | --- | --- |
| `0x0001` | 输出率 | u16 | 1 ~ 100 Hz（受型号/OSR 转换预算约束，超预算返回 `BAD_VALUE`） | 25 | `00 01 00 04 02 19 00` → **25 Hz** |
| `0x0101` | OSR | u16 | 256 / 512 / 1024 / 2048 / 4096 / 8192 | 4096 | `00 01 01 04 02 00 10` → **0x1000 = 4096** |
| `0x0102` | 水密度 | f32 | 900 ~ 1300 kg/m³ | 1029 | `00 02 01 07 04 00 A0 80 44` → **1029.0** |
| `0x0103` | 水面零点压力 | f32 | 10000 ~ 200000 Pa | 未设置 | **`RESULT=8 NOT_READY`**（还没归零） |
| `0x0104` | 滤波系数 K | f32 | 0 ~ 0.99 | 0 | `00 04 01 07 04 00 00 00 00` → **0.0**（不滤波） |
| `0x0105` | 传感器型号 | u8 | 0 = unknown，2 = 02BA，30 = 30BA | 0 | `00 05 01 02 01 02` → **2（02BA）** |

> `GET_PARAMETER` 的回复 = `RESULT(1) + ID(2) + TYPE(1) + LEN(1) + VALUE`；
> `SET_PARAMETER` 的请求负载就是 `ID(2) + TYPE(1) + LEN(1) + VALUE`，回复原样回显。
> 实测 `SET_PARAMETER 0105 = 2`：请求负载 `05 01 02 01 02`，回复负载 `00 05 01 02 01 02`。

## 6. `GET_INFO` 回复负载（30 字节描述体，`LEN=31` 含 RESULT）

| 偏移 | 长度 | 字段 | 说明 |
| --- | --- | --- | --- |
| 0 | 1 | `RESULT` | 0 = OK |
| 1 | 1 | `sensor_type` | 固定 2（深度计） |
| 2 | 1 | `model` | 0 = unknown，2 = 02BA，30 = 30BA |
| 3 | 4 | `capabilities` | u32 位图：bit3 压力、bit4 温度、bit5 原始深度、bit10 零点、bit11 配置；实测 `0x00000C38` |
| 7 | 16 | `name` | ASCII，`"MS5837"` 后补 `\0` |
| 23 | 8 | `firmware` | ASCII，实测 `"host-v1"`（后补 `\0`） |

实测原始帧（SEQ=0x5101）：

```
TX: AA 5B 01 01 01 02 01 51 00 00 00 00 00 00 00 00 A5 FB 5B AA
RX: AA 5B 01 41 02 02 01 51 00 00 1F 00 68 D5 A9 7C
    00 02 02 38 0C 00 00 4D 53 35 38 33 37 00 00 00 00
    00 00 00 00 00 00 68 6F 73 74 2D 76 31 00
    B2 3D 5B AA

    CMD=0x41 FLAGS=0x02 TARGET=2 SEQ=0x5101 LEN=31 TIMESTAMP_US=2091505000
负载: 00 | 02 | 02 | 38 0C 00 00 | "MS5837"(后补0) | 全 0 | "host-v1"(后补0)
      ^RESULT  ^type  ^model=02BA  ^caps=0x00000C38
```

## 7. `GET_STATUS` 回复负载（20 字节统计体，`LEN=21` 含 RESULT）

| 偏移 | 长度 | 字段 | 说明 |
| --- | --- | --- | --- |
| 0 | 1 | `RESULT` | 0 = OK |
| 1 | 4 | `status` | 状态字（§8），实测 `0x00000933` |
| 5 | 4 | `sample_seq` | 已完成采样数（有效样本才 +1） |
| 9 | 4 | `age_ms` | 最近一个样本距今毫秒；**从未有样本时为 `0xFFFFFFFF`** |
| 13 | 4 | `good_frames` | 良好样本数 |
| 17 | 4 | `errors` | 累计错误数（实测 0） |

实测原始帧（设置好型号、运行中的状态，SEQ=0x5102）：

```
TX: AA 5B 01 02 01 02 02 51 00 00 00 00 00 00 00 00 CF 85 5B AA
RX: AA 5B 01 42 02 02 02 51 00 00 15 00 D0 D5 B4 7C
    00 33 09 00 00 51 CC 00 00 15 00 00 00 51 CC 00 00 00 00 00 00
    31 CC 5B AA

    CMD=0x42 FLAGS=0x02 TARGET=2 SEQ=0x5102 LEN=21 TIMESTAMP_US=2092226000
负载: 00 | 33 09 00 00 | 51 CC 00 00 | 15 00 00 00 | 51 CC 00 00 | 00 00 00 00
      ^RESULT  status=0x933  seq=52305     age=21 ms     good=52305    errors=0
```

另一种状态（**刚复位、还没设型号**，同一条命令）：

```
负载: 00 | 03 05 00 00 | 3D 01 00 00 | 11 00 00 00 | 3D 01 00 00 | 00 00 00 00
      ^RESULT  status=0x503   seq=317      age=17 ms     good=317      errors=0
```

## 8. 状态字位定义（`status`，u32）

| 位 | 名称 | 含义 |
| --- | --- | --- |
| 0 | `ONLINE` | 传感器在线 |
| 1 | `RAW_VALID` | 原始 D1/D2 有效 |
| 2 / 3 | （IMU 用：四元数/欧拉角） | 深度计不使用 |
| 4 | `PRESSURE_VALID` | 压力有效 |
| 5 | `TEMPERATURE_VALID` | 温度有效 |
| 6 | `DEPTH_VALID` | 深度有效（需已归零） |
| 7 | `ZERO_VALID` | 零点已采集 |
| 8 | `PROM_VALID` | PROM 读取且 CRC4 通过 |
| 9 | `PIN_BLOCKED` | 深度计不使用 |
| 10 | `CONFIG_UNKNOWN` | 型号未确认（此时压力/温度/深度**都无效**） |
| 11 | `MODEL_CONFIRMED` | 型号已明确（需上位机 `SET 0105`） |

运行正常时实测：`0x00000933` = `ONLINE|RAW_VALID|PRESSURE_VALID|TEMPERATURE_VALID|PROM_VALID|MODEL_CONFIRMED`。
**归零且在水面以下后应变成 `0x000009F3`**（多 `DEPTH_VALID|ZERO_VALID` 两位）。

## 9. 遥测：`0x82` `DEPTH_DATA`（32 字节，默认 25 Hz）

| 偏移 | 长度 | 字段 | 单位 | 说明 |
| --- | --- | --- | --- | --- |
| 0 | 4 | `status` | —— | 与 §8 同一状态字 |
| 4 | 4 | `pressure_pa` | Pa | f32 |
| 8 | 4 | `temperature_c` | °C | f32 |
| 12 | 4 | `depth_raw_m` | m | f32，未滤波 |
| 16 | 4 | `depth_filtered_m` | m | f32，滤波后（`K=0` 时与 raw 相同） |
| 20 | 4 | `surface_pressure_pa` | Pa | f32，已采集的零点压力（未采集时为 0） |
| 24 | 4 | `D1` | —— | u32，原始压力 ADC |
| 28 | 4 | `D2` | —— | u32，原始温度 ADC |

**无效值规则**：任何物理量无效时，线上**填 `0.0` 并清掉对应 `*_VALID` 位**（内部是 NaN，不会把 NaN 发给上位机）。
所以上位机**必须先看 `status` 位，再看数值**；`0.0` 不代表"零深度/零压力"。

实测原始帧（25 Hz 流中的一帧，SEQ=861，可整帧当单测向量用）：

```
AA 5B 01 82 08 02 5D 03 00 00 20 00 88 A8 62 6C
33 09 00 00 80 B1 C5 47 71 3D C4 41 00 00 00 00
00 00 00 00 00 00 00 00 FF 43 60 00 C3 1C 78 00
9D 9F 5B AA

CMD=0x82 FLAGS=0x08 TARGET=2 SEQ=861 LEN=32 TIMESTAMP_US=1818405000 (= 上电后 1818.405 s)
负载: 33 09 00 00 | 80 B1 C5 47 | 71 3D C4 41 | 00 00 00 00 | 00 00 00 00 |
      00 00 00 00 | FF 43 60 00 | C3 1C 78 00
解码: status=0x00000933
      pressure_pa  = 101219.00 Pa (1012.19 mbar)
      temperature_c= 24.530 C
      depth_raw_m  = 0.000  m   (DEPTH_VALID 未置位：尚未归零)
      depth_filtered_m = 0.000 m
      surface_pressure_pa = 0.00 Pa
      D1 = 6308863   D2 = 7871683
```

## 10. 遥测：`0x83` `SENSOR_STATUS`（20 字节，默认约 1 Hz/TARGET）

| 偏移 | 长度 | 字段 |
| --- | --- | --- |
| 0 | 4 | `status`（§8） |
| 4 | 4 | `sample_seq` |
| 8 | 4 | `age_ms`（从未有样本 = `0xFFFFFFFF`） |
| 12 | 4 | `good_frames` |
| 16 | 4 | `errors` |

负载**没有 `RESULT` 字节**。注意 `0x83` 是 **IMU（TARGET=1）和深度计（TARGET=2）共用**的命令码，
**必须按 `TARGET` 区分**。实测同一秒内两个 TARGET 各一帧：

```
TARGET=1 (IMU)    负载: 00 06 00 00 | 00 00 00 00 | FF FF FF FF | 00 00 00 00 | 00 00 00 00
                        status=0x600(未上线) seq=0  age=never     good=0        errors=0
TARGET=2 (深度计) 负载: 33 09 00 00 | A3 B1 00 00 | 1E 00 00 00 | A3 B1 00 00 | 00 00 00 00
                        status=0x933  seq=45475    age=30 ms     good=45475    errors=0
```

## 11. 时间戳语义（重要）

`TIMESTAMP_US`（帧头偏移 12）= **数据产生时刻**，不是发送时刻：

| 帧 | 时间戳取值 |
| --- | --- |
| `0x82` | 该样本 **D2 结果读完那一刻**的 `HAL_GetTick()` × 1000（分辨率 1 ms，u32 微秒值，约 71.6 分钟回绕一次） |
| `0x83` | 生成该状态帧的时刻 |
| 命令回复 | **回复生成时刻**（实测 `GET_INFO` 回复 `TIMESTAMP_US=2091505000` = 上电后 2091.5 s，即当刻） |

也就是说：`0x82` 的时间戳直接可用于"这条数据是什么时候测出来的"，
上位机做时间对齐/延迟统计时**不要**用接收时间代替。

## 12. 推荐交互时序

1. **连接后**：设备默认**会自动开始推流**（链路建立即开流），所以你会立刻收到 `0x82`/`0x83`；
   不想要就发 `STOP_STREAM`。
2. `GET_INFO`：确认 `sensor_type=2`、读 `model`（复位后是 0）。
3. **必须**：`SET_PARAMETER 0105 = 2`（02BA）或 `30`（30BA）。
   不设型号 → `CONFIG_UNKNOWN`，压力/温度/深度全部无效（**驱动不会猜型号**）。
4. 等 `GET_STATUS.status` 出现 `PRESSURE_VALID | TEMPERATURE_VALID | PROM_VALID`（设好型号后约 1 秒内，
   实测 `status` 从 `0x503` 变为 `0x933`）。
5. 把传感器放在**水面**，发 `ZERO_DEPTH`；成功后 `status` 多出 `ZERO_VALID | DEPTH_VALID`，深度开始有效。
6. 之后正常解析 `0x82`；需要更慢/更快就 `SET 0001`（1~100 Hz），需要更稳就加大 `0101` OSR 或调 `0104` 滤波系数。
7. 若中途改型号/OSR/输出率：驱动会自动放弃当前半周期并重新开始，**不会有跨配置的错配样本**；
   但改型号后 `DEPTH_VALID` 会重新变无效，需要重新确认状态。

## 13. 上位机必须知道的注意事项

1. **帧可能偶尔丢失（设计如此）**：传感器帧入队前有一道背压 —— 当 USB 发送队列占用 ≥ 25/255 时，
   传感器帧直接丢弃（`sensor_board.c:104`），保证 CAN/固件流不被传感器饿死。
   所以"偶尔少一帧 `0x82`"是正常的，用 `sample_seq`/`SEQ` 判断连续性即可。
2. **无持久化**：设备掉电/复位后 `model`、零点、滤波系数全部回到默认（`SAVE_CONFIG` 返回 UNSUPPORTED）。
   上位机每次连接都应重做第 3、5 步。
3. **深度必须显式归零**：不归零时 `depth_*` 恒为 0 且 `DEPTH_VALID=0`（本轮实测就是这种状态，`P0=0`）。
4. **`GET_PARAMETER 0103` 在未归零时返回 `RESULT=8 NOT_READY`**，不要当作通信失败。
5. 参数非法（如型号 7、密度 5000、OSR 3000、输出率 200）→ `RESULT=2 BAD_VALUE`，
   且**设备端数值不变**；上位机应把回显值和期望值比对。
6. `CALIBRATE`/`SELF_TEST`/`RESTORE_DEFAULTS`/`SAVE_CONFIG` 对深度计**实测统统 `RESULT=1 UNSUPPORTED`**，
   不要在上位机暴露成可用按钮。
7. `0x83` 的 `age_ms` 为 `0xFFFFFFFF` 表示"还没有任何样本"（别当成 49 天的延迟）。

## 14. 待实现（提议，**当前固件没有**，上位机先别实现）

用户已提出"默认不主动上传、只上传需要的字段（压力+温度+深度+数据产生时间戳）、收到专用指令再回一帧"。
提议的新命令（尚未在固件里实现，故本文档只作预告）：

| 项 | 值 |
| --- | --- |
| 请求 | `CMD 0x0D`（`SENSOR_READ_DEPTH_SAMPLE = 13`），`FLAGS=0x01`，`TARGET=2`，负载空 |
| 回复 | `CMD 0x4D`，`FLAGS=0x02`，负载 **21 字节** |
| 负载 | `RESULT:u8 + status:u32 + pressure_pa:f32 + temperature_c:f32 + depth_m:f32 + timestamp_ms:u32` |

配套行为：默认**不自动开流**（去掉"连接即推流"），`0x82` 的 32 字节布局由本回复取代（破坏性变更）。
实现细节与集成改动点见 `docs/ms5837.md` 第 8 节。
