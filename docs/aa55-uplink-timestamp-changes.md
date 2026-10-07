# AA55 上行帧时间戳变更说明（上位机适配用）

**适用固件**：`电机优先调度` 分支，提交 `2119add` 及之后烧录的固件。
**变更性质**：仅**板卡 → 上位机**方向的 AA55 帧格式变化；**上位机 → 板卡**方向（下行）格式完全不变。本变更与"电机优先调度"功能相互独立，拆包改造可单独上线。

## 1. 变更摘要

| 帧族 | 方向 | 是否变化 | 时间戳位置 |
| --- | --- | --- | --- |
| AA55 普通 CAN 数据上报 | 板卡→上位机 | **变了**：DATA 后新增 4 字节 | 帧尾，DATA 之后、CRC 之前 |
| AA55 状态/启动提示（`UART_TX!` 等） | 板卡→上位机 | **变了**：同上 | 同上 |
| AA55 波特率配置回复（FLAGS=80/81） | 板卡→上位机 | **变了**：包体 17→21 字节，总长 23→27 字节 | 参数之后、CRC 之前 |
| AA55 下行命令 | 上位机→板卡 | 不变，仍为 `BODY_LEN=8+N` | — |
| AA58 心跳 | 板卡→上位机 | 不变（原本就有 TIMESTAMP_US，偏移 12..15） | 偏移 12..15 |
| AA59 固件块 ACK | 板卡→上位机 | 不变（原本就有） | 偏移 12..15 |
| AA5B 传感器帧 | 板卡→上位机 | 不变（原本就有） | 偏移 12..15 |

注意：AA58/AA59/AA5B 的时间戳在**固定偏移 12..15**；AA55 上行帧因为沿用旧的 8 位 LEN 布局，时间戳放在**帧尾**，两处不要混淆。

## 2. 上行 AA55 新帧格式

```text
AA 55 | BODY_LEN | SEQ(2) | CAN_ID(4) | FLAGS | LEN | DATA(N) | TIMESTAMP_US(4) | CRC8 | 55 AA
```

| 字节偏移 | 字节数 | 含义 |
| --- | --- | --- |
| 0..1 | 2 | 帧头 `AA 55` |
| 2 | 1 | BODY_LEN = `12 + N`（旧固件为 `8 + N`） |
| 3..4 | 2 | SEQ，16 位序号，小端，板卡独立递增 |
| 5..8 | 4 | CAN ID，小端（电机优先级分类也按此偏移，未变） |
| 9 | 1 | FLAGS，同旧版 |
| 10 | 1 | LEN = N |
| 11..10+N | N | DATA |
| 11+N | 4 | **TIMESTAMP_US，新增**，uint32 小端，微秒 |
| 15+N | 1 | CRC8（覆盖 BODY_LEN 至 TIMESTAMP_US 末尾，共 `BODY_LEN+1` 字节） |
| 16+N | 2 | 帧尾 `55 AA` |

总长 `18 + N`（旧版 `14 + N`）。所有多字节整数均为小端。

## 3. TIMESTAMP_US 语义

- **单位**：微秒，但时基是毫秒 tick ×1000，**分辨率 1 ms**——同一毫秒内到达的多帧时间戳相同，不能用于毫秒内的细粒度排序（帧的到达顺序由串口流本身保证）。
- **捕获时刻**：CAN 数据上报帧记录**中断从 FDCAN RX FIFO 读出该帧的时刻**（总线到达时间，不含软件队列排队延迟）；状态/提示帧与配置回复记录**封装时刻**。
- **回绕**：32 位无符号，约 **71.6 分钟**回绕一次，与 AA58/AA59/AA5B 的时间戳同一时基，可直接跨帧族差分。上位机建议用 64 位扩展：每次检测到 `new < old` 就给高位加 1（参照 AA5B `sample_seq` 的连续性处理）。

## 4. 新旧固件自动识别（建议做兼容）

LEN 仍固定在偏移 10，两代格式可以无歧义共存，解析器按 BODY_LEN 自适应即可：

```text
body_len == 12 + N  → 新格式（上行含时间戳）
body_len ==  8 + N  → 旧格式（无时间戳；仅旧固件会出现）
```

若确定现场只跑新固件，直接按新格式解析即可。若需要双版本兼容，注意 `body_len == 20` 时既可能是新格式 N=8、也可能是旧格式 N=12，**单看 BODY_LEN 有歧义**：请先按新格式验 CRC，失败再按旧格式（无时间戳）重试一次，以 CRC 通过者为最终判据。

## 5. 配置回复帧变化（FLAGS=0x80，命令字 0x81）

仅当上位机实现了"修改 CAN 速率"功能时需要关注。包体 17→21 字节，总长 23→27 字节：

| 回复偏移 | 字节数 | 含义 |
| --- | --- | --- |
| 0..1 | 2 | AA 55 |
| 2 | 1 | 0x15（包体 21 字节；旧版 0x11） |
| 3..4 | 2 | 请求序号 |
| 5..8 | 4 | 00 00 00 00 |
| 9 | 1 | 0x80 |
| 10 | 1 | 0x81 |
| 11 | 1 | STATUS |
| 12..15 | 4 | 仲裁速率，小端，bit/s |
| 16..19 | 4 | 数据速率，小端，bit/s |
| **20..23** | **4** | **TIMESTAMP_US（新增）** |
| 24 | 1 | CRC8（覆盖偏移 2..23 共 22 字节；旧版覆盖 2..19 共 18 字节） |
| 25..26 | 2 | 55 AA |

## 6. CRC8 算法（未变，覆盖范围变了）

多项式 `0x07`，初值 `0x00`，不反射，无异或输出。覆盖从 BODY_LEN 字节到 BODY 最后一个字节（上行即到 TIMESTAMP_US 末尾），共 `BODY_LEN + 1` 字节；不含帧头、CRC 自身、帧尾。

## 7. 参考拆包代码（Python 3，可直接改抄）

```python
def crc8(data):
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def unpack_aa55_uplink(buf, ts_state=[0, 0, 0]):
    """从缓存 buf 尝试解析一帧板卡上行 AA55（新格式）。

    返回 (packet, consumed)：
      packet != None  解析出一帧，consumed 为该帧总长；
      packet == None  consumed 为应丢弃的字节数（0 表示等待更多数据）。
    ts_state = [unused, last_raw, wrap_base]：用于把 32 位回绕时间戳
    扩展成 64 位微秒；首次调用前初始化为 [0, 0, 0]。
    """
    i = buf.find(b'\xAA\x55')
    if i > 0:
        return None, i                      # 丢弃帧头前杂字节
    if len(buf) < 3:
        return None, 0
    body_len = buf[2]
    if not (12 <= body_len <= 76):          # 上行合法范围（12+N 数据帧与 0x15 配置回复）
        return None, 2                      # 假帧头，跳过 AA 后重找
    total = body_len + 6
    if len(buf) < total:
        return None, 0                      # 半包，等待更多字节
    if buf[total - 2:total] != b'\x55\xAA':
        return None, 2
    ok = crc8(buf[2:3 + body_len]) == buf[body_len + 3]
    seq = int.from_bytes(buf[3:5], 'little')
    raw_ts = int.from_bytes(buf[body_len - 1:body_len + 3], 'little')

    # 32 位回绕 → 64 位扩展：仅当原始值大幅倒退（超过半程，即真实回绕）
    # 才累加扩展位；小幅倒退来自双队列优先级重排或同毫秒抖动，不调整。
    prev_raw = ts_state[1]
    if raw_ts < prev_raw and (prev_raw - raw_ts) > 0x80000000:
        ts_state[2] += 1 << 32
    ts_state[1] = raw_ts
    ts64 = ts_state[2] | raw_ts

    if buf[9] == 0x80:                      # 配置回复，body_len 应为 0x15
        if body_len != 0x15:
            return None, 2
        packet = {'type': 'config_rsp', 'seq': seq, 'cmd': buf[10],
                  'status': buf[11],
                  'nominal_bps': int.from_bytes(buf[12:16], 'little'),
                  'data_bps': int.from_bytes(buf[16:20], 'little'),
                  'timestamp_us': ts64, 'crc_ok': ok}
    else:                                   # 普通 CAN 上报 / 状态帧
        n = buf[10]
        if body_len != 12 + n:
            return None, 2
        packet = {'type': 'can', 'seq': seq,
                  'can_id': int.from_bytes(buf[5:9], 'little'),
                  'flags': buf[9], 'len': n,
                  'data': bytes(buf[11:11 + n]),
                  'timestamp_us': ts64, 'crc_ok': ok}
    return packet, total
```

要点：

- `TIMESTAMP_US` 取 `buf[body_len-1 : body_len+3]`（CRC 前推 4 字节），CAN 帧、状态帧、配置回复都满足这个关系，不要按帧类型分别数偏移。
- CRC 输入为 `buf[2 : 3+body_len]`（共 `body_len+1` 字节），与旧版算法相同、范围随包体加长。
- 若需兼容旧固件（`body_len == 8+N`，无时间戳），见第 4 节的 CRC 判据说明。

## 8. 测试向量（可直接做单元测试）

以下包均合法、CRC 已按固件算法算好；时间戳字段固定为可预期的值便于断言。**各向量相互独立，单测时每个向量用全新解析状态**（回绕扩展清零），否则前一个向量的时间戳会影响后一个的扩展位。

**① CAN 数据上报帧**（新格式，N=8，ID=0x123，FLAGS=00，SEQ=1，TS=0）：

```text
AA 55 14 01 00 23 01 00 00 00 08 11 22 33 44 55 66 77 88 00 00 00 00 69 55 AA
```

期望解析：can_id=0x123，len=8，data=11 22 33 44 55 66 77 88，timestamp_us=0，总长 26。

**② 状态包 UART_RX!**（N=8，ID=0x7FE，SEQ=1，TS=0）：

```text
AA 55 14 01 00 FE 07 00 00 00 08 55 41 52 54 5F 52 58 21 00 00 00 00 B2 55 AA
```

期望解析：data 的 ASCII 为 `UART_RX!`，timestamp_us=0。

**③ CAN FD 上报帧**（N=12，ID=0x123，FLAGS=02，SEQ=1，TS=1000µs）：

```text
AA 55 18 01 00 23 01 00 00 02 0C AA BB CC DD EE FF 01 02 03 04 05 06 E8 03 00 00 9E 55 AA
```

期望解析：len=12，timestamp_us=1000，总长 30。

**④ 波特率配置回复**（请求 SEQ=2，STATUS=00，500 kbit/s + 5 Mbit/s，TS=0）：

```text
AA 55 15 02 00 00 00 00 00 80 81 00 20 A1 07 00 40 4B 4C 00 00 00 00 00 C9 55 AA
```

期望解析：status=0，nominal_bps=500000，data_bps=5000000，总长 27。

## 9. 常见坑

1. **不要把时间戳当 DATA**：LEN 仍是真实数据长度，DATA 截取 `buf[11:11+N]`，时间戳在它后面；按 `body_len == 12 + N` 校验可以兜底。
2. **不能再用"总长 78"做缓冲区上限假设**：新帧最长 82 字节（N=64 的 FD 帧），接收缓冲至少留 96 字节余量。
3. **同毫秒时间戳相同**是正常现象（分辨率 1 ms），排序以上位机接收顺序为准，时间戳用于测量排队/转发延迟和跨帧族对齐。
4. **半包/粘包**处理逻辑与旧版完全一致：按 `BODY_LEN + 6` 收齐再校验帧尾和 CRC，坏包从候选帧头后重扫。
5. 旧版无帧尾的 76 字节报文依然不兼容；下行帧（上位机发给板卡）不要自行加时间戳，板卡按 `8+N` 校验会拒绝。
