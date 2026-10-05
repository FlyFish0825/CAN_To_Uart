# STM32H750 CAN / USB CDC 网关使用说明

本项目通过 USB CDC 虚拟串口接收电脑命令，将 CAN 帧提交到 FDCAN1；同时将 CAN 接收帧封装后返回电脑。当前采用帧头、长度、CRC、帧尾协议，以及 USB 接收/发送队列和循环缓冲区。UART 外设文件仍可能由 CubeMX 保留，但不属于 CAN 网关运行时通信路径。

## 文档索引

| 分类 | 入口 | 内容 |
| --- | --- | --- |
| 使用与协议 | [快速上手](#1-快速上手)、[普通 CAN 帧协议](#3-普通-can-帧的串口协议)、[状态回复](#4-状态回复怎么读) | USB CDC、AA55/AA58/AA59、CRC、流控和状态包 |
| 构建与维护 | [编译、烧录与维护](#6-编译烧录与维护)、[测试与验证](#10-测试验证入口与审查清单) | CMake 构建、烧录边界、主机测试和板上验证范围 |
| 源码索引 | [工程边界与文件分工](#8-工程边界与文件分工)、[手写模块与配置文件索引](#9-手写模块与配置文件索引) | CubeMX 生成区、手写模块、配置文件和职责划分 |
| 硬件参考 | [硬件资料目录](docs/hardware/) | 原理图和 STM32H750 数据手册，按资料归档，不作为自动化构建输入 |

项目根目录保留工程入口文件；补充资料统一放在 `docs/` 二级目录下。第三方库自带的 LICENSE、示例说明和测试说明继续随其所属库目录保存。

## 1. 快速上手

1. 编译并烧录本项目固件，使用 USB 数据线连接板卡与电脑。
2. 电脑端打开枚举出来的 USB CDC 虚拟串口，发送与接收均选择 **Hex**，发送时选择“无追加”（不加换行）。USB CDC 不需要配置波特率、校验位或停止位。
3. 板卡复位后会通过 USB CDC 输出启动提示。该提示不发送到 CAN 总线。
4. 将下面完整的 22 字节复制到发送框，单击发送：

```text
AA 55 10 01 00 23 01 00 00 00 08 11 22 33 44 55 66 77 88 BC 55 AA
```

这条命令发送标准经典 CAN 数据帧：ID 为 `0x123`，8 字节数据为 `11 22 33 44 55 66 77 88`，默认 CAN 速率为 **1 Mbit/s**。

低速手动单次测试通常会看到输入确认和 `CAN_PUT!` 两条状态。它们证明协议解析、软件入队和硬件 FIFO 提交已执行，**不能证明物理总线发送成功或其他节点收到数据**。

### 1.1 独立的上位机链路心跳

设备还会每隔 **1000 ms** 向上位机发送一帧独立的 `AA 58` System PING。它只表示“设备与上位机之间的公共通信链路仍在运行”，不经过 CAN 解析器，不包含 CAN ID、CAN FLAGS、CAN LEN 或 CAN DATA，也不会发送到 CAN 总线；同时携带关键软件缓冲区占用百分比，便于上位机诊断是否接近拥塞。

固定长度为 25 字节，所有多字节字段均为小端序：

```text
AA 58 01 01 00 00 SEQ(4) 05 00 TIMESTAMP_US(4) STATUS(5) CRC16 58 AA
```

| 偏移 | 长度 | 含义 |
| --- | --- | --- |
| 0 | 1 | 起始字节 `AA` |
| 1 | 1 | 协议族 `58`，表示 System/ROV |
| 2 | 1 | 版本 `01` |
| 3 | 1 | 命令 `01`，表示 `PING` |
| 4 | 1 | FLAGS，当前为 `00` |
| 5 | 1 | TARGET，当前为 `00`（系统） |
| 6..9 | 4 | SEQ，成功入队的心跳序号 |
| 10..11 | 2 | PAYLOAD_LEN，当前固定为 `0005` |
| 12..15 | 4 | TIMESTAMP_US，设备运行时间的微秒近似值 |
| 16 | 1 | USB RX 缓冲区占用百分比 |
| 17 | 1 | USB TX 缓冲区占用百分比 |
| 18 | 1 | CAN RX 软件队列占用百分比 |
| 19 | 1 | CAN TX 软件队列占用百分比 |
| 20 | 1 | AA59 逻辑块队列占用百分比 |
| 21..22 | 2 | CRC16-CCITT，低字节在前 |
| 23..24 | 2 | 帧尾 `58 AA` |

CRC16-CCITT 参数为多项式 `0x1021`、初值 `0xFFFF`，校验范围是偏移 `1..20`（从 `58` 到 5 个状态字节），不包含起始 `AA`、CRC 自身和帧尾。百分比为无符号整数 `0~100`，100 表示对应软件队列达到可用容量上限。上位机连续约 **2~3 秒**没有收到合法心跳时，可将公共链路标记为离线；心跳离线不等同于 CAN 总线没有节点回复。

### 1.2 连续传输时的流控与卡死保护

连续发送 AA55 数据时不需要在每个包后固定延时。固件通过队列水位和 USB 反压自动调节：

- USB RX 每轮最多解析 256 字节，主循环会持续服务 CAN 和 USB TX；
- USB RX 缓冲不足、USB TX 队列达到 192/255、或 CAN 软件发送队列达到 48/63 时，暂不重新提交下一次 USB OUT 接收；主机收到 USB NAK 后会自然减速；
- USB TX 完成回调超过 1 秒没有回来时，刷新 IN 端点并重试当前包；
- CAN→USB 发送队列忙时保留当前 CAN 帧，不提前移除；FDCAN 明确拒绝某帧时释放该失败帧并返回 `CAN_FAIL`，防止单帧把队列永久锁住；
- 成功诊断提示（`USB_RX!!`、`CAN_PUT!`）按 100 ms 合并发送，不再为每个输入帧制造一个回包，避免诊断数据反过来占满 USB TX 队列；
- FDCAN 进入 Bus-Off 时只在中断里记录事件，由主循环停止并重新启动控制器；软件队列保持不变，恢复后继续提交未发送帧，并报告 `BUS_OFF!`；
- 队列满或下游暂忙不会覆盖旧数据，也不会把当前队列项静默当作已完成。

这里的水位是传输层保护，不改变 AA55 报文格式。当前 AA55 协议只描述单个 CAN 帧，没有定义 `block_index`、`valid_len` 等固件逻辑块字段，因此不能在没有上层格式约定的情况下伪造逻辑块 Credit/ACK。

### 1.3 AA59 固件逻辑块通道

如果上位机需要按固件逻辑块连续传输，使用独立的 AA59 通道；AA55 仍然
保持原来的“一个包对应一个 CAN 帧”兼容格式。AA59 的公共帧格式为：

```text
AA 59 VERSION CMD FLAGS TARGET SEQ(4) PAYLOAD_LEN(2) TIMESTAMP_US(4)
PAYLOAD CRC16-CCITT(2) 59 AA
```

所有多字节字段均为小端序。CRC16-CCITT 多项式为 `0x1021`、初值为
`0xFFFF`，覆盖从 `59` 到 PAYLOAD 末尾的所有字节，不包含起始 `AA`、
CRC 自身和帧尾 `59 AA`。

#### 1.3.1 命令顺序

一次传输按以下顺序执行：

1. `CMD=0x06` BEGIN，PAYLOAD 为 `firmware_size(4)`；
2. 连续发送一个或多个 `CMD=0x10` DATA_BLOCK；
3. 等待最后一块完成后发送 `CMD=0x11` END。

USB 字节流由传输层先按 `AA 55` / `AA 59` 帧族和长度路由，再交给对应
解析器；AA59 数据内容中的 `AA 55` 不会被误当成 AA55 CAN 命令。

BEGIN 成功后初始 credit 为 16。上位机最多连续放入 16 个未完成逻辑块；
收到累计 ACK 后，把 `credit_return` 加回本地 credit，再继续发送新块。

#### 1.3.2 DATA_BLOCK 负载

```text
Offset  Size  字段
0       4     CAN_ID
4       1     CAN_FLAGS（bit0 扩展 ID，bit1 FD，bit2 BRS；bit3 不允许）
5       4     block_index
9       4     offset
13      1     valid_len（1~64）
14      N     实际数据，N 必须等于 valid_len
```

`block_index` 必须从 0 连续递增，`offset` 必须等于前一块的结束偏移；
网关会校验 `offset + valid_len <= firmware_size`。最后一块可以是 1、7、8、
9、23、63 或任意不超过 64 的长度，padding 不计入有效数据。

#### 1.3.3 CAN 拆分规则

- Classic CAN：以 `CAN_ID` 为首 ID，每帧最多 8 字节，连续使用
  `CAN_ID + 0`、`CAN_ID + 1`……；`valid_len=23` 时只发送长度为 8、8、7
  的三帧，不补发空分片。
- CAN FD：一个逻辑块发送为一个 CAN FD 帧。物理 DLC 采用合法长度
  `0~8、12、16、20、24、32、48、64` 的向上取值；例如 `valid_len=23`
  时发送 DLC=24，但 ACK 和逻辑长度仍按 23 字节计算，最后 1 字节仅作
  物理 padding。

#### 1.3.4 累计 ACK

网关回复 `CMD=0x80`，PAYLOAD 固定 14 字节：

```text
ack_block_index(4) | credit_return(2) | free_blocks(2) |
status(1) | reserved(3)
```

`ack_block_index` 是已经完成转发的最后一个块；没有完成块时为
`0xFFFFFFFF`。`credit_return` 只统计本次 ACK 新返还的 credit，不能重复
计算；`free_blocks` 是固定 32 块队列当前剩余的实际槽位，发送端仍必须同时
遵守本次会话的 credit。正常 ACK 不逐块发送，满足
以下任一条件才发送：累计完成 4 块、队列降到低水位、距上次 ACK 超过 10 ms、
完成最后一块或发生错误。

状态码：`00 OK`、`01 BUSY`、`02 QUEUE_FULL`、`03 INVALID_BLOCK`、
`04 FORWARD_FAILED`、`05 INTERNAL_ERROR`。

固定参数为：逻辑块队列 32 块、初始 credit 16、高水位 24、低水位 12、
累计 ACK 间隔 4 块。网关只有在一块的所有物理 CAN 分片成功进入核心发送
并且被 FDCAN 硬件 TX FIFO 接受后才返还该块 credit；这只表示控制器接受
了发送请求，不表示总线节点已经 ACK。分片入队失败会保留当前分片位置并重试，明确
失败则返回 `FORWARD_FAILED`，不会返还对应 credit。

## 2. 接线与默认配置

| 接口 | MCU 引脚 | 接法 / 用途 |
| --- | --- | --- |
| USB | USB 接口 | 连接电脑，枚举为 USB CDC 虚拟串口 |
| GND | GND | 板卡与电脑共地（由 USB 线提供） |
| FDCAN1 TX | PD1 | MCU 到 CAN 收发器 TXD |
| FDCAN1 RX | PD0 | CAN 收发器 RXD 到 MCU |
| CANH / CANL | 收发器总线侧 | 接 CAN 总线，不能接 TTL 串口 |

本固件使用 USB CDC 虚拟串口作为电脑接口，不需要外接 USB 转串口模块，也不需要配置传统串口波特率。请根据实际板卡原理图确认 USB 接口连接和供电。

FDCAN 内核时钟配置为 80 MHz；默认仲裁段 1 Mbit/s、FD 数据段 8 Mbit/s。工作模式为 Normal，自动重发关闭。经典 CAN 帧不使用 FD 数据段速率。支持 FD 报文的软件配置不等于板卡收发器及布线已通过对应高速验证。

## 3. 普通 CAN 帧的串口协议

### 3.1 先认识符号与数据方向

这里的“串口帧”是电脑与 H750 之间约定的一包二进制数据；“CAN 帧”是 H750 通过 CAN 控制器发送到总线的数据。串口帧头、序号、长度、CRC 和帧尾不会作为 CAN 数据发送，只有 CAN_ID、FLAGS、LEN、DATA 用于构造 CAN 帧。

电脑 → H750：表示“请发送这条 CAN 帧”。H750 → 电脑：可能是真实 CAN 接收帧，也可能是第 4 节的本地调试提示。方向由连接两端决定，普通报文内没有方向字段。

本文报文框里的数字全部为十六进制，每两个数字表示一个字节：`10` 是十进制 16，`48` 是十进制 72，`AA` 是十进制 170。`0x` 仅是文字中的十六进制前缀。向 Hex 发送框粘贴时使用 `AA 55 ...`，不输入 `0x`、竖线、括号、字段名称或省略号。空格用于分隔字节，不发送为空格字符。若选成文本发送，字符 `AA` 会变成两个 ASCII 字节 `41 41`，不是所需的一个字节 `AA`。

`|` 只是文档分隔符；`SEQ(2)` 表示 SEQ 占 2 字节；`DATA(N)` 表示 DATA 占 N 字节，N 由 LEN 指定。偏移从 0 开始，因此偏移 9 是第 10 个字节。

电脑发送与板卡上报的普通 CAN 帧使用同一格式。所有多字节整数均为**小端序**。

```text
AA 55 | BODY_LEN | SEQ(2) | CAN_ID(4) | FLAGS | LEN | DATA(N) | CRC8 | 55 AA
```

| 字节偏移（从 0 开始） | 字节数 | 含义 |
| --- | --- | --- |
| 0 | 2 | 帧头 `AA 55` |
| 2 | 1 | BODY_LEN，紧凑格式为 `8 + N` |
| 3 | 2 | SEQ，16 位序号 |
| 5 | 4 | CAN ID，标准帧 0..0x7FF，扩展帧 0..0x1FFFFFFF |
| 9 | 1 | FLAGS，见下表 |
| 10 | 1 | LEN，实际数据字节数 N，不是 CAN 硬件 DLC 编码 |
| 11 | N | DATA |
| 11 + N | 1 | CRC8 |
| 12 + N | 2 | 帧尾 `55 AA` |

紧凑格式总长度为 `14 + N`，也就是 `BODY_LEN + 6`。上例 BODY_LEN=`0x10`，LEN=`0x08`，总长 22 字节。

### 3.2 每个字段怎么填写

**帧头 AA 55**：固定两个字节，帮助接收端找到报文起点。不是 CAN ID，也不计入 BODY_LEN。帧尾固定为反向的 `55 AA`，不要写成 `AA 55`。

**BODY_LEN（包体长度）**：包体从 SEQ 开始，到 DATA 最后一个字节结束。其固定部分为 `SEQ 2 + CAN_ID 4 + FLAGS 1 + LEN 1 = 8` 字节，所以紧凑格式填 `8 + N`。不包含帧头、BODY_LEN 自己、CRC、帧尾。例如 N=0 填 `08`；N=8 填 `10`；N=12 填 `14`；N=64 填 `48`。它与 LEN 不是同一个长度。

**SEQ（序号）**：占 2 字节，范围 0..65535。手工测试可固定为 `01 00`；上位机可以每发一帧加 1，65535 后回到 0。当前固件不检查输入序号是否连续，也不去重，不要把重复序号当成重发保护。普通输出序号由板卡独立生成，不能拿它匹配某条输入命令。

**CAN_ID（报文标识符）**：占 4 字节，即使标准帧也不能省掉高位零。它用于 CAN 报文识别和仲裁，不是本协议指定的“节点地址”；哪个 ID 对应哪个设备或动作，由目标设备的 CAN 协议决定。标准 ID 为 11 位，范围 0..0x7FF；扩展 ID 为 29 位，范围 0..0x1FFFFFFF。扩展或标准由 FLAGS bit0 决定，不能只看 ID 数值大小；扩展帧也可以使用数值 0x123。

小端序表示“低位字节先发”，按字节倒序，不是把每个字节内部数字倒序：

| 数值 | 字段长度 | 线上字节 |
| --- | --- | --- |
| SEQ=1 | 2 | `01 00` |
| SEQ=0x1234 | 2 | `34 12` |
| 标准 ID=0x123 | 4 | `23 01 00 00` |
| 扩展 ID=0x18FF50E5 | 4 | `E5 50 FF 18` |

**FLAGS（帧属性）**：占 1 字节，用其中不同的二进制位表达几项独立选择，详见 3.3。它不是要发送给 CAN 节点的数据。

**LEN（数据长度）**：占 1 字节，填写实际字节数 N。例如 8 字节填 `08`，12 字节填 `0C`，64 字节填 `40`。头文件中也称其为 DLC，但线上传的是字节数，不是控制器的编码。

**DATA（数据区）**：填写目标 CAN 协议要求的原始字节。固件按顺序复制，不自动倒序或转换单位。例如 DATA=`12 34`，总线上仍为 `12 34`。若目标协议要求把某个 16 位值按小端编码，需要上位机先完成编码。仅把 CAN ID 改对不能保证设备执行动作，还需遵循目标设备的数据定义。

**CRC8（校验）**：占 1 字节，用于发现串口包内容错误，算法和代码见 3.7。它不是 CAN 总线自身的 CRC，CAN 控制器会自行处理 CAN 帧的总线校验。

**帧尾 55 AA**：位于长度指定的位置，用于检查完整性；不能用“找到任意 55 AA 就结束”代替长度解析，因为 DATA 中也允许出现这两个字节。

### 3.3 FLAGS 每一位与全部合法组合

一个字节共有 8 位，从右往左编号 bit0 到 bit7。某位为 1 表示启用该属性，为 0 表示未启用；选择多项时按位或（这些不同位的数值也可相加）。例如扩展 + FD + BRS 为 `0x01 | 0x02 | 0x04 = 0x07`，线上只发送一个字节 `07`。

| 位 | 掩码 | 为 0 时 | 为 1 时 | 限制 |
| --- | --- | --- | --- | --- |
| bit0 | 0x01 | 标准 ID（11 位） | 扩展 ID（29 位） | 决定 ID 的解释方式与上限 |
| bit1 | 0x02 | 经典 CAN | CAN FD | FD 允许更长数据，不能使用远程帧 |
| bit2 | 0x04 | 不切换数据段速率 | BRS，切换到配置的数据段速率 | 只能与 bit1=1 同用 |
| bit3 | 0x08 | 数据帧 | 远程帧 RTR | 只能与 bit1=0、bit2=0 同用 |
| bit4 | 0x10 | 普通帧必须为 0 | 未定义 | 拒绝作为普通帧发送 |
| bit5 | 0x20 | 普通帧必须为 0 | 未定义 | 拒绝作为普通帧发送 |
| bit6 | 0x40 | 普通帧必须为 0 | 未定义 | 拒绝作为普通帧发送 |
| bit7 | 0x80 | 普通 CAN 报文 | 进入配置命令解析 | 配置必须使用 FLAGS=80，布局见第 5 节 |

经典 CAN 的单帧数据最多 8 字节；FD 的单帧数据最多 64 字节。BRS 是 FD 中的可选属性：FD 不开 BRS 时使用仲裁速率传输，开 BRS 时数据段采用配置的数据段速率。FLAGS 不直接填写速率值，也不改变 USART1 的 115200 波特率。

远程帧是经典 CAN 中用于请求某 ID 数据的帧类型，本身没有数据载荷。网关发送远程帧后不会等待、强制或自动生成节点回复；对端是否回复由对端协议决定。

下面列出**所有合法的普通 CAN FLAGS**，不只是常用值：

| FLAGS | 二进制 bit7..bit0 | ID 类型 | CAN 类型 | BRS | 帧类型 | LEN |
| --- | --- | --- | --- | --- | --- | --- |
| 00 | 00000000 | 标准 | 经典 | 关闭 | 数据 | 0..8 |
| 01 | 00000001 | 扩展 | 经典 | 关闭 | 数据 | 0..8 |
| 02 | 00000010 | 标准 | FD | 关闭 | 数据 | FD 合法长度 |
| 03 | 00000011 | 扩展 | FD | 关闭 | 数据 | FD 合法长度 |
| 06 | 00000110 | 标准 | FD | 开启 | 数据 | FD 合法长度 |
| 07 | 00000111 | 扩展 | FD | 开启 | 数据 | FD 合法长度 |
| 08 | 00001000 | 标准 | 经典 | 关闭 | 远程 | 0..8 个占位字节 |
| 09 | 00001001 | 扩展 | 经典 | 关闭 | 远程 | 0..8 个占位字节 |

`04/05` 为经典 CAN 却开启 BRS，非法；`0A/0B/0E/0F` 同时选择 FD 与远程帧，非法；`0C/0D` 为远程经典帧却开启 BRS，非法。`80` 是配置命令，不是“另一种 CAN 帧”；`81` 等带其他属性的控制 FLAGS 也不被当前固件接受。

常用 FLAGS：`00` 标准经典 CAN，`01` 扩展经典 CAN，`06` 标准 FD+BRS，`07` 扩展 FD+BRS。普通 CAN 帧其余位须为 0；`0x80` 留给配置命令。

经典 CAN 的 LEN 为 0..8；FD 为 0..8、12、16、20、24、32、48、64。FD 不支持远程帧。远程帧协议仍携带 LEN 个占位字节，建议填零，它们不作为 CAN 远程帧的数据发送。

### 3.4 长度速查与固定格式

| N（十进制） | LEN（Hex） | BODY_LEN（Hex） | 紧凑包总字节数 | CAN 硬件 DLC 编码（不要填入 LEN） |
| --- | --- | --- | --- | --- |
| 0..8 | 00..08 | 08..10 | 14..22 | 0..8 |
| 12 | 0C | 14 | 26 | 9 |
| 16 | 10 | 18 | 30 | 10 |
| 20 | 14 | 1C | 34 | 11 |
| 24 | 18 | 20 | 38 | 12 |
| 32 | 20 | 28 | 46 | 13 |
| 48 | 30 | 38 | 62 | 14 |
| 64 | 40 | 48 | 78 | 15 |

例如想用 FD 发送 10 字节，不能填写 LEN=`0A`。应按目标设备协议允许的方式补到 12 字节，然后填 LEN=`0C`，BODY_LEN=`14`，重新计算 CRC。固件不会自动补齐不合法长度。

接收端还接受 BODY_LEN=`0x48` 的固定长度格式：DATA 区占满 64 字节，不足部分填零，CRC 覆盖填充字节，总长 78 字节。优先使用紧凑格式。**旧版无帧尾的 76 字节报文不兼容，不能直接发送。**

固定格式的 SEQ、ID、FLAGS、LEN 偏移不变；DATA 区为偏移 11..74，CRC 为偏移 75，帧尾为偏移 76..77。LEN 仍写真实数据长度，例如经典 CAN 8 字节仍填 `08`。后续填充字节不发送到 CAN，但参与串口 CRC。板卡输出普通帧统一采用紧凑格式，不跟随电脑输入格式。

### 3.5 将快速示例逐项拆开

```text
AA 55 | 10 | 01 00 | 23 01 00 00 | 00 | 08 | 11 22 33 44 55 66 77 88 | BC | 55 AA
```

| 偏移 | 线上字节 | 解释 |
| --- | --- | --- |
| 0..1 | AA 55 | 起始标记 |
| 2 | 10 | 包体 16 字节，8 字节固定字段 + 8 字节数据 |
| 3..4 | 01 00 | 序号 1 |
| 5..8 | 23 01 00 00 | CAN ID=0x00000123 |
| 9 | 00 | 标准 ID、经典 CAN、无 BRS、数据帧 |
| 10 | 08 | 8 字节有效数据 |
| 11..18 | 11 22 33 44 55 66 77 88 | 实际 CAN 数据 |
| 19 | BC | 对偏移 2..18 共 17 字节计算的 CRC |
| 20..21 | 55 AA | 结束标记 |

这条命令中的 CRC 输入完整为 `10 01 00 23 01 00 00 00 08 11 22 33 44 55 66 77 88`，结果 `BC`。只想修改最后一个数据字节时，也必须更新 CRC，不能继续沿用 BC。

### 3.6 接收程序如何拆包

1. 将串口收到的字节追加到接收缓存，查找帧头 `AA 55`，丢弃它前面的杂字节。
2. 至少有 3 字节时读取 BODY_LEN，合法范围为 8..72；范围错误则继续寻找帧头。
3. 缓存不足 `BODY_LEN + 6` 字节时继续等待，不能将当前半包当成完整包。
4. 检查偏移 `BODY_LEN + 4` 与 `BODY_LEN + 5` 为 `55 AA`，检查偏移 `BODY_LEN + 3` 的 CRC。
5. 再按 FLAGS 区分普通帧与配置回复，检查各字段语义；消费这一包后继续处理缓存中剩余数据。

这是上位机的建议拆包流程。上位机遇到坏包可从候选帧头后一字节重新搜索，并自行设计半包超时。当前固件在已接受的长度收满后才判定坏包，并不会回扫该坏包内部寻找下一个帧头；也未实现半包超时。因此不能宣称丢字节后下一帧必定立即恢复。

数据中允许出现 `AA 55` 或 `55 AA`，无需转义；解析器根据 BODY_LEN 定位帧尾。串口读操作可能得到半帧或多帧，上位机应累计字节后按长度拆包，不能把一次串口读取当成一帧。

普通输入帧的 SEQ 不用于去重，重复发送同一 SEQ 会再次入队。板卡普通输出使用自己的递增序号，不回显命令 SEQ；配置回复例外。

### 3.7 CRC 与生成示例

CRC-8：多项式 `0x07`，初值 `0x00`，输入/输出均不反射，最终异或 `0x00`。计算范围从 BODY_LEN 到 BODY 最后一个字节，包含长度字节，共 `BODY_LEN + 1` 字节；不包含帧头、CRC 自身和帧尾。

以下 Python 3 代码无需第三方库，只生成十六进制报文，不打开串口。修改 ID、FLAGS、DATA 后必须重新计算 CRC。

```python
def crc8(data):
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ (0x07 if crc & 0x80 else 0)) & 0xFF
    return crc


def wrap(body):
    checked = bytes([len(body)]) + body
    return b'\xAA\x55' + checked + bytes([crc8(checked)]) + b'\x55\xAA'


def can_packet(can_id, data, seq=1, flags=0):
    data = bytes(data)
    assert 0 <= seq <= 0xFFFF
    assert 0 <= flags <= 0x0F
    assert 0 <= can_id <= (0x1FFFFFFF if flags & 1 else 0x7FF)
    if flags & 2:
        assert not flags & 8
        assert len(data) in (*range(9), 12, 16, 20, 24, 32, 48, 64)
    else:
        assert not flags & 4
        assert len(data) <= 8
    body = (seq.to_bytes(2, 'little') + can_id.to_bytes(4, 'little')
            + bytes([flags, len(data)]) + data)
    return wrap(body)


packet = can_packet(0x123, bytes.fromhex('11 22 33 44 55 66 77 88'))
assert packet == bytes.fromhex(
    'AA 55 10 01 00 23 01 00 00 00 08 11 22 33 44 55 66 77 88 BC 55 AA')
print(packet.hex(' ').upper())
```

将上述代码保存为 `make_packet.py`，安装 Python 3 后运行 `python make_packet.py`，把打印的一整行复制进串口助手 Hex 发送框。配置生成示例需要放在同一文件、上述函数定义之后。这里的 assert 用于提示示例参数错误，不应使用 Python 的 `-O` 选项运行这些示例。

### 3.8 八种帧类型的完整发送示例

下面全部为紧凑格式，SEQ=1，已按软件算法校验长度与 CRC。标准 ID 使用 0x123，扩展 ID 使用 0x18FF50E5；数据帧用递增数据，远程帧用零占位。它们说明如何编码，不代表这些 CAN ID/数据适合你的目标设备，也不代表所有类型已在板上实测。

标准经典数据帧，FLAGS=00，8 字节数据：

```text
AA 55 10 01 00 23 01 00 00 00 08 01 02 03 04 05 06 07 08 55 55 AA
```

扩展经典数据帧，FLAGS=01，8 字节数据：

```text
AA 55 10 01 00 E5 50 FF 18 01 08 01 02 03 04 05 06 07 08 03 55 AA
```

标准 FD 数据帧，不开启 BRS，FLAGS=02，12 字节数据：

```text
AA 55 14 01 00 23 01 00 00 02 0C 01 02 03 04 05 06 07 08 09 0A 0B 0C AA 55 AA
```

扩展 FD 数据帧，不开启 BRS，FLAGS=03，12 字节数据：

```text
AA 55 14 01 00 E5 50 FF 18 03 0C 01 02 03 04 05 06 07 08 09 0A 0B 0C 22 55 AA
```

标准 FD 数据帧，开启 BRS，FLAGS=06，12 字节数据：

```text
AA 55 14 01 00 23 01 00 00 06 0C 01 02 03 04 05 06 07 08 09 0A 0B 0C 37 55 AA
```

扩展 FD 数据帧，开启 BRS，FLAGS=07，12 字节数据：

```text
AA 55 14 01 00 E5 50 FF 18 07 0C 01 02 03 04 05 06 07 08 09 0A 0B 0C BF 55 AA
```

标准经典远程帧，FLAGS=08，请求长度 8，串口带 8 个零占位：

```text
AA 55 10 01 00 23 01 00 00 08 08 00 00 00 00 00 00 00 00 22 55 AA
```

扩展经典远程帧，FLAGS=09，请求长度 8，串口带 8 个零占位：

```text
AA 55 10 01 00 E5 50 FF 18 09 08 00 00 00 00 00 00 00 00 74 55 AA
```

第一条最后三个字节为 `55 55 AA`，第一个 `55` 是 CRC，后两个才是帧尾，这是合法报文。不要因为 CRC 恰好等于标记字节而删除它。

## 4. 状态回复怎么读

状态包只在串口输出，使用普通 CAN 帧形状，FLAGS=0，LEN=8。

例如以下为 SEQ=1 的 UART_RX! 状态包（实际 SEQ、CRC 随运行改变）：

```text
AA 55 10 01 00 FE 07 00 00 00 08 55 41 52 54 5F 52 58 21 6D 55 AA
```

`FE 07 00 00` 解码为 ID=0x7FE；`55 41 52 54 5F 52 58 21` 按 ASCII 解码为 `UART_RX!`。串口助手选择 Hex 时只会显示这些字节，不一定直接显示英文。不能把整包都当作文本，因为其中还包含二进制头部和 CRC。

| ID | DATA 的 ASCII 内容 | 含义 |
| --- | --- | --- |
| 0x7FF | UART_TX! | 启动提示，串口发送链路有输出 |
| 0x7FE | UART_RX! | 输入校验通过且已进入 CAN 软件发送队列 |
| 0x7FD | CAN_PUT! | HAL 已接受帧并写入 CAN 硬件发送 FIFO |
| 0x7FC | CRC_ERR! | CRC 不匹配 |
| 0x7FB | PKT_ERR! | 帧尾、长度、标志、ID 或命令等格式错误 |
| 0x7FA | CAN_FAIL | HAL 提交 CAN 发送 FIFO 失败 |

这些是调试事件提示，不是逐帧可靠 ACK：主循环会合并相同类型的多次事件；输出队列满时状态包也可能丢失。不能按提示条数统计 CAN 帧数，也不能靠它确认节点回复。

真实 CAN 接收帧会使用相同的串口格式转发，因此总线上 ID=0x7FA..0x7FF 且数据相同的报文可能与提示混淆。当前协议没有独立状态消息类型，上位机集成时应避免此冲突。

## 5. 修改 CAN 速率（可选）

配置命令只在当前运行期间生效，不写入 Flash，复位恢复默认值。仲裁段支持 50000、100000、125000、250000、500000、800000、1000000 bit/s；数据段支持 500000、1000000、2000000、4000000、5000000、8000000 bit/s。

请求格式为：

```text
AA 55 | 10 | SEQ(2) | 00 00 00 00 | 80 | 01 | 仲裁速率(4) | 数据速率(4) | CRC | 55 AA
```

两个速率均为小端 uint32，单位 bit/s。这里偏移 10 是命令字 `01`，不是数据长度。使用上面的 wrap 函数可生成命令：

| 请求偏移 | 字节数 | 含义 / 填法 |
| --- | --- | --- |
| 0..1 | 2 | AA 55 |
| 2 | 1 | 10，包体 16 字节 |
| 3..4 | 2 | 自选序号，回复会原样带回 |
| 5..8 | 4 | 保留 ID，发送 00 00 00 00，不发送到 CAN |
| 9 | 1 | 80，配置标志 |
| 10 | 1 | 01，设置 CAN 速率命令 |
| 11..14 | 4 | 仲裁速率，例如 500000=0x0007A120，填 20 A1 07 00 |
| 15..18 | 4 | 数据速率，例如 5000000=0x004C4B40，填 40 4B 4C 00 |
| 19 | 1 | 对偏移 2..18 计算 CRC |
| 20..21 | 2 | 55 AA |

即使仅使用经典 CAN，也必须填写受支持的数据段速率值。此命令不改变串口波特率。当前固件接受包体长度至少 16 的配置请求并忽略额外包体字节，但发送程序应统一使用这里定义的 16 字节包体，不依赖这种宽松处理。

```python
body = ((2).to_bytes(2, 'little') + bytes(4) + bytes([0x80, 0x01])
        + (500000).to_bytes(4, 'little') + (5000000).to_bytes(4, 'little'))
print(wrap(body).hex(' ').upper())
```

对应完整命令为（SEQ=2，500 kbit/s + 5 Mbit/s）：

```text
AA 55 10 02 00 00 00 00 00 80 01 20 A1 07 00 40 4B 4C 00 02 55 AA
```

回复共 23 字节，格式为：

```text
AA 55 | 11 | 请求SEQ(2) | 00 00 00 00 | 80 | 81 | STATUS | 当前仲裁速率(4) | 当前数据速率(4) | CRC | 55 AA
```

| 回复偏移 | 字节数 | 含义 |
| --- | --- | --- |
| 0..1 | 2 | AA 55 |
| 2 | 1 | 11，即包体 17 字节，不是十进制 11 |
| 3..4 | 2 | 请求序号 |
| 5..8 | 4 | 00 00 00 00 |
| 9 | 1 | 80，配置消息 |
| 10 | 1 | 81，设置速率的回复命令字；不要与偏移 9 的 FLAGS 混淆 |
| 11 | 1 | STATUS，结果码 |
| 12..15 | 4 | 软件记录的仲裁速率，小端，bit/s |
| 16..19 | 4 | 软件记录的数据速率，小端，bit/s |
| 20 | 1 | 对偏移 2..19 共 18 字节计算 CRC |
| 21..22 | 2 | 55 AA |

配置回复没有普通帧的 LEN 字段，上位机不能用偏移 10 的 `81` 当作数据长度。应先按外层 BODY_LEN 收齐包并验 CRC，再根据 FLAGS 和命令字解析。不要把板卡发回的 `81` 回复直接回发给板卡：它不是有效的设置请求。

STATUS：`00` 成功，`01` 不支持的速率，`02` 应用失败。应用失败后的速率字段仅反映软件最后记录值，不保证控制器仍正常工作，应复位排查。切换会停止并重新初始化 FDCAN；停止业务流量后单条发送配置命令，等回复再恢复业务，避免连续配置覆盖待处理命令或影响在途帧。此功能尚未完成板上验证。

## 6. 编译、烧录与维护

需要 CMake（项目要求至少 3.22）、Ninja 和 PATH 中可用的 Arm GNU 工具链 `arm-none-eabi-gcc`。在项目根目录执行：

```powershell
cmake --preset Debug
cmake --build --preset Debug --parallel
cmake --preset Release
cmake --build --preset Release --parallel
```

产物分别为 `build/Debug/CAN_To_Uart.elf`、`build/Release/CAN_To_Uart.elf`。通过 ST-LINK/SWD 和 STM32CubeProgrammer 或调试器加载对应 ELF，完成下载后复位。编译不会自动烧录。不要把构建目录加入版本控制。

| 文件 | 维护内容 |
| --- | --- |
| Core/Src/can_gateway_core.c | 协议、CRC、环形队列、CAN 转发、状态与速率命令 |
| Core/Inc/can_gateway_core.h | 核心函数指针接口、初始化、轮询和输入入口 |
| Core/Src/firmware_flow.c | AA59 逻辑块解析、固定队列、CAN 拆分和累计 ACK |
| Core/Inc/firmware_flow.h | AA59 块通道命令、队列水位和状态码定义 |
| Core/Inc/can_gateway_protocol.h | AA55 协议字段和数据格式说明 |
| Core/Src/main.c | 网关初始化及主循环调用 |
| Core/Src/usart.c | USART1 参数及 RX/TX DMA 初始化 |
| Core/Src/stm32h7xx_it.c | USART1 和 DMA 中断入口 |
| Core/Src/fdcan.c | CAN 引脚、时钟源、默认时序和 FIFO |
| STM32H750XX_FLASH.ld | .dma_buffer 放置到 RAM_D2 |
| CAN_To_Uart.ioc | CubeMX 外设配置 |
| CMakeLists.txt | 网关源文件加入构建 |

RX 使用 DMA1 Stream0 循环模式，256 字节 DMA 缓冲，通过 IDLE、半满和全满事件搬入软件环形缓冲。软件环形缓冲分配 1024 字节、可用 1023 字节。TX 使用 DMA1 Stream1 普通模式，16 个队列槽、可用 15 个。CAN 收发软件队列各 64 槽、可用 63 帧；CAN 硬件 TX FIFO 为 3 帧。

DMA 缓冲区位于 D2 SRAM，32 字节对齐，避免落入 DMA1 不可访问的 DTCM。当前未启用 D-Cache；后续启用时必须处理 DMA 缓存一致性（非缓存区或正确的缓存维护），仅地址对齐并不足够。保持主循环持续调用 CanGateway_Process 和传输层服务函数，避免加入长时间阻塞操作。

CubeMX 再生成代码后检查 DMA 初始化顺序、中断入口、链接脚本 .dma_buffer 和 CMake 网关源文件条目；当前再生成流程未验证，不能仅凭 .ioc 存在就认为自定义内容一定保留。

## 7. 排查与验证范围

AA59 固件块状态机提供了不依赖开发板的主机单元测试：

```powershell
gcc -std=c11 -Wall -Wextra -Werror -O0 -ICore/Inc `
  tests/firmware_flow_host_test.c -o build/firmware_flow_host_test.exe
.\build\firmware_flow_host_test.exe
```

测试直接编译工程中的 `firmware_flow.c`，覆盖 1/7/8/9/23/63/64 字节
尾块、128 字节、`64*16`、1000 字节、10243 字节、Classic 分片、CAN FD
DLC、初始 credit、累计 ACK、队列满后重试、CAN BUSY、CAN 失败、ACK
发送 BUSY 后重试以及 CRC 错误。通过时输出：

```text
firmware_flow_host_test: PASS
```

| 现象 | 检查方向 |
| --- | --- |
| 复位没有 UART_TX! | 确认固件已下载、COM 口、115200/8N1、PA9 接模块 RX、共地及供电 |
| 有启动提示，发送没有 UART_RX! | 检查模块 TX 到 PA10，发送 Hex、无追加、完整帧长和帧尾 |
| CRC_ERR! | ID/序号/数据变化后是否重算 CRC；是否遗漏字节或混用旧协议 |
| PKT_ERR! | BODY_LEN、LEN、帧尾、ID 范围和 FLAGS 组合是否合法 |
| 有 UART_RX!，没有 CAN_PUT! | 检查 CAN FIFO 是否满、控制器状态和发送队列计数 |
| 有 CAN_PUT!，总线无波形 | 先测 PD1，再测收发器 CANH/CANL；核对供电、待机脚、连接的 CAN 通道及终端电阻 |

可以在没有其他节点回复的情况下用示波器检查发送尝试；CAN_PUT! 不代表收到 ACK。FDCAN 的 Error-Warning、Error-Passive、Bus-Off 状态会被记录；Bus-Off 或硬件 TX FIFO 连续 100 ms 不释放时，主循环会重新启动控制器，避免软件队列永久停住。持续无节点、收发器断开或物理层异常仍需在实机上确认波形和恢复效果。

缓冲采用水位反压，正常达到处理能力上限时会暂停 USB OUT，而不是覆盖旧数据；极端情况下仍应观察 `uart_rx_ring_drop_count`、`uart_tx_queue_drop_count`、`can_tx_drop_count`、`can_rx_drop_count`、`can_rx_hw_lost_count`、`uart_rx_error_count` 和 `can_tx_fail_count`。这些计数目前不通过独立查询命令输出。AA55、AA59 和 USB 协议路由器均设置了 1000 ms 半帧超时；输入残缺帧超过该时间后会自动丢弃并重新寻找帧头，不需要复位设备。

截至本次交接：Debug/Release 已编译通过；用户板上截图显示启动提示，且两次完整 22 字节命令均产生 UART_RX! 与 CAN_PUT!。CAN 物理波形、节点接收、CAN 转串口方向、FD 高速、速率切换、长时间连续流量以及异常恢复仍需板上验证。

## 8. 工程边界与文件分工

### 8.1 工程用途与边界

本工程面向 **STM32H750VBTx**，把 USB FS CDC 虚拟串口与 FDCAN1 连接起来：电脑侧输入 AA55 或 AA59 协议报文，固件解析后把 CAN 帧提交给 FDCAN1；FDCAN1 收到的报文以及协议状态再经 USB CDC 返回电脑。AA58 心跳属于电脑与设备之间的诊断链路，不经过 CAN。

本工程不是 CAN 总线分析仪、上位机软件、CAN 收发器或物理层测试夹具；固件也不负责解释目标节点的业务数据。`CAN_PUT!`、AA59 的成功 ACK 只表示软件/控制器接受了发送请求，不等价于总线上已有节点收到并确认。收发器供电、终端电阻、布线、节点 ACK 和目标设备协议必须另行验证。

### 8.2 CubeMX 生成区与手写区

`.ioc` 将目标工具链设为 CMake、芯片设为 STM32H750VBTx、USB 类设为 CDC，并配置 FDCAN1、USART1、DMA 和时钟。由 CubeMX/STM32Cube 固件提供或参与生成的内容主要包括：

- [Core/Inc](Core/Inc/) 中的 HAL 配置、外设声明和中断声明；[Core/Src/main.c](Core/Src/main.c)、[Core/Src/gpio.c](Core/Src/gpio.c)、[Core/Src/dma.c](Core/Src/dma.c)、[Core/Src/fdcan.c](Core/Src/fdcan.c)、[Core/Src/usart.c](Core/Src/usart.c)、[Core/Src/stm32h7xx_it.c](Core/Src/stm32h7xx_it.c)、[Core/Src/stm32h7xx_hal_msp.c](Core/Src/stm32h7xx_hal_msp.c)、[Core/Src/system_stm32h7xx.c](Core/Src/system_stm32h7xx.c)、[Core/Src/syscalls.c](Core/Src/syscalls.c)、[Core/Src/sysmem.c](Core/Src/sysmem.c)；
- [USB_DEVICE](USB_DEVICE/)、[Drivers](Drivers/) 和 [Middlewares](Middlewares/) 下的 USB、HAL、CMSIS 及供应商库；这些目录不在本文逐个列出第三方文件；
- [startup_stm32h750xx.s](startup_stm32h750xx.s)、[STM32H750XX_FLASH.ld](STM32H750XX_FLASH.ld) 以及 [cmake/stm32cubemx/CMakeLists.txt](cmake/stm32cubemx/CMakeLists.txt) 中的生成构建输入。

手写/应用层入口是 [Core/Inc/can_gateway_core.h](Core/Inc/can_gateway_core.h)、[Core/Src/can_gateway_core.c](Core/Src/can_gateway_core.c)、[Core/Inc/can_gateway_protocol.h](Core/Inc/can_gateway_protocol.h)、[Core/Inc/firmware_flow.h](Core/Inc/firmware_flow.h)、[Core/Src/firmware_flow.c](Core/Src/firmware_flow.c)、[Core/Inc/system_heartbeat.h](Core/Inc/system_heartbeat.h)、[Core/Src/system_heartbeat.c](Core/Src/system_heartbeat.c)、[Core/Inc/usb_can_gateway.h](Core/Inc/usb_can_gateway.h) 和 [Core/Src/usb_can_gateway.c](Core/Src/usb_can_gateway.c)。[Core/Src/main.c](Core/Src/main.c) 中 `USER CODE` 区域把这些模块注册到主循环；因此它既包含生成初始化，也包含应用集成代码，不能简单视为纯生成文件。

重新生成前后应重点比对 `main.c` 的初始化/主循环调用、CMake 中的四个应用源文件、链接脚本中的 `.dma_buffer` 放置，以及 USB CDC 回调是否仍把数据交给 [Core/Src/usb_can_gateway.c](Core/Src/usb_can_gateway.c)。

## 9. 手写模块与配置文件索引

### 9.1 核心接口与实现

| 文件 | 当前职责 |
| --- | --- |
| [can_gateway_protocol.h](Core/Inc/can_gateway_protocol.h) | AA55 字段、标志位、长度和协议约束的共享定义 |
| [can_gateway_core.h](Core/Inc/can_gateway_core.h) / [can_gateway_core.c](Core/Src/can_gateway_core.c) | AA55 解析、CRC、CAN 收发队列、状态回复、FDCAN 提交和速率命令 |
| [firmware_flow.h](Core/Inc/firmware_flow.h) / [firmware_flow.c](Core/Src/firmware_flow.c) | AA59 会话、逻辑块队列、Classic CAN 分片、CAN FD DLC、累计 ACK |
| [system_heartbeat.h](Core/Inc/system_heartbeat.h) / [system_heartbeat.c](Core/Src/system_heartbeat.c) | AA58 PING 和队列占用状态 |
| [usb_can_gateway.h](Core/Inc/usb_can_gateway.h) / [usb_can_gateway.c](Core/Src/usb_can_gateway.c) | USB CDC 传输抽象、RX 环形缓存、TX 队列和回调衔接 |

### 9.2 Core 头文件与源文件

头文件：[main.h](Core/Inc/main.h)、[fdcan.h](Core/Inc/fdcan.h)、[dma.h](Core/Inc/dma.h)、[gpio.h](Core/Inc/gpio.h)、[usart.h](Core/Inc/usart.h)、[stm32h7xx_it.h](Core/Inc/stm32h7xx_it.h)、[stm32h7xx_hal_conf.h](Core/Inc/stm32h7xx_hal_conf.h)，以及上表列出的手写头文件。

源文件：[main.c](Core/Src/main.c)、[fdcan.c](Core/Src/fdcan.c)、[dma.c](Core/Src/dma.c)、[gpio.c](Core/Src/gpio.c)、[usart.c](Core/Src/usart.c)、[stm32h7xx_it.c](Core/Src/stm32h7xx_it.c)、[stm32h7xx_hal_msp.c](Core/Src/stm32h7xx_hal_msp.c)、[system_stm32h7xx.c](Core/Src/system_stm32h7xx.c)、[syscalls.c](Core/Src/syscalls.c)、[sysmem.c](Core/Src/sysmem.c)，以及上表列出的手写源文件。

### 9.3 构建和 CubeMX 配置

- [CMakeLists.txt](CMakeLists.txt)：顶层目标、应用层源文件和 `stm32cubemx` 目标的装配。
- [CMakePresets.json](CMakePresets.json)：`Debug`/`Release` 配置和 Ninja 构建入口。
- [cmake/gcc-arm-none-eabi.cmake](cmake/gcc-arm-none-eabi.cmake)、[cmake/starm-clang.cmake](cmake/starm-clang.cmake)：工具链文件。
- [cmake/stm32cubemx/CMakeLists.txt](cmake/stm32cubemx/CMakeLists.txt)：CubeMX 生成源文件、HAL/USB/CMSIS 包含目录和目标库。
- [CAN_To_Uart.ioc](CAN_To_Uart.ioc)：当前 CubeMX 外设、引脚、时钟、USB CDC、USART1 和 FDCAN1 配置。

## 10. 测试、验证入口与审查清单

[tests/firmware_flow_host_test.c](tests/firmware_flow_host_test.c) 是主机侧 C 单元测试。它通过最小 HAL/CAN/传输桩直接包含当前的 [Core/Src/firmware_flow.c](Core/Src/firmware_flow.c)，覆盖 Classic CAN 分片、FD DLC、不同尾块长度、多块 credit/ACK、队列满、CAN BUSY/失败、ACK 发送忙和 CRC 错误。它不是完整的 MCU 集成测试，也不会验证 USB 电气层、真实 FDCAN 中断、收发器或总线 ACK。

在工程根目录执行以下验证入口：

```powershell
cmake --preset Debug
cmake --build --preset Debug --parallel
cmake --preset Release
cmake --build --preset Release --parallel
gcc -std=c11 -Wall -Wextra -Werror -O0 -ICore/Inc tests/firmware_flow_host_test.c -o build/firmware_flow_host_test.exe
.\build\firmware_flow_host_test.exe
```

审查时至少确认：

1. `CMakeLists.txt` 仍包含 `can_gateway_core.c`、`firmware_flow.c`、`system_heartbeat.c`、`usb_can_gateway.c`；
2. `main.c` 初始化顺序仍为 USB 传输注册、AA55 核心、AA59 通道、AA58 心跳，并持续轮询四个模块；
3. [STM32H750XX_FLASH.ld](STM32H750XX_FLASH.ld) 的 `.dma_buffer` 与 DMA 可访问 RAM 保持一致，启用 D-Cache 时补做缓存一致性处理；
4. 主机测试通过且输出 `firmware_flow_host_test: PASS`；
5. 板上分别验证 USB 枚举、USB 半包/粘包、FDCAN 波形、节点 ACK、CAN 接收回传、FD+BRS、Bus-Off 恢复和长时间流量。

## 11. 已知限制、风险与文档疑点

- 当前测试只覆盖 `firmware_flow`，没有覆盖 AA55、AA58、USB CDC 回调及真实 FDCAN 外设。
- [Core/Src/usart.c](Core/Src/usart.c)、[Core/Src/dma.c](Core/Src/dma.c) 和 `.ioc` 仍配置 USART1 与 DMA；但 [Core/Src/main.c](Core/Src/main.c) 的网关运行时接口是 USB CDC。任何把 USART1 当作电脑运行时接口的操作说明都需要与当前实现重新核对。
- 速率切换在本文前文已标明“尚未完成板上验证”；应用失败时软件记录值与控制器真实状态可能不同。
- FDCAN 配置声明了 FD+BRS 和高速数据段，但收发器、布线、终端和对端能力未由仓库文件证明；不能仅凭 `.ioc` 推断物理层已验证。
- `Error_Handler()` 为关中断死循环；初始化失败时不会自动恢复，也没有文档化的错误回传通道。
- 构建目录、烧录工具和板卡实测日志不属于源码仓库的可复现输入；“已编译通过”和“已在板上验证”必须分开记录。
- 本次文档依据的是当前工作区实际文件；若后续重新运行 CubeMX、修改协议字段或改变 USB/USART 选路，必须同步复核本 README 的示例、文件边界和验证结论。
