/**
 * @file imu_sensor.h
 * @brief 【可集成 IMU 后端】7E23 帧解码 + 原生命令构建 + 数据快照接口。
 *
 * 本模块是正式数据路径的 IMU 后端（区别于调试分支的 imu_uart_debug 文本
 * 直打链路）：只做纯解码、原生命令组包和结构化数据快照，不直接驱动
 * UART/DMA/Pin，也不产生调试文本。上位机 AA5B 层（由统筹方实现）通过
 * GetSample/GetParameter/Request/PopResult 消费本模块。
 *
 * 数据路径约定（集成方负责传输层，见 docs/imu-sensor-progress.md）：
 *   USART1 RX（DMA 循环 + IDLE）→ 集成层在主循环里按 NDTR 游标取有界
 *   分段（读前对 D-Cache 做 Invalidate，缓冲必须放在 DMA 可访问内存如
 *   .dma_buffer/RAM_D2，32 字节对齐）→ ImuSensor_Feed() 有界拷贝入本
 *   模块环形缓冲 → ImuSensor_Process() 在主循环完成帧解析（不在中断里
 *   解析）。Feed/Process/GetSample/Request 必须在同一线程上下文调用。
 *
 * 时间体系：模块自身无时钟，全部时间由调用方以 uint64_t 微秒显式传入
 * （Feed 的 now_us 为该分块的到达时刻，Process 的 now_us 驱动超时）。
 * 各数据组（raw/quat/euler/baro）各自持有独立的时间戳和递增序号，组间
 * 互不刷新新鲜度。
 *
 * 协议勘误（以 SENSOR_COORDINATION_20261006.md 为准）：
 *   - 原生 0x60（速率 10..100Hz）、0x61（模式 6|9）、A0 复位明确无回复，
 *     发送完成后只能给 UNCONFIRMED，绝不等待 0x81 假装成功；
 *   - 原生 0x70/0x71 校准命令才有 0x81 [原命令,状态] 回包，同一时刻只
 *     允许一个待确认请求（原生协议无序号字段），超时回复 TIMEOUT；
 *   - 原生 0x80 版本请求有 0x01 回包（总长 8，三个版本字节）；
 *   - 0x73 温度命令长度表自相矛盾，未核实，一律 UNSUPPORTED；
 *   - 参数无原生读回：GET 只能返回缓存值并标注 UNCONFIRMED。
 *   - 单位：accel 为 g（16/32767），gyro 为 rad/s（2000/32767*π/180），
 *     euler 为 rad，MAG 的 800/32767 物理单位未证实，命名保持中性。
 */
#ifndef __IMU_SENSOR_H__
#define __IMU_SENSOR_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdint.h>

/** 数据组新鲜度窗口（微秒）；集成方可在包含本头文件前覆盖。 */
#ifndef IMU_SENSOR_FRESH_US
#define IMU_SENSOR_FRESH_US 500000ULL
#endif

/** 请求操作码；与 AA5B CMD 的映射关系见 docs/imu-sensor-progress.md。 */
enum ImuSensorRequestOp
{
  IMU_SENSOR_REQ_SET_RATE = 1,            /* 原生 0x60，arg=Hz(10..100)，无回复。 */
  IMU_SENSOR_REQ_SET_MODE = 2,            /* 原生 0x61，arg=6|9，无回复。 */
  IMU_SENSOR_REQ_GET_VERSION = 3,         /* 原生 0x80，回复 0x01。 */
  IMU_SENSOR_REQ_CAL_ACCEL_GYRO_START = 4, /* 原生 0x70 action=1。 */
  IMU_SENSOR_REQ_CAL_ACCEL_GYRO_CLEAR = 5, /* 原生 0x70 action=0。 */
  IMU_SENSOR_REQ_CAL_MAG_START = 6,       /* 原生 0x71 action=1。 */
  IMU_SENSOR_REQ_CAL_MAG_CLEAR = 7,       /* 原生 0x71 action=0。 */
  IMU_SENSOR_REQ_CAL_TEMP = 8,            /* 0x73 未核实，恒 UNSUPPORTED。 */
  IMU_SENSOR_REQ_SAVE_CONFIG = 9,         /* 预留，恒 UNSUPPORTED。 */
  IMU_SENSOR_REQ_SELF_TEST = 10,          /* 预留，恒 UNSUPPORTED。 */
  IMU_SENSOR_REQ_REBOOT = 11              /* 预留，恒 UNSUPPORTED。 */
};

/**
 * 结果码。编号与 AA5B v1 的 RESULT 字段对齐，AA5B 层可直抄。
 * UNCONFIRMED = UART 已发出但设备无 ACK/读回，不是成功。
 */
enum ImuSensorResultCode
{
  IMU_SENSOR_RES_OK = 0,
  IMU_SENSOR_RES_UNSUPPORTED = 1,
  IMU_SENSOR_RES_BAD_VALUE = 2,
  IMU_SENSOR_RES_BUSY = 3,
  IMU_SENSOR_RES_TIMEOUT = 4,
  IMU_SENSOR_RES_OFFLINE = 5,
  IMU_SENSOR_RES_IO_ERROR = 6,
  IMU_SENSOR_RES_UNCONFIRMED = 7,
  IMU_SENSOR_RES_NOT_READY = 8,
  IMU_SENSOR_RES_PIN_BLOCKED = 9
};

/** 状态位（GetSample::status），编号与 AA5B GET_STATUS status 对齐。 */
#define IMU_SENSOR_STATUS_ONLINE           (1UL << 0)
#define IMU_SENSOR_STATUS_RAW_VALID        (1UL << 1)
#define IMU_SENSOR_STATUS_QUAT_VALID       (1UL << 2)
#define IMU_SENSOR_STATUS_EULER_VALID      (1UL << 3)
#define IMU_SENSOR_STATUS_PRESSURE_VALID   (1UL << 4)
#define IMU_SENSOR_STATUS_TEMPERATURE_VALID (1UL << 5)
#define IMU_SENSOR_STATUS_PIN_BLOCKED      (1UL << 9)
/* bit10 CONFIG_UNKNOWN 恒置位：原生协议无读回，配置永远无法确认，
 * UNCONFIRMED 的 rate/mode 下发不改变该位。 */
#define IMU_SENSOR_STATUS_CONFIG_UNKNOWN   (1UL << 10)
/* bit11 MODEL_CONFIRMED 恒为 0：原生协议无型号读回，禁止凭空确认。 */

/** 参数 ID（AA5B GET/SET_PARAMETER 的 PARAM_ID）。 */
#define IMU_SENSOR_PARAM_OUTPUT_RATE_HZ 0x0001U /* u16，10..100。 */
#define IMU_SENSOR_PARAM_ALGORITHM_MODE 0x0003U /* u8，6|9。 */

/** UART 发送回调：把 len 字节交给传输层，返回 0 表示已受理/发出。 */
typedef int (*ImuSensor_TxFn)(void *user, const uint8_t *data, uint16_t len);

/** 初始化配置；request_timeout_us 传 0 取默认 1 秒。 */
typedef struct
{
  ImuSensor_TxFn tx;            /* 可为 NULL：所有请求异步回 IO_ERROR。 */
  void *tx_user;                /* 透传给 tx 的上下文。 */
  uint64_t request_timeout_us;  /* 0 = 默认 1000000us。 */
} ImuSensor_Config;

/** 单个异步请求的完成结果（PopResult 弹出）。 */
typedef struct
{
  uint8_t op;             /* 发起时的 ImuSensorRequestOp。 */
  uint32_t host_seq;      /* Request 分配的主机侧序号。 */
  uint8_t result;         /* ImuSensorResultCode。 */
  uint8_t version[3];     /* 仅 GET_VERSION 且 OK 时有效。 */
} ImuSensor_Result;

/** 数据快照：各组独立时间戳/序号 + 原始值 + 物理值 + 统计。 */
typedef struct
{
  uint32_t status; /* IMU_SENSOR_STATUS_* 组合，按 now_us 实时评估。 */

  /* RAW 组：原生 0x04，23 字节，9 个有符号 int16。 */
  uint32_t raw_seq;
  uint64_t raw_time_us;
  int16_t accel_raw[3];  /* 原始计数。 */
  int16_t gyro_raw[3];
  int16_t mag_raw[3];
  float accel_g[3];      /* raw * 16/32767，单位 g。 */
  float gyro_rad_s[3];   /* raw * 2000/32767 * pi/180，单位 rad/s。 */
  float mag_units[3];    /* raw * 800/32767，物理单位未证实，勿标 uT。 */

  /* 四元数组：原生 0x16，21 字节，float32 wxyz，已做 isfinite 检查。 */
  uint32_t quat_seq;
  uint64_t quat_time_us;
  float quat_wxyz[4];

  /* 欧拉数组：原生 0x26，17 字节，float32 roll/pitch/yaw，单位 rad。 */
  uint32_t euler_seq;
  uint64_t euler_time_us;
  float euler_rpy_rad[3];

  /* 气压计组：原生 0x32，21 字节，float32。 */
  uint32_t baro_seq;
  uint64_t baro_time_us;
  float baro_height_m;
  float baro_temp_c;
  float baro_pressure_pa;
  float baro_ref_pa;

  /* 版本：原生 0x80 请求 → 0x01 回包（三个字节）。 */
  uint8_t version[3];
  uint64_t version_time_us;
  uint8_t version_valid;

  /* 状态/错误统计（自开机累计）。 */
  uint32_t good_frames;         /* 校验和正确的已知类型帧。 */
  uint32_t bad_checksum_frames; /* 累加和错误。 */
  uint32_t bad_length_frames;   /* 校验和过但长度与功能字不符。 */
  uint32_t nonfinite_frames;    /* 浮点帧含 NaN/Inf，整帧拒收。 */
  uint32_t unknown_func_frames; /* 校验和过但功能字未定义。 */
  uint32_t unexpected_replies;  /* 无匹配待确认请求的 0x01/0x81。 */
  uint32_t rx_dropped_bytes;    /* 环形缓冲满被整块丢弃的字节。 */
  uint32_t rx_overflow_chunks;  /* 整块丢弃次数。 */
  uint32_t results_dropped;     /* 结果队列满被丢弃的结果数。 */
  uint32_t tx_frames;           /* 成功交给传输层的原生帧数。 */
  uint32_t requests_total;      /* 受理的请求数（含异步失败）。 */
  uint32_t timeouts_total;      /* 待确认请求超时次数。 */
} ImuSensor_Sample;

/**
 * @brief 初始化模块并锁定引脚（pins_blocked 恒为 1）。
 * @param config 可为 NULL：等价于 tx=NULL、默认超时。
 *
 * 不触碰任何硬件引脚；即使 tx 已接线，引脚解除占用也必须由集成层在
 * 用户明确确认后调用 ImuSensor_SetPinsBlocked(0)。
 */
void ImuSensor_Init(const ImuSensor_Config *config);

/** @return 当前引脚占用状态：1=被占用（默认），0=已由集成层解锁。 */
uint8_t ImuSensor_IsPinsBlocked(void);

/**
 * @brief 解锁/占用引脚。仅供集成层在用户明确确认后调用；
 *        本模块自身永远不会调用它。
 */
void ImuSensor_SetPinsBlocked(uint8_t blocked);

/**
 * @brief 喂入一个到达分块（非 ISR 安全，须与 Process 同上下文）。
 * @param data   分块字节（来自 DMA 缓冲的已 Cache-Invalidate 拷贝）。
 * @param len    字节数。
 * @param now_us 本分块最后字节的到达时刻（微秒，集成方时钟）。
 * @return 接受的字节数；0 表示整块因缓冲不足被丢弃（整块原子，
 *         保证块时间戳一致），丢弃计入 rx_dropped_bytes。
 *
 * 有界：内部环形缓冲固定 1024 字节 + 16 个分块头，永不增长、永不阻塞。
 */
int ImuSensor_Feed(const uint8_t *data, uint16_t len, uint64_t now_us);

/**
 * @brief 主循环轮询：解析环形缓冲内全部分块并处理请求超时。
 * @param now_us 当前时刻（微秒，与 Feed/Request 同一时钟）。
 */
void ImuSensor_Process(uint64_t now_us);

/**
 * @brief 取数据快照。
 * @param out     输出快照（可为 NULL，仅用于判空检查则不允许）。
 * @param now_us  当前时刻，用于评估 status 里的新鲜度位。
 * @return 1 = 开机后至少收到过一帧已知类型数据；0 = 尚无任何数据。
 */
int ImuSensor_GetSample(ImuSensor_Sample *out, uint64_t now_us);

/**
 * @brief 读参数缓存。
 * @param id        IMU_SENSOR_PARAM_*。
 * @param value_out 缓存值（u16 参数在低 16 位，u8 在低 8 位）。
 * @return  1 = 有缓存（仍属 UNCONFIRMED，无原生读回）；
 *          0 = 从未设置过，无缓存（AA5B 应回 NOT_READY）；
 *         -1 = 未知参数 ID（AA5B 应回 UNSUPPORTED）。
 */
int ImuSensor_GetParameter(uint16_t id, uint32_t *value_out);

/**
 * @brief 发起一次原生命令请求（异步，结果经 PopResult 弹出）。
 * @param op            ImuSensorRequestOp。
 * @param arg           SET_RATE=Hz(10..100)；SET_MODE=6|9；其余忽略。
 * @param now_us        当前时刻，用于待确认请求的超时判定。
 * @param host_seq_out  可为 NULL；受理时写回本次请求的主机侧序号。
 * @return 0 = 受理（异步结果稍后从 PopResult 出）；
 *         否则同步拒绝码（PIN_BLOCKED/BAD_VALUE/BUSY/UNSUPPORTED）。
 *
 * 受理即可能发送 UART 帧；0x60/0x61 发出后立即产生 UNCONFIRMED 结果，
 * 不等待任何回包。0x80/0x70/0x71 为待确认请求，同一时刻仅一个；待确认
 * 请求在飞期间所有新请求（含无回复命令）保守返回 BUSY，超时产生
 * TIMEOUT 结果。
 */
int ImuSensor_Request(uint8_t op, uint32_t arg, uint64_t now_us,
                      uint32_t *host_seq_out);

/** @brief 弹出一个异步结果。@return 1 = 已弹出，0 = 队列空。 */
int ImuSensor_PopResult(ImuSensor_Result *out);

#ifdef __cplusplus
}
#endif

#endif /* __IMU_SENSOR_H__ */
