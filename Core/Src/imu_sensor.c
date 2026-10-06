/**
 * @file imu_sensor.c
 * @brief 【可集成 IMU 后端】实现：7E23 解析、原生命令组包、快照与异步请求。
 *
 * 设计约束（对应统筹文档 SENSOR_COORDINATION_20261006.md）：
 *   - 纯 C、无 HAL 依赖，可在主机测试中直接编译；
 *   - 接收侧有界：1024 字节环形缓冲 + 16 个分块头，溢出整块丢弃并计数，
 *     解析全部发生在 ImuSensor_Process()（主循环），不在中断里解析；
 *   - 无序号的原生协议同一时刻只允许一个待确认请求（0x80/0x70/0x71/
 *     0x73）；0x60/0x61/0xA0 明确无回复，发出后立即回 UNCONFIRMED，不
 *     等 0x81；命令帧均按通信协议.xlsx 原文组包（0x60 频率为单字节参
 *     数、0x73 长度单元格 07 为笔误按下标行取 8 字节）；
 *   - 浮点访问显式按字节拼 LE 再 memcpy 到对齐临时变量，禁止从环形缓冲
 *     直接按指针取 32 位值；浮点帧全部做 isfinite 检查，非有限整帧拒收；
 *   - 引脚默认锁定（pins_blocked=1），模块自身永远不解锁、不自环、
 *     不自动校准、不发送任何自发帧。
 */
#include "imu_sensor.h"

#include <math.h>
#include <string.h>

/* ---- 常量 ---- */
#define IMU_SENSOR_RX_RING_SIZE 1024U /* 必须为 2 的幂。 */
#define IMU_SENSOR_RX_RING_MASK (IMU_SENSOR_RX_RING_SIZE - 1U)
#define IMU_SENSOR_CHUNK_MAX 16U     /* 未解析分块头队列深度。 */
#define IMU_SENSOR_RESULT_Q_SIZE 8U  /* 异步结果 FIFO 深度。 */
#define IMU_SENSOR_FRAME_MAX 64U     /* 原生帧总长上限。 */
#define IMU_SENSOR_FRAME_MIN 5U      /* 7E 23 LEN FUNC SUM 的理论下限。 */
/* request_timeout_us=0 时的默认待确认超时：版本查询 1s，0x70/0x71/0x73
 * 校准 30s（校准实机耗时远长于版本查询）。显式非 0 值对全部命令统一覆盖。 */
#define IMU_SENSOR_VERSION_DEFAULT_TIMEOUT_US 1000000ULL
#define IMU_SENSOR_CAL_DEFAULT_TIMEOUT_US 30000000ULL

/* 原生功能字。 */
#define IMU_NATIVE_FUNC_VERSION_RSP 0x01U /* 0x80 的回包：3 个版本字节。 */
#define IMU_NATIVE_FUNC_RAW 0x04U         /* 23 字节：9 个有符号 int16。 */
#define IMU_NATIVE_FUNC_QUAT 0x16U        /* 21 字节：float32 wxyz。 */
#define IMU_NATIVE_FUNC_EULER 0x26U       /* 17 字节：float32 rad。 */
#define IMU_NATIVE_FUNC_BARO 0x32U        /* 21 字节：float32 高度/温度/气压/参考压。 */
#define IMU_NATIVE_FUNC_VERSION_REQ 0x80U /* 请求 01 00 5F 前缀。 */
#define IMU_NATIVE_FUNC_CAL_AGM 0x70U     /* 校准陀螺仪+加速度计（原文已核实）。 */
#define IMU_NATIVE_FUNC_CAL_MAG 0x71U     /* 校准磁力计（原文已核实）。 */
#define IMU_NATIVE_FUNC_CAL_TEMP 0x73U    /* 温度校准（长度单元格 07 为笔误，按下标行取 8 字节）。 */
#define IMU_NATIVE_FUNC_RESET 0xA0U       /* 重置用户数据，字面量 7E 23 07 A0 01 5F A8。 */
#define IMU_NATIVE_FUNC_CAL_ACK 0x81U     /* 校准回包：[原命令, 状态0|1]。 */

/* 各已知功能字要求的帧总长（LEN 字段 = 整帧长度）。 */
#define IMU_NATIVE_LEN_VERSION_RSP 8U
#define IMU_NATIVE_LEN_RAW 23U
#define IMU_NATIVE_LEN_QUAT 21U
#define IMU_NATIVE_LEN_EULER 17U
#define IMU_NATIVE_LEN_BARO 21U
#define IMU_NATIVE_LEN_CAL_ACK 7U

/* 换算系数（统筹文档勘误：accel g / gyro rad/s / mag 单位未证实）。 */
#define IMU_SENSOR_ACCEL_SCALE (16.0f / 32767.0f)
#define IMU_SENSOR_GYRO_SCALE \
  ((2000.0f / 32767.0f) * (3.14159265358979323846f / 180.0f))
#define IMU_SENSOR_MAG_SCALE (800.0f / 32767.0f)

/* 帧解析状态机状态。 */
enum
{
  IMU_SENSOR_ST_HEAD1 = 0, /* 等待包头 0x7E。 */
  IMU_SENSOR_ST_HEAD2,     /* 已见 0x7E，等待 0x23。 */
  IMU_SENSOR_ST_LEN,       /* 等待总长字节。 */
  IMU_SENSOR_ST_BODY       /* 收集帧体直到 LEN 个字节。 */
};

/* 到达分块头：分块长度 + 该分块的到达时刻。 */
typedef struct
{
  uint16_t len;
  uint64_t t_us;
} ImuSensor_ChunkHdr;

/* 待确认请求（原生协议无序号，单一在飞）。 */
typedef struct
{
  uint8_t active;
  uint8_t op;        /* 发起时的请求操作码。 */
  uint8_t native;    /* 期望匹配的原生命令（0x80/0x70/0x71）。 */
  uint32_t host_seq;
  uint64_t deadline_us;
} ImuSensor_Pending;

/* 模块状态（单实例）。 */
static ImuSensor_Config imu_cfg;
static uint8_t imu_pins_blocked = 1U; /* 默认锁定，永不自行解锁。 */

/* RX 环形缓冲与分块队列。 */
static uint8_t imu_rx_ring[IMU_SENSOR_RX_RING_SIZE];
static uint16_t imu_rx_head; /* 自由递增写游标（按 &MASK 取索引）。 */
static uint16_t imu_rx_tail; /* 自由递增读游标。 */
static ImuSensor_ChunkHdr imu_chunk_q[IMU_SENSOR_CHUNK_MAX];
static uint8_t imu_chunk_head;
static uint8_t imu_chunk_tail;
static uint8_t imu_chunk_count;

/* 帧解析状态机。 */
static uint8_t imu_pstate;
static uint8_t imu_frame[IMU_SENSOR_FRAME_MAX];
static uint16_t imu_frame_pos;
static uint16_t imu_frame_len;

/* 数据组：独立序号/时间戳/数据。 */
static uint32_t imu_raw_seq;
static uint64_t imu_raw_time_us;
static int16_t imu_accel_raw[3];
static int16_t imu_gyro_raw[3];
static int16_t imu_mag_raw[3];
static float imu_accel_g[3];
static float imu_gyro_rad_s[3];
static float imu_mag_units[3];
static uint8_t imu_have_raw;

static uint32_t imu_quat_seq;
static uint64_t imu_quat_time_us;
static float imu_quat_wxyz[4];
static uint8_t imu_have_quat;

static uint32_t imu_euler_seq;
static uint64_t imu_euler_time_us;
static float imu_euler_rpy_rad[3];
static uint8_t imu_have_euler;

static uint32_t imu_baro_seq;
static uint64_t imu_baro_time_us;
static float imu_baro_height_m;
static float imu_baro_temp_c;
static float imu_baro_pressure_pa;
static float imu_baro_ref_pa;
static uint8_t imu_have_baro;

/* 版本与参数缓存（无原生读回，缓存仅为最后下发值的记录）。 */
static uint8_t imu_version[3];
static uint64_t imu_version_time_us;
static uint8_t imu_version_valid;
static uint16_t imu_rate_cache;
static uint8_t imu_rate_have;
static uint8_t imu_mode_cache;
static uint8_t imu_mode_have;

/* 请求与结果。 */
static ImuSensor_Pending imu_pending;
static ImuSensor_Result imu_result_q[IMU_SENSOR_RESULT_Q_SIZE];
static uint8_t imu_result_head;
static uint8_t imu_result_tail;
static uint8_t imu_result_count;
static uint32_t imu_host_seq_counter;

/* 统计。 */
static uint32_t imu_good_frames;
static uint32_t imu_bad_checksum_frames;
static uint32_t imu_bad_length_frames;
static uint32_t imu_nonfinite_frames;
static uint32_t imu_unknown_func_frames;
static uint32_t imu_unexpected_replies;
static uint32_t imu_rx_dropped_bytes;
static uint32_t imu_rx_overflow_chunks;
static uint32_t imu_results_dropped;
static uint32_t imu_tx_frames;
static uint32_t imu_requests_total;
static uint32_t imu_timeouts_total;

/* ---- 小工具 ---- */

/** 按字节拼 16 位 LE，避免从环形缓冲按指针取多字节值。 */
static uint16_t ImuSensor_LdU16Le(const uint8_t *p)
{
  return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

/** 按字节拼 32 位 LE 后 memcpy 到对齐 float，兼容任意主机对齐要求。 */
static float ImuSensor_LdF32Le(const uint8_t *p)
{
  uint32_t u = (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
               ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
  float f;

  memcpy(&f, &u, 4U);
  return f;
}

/** 压入一个异步结果；队列满时丢弃新结果并计数。 */
static void ImuSensor_PushResult(uint8_t op, uint32_t host_seq,
                                 uint8_t result, const uint8_t version[3])
{
  ImuSensor_Result *slot = &imu_result_q[imu_result_head];

  if (imu_result_count >= IMU_SENSOR_RESULT_Q_SIZE)
  {
    imu_results_dropped++;
    return;
  }
  slot->op = op;
  slot->host_seq = host_seq;
  slot->result = result;
  if (version != NULL)
  {
    slot->version[0] = version[0];
    slot->version[1] = version[1];
    slot->version[2] = version[2];
  }
  else
  {
    slot->version[0] = 0U;
    slot->version[1] = 0U;
    slot->version[2] = 0U;
  }
  imu_result_head = (uint8_t)((imu_result_head + 1U) % IMU_SENSOR_RESULT_Q_SIZE);
  imu_result_count++;
}

/** 组包：7E 23 LEN FUNC payload... SUM8，LEN 为整帧长度。 */
static uint16_t ImuSensor_BuildFrame(uint8_t func, const uint8_t *payload,
                                     uint16_t payload_len, uint8_t *out)
{
  uint16_t total = (uint16_t)(payload_len + 5U);
  uint16_t i;
  uint8_t sum = 0U;

  out[0] = 0x7EU;
  out[1] = 0x23U;
  out[2] = (uint8_t)total;
  out[3] = func;
  if ((payload != NULL) && (payload_len > 0U))
  {
    memcpy(&out[4], payload, payload_len);
  }
  for (i = 0U; i < (uint16_t)(total - 1U); i++)
  {
    sum = (uint8_t)(sum + out[i]);
  }
  out[total - 1U] = sum;
  return total;
}

/** 让传输层发送一帧。@return 0 = 已受理。 */
static int ImuSensor_Transmit(const uint8_t *frame, uint16_t len)
{
  if (imu_cfg.tx == NULL)
  {
    return -1;
  }
  if (imu_cfg.tx(imu_cfg.tx_user, frame, len) != 0)
  {
    return -1;
  }
  imu_tx_frames++;
  return 0;
}

/* ---- 解析与分发 ---- */

/** 发布 RAW 帧（0x04）：9 个 int16，缩放为 g / rad/s / 协议单位。 */
static void ImuSensor_PublishRaw(uint64_t stamp)
{
  uint16_t i;

  for (i = 0U; i < 3U; i++)
  {
    int16_t a = (int16_t)ImuSensor_LdU16Le(&imu_frame[4U + i * 2U]);
    int16_t g = (int16_t)ImuSensor_LdU16Le(&imu_frame[10U + i * 2U]);
    int16_t m = (int16_t)ImuSensor_LdU16Le(&imu_frame[16U + i * 2U]);

    imu_accel_raw[i] = a;
    imu_gyro_raw[i] = g;
    imu_mag_raw[i] = m;
    imu_accel_g[i] = (float)a * IMU_SENSOR_ACCEL_SCALE;
    imu_gyro_rad_s[i] = (float)g * IMU_SENSOR_GYRO_SCALE;
    imu_mag_units[i] = (float)m * IMU_SENSOR_MAG_SCALE;
  }
  imu_raw_seq++;
  imu_raw_time_us = stamp;
  imu_have_raw = 1U;
  imu_good_frames++;
}

/** 发布浮点帧的公共检查：任一值非有限则整帧拒收。 */
static int ImuSensor_FloatsFinite(const uint8_t *offsets, uint16_t count)
{
  uint16_t i;

  for (i = 0U; i < count; i++)
  {
    float f = ImuSensor_LdF32Le(&imu_frame[offsets[i]]);

    if (!isfinite(f))
    {
      return 0;
    }
  }
  return 1;
}

/** 发布四元数帧（0x16）。 */
static void ImuSensor_PublishQuat(uint64_t stamp)
{
  static const uint8_t offsets[4] = {4U, 8U, 12U, 16U};
  uint16_t i;

  if (!ImuSensor_FloatsFinite(offsets, 4U))
  {
    imu_nonfinite_frames++;
    return;
  }
  for (i = 0U; i < 4U; i++)
  {
    imu_quat_wxyz[i] = ImuSensor_LdF32Le(&imu_frame[offsets[i]]);
  }
  imu_quat_seq++;
  imu_quat_time_us = stamp;
  imu_have_quat = 1U;
  imu_good_frames++;
}

/** 发布欧拉角帧（0x26）。 */
static void ImuSensor_PublishEuler(uint64_t stamp)
{
  static const uint8_t offsets[3] = {4U, 8U, 12U};
  uint16_t i;

  if (!ImuSensor_FloatsFinite(offsets, 3U))
  {
    imu_nonfinite_frames++;
    return;
  }
  for (i = 0U; i < 3U; i++)
  {
    imu_euler_rpy_rad[i] = ImuSensor_LdF32Le(&imu_frame[offsets[i]]);
  }
  imu_euler_seq++;
  imu_euler_time_us = stamp;
  imu_have_euler = 1U;
  imu_good_frames++;
}

/** 发布气压计帧（0x32）。 */
static void ImuSensor_PublishBaro(uint64_t stamp)
{
  static const uint8_t offsets[4] = {4U, 8U, 12U, 16U};
  uint16_t i;
  float values[4];

  if (!ImuSensor_FloatsFinite(offsets, 4U))
  {
    imu_nonfinite_frames++;
    return;
  }
  for (i = 0U; i < 4U; i++)
  {
    values[i] = ImuSensor_LdF32Le(&imu_frame[offsets[i]]);
  }
  imu_baro_height_m = values[0];
  imu_baro_temp_c = values[1];
  imu_baro_pressure_pa = values[2];
  imu_baro_ref_pa = values[3];
  imu_baro_seq++;
  imu_baro_time_us = stamp;
  imu_have_baro = 1U;
  imu_good_frames++;
}

/** 处理版本回包（0x01）：更新版本信息并结束匹配的待确认请求。 */
static void ImuSensor_HandleVersionReply(uint64_t stamp)
{
  if (imu_pending.active && (imu_pending.native == IMU_NATIVE_FUNC_VERSION_REQ))
  {
    uint8_t version[3];

    version[0] = imu_frame[4];
    version[1] = imu_frame[5];
    version[2] = imu_frame[6];
    ImuSensor_PushResult(imu_pending.op, imu_pending.host_seq,
                         IMU_SENSOR_RES_OK, version);
    imu_pending.active = 0U;
  }
  else
  {
    imu_unexpected_replies++;
  }
  imu_version[0] = imu_frame[4];
  imu_version[1] = imu_frame[5];
  imu_version[2] = imu_frame[6];
  imu_version_time_us = stamp;
  imu_version_valid = 1U;
  imu_good_frames++;
}

/** 处理校准回包（0x81）：状态 1=成功，0=设备报告失败。 */
static void ImuSensor_HandleCalAck(void)
{
  uint8_t orig_cmd = imu_frame[4];
  uint8_t status = imu_frame[5];

  if (imu_pending.active && (imu_pending.native == orig_cmd))
  {
    ImuSensor_PushResult(imu_pending.op, imu_pending.host_seq,
                         (status != 0U) ? IMU_SENSOR_RES_OK
                                        : IMU_SENSOR_RES_IO_ERROR,
                         NULL);
    imu_pending.active = 0U;
  }
  else
  {
    imu_unexpected_replies++;
  }
  imu_good_frames++;
}

/** 按功能字分发一个校验和正确的完整帧。 */
static void ImuSensor_DispatchFrame(uint64_t stamp)
{
  uint8_t func = imu_frame[3];

  switch (func)
  {
  case IMU_NATIVE_FUNC_RAW:
    if (imu_frame_len != IMU_NATIVE_LEN_RAW)
    {
      imu_bad_length_frames++;
    }
    else
    {
      ImuSensor_PublishRaw(stamp);
    }
    break;

  case IMU_NATIVE_FUNC_QUAT:
    if (imu_frame_len != IMU_NATIVE_LEN_QUAT)
    {
      imu_bad_length_frames++;
    }
    else
    {
      ImuSensor_PublishQuat(stamp);
    }
    break;

  case IMU_NATIVE_FUNC_EULER:
    if (imu_frame_len != IMU_NATIVE_LEN_EULER)
    {
      imu_bad_length_frames++;
    }
    else
    {
      ImuSensor_PublishEuler(stamp);
    }
    break;

  case IMU_NATIVE_FUNC_BARO:
    if (imu_frame_len != IMU_NATIVE_LEN_BARO)
    {
      imu_bad_length_frames++;
    }
    else
    {
      ImuSensor_PublishBaro(stamp);
    }
    break;

  case IMU_NATIVE_FUNC_VERSION_RSP:
    if (imu_frame_len != IMU_NATIVE_LEN_VERSION_RSP)
    {
      imu_bad_length_frames++;
    }
    else
    {
      ImuSensor_HandleVersionReply(stamp);
    }
    break;

  case IMU_NATIVE_FUNC_CAL_ACK:
    if (imu_frame_len != IMU_NATIVE_LEN_CAL_ACK)
    {
      imu_bad_length_frames++;
    }
    else
    {
      ImuSensor_HandleCalAck();
    }
    break;

  default:
    imu_unknown_func_frames++;
    break;
  }
}

/** 喂入一个字节给帧解析状态机（时间戳取所属分块的到达时刻）。 */
static void ImuSensor_ParseByte(uint8_t byte, uint64_t stamp)
{
  switch (imu_pstate)
  {
  case IMU_SENSOR_ST_HEAD1:
    if (byte == 0x7EU)
    {
      imu_frame[0] = byte;
      imu_pstate = IMU_SENSOR_ST_HEAD2;
    }
    break;

  case IMU_SENSOR_ST_HEAD2:
    if (byte == 0x23U)
    {
      imu_frame[1] = byte;
      imu_pstate = IMU_SENSOR_ST_LEN;
    }
    else if (byte != 0x7EU)
    {
      imu_pstate = IMU_SENSOR_ST_HEAD1;
    }
    break;

  case IMU_SENSOR_ST_LEN:
    /* 长度越界的字节按噪声处理静默重同步，不计入错误统计。 */
    if ((byte >= IMU_SENSOR_FRAME_MIN) && (byte <= IMU_SENSOR_FRAME_MAX))
    {
      imu_frame[2] = byte;
      imu_frame_len = byte;
      imu_frame_pos = 3U;
      imu_pstate = IMU_SENSOR_ST_BODY;
    }
    else
    {
      imu_pstate = IMU_SENSOR_ST_HEAD1;
    }
    break;

  case IMU_SENSOR_ST_BODY:
  default:
    imu_frame[imu_frame_pos++] = byte;
    if (imu_frame_pos >= imu_frame_len)
    {
      uint8_t checksum = 0U;
      uint16_t i;

      for (i = 0U; (uint16_t)(i + 1U) < imu_frame_len; i++)
      {
        checksum = (uint8_t)(checksum + imu_frame[i]);
      }
      if (checksum == imu_frame[imu_frame_len - 1U])
      {
        ImuSensor_DispatchFrame(stamp);
      }
      else
      {
        imu_bad_checksum_frames++;
      }
      imu_pstate = IMU_SENSOR_ST_HEAD1;
    }
    break;
  }
}

/* ---- 公开接口 ---- */

void ImuSensor_Init(const ImuSensor_Config *config)
{
  if (config != NULL)
  {
    imu_cfg = *config;
  }
  else
  {
    imu_cfg.tx = NULL;
    imu_cfg.tx_user = NULL;
    imu_cfg.request_timeout_us = 0U;
  }

  imu_pins_blocked = 1U; /* 默认锁定，初始化永不解锁。 */
  imu_rx_head = 0U;
  imu_rx_tail = 0U;
  imu_chunk_head = 0U;
  imu_chunk_tail = 0U;
  imu_chunk_count = 0U;
  imu_pstate = IMU_SENSOR_ST_HEAD1;
  imu_frame_pos = 0U;
  imu_frame_len = 0U;

  imu_raw_seq = 0U;
  imu_raw_time_us = 0ULL;
  imu_have_raw = 0U;
  imu_quat_seq = 0U;
  imu_quat_time_us = 0ULL;
  imu_have_quat = 0U;
  imu_euler_seq = 0U;
  imu_euler_time_us = 0ULL;
  imu_have_euler = 0U;
  imu_baro_seq = 0U;
  imu_baro_time_us = 0ULL;
  imu_have_baro = 0U;
  imu_version_valid = 0U;
  imu_version_time_us = 0ULL;
  imu_rate_have = 0U;
  imu_mode_have = 0U;

  imu_pending.active = 0U;
  imu_result_head = 0U;
  imu_result_tail = 0U;
  imu_result_count = 0U;
  imu_host_seq_counter = 0U;

  imu_good_frames = 0U;
  imu_bad_checksum_frames = 0U;
  imu_bad_length_frames = 0U;
  imu_nonfinite_frames = 0U;
  imu_unknown_func_frames = 0U;
  imu_unexpected_replies = 0U;
  imu_rx_dropped_bytes = 0U;
  imu_rx_overflow_chunks = 0U;
  imu_results_dropped = 0U;
  imu_tx_frames = 0U;
  imu_requests_total = 0U;
  imu_timeouts_total = 0U;
}

uint8_t ImuSensor_IsPinsBlocked(void)
{
  return imu_pins_blocked;
}

void ImuSensor_SetPinsBlocked(uint8_t blocked)
{
  imu_pins_blocked = (blocked != 0U) ? 1U : 0U;
}

int ImuSensor_Feed(const uint8_t *data, uint16_t len, uint64_t now_us)
{
  uint16_t buffered;
  uint16_t free_bytes;
  uint16_t first;
  ImuSensor_ChunkHdr *chunk;

  if ((data == NULL) || (len == 0U))
  {
    return 0;
  }

  buffered = (uint16_t)(imu_rx_head - imu_rx_tail);
  free_bytes = (uint16_t)(IMU_SENSOR_RX_RING_SIZE - buffered);
  /* 整块原子接收：块内字节必须共享同一到达时间戳。 */
  if ((len > free_bytes) || (imu_chunk_count >= IMU_SENSOR_CHUNK_MAX))
  {
    imu_rx_overflow_chunks++;
    imu_rx_dropped_bytes = (uint32_t)(imu_rx_dropped_bytes + len);
    return 0;
  }

  chunk = &imu_chunk_q[imu_chunk_head];
  chunk->len = len;
  chunk->t_us = now_us;
  imu_chunk_head = (uint8_t)((imu_chunk_head + 1U) % IMU_SENSOR_CHUNK_MAX);
  imu_chunk_count++;

  first = (uint16_t)(IMU_SENSOR_RX_RING_SIZE - (imu_rx_head & IMU_SENSOR_RX_RING_MASK));
  if (first > len)
  {
    first = len;
  }
  memcpy(&imu_rx_ring[imu_rx_head & IMU_SENSOR_RX_RING_MASK], data, first);
  if (len > first)
  {
    memcpy(&imu_rx_ring[0], &data[first], (uint16_t)(len - first));
  }
  imu_rx_head = (uint16_t)(imu_rx_head + len);
  return (int)len;
}

void ImuSensor_Process(uint64_t now_us)
{
  while (imu_chunk_count > 0U)
  {
    const ImuSensor_ChunkHdr *chunk = &imu_chunk_q[imu_chunk_tail];
    uint16_t remaining = chunk->len;
    uint64_t stamp = chunk->t_us;

    while (remaining > 0U)
    {
      ImuSensor_ParseByte(imu_rx_ring[imu_rx_tail & IMU_SENSOR_RX_RING_MASK],
                          stamp);
      imu_rx_tail = (uint16_t)(imu_rx_tail + 1U);
      remaining--;
    }
    imu_chunk_tail = (uint8_t)((imu_chunk_tail + 1U) % IMU_SENSOR_CHUNK_MAX);
    imu_chunk_count--;
  }

  if (imu_pending.active && (now_us >= imu_pending.deadline_us))
  {
    ImuSensor_PushResult(imu_pending.op, imu_pending.host_seq,
                         IMU_SENSOR_RES_TIMEOUT, NULL);
    imu_timeouts_total++;
    imu_pending.active = 0U;
  }
}

int ImuSensor_GetSample(ImuSensor_Sample *out, uint64_t now_us)
{
  uint32_t status = 0U;
  uint8_t raw_fresh;
  uint8_t quat_fresh;
  uint8_t euler_fresh;
  uint8_t baro_fresh;

  if (out == NULL)
  {
    return 0;
  }

  raw_fresh = (uint8_t)(imu_have_raw &&
                        ((now_us - imu_raw_time_us) <= IMU_SENSOR_FRESH_US));
  quat_fresh = (uint8_t)(imu_have_quat &&
                         ((now_us - imu_quat_time_us) <= IMU_SENSOR_FRESH_US));
  euler_fresh = (uint8_t)(imu_have_euler &&
                          ((now_us - imu_euler_time_us) <= IMU_SENSOR_FRESH_US));
  baro_fresh = (uint8_t)(imu_have_baro &&
                         ((now_us - imu_baro_time_us) <= IMU_SENSOR_FRESH_US));

  if (raw_fresh)
  {
    status |= IMU_SENSOR_STATUS_ONLINE | IMU_SENSOR_STATUS_RAW_VALID;
  }
  if (quat_fresh)
  {
    status |= IMU_SENSOR_STATUS_QUAT_VALID;
  }
  if (euler_fresh)
  {
    status |= IMU_SENSOR_STATUS_EULER_VALID;
  }
  if (baro_fresh)
  {
    status |= IMU_SENSOR_STATUS_PRESSURE_VALID |
              IMU_SENSOR_STATUS_TEMPERATURE_VALID;
  }
  if (imu_pins_blocked != 0U)
  {
    status |= IMU_SENSOR_STATUS_PIN_BLOCKED;
  }
  /* 无原生读回：配置永远无法确认，CONFIG_UNKNOWN 恒置位；
   * UNCONFIRMED 的 rate/mode 下发（缓存更新）不改变该位。 */
  status |= IMU_SENSOR_STATUS_CONFIG_UNKNOWN;

  memset(out, 0, sizeof(*out));
  out->status = status;

  out->raw_seq = imu_raw_seq;
  out->raw_time_us = imu_raw_time_us;
  if (imu_have_raw != 0U)
  {
    memcpy(out->accel_raw, imu_accel_raw, sizeof(out->accel_raw));
    memcpy(out->gyro_raw, imu_gyro_raw, sizeof(out->gyro_raw));
    memcpy(out->mag_raw, imu_mag_raw, sizeof(out->mag_raw));
    memcpy(out->accel_g, imu_accel_g, sizeof(out->accel_g));
    memcpy(out->gyro_rad_s, imu_gyro_rad_s, sizeof(out->gyro_rad_s));
    memcpy(out->mag_units, imu_mag_units, sizeof(out->mag_units));
  }

  out->quat_seq = imu_quat_seq;
  out->quat_time_us = imu_quat_time_us;
  if (imu_have_quat != 0U)
  {
    memcpy(out->quat_wxyz, imu_quat_wxyz, sizeof(out->quat_wxyz));
  }

  out->euler_seq = imu_euler_seq;
  out->euler_time_us = imu_euler_time_us;
  if (imu_have_euler != 0U)
  {
    memcpy(out->euler_rpy_rad, imu_euler_rpy_rad, sizeof(out->euler_rpy_rad));
  }

  out->baro_seq = imu_baro_seq;
  out->baro_time_us = imu_baro_time_us;
  if (imu_have_baro != 0U)
  {
    out->baro_height_m = imu_baro_height_m;
    out->baro_temp_c = imu_baro_temp_c;
    out->baro_pressure_pa = imu_baro_pressure_pa;
    out->baro_ref_pa = imu_baro_ref_pa;
  }

  out->version[0] = imu_version[0];
  out->version[1] = imu_version[1];
  out->version[2] = imu_version[2];
  out->version_time_us = imu_version_time_us;
  out->version_valid = imu_version_valid;

  out->good_frames = imu_good_frames;
  out->bad_checksum_frames = imu_bad_checksum_frames;
  out->bad_length_frames = imu_bad_length_frames;
  out->nonfinite_frames = imu_nonfinite_frames;
  out->unknown_func_frames = imu_unknown_func_frames;
  out->unexpected_replies = imu_unexpected_replies;
  out->rx_dropped_bytes = imu_rx_dropped_bytes;
  out->rx_overflow_chunks = imu_rx_overflow_chunks;
  out->results_dropped = imu_results_dropped;
  out->tx_frames = imu_tx_frames;
  out->requests_total = imu_requests_total;
  out->timeouts_total = imu_timeouts_total;

  return (int)((imu_have_raw != 0U) || (imu_have_quat != 0U) ||
               (imu_have_euler != 0U) || (imu_have_baro != 0U) ||
               (imu_version_valid != 0U));
}

int ImuSensor_GetParameter(uint16_t id, uint32_t *value_out)
{
  switch (id)
  {
  case IMU_SENSOR_PARAM_OUTPUT_RATE_HZ:
    if (imu_rate_have == 0U)
    {
      return 0;
    }
    if (value_out != NULL)
    {
      *value_out = (uint32_t)imu_rate_cache;
    }
    return 1;

  case IMU_SENSOR_PARAM_ALGORITHM_MODE:
    if (imu_mode_have == 0U)
    {
      return 0;
    }
    if (value_out != NULL)
    {
      *value_out = (uint32_t)imu_mode_cache;
    }
    return 1;

  default:
    return -1;
  }
}

int ImuSensor_Request(uint8_t op, uint32_t arg, uint64_t now_us,
                      uint32_t *host_seq_out)
{
  uint8_t payload[3];
  uint16_t payload_len = 0U;
  uint8_t native = 0U;
  uint8_t expect_reply = 0U;
  uint8_t frame[16];
  uint16_t frame_len;
  uint32_t seq;

  if (host_seq_out != NULL)
  {
    *host_seq_out = 0U;
  }
  if (imu_pins_blocked != 0U)
  {
    return IMU_SENSOR_RES_PIN_BLOCKED;
  }

  switch (op)
  {
  case IMU_SENSOR_REQ_SET_RATE:
    /* 统筹勘误：0x60 明确无回复，发出后立即 UNCONFIRMED。
     * 协议表原文：长度 07，频率为单字节参数1，参数2 固定 0x5F。 */
    if ((arg < 10U) || (arg > 100U))
    {
      return IMU_SENSOR_RES_BAD_VALUE;
    }
    native = 0x60U;
    payload[0] = (uint8_t)arg;
    payload[1] = 0x5FU;
    payload_len = 2U;
    break;

  case IMU_SENSOR_REQ_SET_MODE:
    if ((arg != 6U) && (arg != 9U))
    {
      return IMU_SENSOR_RES_BAD_VALUE;
    }
    native = 0x61U;
    payload[0] = (uint8_t)arg;
    payload[1] = 0x5FU;
    payload_len = 2U;
    break;

  case IMU_SENSOR_REQ_GET_VERSION:
    /* 0x80 有 0x01 回包，按待确认请求处理。 */
    native = IMU_NATIVE_FUNC_VERSION_REQ;
    payload[0] = 0x01U;
    payload[1] = 0x00U;
    payload_len = 2U;
    expect_reply = 1U;
    break;

  case IMU_SENSOR_REQ_CAL_ACCEL_GYRO_START:
    native = IMU_NATIVE_FUNC_CAL_AGM;
    payload[0] = 1U;
    payload[1] = 0x5FU;
    payload_len = 2U;
    expect_reply = 1U;
    break;

  case IMU_SENSOR_REQ_CAL_ACCEL_GYRO_CLEAR:
    native = IMU_NATIVE_FUNC_CAL_AGM;
    payload[0] = 0U;
    payload[1] = 0x5FU;
    payload_len = 2U;
    expect_reply = 1U;
    break;

  case IMU_SENSOR_REQ_CAL_MAG_START:
    native = IMU_NATIVE_FUNC_CAL_MAG;
    payload[0] = 1U;
    payload[1] = 0x5FU;
    payload_len = 2U;
    expect_reply = 1U;
    break;

  case IMU_SENSOR_REQ_CAL_MAG_CLEAR:
    native = IMU_NATIVE_FUNC_CAL_MAG;
    payload[0] = 0U;
    payload[1] = 0x5FU;
    payload_len = 2U;
    expect_reply = 1U;
    break;

  case IMU_SENSOR_REQ_CAL_TEMP:
    /* 0x73：温度×100 按两字节小端发送（参数1=低、参数2=高），参数3 固定
     * 0x5F，应答 0x81 [0x73, 状态]。表格长度单元格写 07 但下标行到 7，
     * 矛盾按下标行取总长 8（与 7 字节的 0x70/0x71 帧同构多一个数据字节）。 */
    native = IMU_NATIVE_FUNC_CAL_TEMP;
    payload[0] = (uint8_t)(arg & 0xFFU);
    payload[1] = (uint8_t)((arg >> 8) & 0xFFU);
    payload[2] = 0x5FU;
    payload_len = 3U;
    expect_reply = 1U;
    break;

  case IMU_SENSOR_REQ_RESET:
    /* 0xA0 重置用户数据：字面量 7E 23 07 A0 01 5F A8，明确无回复。 */
    native = IMU_NATIVE_FUNC_RESET;
    payload[0] = 0x01U;
    payload[1] = 0x5FU;
    payload_len = 2U;
    break;

  case IMU_SENSOR_REQ_SAVE_CONFIG:
  case IMU_SENSOR_REQ_SELF_TEST:
  case IMU_SENSOR_REQ_REBOOT:
    /* 原生协议无对应命令（0xA0 是重置用户数据，不是复位/保存/自检）。 */
    return IMU_SENSOR_RES_UNSUPPORTED;

  default:
    return IMU_SENSOR_RES_UNSUPPORTED;
  }

  /* 单一在飞：待确认请求未完成时保守拒绝一切新请求，避免在设备
   * 校准/应答期间混入 0x60/0x61 等未定义时序的原生命令。 */
  if (imu_pending.active)
  {
    return IMU_SENSOR_RES_BUSY;
  }

  seq = ++imu_host_seq_counter;
  if (host_seq_out != NULL)
  {
    *host_seq_out = seq;
  }
  imu_requests_total++;

  frame_len = ImuSensor_BuildFrame(native, payload, payload_len, frame);
  if (ImuSensor_Transmit(frame, frame_len) == 0)
  {
    if (expect_reply != 0U)
    {
      uint64_t timeout;

      if (imu_cfg.request_timeout_us != 0ULL)
      {
        timeout = imu_cfg.request_timeout_us;
      }
      else if ((native == IMU_NATIVE_FUNC_CAL_AGM) ||
               (native == IMU_NATIVE_FUNC_CAL_MAG) ||
               (native == IMU_NATIVE_FUNC_CAL_TEMP))
      {
        timeout = IMU_SENSOR_CAL_DEFAULT_TIMEOUT_US;
      }
      else
      {
        timeout = IMU_SENSOR_VERSION_DEFAULT_TIMEOUT_US;
      }

      imu_pending.active = 1U;
      imu_pending.op = op;
      imu_pending.native = native;
      imu_pending.host_seq = seq;
      imu_pending.deadline_us = now_us + timeout;
    }
    else
    {
      /* 无回复命令：只更新缓存并立刻回 UNCONFIRMED，绝不假装成功。 */
      if (op == IMU_SENSOR_REQ_SET_RATE)
      {
        imu_rate_cache = (uint16_t)arg;
        imu_rate_have = 1U;
      }
      else if (op == IMU_SENSOR_REQ_SET_MODE)
      {
        imu_mode_cache = (uint8_t)arg;
        imu_mode_have = 1U;
      }
      ImuSensor_PushResult(op, seq, IMU_SENSOR_RES_UNCONFIRMED, NULL);
    }
  }
  else
  {
    /* 发送失败：不进待确认、不更新缓存，直接回 IO_ERROR。 */
    ImuSensor_PushResult(op, seq, IMU_SENSOR_RES_IO_ERROR, NULL);
  }
  return 0;
}

int ImuSensor_PopResult(ImuSensor_Result *out)
{
  const ImuSensor_Result *slot;

  if ((out == NULL) || (imu_result_count == 0U))
  {
    return 0;
  }
  slot = &imu_result_q[imu_result_tail];
  *out = *slot;
  imu_result_tail = (uint8_t)((imu_result_tail + 1U) % IMU_SENSOR_RESULT_Q_SIZE);
  imu_result_count--;
  return 1;
}
