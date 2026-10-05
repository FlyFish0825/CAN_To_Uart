/**
 * @file imu_sensor_host_test.c
 * @brief 不依赖 STM32 HAL 的 IMU 后端主机测试。
 *
 * 直接编译 Core/Src/imu_sensor.c（纯 C、无 HAL 依赖），覆盖统筹文档
 * SENSOR_COORDINATION_20261006.md 要求的测试项：碎包/粘包/噪声/校验和/
 * 错误长度/负数/float/参数边界/无ACK/有ACK/超时/引脚占用，另覆盖独立
 * 时间序号、非有限浮点、未对齐偏移、缓冲溢出、BUSY/UNSUPPORTED 与发送
 * 失败路径。它不代表已在真实 UART、DMA 或目标板上运行。
 *
 * 编译（见 docs/imu-sensor-progress.md）：
 *   gcc -std=c11 -Wall -Wextra -Werror -O0 -ICore/Inc \
 *       tests/imu_sensor_host_test.c -o build/imu_sensor_host_test.exe
 */
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "../Core/Inc/imu_sensor.h"

/*
 * 主机测试直接包含固件实现（imu_sensor.c 为纯 C、无 HAL 依赖，
 * 可被 Windows GCC 直接编译）。
 */
#include "../Core/Src/imu_sensor.c"

/* ---- 测试桩 ---- */

#define TX_LOG_MAX 16U /* 发送桩最多记录的帧数。 */

static uint8_t tx_log[TX_LOG_MAX][16];
static uint16_t tx_len_log[TX_LOG_MAX];
static uint16_t tx_count;
static int tx_fail; /* 1 = 发送桩返回失败。 */

static int Test_Tx(void *user, const uint8_t *data, uint16_t len)
{
  (void)user;
  if (tx_fail != 0)
  {
    return -1;
  }
  if ((tx_count < TX_LOG_MAX) && (len <= 16U))
  {
    memcpy(tx_log[tx_count], data, len);
    tx_len_log[tx_count] = len;
  }
  tx_count++;
  return 0;
}

/** 重建模块：tx_fail 选择发送桩行为，timeout_us 传 0 取默认。 */
static void TestReset(int fail_tx, uint64_t timeout_us)
{
  ImuSensor_Config cfg;

  cfg.tx = Test_Tx;
  cfg.tx_user = NULL;
  cfg.request_timeout_us = timeout_us;
  tx_fail = fail_tx;
  tx_count = 0U;
  ImuSensor_Init(&cfg);
}

/* ---- 帧构造工具（测试侧独立实现；算法正确性由统筹文档给定的
 * 版本请求字面量 7E 23 07 80 01 00 29 在 Test_ParamAndRequests 中
 * 锚定，避免与被测实现共享同一处 bug）。 ---- */

static void PutU16Le(uint8_t *p, uint16_t v)
{
  p[0] = (uint8_t)(v & 0xFFU);
  p[1] = (uint8_t)((v >> 8) & 0xFFU);
}

static void PutI16Le(uint8_t *p, int16_t v)
{
  PutU16Le(p, (uint16_t)v);
}

static void PutF32Le(uint8_t *p, float f)
{
  memcpy(p, &f, 4U);
}

/** 构造 7E 23 LEN FUNC payload... SUM8 帧，LEN 为整帧长度。 */
static uint16_t Build7e23(uint8_t func, const uint8_t *payload,
                          uint16_t payload_len, uint8_t *out)
{
  uint16_t total = (uint16_t)(payload_len + 5U);
  uint16_t i;
  uint8_t sum = 0U;

  out[0] = 0x7EU;
  out[1] = 0x23U;
  out[2] = (uint8_t)total;
  out[3] = func;
  for (i = 0U; i < payload_len; i++)
  {
    out[4U + i] = payload[i];
  }
  for (i = 0U; (uint16_t)(i + 1U) < total; i++)
  {
    sum = (uint8_t)(sum + out[i]);
  }
  out[total - 1U] = sum;
  return total;
}

/** 构造 RAW04 帧（9 个有符号 int16）。 */
static uint16_t BuildRawFrame(const int16_t v[9], uint8_t *out)
{
  uint8_t payload[18];
  uint16_t i;

  for (i = 0U; i < 9U; i++)
  {
    PutI16Le(&payload[i * 2U], v[i]);
  }
  return Build7e23(0x04U, payload, sizeof(payload), out);
}

/** 构造四元数帧（float32 wxyz）。 */
static uint16_t BuildQuatFrame(const float q[4], uint8_t *out)
{
  uint8_t payload[16];
  uint16_t i;

  for (i = 0U; i < 4U; i++)
  {
    PutF32Le(&payload[i * 4U], q[i]);
  }
  return Build7e23(0x16U, payload, sizeof(payload), out);
}

/** 构造欧拉角帧（float32 rad）。 */
static uint16_t BuildEulerFrame(const float e[3], uint8_t *out)
{
  uint8_t payload[12];
  uint16_t i;

  for (i = 0U; i < 3U; i++)
  {
    PutF32Le(&payload[i * 4U], e[i]);
  }
  return Build7e23(0x26U, payload, sizeof(payload), out);
}

/** 构造气压计帧（float32 高度/温度/气压/参考压）。 */
static uint16_t BuildBaroFrame(const float b[4], uint8_t *out)
{
  uint8_t payload[16];
  uint16_t i;

  for (i = 0U; i < 4U; i++)
  {
    PutF32Le(&payload[i * 4U], b[i]);
  }
  return Build7e23(0x32U, payload, sizeof(payload), out);
}

/** 构造校准回包 0x81 [原命令, 状态]。 */
static uint16_t BuildCalAck(uint8_t orig_cmd, uint8_t status, uint8_t *out)
{
  uint8_t payload[2];

  payload[0] = orig_cmd;
  payload[1] = status;
  return Build7e23(0x81U, payload, sizeof(payload), out);
}

static void AssertTxFrame(uint16_t idx, const uint8_t *expected,
                          uint16_t len)
{
  assert(idx < tx_count);
  assert(tx_len_log[idx] == len);
  assert(memcmp(tx_log[idx], expected, len) == 0);
}

static void AssertFloatNear(float a, float b)
{
  assert(fabsf(a - b) <= (fabsf(b) * 1.0e-6f) + 1.0e-9f);
}

/* ---- 用例 ---- */

/** RAW 帧缩放、负数、原始值与时间戳/序号。 */
static void Test_RawScalingAndNegatives(void)
{
  ImuSensor_Sample s;
  uint8_t frame[32];
  uint16_t len;
  int16_t v[9] = {0, 1, -32767, -16384, 16384, 0, -32767, 32767, -1};

  TestReset(0, 0);
  len = BuildRawFrame(v, frame);
  assert(len == 23U);
  assert(ImuSensor_Feed(frame, len, 1000ULL) == (int)len);
  ImuSensor_Process(1000ULL);

  assert(ImuSensor_GetSample(&s, 1000ULL) == 1);
  assert(s.raw_seq == 1U);
  assert(s.raw_time_us == 1000ULL);
  assert(s.accel_raw[0] == 0 && s.accel_raw[1] == 1 &&
         s.accel_raw[2] == -32767);
  assert(s.gyro_raw[0] == -16384 && s.gyro_raw[1] == 16384 &&
         s.gyro_raw[2] == 0);
  assert(s.mag_raw[0] == -32767 && s.mag_raw[1] == 32767 &&
         s.mag_raw[2] == -1);
  AssertFloatNear(s.accel_g[0], 0.0f);
  AssertFloatNear(s.accel_g[1], (float)1 * (16.0f / 32767.0f));
  AssertFloatNear(s.accel_g[2], (float)-32767 * (16.0f / 32767.0f));
  AssertFloatNear(s.gyro_rad_s[0],
                  (float)-16384 * ((2000.0f / 32767.0f) *
                                   (3.14159265358979323846f / 180.0f)));
  AssertFloatNear(s.gyro_rad_s[1],
                  (float)16384 * ((2000.0f / 32767.0f) *
                                  (3.14159265358979323846f / 180.0f)));
  AssertFloatNear(s.mag_units[0],
                  (float)-32767 * (800.0f / 32767.0f));
  AssertFloatNear(s.mag_units[2], (float)-1 * (800.0f / 32767.0f));
  assert(s.good_frames == 1U);
  assert(s.bad_checksum_frames == 0U);
}

/** 碎包：一帧按 3 字节一组分多次 Feed。 */
static void Test_FragmentedFeed(void)
{
  ImuSensor_Sample s;
  uint8_t frame[32];
  uint16_t len;
  uint16_t off;
  float e[3] = {0.1f, -0.2f, 3.1f};

  TestReset(0, 0);
  len = BuildEulerFrame(e, frame);
  for (off = 0U; off < len; off += 3U)
  {
    uint16_t remain = (uint16_t)(len - off);
    uint16_t n = (remain < 3U) ? remain : 3U;

    assert(ImuSensor_Feed(&frame[off], n, 2000ULL + off) == (int)n);
    ImuSensor_Process(2000ULL + off);
  }
  assert(ImuSensor_GetSample(&s, 3000ULL) == 1);
  assert(s.euler_seq == 1U);
  AssertFloatNear(s.euler_rpy_rad[0], 0.1f);
  AssertFloatNear(s.euler_rpy_rad[1], -0.2f);
  AssertFloatNear(s.euler_rpy_rad[2], 3.1f);
  assert(s.good_frames == 1U);
}

/** 粘包 + 噪声：多帧与噪声混在一个 Feed 块内，含伪包头重同步。 */
static void Test_CoalescedAndNoise(void)
{
  ImuSensor_Sample s;
  uint8_t buf[64];
  uint16_t len = 0U;
  uint16_t n;
  float q1[4] = {1.0f, 0.0f, 0.0f, 0.0f};
  float q2[4] = {0.5f, -0.5f, 0.5f, -0.5f};
  /* 伪包头与杂散字节：7E 7E 00 / 7E 23 FF 均应静默重同步。 */
  static const uint8_t noise[] = {0x00U, 0x55U, 0x7EU, 0x7EU, 0x00U,
                                  0x7EU, 0x23U, 0xFFU, 0x13U};

  TestReset(0, 0);
  memcpy(&buf[len], noise, sizeof(noise));
  len = (uint16_t)(len + sizeof(noise));
  n = BuildQuatFrame(q1, &buf[len]);
  len = (uint16_t)(len + n);
  n = BuildQuatFrame(q2, &buf[len]);
  len = (uint16_t)(len + n);

  assert(ImuSensor_Feed(buf, len, 4000ULL) == (int)len);
  ImuSensor_Process(4000ULL);

  assert(ImuSensor_GetSample(&s, 4000ULL) == 1);
  assert(s.quat_seq == 2U);
  AssertFloatNear(s.quat_wxyz[0], 0.5f);
  AssertFloatNear(s.quat_wxyz[1], -0.5f);
  assert(s.good_frames == 2U);
  assert(s.bad_checksum_frames == 0U);
  assert(s.bad_length_frames == 0U);
  assert(s.unknown_func_frames == 0U);
}

/** 校验和错误：坏帧被拒收且不破坏后续帧解析。 */
static void Test_BadChecksum(void)
{
  ImuSensor_Sample s;
  uint8_t frame[32];
  uint16_t len;
  float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
  float e[3] = {0.1f, 0.0f, 0.0f};

  TestReset(0, 0);
  len = BuildQuatFrame(q, frame);
  frame[6] = 0x5AU; /* 翻转帧体字节，累加和必然失配。 */
  assert(ImuSensor_Feed(frame, len, 5000ULL) == (int)len);
  len = BuildEulerFrame(e, frame);
  assert(ImuSensor_Feed(frame, len, 5100ULL) == (int)len);
  ImuSensor_Process(5100ULL);

  assert(ImuSensor_GetSample(&s, 5100ULL) == 1);
  assert(s.quat_seq == 0U); /* 坏四元数帧未发布。 */
  assert(s.euler_seq == 1U);
  assert(s.bad_checksum_frames == 1U);
}

/** 错误长度与未知功能字。 */
static void Test_BadLengthAndUnknownFunc(void)
{
  ImuSensor_Sample s;
  uint8_t frame[32];
  uint16_t len;
  uint8_t payload[17];
  int16_t v[9] = {1, 2, 3, 4, 5, 6, 7, 8, 9};

  TestReset(0, 0);
  /* RAW 帧故意少一个 int16：总长 22，校验和按 22 字节重算并通过。 */
  memset(payload, 0, sizeof(payload));
  (void)BuildRawFrame(v, frame);
  memcpy(payload, &frame[4], 17U);
  len = Build7e23(0x04U, payload, sizeof(payload), frame);
  assert(len == 22U);
  assert(ImuSensor_Feed(frame, len, 6000ULL) == (int)len);

  len = Build7e23(0x55U, payload, 1U, frame); /* 未知功能字。 */
  assert(ImuSensor_Feed(frame, len, 6100ULL) == (int)len);
  ImuSensor_Process(6100ULL);

  assert(ImuSensor_GetSample(&s, 6100ULL) == 0);
  assert(s.bad_length_frames == 1U);
  assert(s.unknown_func_frames == 1U);
  assert(s.good_frames == 0U);
}

/** 浮点帧含 NaN/Inf：整帧拒收并计数。 */
static void Test_NonfiniteFloat(void)
{
  ImuSensor_Sample s;
  uint8_t frame[32];
  uint16_t len;
  float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
  float e[3] = {0.1f, 0.0f, 0.0f};

  TestReset(0, 0);
  len = BuildQuatFrame(q, frame);
  /* 第二个 float 改为 qNaN（LE 字节 00 00 C0 7F），并重算累加和，
   * 保证失败发生在 isfinite 检查而不是校验和。 */
  frame[8] = 0x00U;
  frame[9] = 0x00U;
  frame[10] = 0xC0U;
  frame[11] = 0x7FU;
  {
    uint8_t sum = 0U;
    uint16_t i;

    for (i = 0U; (uint16_t)(i + 1U) < len; i++)
    {
      sum = (uint8_t)(sum + frame[i]);
    }
    frame[len - 1U] = sum;
  }
  assert(ImuSensor_Feed(frame, len, 7000ULL) == (int)len);
  len = BuildEulerFrame(e, frame);
  assert(ImuSensor_Feed(frame, len, 7100ULL) == (int)len);
  ImuSensor_Process(7100ULL);

  assert(ImuSensor_GetSample(&s, 7100ULL) == 1);
  assert(s.quat_seq == 0U);
  assert(s.euler_seq == 1U);
  assert(s.nonfinite_frames == 1U);
}

/** 未对齐偏移：帧起始位于奇数偏移仍精确解码。 */
static void Test_UnalignedOffsets(void)
{
  ImuSensor_Sample s;
  uint8_t buf[64];
  uint16_t len = 0U;
  uint16_t n;
  float e[3] = {0.25f, -0.5f, 1.0f};
  float q[4] = {0.0f, 1.0f, 0.0f, 0.0f};
  static const uint8_t pad1[1] = {0x00U};
  static const uint8_t pad3[3] = {0x00U, 0x00U, 0x00U};

  TestReset(0, 0);
  memcpy(&buf[len], pad1, sizeof(pad1));
  len = (uint16_t)(len + sizeof(pad1));
  n = BuildEulerFrame(e, &buf[len]);
  len = (uint16_t)(len + n);
  memcpy(&buf[len], pad3, sizeof(pad3));
  len = (uint16_t)(len + sizeof(pad3));
  n = BuildQuatFrame(q, &buf[len]);
  len = (uint16_t)(len + n);

  assert(ImuSensor_Feed(buf, len, 8000ULL) == (int)len);
  ImuSensor_Process(8000ULL);

  assert(ImuSensor_GetSample(&s, 8000ULL) == 1);
  AssertFloatNear(s.euler_rpy_rad[0], 0.25f);
  AssertFloatNear(s.euler_rpy_rad[1], -0.5f);
  AssertFloatNear(s.quat_wxyz[1], 1.0f);
}

/** 参数边界、缓存语义、无 ACK 的 UNCONFIRMED、字面量锚定。 */
static void Test_ParamAndRequests(void)
{
  ImuSensor_Sample s;
  ImuSensor_Result r;
  uint8_t expected[16];
  uint16_t len;
  uint32_t seq = 0U;
  uint32_t value = 0U;
  /* 统筹文档给定的版本请求字面量（校验和算法锚点）。 */
  static const uint8_t ver_req[7] = {0x7EU, 0x23U, 0x07U, 0x80U,
                                     0x01U, 0x00U, 0x29U};
  /* 版本回包字面量：7E 23 08 01 12 34 56 46。 */
  static const uint8_t ver_rsp[8] = {0x7EU, 0x23U, 0x08U, 0x01U,
                                     0x12U, 0x34U, 0x56U, 0x46U};

  TestReset(0, 0);

  /* 引脚占用默认值：一切请求被拒，UART 不发帧。 */
  assert(ImuSensor_IsPinsBlocked() == 1U);
  assert(ImuSensor_Request(IMU_SENSOR_REQ_SET_RATE, 50U, 1000ULL, &seq) ==
         IMU_SENSOR_RES_PIN_BLOCKED);
  assert(ImuSensor_Request(IMU_SENSOR_REQ_GET_VERSION, 0U, 1000ULL, &seq) ==
         IMU_SENSOR_RES_PIN_BLOCKED);
  assert(tx_count == 0U);
  assert(ImuSensor_PopResult(&r) == 0);
  ImuSensor_SetPinsBlocked(0U);
  assert(ImuSensor_IsPinsBlocked() == 0U);

  /* 无缓存读回。 */
  assert(ImuSensor_GetParameter(IMU_SENSOR_PARAM_OUTPUT_RATE_HZ, &value) == 0);

  /* 参数边界。 */
  assert(ImuSensor_Request(IMU_SENSOR_REQ_SET_RATE, 9U, 1000ULL, &seq) ==
         IMU_SENSOR_RES_BAD_VALUE);
  assert(ImuSensor_Request(IMU_SENSOR_REQ_SET_RATE, 101U, 1000ULL, &seq) ==
         IMU_SENSOR_RES_BAD_VALUE);
  assert(tx_count == 0U);

  /* 0x60 受理：字面帧 + 立即 UNCONFIRMED（明确无回复，不等 0x81）。 */
  assert(ImuSensor_Request(IMU_SENSOR_REQ_SET_RATE, 10U, 1000ULL, &seq) == 0);
  assert(seq == 1U);
  len = Build7e23(0x60U, (const uint8_t *)"\x0A\x00\x5F", 3U, expected);
  AssertTxFrame(0U, expected, len);
  assert(ImuSensor_PopResult(&r) == 1);
  assert(r.op == IMU_SENSOR_REQ_SET_RATE && r.host_seq == 1U);
  assert(r.result == IMU_SENSOR_RES_UNCONFIRMED);
  assert(ImuSensor_PopResult(&r) == 0); /* 没有第二个结果。 */
  assert(ImuSensor_GetParameter(IMU_SENSOR_PARAM_OUTPUT_RATE_HZ, &value) == 1);
  assert(value == 10U);

  /* 0x61 模式边界。 */
  assert(ImuSensor_Request(IMU_SENSOR_REQ_SET_MODE, 7U, 1000ULL, &seq) ==
         IMU_SENSOR_RES_BAD_VALUE);
  assert(ImuSensor_Request(IMU_SENSOR_REQ_SET_MODE, 6U, 1000ULL, &seq) == 0);
  len = Build7e23(0x61U, (const uint8_t *)"\x06\x5F", 2U, expected);
  AssertTxFrame(1U, expected, len);
  assert(ImuSensor_PopResult(&r) == 1);
  assert(r.result == IMU_SENSOR_RES_UNCONFIRMED);
  assert(ImuSensor_GetParameter(IMU_SENSOR_PARAM_ALGORITHM_MODE, &value) == 1);
  assert(value == 6U);

  /* 0x80 版本请求：与统筹文档字面量逐字节一致。 */
  assert(ImuSensor_Request(IMU_SENSOR_REQ_GET_VERSION, 0U, 1000ULL, &seq) == 0);
  AssertTxFrame(2U, ver_req, sizeof(ver_req));

  /* 0x01 版本回包（字面量字节序列）完成请求。 */
  assert(ImuSensor_Feed(ver_rsp, sizeof(ver_rsp), 2000ULL) == (int)sizeof(ver_rsp));
  ImuSensor_Process(2000ULL);
  assert(ImuSensor_PopResult(&r) == 1);
  assert(r.op == IMU_SENSOR_REQ_GET_VERSION && r.host_seq == seq);
  assert(r.result == IMU_SENSOR_RES_OK);
  assert(r.version[0] == 0x12U && r.version[1] == 0x34U &&
         r.version[2] == 0x56U);
  assert(ImuSensor_GetSample(&s, 2000ULL) == 1);
  assert(s.version_valid == 1U);
  assert(s.version[0] == 0x12U);
}

/** 待确认请求超时与单一在飞 BUSY。 */
static void Test_TimeoutAndBusy(void)
{
  ImuSensor_Result r;
  uint32_t seq = 0U;

  TestReset(0, 1000ULL); /* 超时 1ms，便于测试。 */
  ImuSensor_SetPinsBlocked(0U);

  assert(ImuSensor_Request(IMU_SENSOR_REQ_GET_VERSION, 0U, 100000ULL,
                           &seq) == 0);
  assert(ImuSensor_Request(IMU_SENSOR_REQ_GET_VERSION, 0U, 100000ULL,
                           &seq) == IMU_SENSOR_RES_BUSY);
  assert(ImuSensor_Request(IMU_SENSOR_REQ_CAL_ACCEL_GYRO_START, 0U,
                           100000ULL, &seq) == IMU_SENSOR_RES_BUSY);
  /* 待确认请求在飞时无回复命令也保守拒绝。 */
  assert(ImuSensor_Request(IMU_SENSOR_REQ_SET_RATE, 50U, 100000ULL,
                           &seq) == IMU_SENSOR_RES_BUSY);
  assert(tx_count == 1U);

  ImuSensor_Process(100999ULL);
  assert(ImuSensor_PopResult(&r) == 0); /* 未到超时。 */
  ImuSensor_Process(101000ULL);
  assert(ImuSensor_PopResult(&r) == 1);
  assert(r.result == IMU_SENSOR_RES_TIMEOUT && r.host_seq == 1U);
  assert(ImuSensor_PopResult(&r) == 0);

  /* 超时后可重新发起；0x60 无回复立即 UNCONFIRMED。 */
  assert(ImuSensor_Request(IMU_SENSOR_REQ_GET_VERSION, 0U, 101000ULL,
                           &seq) == 0);
  assert(ImuSensor_Request(IMU_SENSOR_REQ_SET_RATE, 50U, 101000ULL, &seq) ==
         IMU_SENSOR_RES_BUSY);
  assert(ImuSensor_Feed((const uint8_t *)"\x7E\x23\x08\x01\x01\x02\x03\xB0",
                        8U, 101000ULL) == 8);
  ImuSensor_Process(101000ULL);
  assert(ImuSensor_PopResult(&r) == 1 && r.result == IMU_SENSOR_RES_OK);
  assert(ImuSensor_Request(IMU_SENSOR_REQ_SET_RATE, 50U, 101000ULL, &seq) == 0);
  assert(ImuSensor_PopResult(&r) == 1);
  assert(r.result == IMU_SENSOR_RES_UNCONFIRMED);
}

/** 默认超时（request_timeout_us=0）：版本查询 1s、0x70/0x71 校准 30s。 */
static void Test_DefaultTimeouts(void)
{
  ImuSensor_Result r;
  uint32_t seq = 0U;

  TestReset(0, 0); /* 显式 0 → 按命令取默认。 */
  ImuSensor_SetPinsBlocked(0U);

  assert(ImuSensor_Request(IMU_SENSOR_REQ_GET_VERSION, 0U, 0ULL, &seq) == 0);
  ImuSensor_Process(999999ULL);
  assert(ImuSensor_PopResult(&r) == 0);
  ImuSensor_Process(1000000ULL);
  assert(ImuSensor_PopResult(&r) == 1);
  assert(r.result == IMU_SENSOR_RES_TIMEOUT);

  assert(ImuSensor_Request(IMU_SENSOR_REQ_CAL_ACCEL_GYRO_START, 0U,
                           2000000ULL, &seq) == 0);
  ImuSensor_Process(2000000ULL + 30000000ULL - 1ULL);
  assert(ImuSensor_PopResult(&r) == 0);
  ImuSensor_Process(2000000ULL + 30000000ULL);
  assert(ImuSensor_PopResult(&r) == 1);
  assert(r.result == IMU_SENSOR_RES_TIMEOUT);
}

/** 显式非 0 超时对版本与校准统一覆盖（便于 host 测试缩短等待）。 */
static void Test_ExplicitTimeoutOverride(void)
{
  ImuSensor_Result r;
  uint32_t seq = 0U;

  TestReset(0, 5000ULL); /* 非 0：版本与校准统一 5ms。 */
  ImuSensor_SetPinsBlocked(0U);

  assert(ImuSensor_Request(IMU_SENSOR_REQ_GET_VERSION, 0U, 100000ULL,
                           &seq) == 0);
  ImuSensor_Process(104999ULL);
  assert(ImuSensor_PopResult(&r) == 0);
  ImuSensor_Process(105000ULL);
  assert(ImuSensor_PopResult(&r) == 1);
  assert(r.result == IMU_SENSOR_RES_TIMEOUT);

  assert(ImuSensor_Request(IMU_SENSOR_REQ_CAL_MAG_START, 0U, 200000ULL,
                           &seq) == 0);
  ImuSensor_Process(204999ULL);
  assert(ImuSensor_PopResult(&r) == 0);
  ImuSensor_Process(205000ULL);
  assert(ImuSensor_PopResult(&r) == 1);
  assert(r.result == IMU_SENSOR_RES_TIMEOUT);
}

/** 校准有 ACK：0x81 匹配原命令；状态 0 视为设备报告失败。 */
static void Test_CalAck(void)
{
  ImuSensor_Result r;
  uint8_t expected[16];
  uint8_t frame[16];
  uint16_t len;
  uint32_t seq = 0U;

  TestReset(0, 50000ULL);
  ImuSensor_SetPinsBlocked(0U);

  /* accel+gyro 校准 → 原生 0x70 action=1（映射假设，未实机核实）。 */
  assert(ImuSensor_Request(IMU_SENSOR_REQ_CAL_ACCEL_GYRO_START, 0U,
                           100000ULL, &seq) == 0);
  len = Build7e23(0x70U, (const uint8_t *)"\x01\x5F", 2U, expected);
  AssertTxFrame(0U, expected, len);
  assert(ImuSensor_PopResult(&r) == 0); /* 不立即出结果。 */
  len = BuildCalAck(0x70U, 1U, frame);
  assert(ImuSensor_Feed(frame, len, 110000ULL) == (int)len);
  ImuSensor_Process(110000ULL);
  assert(ImuSensor_PopResult(&r) == 1);
  assert(r.op == IMU_SENSOR_REQ_CAL_ACCEL_GYRO_START && r.host_seq == seq);
  assert(r.result == IMU_SENSOR_RES_OK);

  /* mag 校准 → 原生 0x71；回包状态 0 → IO_ERROR。 */
  assert(ImuSensor_Request(IMU_SENSOR_REQ_CAL_MAG_START, 0U, 120000ULL,
                           &seq) == 0);
  len = Build7e23(0x71U, (const uint8_t *)"\x01\x5F", 2U, expected);
  AssertTxFrame(1U, expected, len);
  len = BuildCalAck(0x71U, 0U, frame);
  assert(ImuSensor_Feed(frame, len, 121000ULL) == (int)len);
  ImuSensor_Process(121000ULL);
  assert(ImuSensor_PopResult(&r) == 1);
  assert(r.result == IMU_SENSOR_RES_IO_ERROR);

  /* 原命令不匹配的 0x81 不能完成请求 → 计 unexpected，随后超时。 */
  assert(ImuSensor_Request(IMU_SENSOR_REQ_CAL_ACCEL_GYRO_CLEAR, 0U,
                           130000ULL, &seq) == 0);
  len = BuildCalAck(0x71U, 1U, frame);
  assert(ImuSensor_Feed(frame, len, 131000ULL) == (int)len);
  ImuSensor_Process(131000ULL);
  assert(ImuSensor_PopResult(&r) == 0);
  ImuSensor_Process(130000ULL + 50000ULL);
  assert(ImuSensor_PopResult(&r) == 1);
  assert(r.result == IMU_SENSOR_RES_TIMEOUT);
}

/** 发送失败：IO_ERROR、不进待确认、不写缓存。 */
static void Test_TxError(void)
{
  ImuSensor_Result r;
  uint32_t seq = 0U;
  uint32_t value = 0U;

  TestReset(1, 0); /* 发送桩返回失败。 */
  ImuSensor_SetPinsBlocked(0U);

  assert(ImuSensor_Request(IMU_SENSOR_REQ_SET_RATE, 10U, 1000ULL, &seq) == 0);
  assert(ImuSensor_PopResult(&r) == 1);
  assert(r.result == IMU_SENSOR_RES_IO_ERROR);
  assert(ImuSensor_GetParameter(IMU_SENSOR_PARAM_OUTPUT_RATE_HZ, &value) == 0);

  assert(ImuSensor_Request(IMU_SENSOR_REQ_GET_VERSION, 0U, 1000ULL, &seq) == 0);
  assert(ImuSensor_PopResult(&r) == 1);
  assert(r.result == IMU_SENSOR_RES_IO_ERROR);
  ImuSensor_Process(1000ULL);
  assert(ImuSensor_PopResult(&r) == 0); /* 无 TIMEOUT：从未在飞。 */
}

/** 预留/未核实命令一律 UNSUPPORTED。 */
static void Test_Unsupported(void)
{
  uint32_t seq = 0U;

  TestReset(0, 0);
  ImuSensor_SetPinsBlocked(0U);
  assert(ImuSensor_Request(IMU_SENSOR_REQ_CAL_TEMP, 0U, 1000ULL, &seq) ==
         IMU_SENSOR_RES_UNSUPPORTED);
  assert(ImuSensor_Request(IMU_SENSOR_REQ_SAVE_CONFIG, 0U, 1000ULL, &seq) ==
         IMU_SENSOR_RES_UNSUPPORTED);
  assert(ImuSensor_Request(IMU_SENSOR_REQ_SELF_TEST, 0U, 1000ULL, &seq) ==
         IMU_SENSOR_RES_UNSUPPORTED);
  assert(ImuSensor_Request(IMU_SENSOR_REQ_REBOOT, 0U, 1000ULL, &seq) ==
         IMU_SENSOR_RES_UNSUPPORTED);
  assert(ImuSensor_Request(99U, 0U, 1000ULL, &seq) ==
         IMU_SENSOR_RES_UNSUPPORTED);
  assert(tx_count == 0U);
  assert(ImuSensor_PopResult(NULL) == 0);
}

/** 各数据组时间戳与序号相互独立，按所属分块到达时刻记录。 */
static void Test_GroupTimestampsIndependent(void)
{
  ImuSensor_Sample s;
  uint8_t frame[32];
  uint16_t len;
  float e[3] = {0.1f, 0.0f, 0.0f};
  float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
  int16_t v[9] = {1, 2, 3, 4, 5, 6, 7, 8, 9};

  TestReset(0, 0);
  len = BuildEulerFrame(e, frame);
  assert(ImuSensor_Feed(frame, len, 1000ULL) == (int)len);
  len = BuildQuatFrame(q, frame);
  assert(ImuSensor_Feed(frame, len, 5000ULL) == (int)len);
  len = BuildRawFrame(v, frame);
  assert(ImuSensor_Feed(frame, len, 9000ULL) == (int)len);
  ImuSensor_Process(9000ULL);

  assert(ImuSensor_GetSample(&s, 9000ULL) == 1);
  assert(s.euler_seq == 1U && s.euler_time_us == 1000ULL);
  assert(s.quat_seq == 1U && s.quat_time_us == 5000ULL);
  assert(s.raw_seq == 1U && s.raw_time_us == 9000ULL);

  len = BuildRawFrame(v, frame);
  assert(ImuSensor_Feed(frame, len, 9500ULL) == (int)len);
  ImuSensor_Process(9500ULL);
  assert(ImuSensor_GetSample(&s, 9500ULL) == 1);
  assert(s.raw_seq == 2U && s.raw_time_us == 9500ULL);
  assert(s.quat_seq == 1U && s.quat_time_us == 5000ULL);
  assert(s.euler_seq == 1U && s.euler_time_us == 1000ULL);
}

/** 有界接收：整块丢弃、分块队列耗尽与恢复。 */
static void Test_RingOverflow(void)
{
  ImuSensor_Sample s;
  uint8_t big[2000];
  uint8_t frame[32];
  uint16_t len;
  uint16_t i;
  int16_t v[9] = {1, 2, 3, 4, 5, 6, 7, 8, 9};

  TestReset(0, 0);
  memset(big, 0, sizeof(big));
  assert(ImuSensor_Feed(big, sizeof(big), 1000ULL) == 0);
  assert(ImuSensor_GetSample(&s, 1000ULL) == 0);
  assert(s.rx_overflow_chunks == 1U);
  assert(s.rx_dropped_bytes == sizeof(big));

  /* 16 个 1 字节分块占满分块队列，第 17 个被整块拒绝。 */
  for (i = 0U; i < 16U; i++)
  {
    assert(ImuSensor_Feed(big, 1U, 1000ULL) == 1);
  }
  assert(ImuSensor_Feed(big, 1U, 1000ULL) == 0);
  assert(s.rx_overflow_chunks == 1U);
  ImuSensor_Process(1000ULL);
  assert(ImuSensor_GetSample(&s, 1000ULL) == 0); /* 16 个零字节只是噪声。 */
  assert(s.rx_overflow_chunks == 2U);

  /* 恢复后正常解析。 */
  len = BuildRawFrame(v, frame);
  assert(ImuSensor_Feed(frame, len, 2000ULL) == (int)len);
  ImuSensor_Process(2000ULL);
  assert(ImuSensor_GetSample(&s, 2000ULL) == 1);
  assert(s.raw_seq == 1U);
}

/** 状态位：PIN_BLOCKED/CONFIG_UNKNOWN/新鲜度/ONLINE。 */
static void Test_StatusBits(void)
{
  ImuSensor_Sample s;
  uint8_t frame[32];
  uint16_t len;
  int16_t v[9] = {1, 2, 3, 4, 5, 6, 7, 8, 9};

  TestReset(0, 0);
  assert(ImuSensor_GetSample(&s, 1000ULL) == 0);
  assert((s.status & IMU_SENSOR_STATUS_PIN_BLOCKED) != 0U);
  assert((s.status & IMU_SENSOR_STATUS_CONFIG_UNKNOWN) != 0U);
  assert((s.status & IMU_SENSOR_STATUS_ONLINE) == 0U);
  /* 型号无读回，MODEL_CONFIRMED 恒不置位。 */
  assert((s.status & (1UL << 11)) == 0U);

  ImuSensor_SetPinsBlocked(0U);
  assert(ImuSensor_GetSample(&s, 1000ULL) == 0); /* 仍无数据，但状态位已更新。 */
  assert((s.status & IMU_SENSOR_STATUS_PIN_BLOCKED) == 0U);

  len = BuildRawFrame(v, frame);
  assert(ImuSensor_Feed(frame, len, 5000ULL) == (int)len);
  ImuSensor_Process(5000ULL);
  assert(ImuSensor_GetSample(&s, 5100ULL) == 1);
  assert((s.status & IMU_SENSOR_STATUS_ONLINE) != 0U);
  assert((s.status & IMU_SENSOR_STATUS_RAW_VALID) != 0U);
  assert((s.status & IMU_SENSOR_STATUS_QUAT_VALID) == 0U);
  assert((s.status & IMU_SENSOR_STATUS_EULER_VALID) == 0U);
  assert((s.status & IMU_SENSOR_STATUS_PRESSURE_VALID) == 0U);

  /* 超过新鲜度窗口（500ms）：RAW 不能刷新 ONLINE。 */
  assert(ImuSensor_GetSample(&s, 5000ULL + IMU_SENSOR_FRESH_US + 1ULL) == 1);
  assert((s.status & IMU_SENSOR_STATUS_ONLINE) == 0U);
  assert((s.status & IMU_SENSOR_STATUS_RAW_VALID) == 0U);

  /* UNCONFIRMED 的 rate/mode 下发不清除 CONFIG_UNKNOWN：无原生读回，
   * 配置永远无法确认（严格解释，统筹 2026-10-06 复核）。 */
  assert(ImuSensor_Request(IMU_SENSOR_REQ_SET_RATE, 100U, 6000ULL, NULL) == 0);
  assert(ImuSensor_GetSample(&s, 6000ULL) == 1);
  assert((s.status & IMU_SENSOR_STATUS_CONFIG_UNKNOWN) != 0U);
  assert(ImuSensor_Request(IMU_SENSOR_REQ_SET_MODE, 9U, 6000ULL, NULL) == 0);
  assert(ImuSensor_GetSample(&s, 6000ULL) == 1);
  assert((s.status & IMU_SENSOR_STATUS_CONFIG_UNKNOWN) != 0U);
}

/** 气压计帧解码与压力/温度新鲜度位。 */
static void Test_Barometer(void)
{
  ImuSensor_Sample s;
  uint8_t frame[32];
  uint16_t len;
  float b[4] = {100.5f, 25.25f, 101325.0f, 101000.0f};

  TestReset(0, 0);
  len = BuildBaroFrame(b, frame);
  assert(ImuSensor_Feed(frame, len, 3000ULL) == (int)len);
  ImuSensor_Process(3000ULL);

  assert(ImuSensor_GetSample(&s, 3000ULL) == 1);
  assert(s.baro_seq == 1U && s.baro_time_us == 3000ULL);
  AssertFloatNear(s.baro_height_m, 100.5f);
  AssertFloatNear(s.baro_temp_c, 25.25f);
  AssertFloatNear(s.baro_pressure_pa, 101325.0f);
  AssertFloatNear(s.baro_ref_pa, 101000.0f);
  assert((s.status & IMU_SENSOR_STATUS_PRESSURE_VALID) != 0U);
  assert((s.status & IMU_SENSOR_STATUS_TEMPERATURE_VALID) != 0U);
  assert((s.status & IMU_SENSOR_STATUS_ONLINE) == 0U); /* 气压计不算数据流。 */
}

/** 边界入参与防御性检查。 */
static void Test_EdgeInputs(void)
{
  uint8_t byte = 0x00U;

  TestReset(0, 0);
  assert(ImuSensor_Feed(NULL, 10U, 0ULL) == 0);
  assert(ImuSensor_Feed(&byte, 0U, 0ULL) == 0);
  assert(ImuSensor_GetSample(NULL, 0ULL) == 0);
  ImuSensor_Process(0ULL);
  assert(ImuSensor_GetParameter(0x1234U, NULL) == -1);
}

int main(void)
{
  Test_RawScalingAndNegatives();
  Test_FragmentedFeed();
  Test_CoalescedAndNoise();
  Test_BadChecksum();
  Test_BadLengthAndUnknownFunc();
  Test_NonfiniteFloat();
  Test_UnalignedOffsets();
  Test_ParamAndRequests();
  Test_TimeoutAndBusy();
  Test_DefaultTimeouts();
  Test_ExplicitTimeoutOverride();
  Test_CalAck();
  Test_TxError();
  Test_Unsupported();
  Test_GroupTimestampsIndependent();
  Test_RingOverflow();
  Test_StatusBits();
  Test_Barometer();
  Test_EdgeInputs();
  puts("imu_sensor_host_test: PASS");
  return 0;
}
