/*
 * MS5837 深度计主机单元测试。
 *
 * 目标：
 *  1. 用独立复现的数据手册公式与官方算例验证补偿、CRC4、型号差异；
 *  2. 用带转换时间模型的 I2C 从设备仿真验证非阻塞状态机、短超时、离线恢复；
 *  3. 验证“型号未确认不补偿”“零点必须显式建立”的约定，确认没有伪零、伪深度。
 *
 * 构建（Windows + w64devkit gcc，工作区根目录执行）：
 *   gcc -std=c11 -Wall -Wextra -Werror -O0 -ICore/Inc tests/ms5837_host_test.c -o build/ms5837_host_test.exe
 *
 * 说明：本文件按仓库既有主机测试风格直接包含被测实现（sensor_i2c_bus.c / ms5837.c），
 * 因此可以做白盒断言（例如检查驱动的状态字和统计）。
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define I2C_HOST_TEST 1

#include "../Core/Inc/sensor_i2c_bus.h"
#include "../Core/Inc/ms5837.h"

/* ---------------------------------------------------------------- 时间桩 */
static uint32_t test_tick; /* 主机侧可控毫秒时钟。 */

uint32_t HAL_GetTick(void)
{
  return test_tick;
}

/* ---------------------------------------------------------------- 从设备仿真 */
/*
 * 仿真只实现 MS5837 真正用到的行为：复位、PROM 读、D1/D2 转换命令、ADC 读。
 * 关键点：转换有真实的最大转换时间（独立复制的数据手册 ADC 表），
 * 驱动若提前读 ADC 会被记为 early_reads 违规。
 */
#define SIM_PROM_WORDS   8U
#define SIM_COMMAND_LOG  512U

typedef struct
{
  uint16_t prom[SIM_PROM_WORDS]; /* 仿真 PROM，字 0 含 CRC。 */
  uint8_t online; /* 0 = 从设备不回 ACK。 */
  uint8_t stall; /* 1 = 总线卡死，返回 HAL_TIMEOUT。 */
  uint8_t busy; /* 1 = 返回 HAL_BUSY。 */
  uint8_t model; /* 仿真设备真实型号，决定转换时间。 */
  uint8_t conversion; /* 0 空闲 / 1 D1 / 2 D2。 */
  uint32_t conversion_command_ms; /* 收到转换命令的时刻。 */
  uint32_t conversion_ready_ms; /* 转换完成的时刻。 */
  uint32_t min_conv_wait_ms; /* 观测到的最小“命令→读 ADC”间隔。 */
  uint8_t min_conv_wait_valid; /* 是否已记录过转换等待。 */
  uint32_t tx_time_ms; /* 每次 HAL I2C 事务占用总线的毫秒数（模拟真实事务耗时）。 */
  uint32_t d1_value; /* D1 目标值。 */
  uint32_t d2_value; /* D2 目标值。 */
  uint32_t early_reads; /* 提前读 ADC 的次数（期望 0）。 */
  uint32_t reset_count; /* 收到复位命令的次数。 */
  uint32_t tx_count; /* 写事务次数。 */
  uint32_t rx_count; /* 读事务次数。 */
  uint32_t last_timeout_ms; /* 最近一次 HAL 调用收到的超时参数。 */
  uint8_t read_buffer[4]; /* 待读数据。 */
  uint8_t read_length; /* 待读字节数。 */
  uint8_t read_pending; /* 是否有待读数据。 */
  uint8_t command_log[SIM_COMMAND_LOG]; /* 命令字节日志。 */
  uint32_t command_count; /* 命令字节数量。 */
} SimDevice_t;

static SimDevice_t sim;

static const uint16_t sim_conv_us_30ba[6] = {600U, 1170U, 2280U, 4540U, 9040U, 18080U};
static const uint16_t sim_conv_us_02ba[6] = {560U, 1100U, 2170U, 4320U, 8610U, 17200U};

static uint32_t sim_conversion_ms(uint8_t model, uint8_t osr_index)
{
  uint32_t us = (model == MS5837_MODEL_02BA) ? sim_conv_us_02ba[osr_index]
                                             : sim_conv_us_30ba[osr_index];
  return (us + 999U) / 1000U;
}

static void sim_log_command(uint8_t command)
{
  if (sim.command_count < SIM_COMMAND_LOG)
  {
    sim.command_log[sim.command_count] = command;
  }
  sim.command_count++;
}

HAL_StatusTypeDef HAL_I2C_Master_Transmit(I2C_HandleTypeDef *hi2c,
                                          uint16_t dev_address,
                                          uint8_t *data,
                                          uint16_t size,
                                          uint32_t timeout)
{
  uint8_t command;

  (void)hi2c;
  sim.last_timeout_ms = timeout;
  sim.tx_count++;

  if (sim.stall != 0U)
  {
    test_tick += timeout; /* 卡死时总线占用整个超时时间。 */
    return HAL_TIMEOUT;
  }
  if (sim.online == 0U)
  {
    return HAL_ERROR;
  }
  if (sim.busy != 0U)
  {
    return HAL_BUSY;
  }
  if ((dev_address != (uint16_t)((uint16_t)I2C_BUS_MS5837_ADDRESS7 << 1)) || (size != 1U) ||
      (data == 0))
  {
    return HAL_ERROR;
  }

  command = data[0];
  sim_log_command(command);
  if (sim.tx_time_ms != 0U)
  {
    test_tick += sim.tx_time_ms; /* 事务本身占用总线时间，验证驱动的等待不受相位影响。 */
  }

  if (command == 0x1EU) /* 复位 */
  {
    sim.conversion = 0U;
    sim.read_pending = 0U;
    sim.reset_count++;
    return HAL_OK;
  }
  if ((command & 0xF0U) == 0x40U) /* D1 转换 */
  {
    uint8_t index = (uint8_t)((command >> 1) & 0x07U);
    if (index > 5U)
    {
      return HAL_ERROR;
    }
    sim.conversion = 1U;
    sim.conversion_command_ms = test_tick;
    sim.conversion_ready_ms = test_tick + sim_conversion_ms(sim.model, index);
    return HAL_OK;
  }
  if ((command & 0xF0U) == 0x50U) /* D2 转换 */
  {
    uint8_t index = (uint8_t)((command >> 1) & 0x07U);
    if (index > 5U)
    {
      return HAL_ERROR;
    }
    sim.conversion = 2U;
    sim.conversion_command_ms = test_tick;
    sim.conversion_ready_ms = test_tick + sim_conversion_ms(sim.model, index);
    return HAL_OK;
  }
  if ((command & 0xF0U) == 0xA0U) /* PROM 读 */
  {
    uint8_t index = (uint8_t)((command - 0xA0U) / 2U);
    if (index > 6U)
    {
      return HAL_ERROR;
    }
    sim.read_buffer[0] = (uint8_t)(sim.prom[index] >> 8);
    sim.read_buffer[1] = (uint8_t)(sim.prom[index] & 0xFFU);
    sim.read_length = 2U;
    sim.read_pending = 1U;
    return HAL_OK;
  }
  if (command == 0x00U) /* ADC 读 */
  {
    uint32_t value = (sim.conversion == 1U) ? sim.d1_value : sim.d2_value;
    uint32_t waited;
    if ((sim.conversion == 0U) || ((int32_t)(test_tick - sim.conversion_ready_ms) < 0))
    {
      sim.early_reads++; /* 转换未完成就被读取：驱动等待时间不足。 */
    }
    waited = test_tick - sim.conversion_command_ms;
    if ((sim.min_conv_wait_valid == 0U) || (waited < sim.min_conv_wait_ms))
    {
      sim.min_conv_wait_ms = waited;
      sim.min_conv_wait_valid = 1U;
    }
    sim.read_buffer[0] = (uint8_t)((value >> 16) & 0xFFU);
    sim.read_buffer[1] = (uint8_t)((value >> 8) & 0xFFU);
    sim.read_buffer[2] = (uint8_t)(value & 0xFFU);
    sim.read_length = 3U;
    sim.read_pending = 1U;
    return HAL_OK;
  }
  return HAL_ERROR;
}

HAL_StatusTypeDef HAL_I2C_Master_Receive(I2C_HandleTypeDef *hi2c,
                                         uint16_t dev_address,
                                         uint8_t *data,
                                         uint16_t size,
                                         uint32_t timeout)
{
  (void)hi2c;
  sim.last_timeout_ms = timeout;
  sim.rx_count++;

  if (sim.stall != 0U)
  {
    test_tick += timeout;
    return HAL_TIMEOUT;
  }
  if (sim.online == 0U)
  {
    return HAL_ERROR;
  }
  if ((sim.read_pending == 0U) || (data == 0) || (size != sim.read_length) ||
      (dev_address != (uint16_t)((uint16_t)I2C_BUS_MS5837_ADDRESS7 << 1)))
  {
    return HAL_ERROR;
  }
  memcpy(data, sim.read_buffer, size);
  sim.read_pending = 0U;
  if (sim.tx_time_ms != 0U)
  {
    test_tick += sim.tx_time_ms;
  }
  return HAL_OK;
}

#include "../Core/Src/sensor_i2c_bus.c"
#include "../Core/Src/ms5837.c"

/* ---------------------------------------------------------------- 测试框架 */
static uint32_t test_failures;
static const char *test_current;

#define TEST_BEGIN(name)                        \
  do                                            \
  {                                             \
    test_current = (name);                      \
    printf("[ RUN  ] %s\n", test_current);      \
  } while (0)

#define CHECK(cond)                                                        \
  do                                                                       \
  {                                                                        \
    if (!(cond))                                                           \
    {                                                                      \
      printf("[ FAIL ] %s: %s:%d: %s\n", test_current, __FILE__, __LINE__, \
             #cond);                                                       \
      test_failures++;                                                     \
      return;                                                              \
    }                                                                      \
  } while (0)

#define TEST_END() printf("[  OK  ] %s\n", test_current)

/* ---------------------------------------------------------------- 参考实现与数据 */
/* 官方算例（30BA）：TE MS5837-30BA 数据手册压力/温度计算示例。 */
static const uint16_t prom_30ba_example[MS5837_PROM_WORDS] = {
    0x0000U, 34982U, 36352U, 20328U, 22354U, 26646U, 26146U, 0x0000U};
/* 官方算例（02BA）：TE MS5837-02BA 数据手册压力/温度计算示例。 */
static const uint16_t prom_02ba_example[MS5837_PROM_WORDS] = {
    0x0000U, 46372U, 43981U, 29059U, 27842U, 31553U, 28165U, 0x0000U};
/* 测试用通用系数（非官方算例，用于深度/滤波/型号对比）。 */
static const uint16_t prom_cross[MS5837_PROM_WORDS] = {
    0x0000U, 40000U, 36000U, 20000U, 22000U, 26646U, 26146U, 0x0000U};

/*
 * 数据手册公式的独立复现（直接照抄流程图，除法用有符号右移 = 向负无穷取整）。
 * 只用于交叉验证驱动结果，不参与驱动内部计算。
 */
static void ref_compensate(uint8_t model,
                           const uint16_t prom[MS5837_PROM_WORDS],
                           uint32_t d1,
                           uint32_t d2,
                           int64_t *pressure_raw,
                           int32_t *temperature_centi_c)
{
  int64_t dt = (int64_t)d2 - ((int64_t)prom[5] << 8);
  int64_t temp = 2000 + ((dt * (int64_t)prom[6]) >> 23);
  int64_t off;
  int64_t sens;
  int64_t ti = 0;
  int64_t off_i = 0;
  int64_t sens_i = 0;
  int64_t pressure;

  if (model == MS5837_MODEL_02BA)
  {
    off = ((int64_t)prom[2] << 17) + (((int64_t)prom[4] * dt) >> 6);
    sens = ((int64_t)prom[1] << 16) + (((int64_t)prom[3] * dt) >> 7);
  }
  else
  {
    off = ((int64_t)prom[2] << 16) + (((int64_t)prom[4] * dt) >> 7);
    sens = ((int64_t)prom[1] << 15) + (((int64_t)prom[3] * dt) >> 8);
  }

  if (temp < 2000)
  {
    if (model == MS5837_MODEL_02BA)
    {
      ti = (11 * dt * dt) >> 35;
      off_i = (31 * (temp - 2000) * (temp - 2000)) >> 3;
      sens_i = (63 * (temp - 2000) * (temp - 2000)) >> 5;
    }
    else
    {
      ti = (3 * dt * dt) >> 33;
      off_i = (3 * (temp - 2000) * (temp - 2000)) >> 1;
      sens_i = (5 * (temp - 2000) * (temp - 2000)) >> 3;
      if (temp < -1500)
      {
        off_i += 7 * (temp + 1500) * (temp + 1500);
        sens_i += 4 * (temp + 1500) * (temp + 1500);
      }
    }
  }
  else
  {
    ti = (2 * dt * dt) >> 37;
    off_i = ((temp - 2000) * (temp - 2000)) >> 4;
    sens_i = 0;
  }

  off -= off_i;
  sens -= sens_i;
  temp -= ti;
  if (model == MS5837_MODEL_02BA)
  {
    pressure = ((((int64_t)d1 * sens) >> 21) - off) >> 15;
  }
  else
  {
    pressure = ((((int64_t)d1 * sens) >> 21) - off) >> 13;
  }
  *pressure_raw = pressure;
  *temperature_centi_c = (int32_t)temp;
}

static float ref_pressure_pa(uint8_t model, const uint16_t prom[MS5837_PROM_WORDS],
                             uint32_t d1, uint32_t d2)
{
  int64_t raw = 0;
  int32_t temp = 0;
  ref_compensate(model, prom, d1, d2, &raw, &temp);
  return (model == MS5837_MODEL_30BA) ? ((float)raw * 10.0f) : ((float)raw);
}

static int nearly_equal(float a, float b, float tolerance)
{
  float diff = a - b;
  if (diff < 0.0f)
  {
    diff = -diff;
  }
  return diff <= tolerance;
}

/* ---------------------------------------------------------------- 测试辅助 */
static I2C_HandleTypeDef fake_handle;

static void sim_set_prom_from_coefficients(const uint16_t coefficients[MS5837_PROM_WORDS])
{
  uint16_t prom[MS5837_PROM_WORDS];
  uint8_t crc;

  memcpy(prom, coefficients, sizeof(prom));
  prom[0] = 0x0100U; /* 低 12 位是厂商标识，参与 CRC 计算，CRC 必须按它计算。 */
  prom[7] = 0U;
  crc = Ms5837_Crc4(prom); /* 用驱动 CRC 写入一个自洽的 PROM，CRC 本身另有独立向量验证。 */
  prom[0] = (uint16_t)(((uint16_t)crc << 12) | 0x0100U);
  memcpy(sim.prom, prom, sizeof(prom));
}

static void sim_reset(void)
{
  memset(&sim, 0, sizeof(sim));
  sim.online = 1U;
  sim.model = MS5837_MODEL_30BA;
  sim.d1_value = 5000000U;
  sim.d2_value = 6800000U;
  sim_set_prom_from_coefficients(prom_cross);
}

/* 驱动状态机每一步推进 1 ms，直到条件满足或超时。 */
static uint8_t run_until(uint8_t (*predicate)(void), uint32_t max_steps)
{
  uint32_t step;
  for (step = 0U; step < max_steps; step++)
  {
    if (predicate() != 0U)
    {
      return 1U;
    }
    Ms5837_Process();
    test_tick++;
  }
  return predicate();
}

static uint8_t pred_prom_valid(void)
{
  return ((Ms5837_GetStatus() & MS5837_STATUS_PROM_VALID) != 0U) ? 1U : 0U;
}

static uint8_t pred_new_sample(void)
{
  return Ms5837_HasNewSample();
}

/* 复位驱动到“已初始化 + PROM 有效 + 有一个新样本”的状态。 */
static void harness_start(uint8_t model, uint32_t d1, uint32_t d2)
{
  sim_reset();
  sim.model = (model == MS5837_MODEL_UNKNOWN) ? MS5837_MODEL_30BA : model;
  sim.d1_value = d1;
  sim.d2_value = d2;
  test_tick = 0U;

  I2c_Init(&fake_handle);
  Ms5837_RestoreDefaults();
  (void)Ms5837_Init();
  if (model != MS5837_MODEL_UNKNOWN)
  {
    (void)Ms5837_SetModel(model);
  }
  if (run_until(pred_prom_valid, 200U) == 0U)
  {
    printf("[ FAIL ] %s: PROM 未在预期时间内变为有效\n", test_current);
    test_failures++;
    return;
  }
  Ms5837_ClearNewSampleFlag();
  if (run_until(pred_new_sample, 400U) == 0U)
  {
    printf("[ FAIL ] %s: 未在预期时间内产生样本\n", test_current);
    test_failures++;
  }
}

/* ---------------------------------------------------------------- 1. CRC4 */
static void Test_Crc4(void)
{
  uint16_t prom[MS5837_PROM_WORDS];
  uint16_t flipped[MS5837_PROM_WORDS];
  uint16_t masked[MS5837_PROM_WORDS];
  uint8_t index;

  TEST_BEGIN("Crc4");
  /* 官方算例系数（C1..C6）对应的 CRC 由数据手册 CRC-4 代码的独立 Python 复现得到。 */
  CHECK(Ms5837_Crc4(prom_30ba_example) == 0x02U);
  CHECK(Ms5837_Crc4(prom_02ba_example) == 0x08U);

  memset(prom, 0, sizeof(prom));
  CHECK(Ms5837_Crc4(prom) == 0x00U);
  memset(prom, 0, sizeof(prom));
  for (index = 1U; index <= 6U; index++)
  {
    prom[index] = 0xFFFFU;
  }
  CHECK(Ms5837_Crc4(prom) == 0x0CU);

  /* 单比特改动必须被检出。 */
  for (index = 0U; index < MS5837_PROM_WORDS; index++)
  {
    flipped[index] = prom_30ba_example[index];
  }
  flipped[1] ^= 0x0001U;
  CHECK(Ms5837_Crc4(flipped) != Ms5837_Crc4(prom_30ba_example));

  /* 字 0 的高 4 位（CRC 本身）必须被屏蔽；低 12 位是厂商标识，参与计算。 */
  for (index = 0U; index < MS5837_PROM_WORDS; index++)
  {
    masked[index] = prom_30ba_example[index];
  }
  masked[0] = 0x0000U;
  CHECK(Ms5837_Crc4(masked) == Ms5837_Crc4(prom_30ba_example));
  masked[0] = 0xF000U;
  CHECK(Ms5837_Crc4(masked) == Ms5837_Crc4(prom_30ba_example));

  /* 第 8 个字是保留字，必须固定按 0 参与。 */
  for (index = 0U; index < MS5837_PROM_WORDS; index++)
  {
    masked[index] = prom_30ba_example[index];
  }
  masked[7] = 0x1234U;
  CHECK(Ms5837_Crc4(masked) == Ms5837_Crc4(prom_30ba_example));
  TEST_END();
}

/* ---------------------------------------------------------------- 2/3. 官方算例 */
static void Test_OfficialExample30ba(void)
{
  int64_t pressure_raw = 0;
  int32_t temperature = 0;

  TEST_BEGIN("OfficialExample30ba");
  /* 数据手册示例：dT=-5962，TEMP=19.81 °C，P=3999.8 mbar（=399980 Pa）。 */
  CHECK(Ms5837_Compensate(MS5837_MODEL_30BA, prom_30ba_example, 4958179U, 6815414U,
                          &pressure_raw, &temperature) == 1U);
  CHECK(temperature == 1981); /* 19.81 °C：验证向负无穷取整与数据手册一致。 */
  CHECK(pressure_raw == 39998); /* 0.1 mbar/LSB。 */
  CHECK(nearly_equal((float)pressure_raw * 10.0f, 399980.0f, 10.0f));

  /* 与独立复现交叉验证。 */
  {
    int64_t ref_raw = 0;
    int32_t ref_temp = 0;
    ref_compensate(MS5837_MODEL_30BA, prom_30ba_example, 4958179U, 6815414U, &ref_raw, &ref_temp);
    CHECK(ref_raw == pressure_raw);
    CHECK(ref_temp == temperature);
  }
  TEST_END();
}

static void Test_OfficialExample02ba(void)
{
  int64_t pressure_raw = 0;
  int32_t temperature = 0;

  TEST_BEGIN("OfficialExample02ba");
  /* 数据手册示例：dT=68，TEMP=20.00 °C，P=1100.02 mbar（=110002 Pa）。 */
  CHECK(Ms5837_Compensate(MS5837_MODEL_02BA, prom_02ba_example, 6465444U, 8077636U,
                          &pressure_raw, &temperature) == 1U);
  CHECK(temperature == 2000);
  CHECK(pressure_raw == 110002); /* 0.01 mbar/LSB = 1 Pa/LSB。 */
  CHECK(nearly_equal((float)pressure_raw, 110002.0f, 0.5f));

  {
    int64_t ref_raw = 0;
    int32_t ref_temp = 0;
    ref_compensate(MS5837_MODEL_02BA, prom_02ba_example, 6465444U, 8077636U, &ref_raw, &ref_temp);
    CHECK(ref_raw == pressure_raw);
    CHECK(ref_temp == temperature);
  }
  TEST_END();
}

/* ---------------------------------------------------------------- 4. 温度分支 */
static void Test_TemperatureBranches(void)
{
  int64_t pressure_raw = 0;
  int32_t temperature = 0;
  int64_t ref_raw = 0;
  int32_t ref_temp = 0;
  uint32_t d2_low;
  uint32_t d2_very_low;
  uint32_t d2_high;

  TEST_BEGIN("TemperatureBranches");

  /* 30BA 极低温（TEMP < -15 °C）：额外叠加 OFFi/SENSi 项。 */
  d2_very_low = (uint32_t)(prom_30ba_example[5] * 256U - 1200000);
  CHECK(Ms5837_Compensate(MS5837_MODEL_30BA, prom_30ba_example, 4958179U, d2_very_low,
                          &pressure_raw, &temperature) == 1U);
  CHECK(temperature == -2243); /* -22.43 °C */
  CHECK(pressure_raw == 38110);
  ref_compensate(MS5837_MODEL_30BA, prom_30ba_example, 4958179U, d2_very_low, &ref_raw, &ref_temp);
  CHECK(ref_raw == pressure_raw);
  CHECK(ref_temp == temperature);

  /* 30BA 低温但高于 -15 °C。 */
  d2_low = (uint32_t)(prom_30ba_example[5] * 256U - 500000);
  CHECK(Ms5837_Compensate(MS5837_MODEL_30BA, prom_30ba_example, 4958179U, d2_low,
                          &pressure_raw, &temperature) == 1U);
  CHECK(temperature < 2000);
  CHECK(temperature > -1500);
  ref_compensate(MS5837_MODEL_30BA, prom_30ba_example, 4958179U, d2_low, &ref_raw, &ref_temp);
  CHECK(ref_raw == pressure_raw);
  CHECK(ref_temp == temperature);

  /* 30BA 高温（>= 20 °C）。 */
  d2_high = (uint32_t)(prom_30ba_example[5] * 256U + 900000);
  CHECK(Ms5837_Compensate(MS5837_MODEL_30BA, prom_30ba_example, 4958179U, d2_high,
                          &pressure_raw, &temperature) == 1U);
  CHECK(temperature == 4794);
  CHECK(pressure_raw == 41506);
  ref_compensate(MS5837_MODEL_30BA, prom_30ba_example, 4958179U, d2_high, &ref_raw, &ref_temp);
  CHECK(ref_raw == pressure_raw);

  /* 02BA 低温分支。 */
  {
    uint32_t d2_cold = (uint32_t)(prom_02ba_example[5] * 256U - 900000);
    CHECK(Ms5837_Compensate(MS5837_MODEL_02BA, prom_02ba_example, 6465444U, d2_cold,
                            &pressure_raw, &temperature) == 1U);
    CHECK(temperature == -1281);
    CHECK(pressure_raw == 102115);
    ref_compensate(MS5837_MODEL_02BA, prom_02ba_example, 6465444U, d2_cold, &ref_raw, &ref_temp);
    CHECK(ref_raw == pressure_raw);
    CHECK(ref_temp == temperature);
  }

  /* 型号未确认时不得补偿，也不得写入输出。 */
  pressure_raw = -1;
  temperature = -1;
  CHECK(Ms5837_Compensate(MS5837_MODEL_UNKNOWN, prom_30ba_example, 4958179U, 6815414U,
                          &pressure_raw, &temperature) == 0U);
  CHECK(pressure_raw == -1);
  CHECK(temperature == -1);
  TEST_END();
}

/* ---------------------------------------------------------------- 5. 型号差异 */
static void Test_ModelDifference(void)
{
  int64_t raw_30ba = 0;
  int64_t raw_02ba = 0;
  int32_t temp_30ba = 0;
  int32_t temp_02ba = 0;
  int64_t ref_raw = 0;
  int32_t ref_temp = 0;

  TEST_BEGIN("ModelDifference");

  CHECK(Ms5837_Compensate(MS5837_MODEL_30BA, prom_cross, 5000000U, 6800000U, &raw_30ba,
                          &temp_30ba) == 1U);
  CHECK(Ms5837_Compensate(MS5837_MODEL_02BA, prom_cross, 5000000U, 6800000U, &raw_02ba,
                          &temp_02ba) == 1U);
  /* 同样的原始数据，两种型号的补偿结果必须不同。 */
  CHECK(raw_30ba != raw_02ba);
  CHECK(raw_30ba == 93432);
  CHECK(raw_02ba == 46715);
  CHECK(temp_30ba == temp_02ba); /* 一阶温度相同，二阶系数不同但此处温度一致。 */

  ref_compensate(MS5837_MODEL_30BA, prom_cross, 5000000U, 6800000U, &ref_raw, &ref_temp);
  CHECK(ref_raw == raw_30ba);
  ref_compensate(MS5837_MODEL_02BA, prom_cross, 5000000U, 6800000U, &ref_raw, &ref_temp);
  CHECK(ref_raw == raw_02ba);

  /* 数据手册 ADC 表：最大转换时间不同，且型号未知时取较慢的一侧。 */
  CHECK(Ms5837_MaxConversionTimeMs(MS5837_MODEL_30BA, MS5837_OSR_256) == 1U); /* 0.60 ms -> 1 */
  CHECK(Ms5837_MaxConversionTimeMs(MS5837_MODEL_30BA, MS5837_OSR_512) == 2U); /* 1.17 */
  CHECK(Ms5837_MaxConversionTimeMs(MS5837_MODEL_30BA, MS5837_OSR_1024) == 3U); /* 2.28 */
  CHECK(Ms5837_MaxConversionTimeMs(MS5837_MODEL_30BA, MS5837_OSR_2048) == 5U); /* 4.54 */
  CHECK(Ms5837_MaxConversionTimeMs(MS5837_MODEL_30BA, MS5837_OSR_4096) == 10U); /* 9.04 */
  CHECK(Ms5837_MaxConversionTimeMs(MS5837_MODEL_30BA, MS5837_OSR_8192) == 19U); /* 18.08 */
  CHECK(Ms5837_MaxConversionTimeMs(MS5837_MODEL_02BA, MS5837_OSR_512) == 2U); /* 1.10 */
  CHECK(Ms5837_MaxConversionTimeMs(MS5837_MODEL_02BA, MS5837_OSR_2048) == 5U); /* 4.32 */
  CHECK(Ms5837_MaxConversionTimeMs(MS5837_MODEL_02BA, MS5837_OSR_4096) == 9U); /* 8.61 */
  CHECK(Ms5837_MaxConversionTimeMs(MS5837_MODEL_02BA, MS5837_OSR_8192) == 18U); /* 17.20 */
  CHECK(Ms5837_MaxConversionTimeMs(MS5837_MODEL_UNKNOWN, MS5837_OSR_8192) == 19U);
  TEST_END();
}

/* ---------------------------------------------------------------- 6. 命令序列 */
static void Test_CommandSequence(void)
{
  uint32_t index;
  uint32_t first_d1 = 0U;
  uint32_t first_d2 = 0U;
  uint8_t prom_reads = 0U;

  TEST_BEGIN("CommandSequence");

  sim_reset();
  sim.model = MS5837_MODEL_30BA;
  test_tick = 0U;
  I2c_Init(&fake_handle);
  Ms5837_RestoreDefaults();
  CHECK(Ms5837_SetModel(MS5837_MODEL_30BA) == MS5837_OK);
  CHECK(Ms5837_Init() == MS5837_OK);

  /* 复位必须是第一条命令，且复位前没有任何 PROM/转换命令。 */
  CHECK(sim.command_count >= 1U);
  CHECK(sim.command_log[0] == 0x1EU);
  CHECK(sim.reset_count == 1U);

  run_until(pred_prom_valid, 200U);
  CHECK(pred_prom_valid() != 0U);

  /*
   * 官方物理 PROM 只有 7 个 16 位字（0xA0..0xAC）。驱动必须按顺序读这 7 个地址，
   * 且绝不读 0xAE；CRC4 需要的第 8 个数组元素由软件置 0。
   * 仿真设备的 index>6 会返回 HAL_ERROR，因此若驱动依赖第 8 个字就永远无法上线。
   */
  {
    static const uint8_t expected_reads[7] = {0xA0U, 0xA2U, 0xA4U, 0xA6U, 0xA8U, 0xAAU, 0xACU};
    uint8_t actual_reads[7] = {0U, 0U, 0U, 0U, 0U, 0U, 0U};
    uint8_t seen = 0U;
    uint32_t c;

    for (c = 0U; c < sim.command_count; c++)
    {
      CHECK(sim.command_log[c] != 0xAEU); /* 不存在第 8 个字，禁止读 0xAE。 */
      if ((sim.command_log[c] & 0xF0U) == 0xA0U)
      {
        CHECK(seen < 7U); /* 多于 7 次 PROM 读同样说明读法不对。 */
        actual_reads[seen] = sim.command_log[c];
        seen++;
      }
    }
    CHECK(seen == 7U);
    for (index = 0U; index < 7U; index++)
    {
      CHECK(actual_reads[index] == expected_reads[index]);
      prom_reads++;
    }
  }
  CHECK(prom_reads == 7U);

  Ms5837_ClearNewSampleFlag();
  CHECK(run_until(pred_new_sample, 400U) != 0U);

  /* 默认 OSR4096 → D1=0x48、D2=0x58。 */
  for (index = 1U; index < sim.command_count; index++)
  {
    if (((sim.command_log[index] & 0xF0U) == 0x40U) && (first_d1 == 0U))
    {
      first_d1 = sim.command_log[index];
    }
    if (((sim.command_log[index] & 0xF0U) == 0x50U) && (first_d2 == 0U))
    {
      first_d2 = sim.command_log[index];
    }
  }
  CHECK(first_d1 == 0x48U);
  CHECK(first_d2 == 0x58U);

  /* OSR256 → 0x40/0x50。 */
  CHECK(Ms5837_SetOsr(MS5837_OSR_256) == MS5837_OK);
  sim.command_count = 0U;
  Ms5837_ClearNewSampleFlag();
  CHECK(run_until(pred_new_sample, 400U) != 0U);
  first_d1 = 0U;
  first_d2 = 0U;
  for (index = 0U; index < sim.command_count; index++)
  {
    if (((sim.command_log[index] & 0xF0U) == 0x40U) && (first_d1 == 0U))
    {
      first_d1 = sim.command_log[index];
    }
    if (((sim.command_log[index] & 0xF0U) == 0x50U) && (first_d2 == 0U))
    {
      first_d2 = sim.command_log[index];
    }
  }
  CHECK(first_d1 == 0x40U);
  CHECK(first_d2 == 0x50U);

  /* OSR8192 的周期预算是 42 ms，25 Hz 放不下：先降到 20 Hz；此时 D1=0x4A、D2=0x5A。 */
  CHECK(Ms5837_SetOsr(MS5837_OSR_8192) == MS5837_ERR_PARAM);
  CHECK(Ms5837_SetOutputRateHz(20U) == MS5837_OK);
  CHECK(Ms5837_SetOsr(MS5837_OSR_8192) == MS5837_OK);
  sim.command_count = 0U;
  Ms5837_ClearNewSampleFlag();
  CHECK(run_until(pred_new_sample, 800U) != 0U);
  first_d1 = 0U;
  first_d2 = 0U;
  for (index = 0U; index < sim.command_count; index++)
  {
    if (((sim.command_log[index] & 0xF0U) == 0x40U) && (first_d1 == 0U))
    {
      first_d1 = sim.command_log[index];
    }
    if (((sim.command_log[index] & 0xF0U) == 0x50U) && (first_d2 == 0U))
    {
      first_d2 = sim.command_log[index];
    }
  }
  CHECK(first_d1 == 0x4AU);
  CHECK(first_d2 == 0x5AU);
  TEST_END();
}

/* ---------------------------------------------------------------- 7. 型号未确认 */
static void Test_UnknownModelDefault(void)
{
  Ms5837Sample_t sample;
  uint16_t prom[MS5837_PROM_WORDS];

  TEST_BEGIN("UnknownModelDefault");

  harness_start(MS5837_MODEL_UNKNOWN, 5000000U, 6800000U);
  CHECK(Ms5837_GetModel() == MS5837_MODEL_UNKNOWN);
  CHECK((Ms5837_GetStatus() & MS5837_STATUS_PROM_VALID) != 0U);
  CHECK((Ms5837_GetStatus() & MS5837_STATUS_RAW_VALID) != 0U);
  CHECK((Ms5837_GetStatus() & MS5837_STATUS_CONFIG_UNKNOWN) != 0U);
  CHECK((Ms5837_GetStatus() & MS5837_STATUS_MODEL_CONFIRMED) == 0U);
  CHECK((Ms5837_GetStatus() & MS5837_STATUS_PRESSURE_VALID) == 0U);
  CHECK((Ms5837_GetStatus() & MS5837_STATUS_TEMPERATURE_VALID) == 0U);
  CHECK((Ms5837_GetStatus() & MS5837_STATUS_DEPTH_VALID) == 0U);

  CHECK(Ms5837_GetSample(&sample) == MS5837_OK);
  /* 原始数据可用，但压力/温度/深度必须是 NaN，不能是 0。 */
  CHECK(sample.d1 == 5000000U);
  CHECK(sample.d2 == 6800000U);
  CHECK(sample.pressure_raw == 0);
  CHECK(MS5837_IS_NAN(sample.pressure_pa));
  CHECK(MS5837_IS_NAN(sample.temperature_c));
  CHECK(MS5837_IS_NAN(sample.depth_raw_m));
  CHECK(MS5837_IS_NAN(sample.depth_filtered_m));
  CHECK(MS5837_IS_NAN(sample.surface_pressure_pa));
  CHECK((sample.status & MS5837_STATUS_PRESSURE_VALID) == 0U);

  /* 未确认型号时不允许建立零点。 */
  CHECK(Ms5837_Zero() == MS5837_ERR_MODEL_UNKNOWN);
  CHECK(Ms5837_IsZeroValid() == 0U);

  /* PROM 内容可以读出，包含 CRC 字。 */
  CHECK(Ms5837_GetProm(prom) == MS5837_OK);
  CHECK((prom[0] >> 12) == Ms5837_Crc4(prom));

  /* 显式设定型号后，下一帧就给出补偿值。 */
  CHECK(Ms5837_SetModel(MS5837_MODEL_30BA) == MS5837_OK);
  CHECK((Ms5837_GetStatus() & MS5837_STATUS_CONFIG_UNKNOWN) == 0U);
  CHECK((Ms5837_GetStatus() & MS5837_STATUS_MODEL_CONFIRMED) != 0U);
  Ms5837_ClearNewSampleFlag();
  CHECK(run_until(pred_new_sample, 400U) != 0U);
  CHECK(Ms5837_GetSample(&sample) == MS5837_OK);
  CHECK((sample.status & MS5837_STATUS_PRESSURE_VALID) != 0U);
  CHECK(!MS5837_IS_NAN(sample.pressure_pa));
  CHECK(nearly_equal(sample.pressure_pa, ref_pressure_pa(MS5837_MODEL_30BA, prom_cross, 5000000U, 6800000U), 1.0f));
  TEST_END();
}

/* ---------------------------------------------------------------- 8. 无零点无深度 */
static void Test_NoZeroMeansNoDepth(void)
{
  Ms5837Sample_t sample;
  float pa = 0.0f;
  uint8_t type = 0U;
  uint8_t length = 0U;
  uint8_t value[4];

  TEST_BEGIN("NoZeroMeansNoDepth");

  harness_start(MS5837_MODEL_30BA, 5000000U, 6800000U);
  CHECK(Ms5837_GetSample(&sample) == MS5837_OK);
  CHECK((sample.status & MS5837_STATUS_PRESSURE_VALID) != 0U);
  CHECK(!MS5837_IS_NAN(sample.pressure_pa));
  /* 没有显式零点：深度必须是 NaN，状态位必须清零，绝不出现“深度 0”。 */
  CHECK((sample.status & MS5837_STATUS_ZERO_VALID) == 0U);
  CHECK((sample.status & MS5837_STATUS_DEPTH_VALID) == 0U);
  CHECK(MS5837_IS_NAN(sample.depth_raw_m));
  CHECK(MS5837_IS_NAN(sample.depth_filtered_m));
  CHECK(MS5837_IS_NAN(sample.surface_pressure_pa));

  /* 参数 0103 在未设定零点时必须返回 NO_ZERO，而不是默认海平面压力。 */
  CHECK(Ms5837_GetParam(MS5837_PARAM_SURFACE_PRESSURE, &type, &length, value) == MS5837_ERR_NO_ZERO);
  CHECK(Ms5837_GetSurfacePressurePa(&pa) == MS5837_ERR_NO_ZERO);

  /* 显式设定零点压力后才允许输出深度。 */
  CHECK(Ms5837_SetSurfacePressurePa(101325.0f) == MS5837_OK);
  CHECK((Ms5837_GetStatus() & MS5837_STATUS_ZERO_VALID) != 0U);
  CHECK(Ms5837_GetSurfacePressurePa(&pa) == MS5837_OK);
  CHECK(nearly_equal(pa, 101325.0f, 0.01f));
  CHECK(Ms5837_GetSample(&sample) == MS5837_OK);
  CHECK((sample.status & MS5837_STATUS_DEPTH_VALID) != 0U);
  CHECK(!MS5837_IS_NAN(sample.depth_raw_m));
  CHECK(nearly_equal(sample.depth_raw_m,
                     (sample.pressure_pa - 101325.0f) / (MS5837_WATER_DENSITY_DEFAULT * MS5837_GRAVITY),
                     1e-3f));

  /* 清除零点后深度重新变为 NaN。 */
  CHECK(Ms5837_ClearZero() == MS5837_OK);
  CHECK(Ms5837_GetSample(&sample) == MS5837_OK);
  CHECK(MS5837_IS_NAN(sample.depth_raw_m));
  CHECK((sample.status & MS5837_STATUS_DEPTH_VALID) == 0U);
  TEST_END();
}

/* ---------------------------------------------------------------- 9. 零点与深度 */
static void Test_ZeroAndDepth(void)
{
  Ms5837Sample_t sample;
  float depth_expected;
  float pa1;
  float pa2;
  uint32_t d2_25c = (uint32_t)(((int64_t)prom_cross[5] * 256) +
                               (((int64_t)(2500 - 2000) * 8388608) / (int64_t)prom_cross[6]));

  TEST_BEGIN("ZeroAndDepth");

  /* 第一帧：D1=5000000 作为水面零点。 */
  sim_reset();
  sim.model = MS5837_MODEL_30BA;
  sim.d1_value = 5000000U;
  sim.d2_value = d2_25c;
  test_tick = 0U;
  I2c_Init(&fake_handle);
  Ms5837_RestoreDefaults();
  CHECK(Ms5837_SetModel(MS5837_MODEL_30BA) == MS5837_OK);
  CHECK(Ms5837_Init() == MS5837_OK);
  CHECK(run_until(pred_prom_valid, 200U) != 0U);
  Ms5837_ClearNewSampleFlag();
  CHECK(run_until(pred_new_sample, 400U) != 0U);
  CHECK(Ms5837_GetSample(&sample) == MS5837_OK);
  pa1 = ref_pressure_pa(MS5837_MODEL_30BA, prom_cross, 5000000U, d2_25c);
  CHECK(nearly_equal(sample.pressure_pa, pa1, 1.0f));

  /* 显式采集零点（AA5B ZERO_DEPTH 语义）。 */
  CHECK(Ms5837_Zero() == MS5837_OK);
  CHECK(Ms5837_IsZeroValid() == 1U);
  CHECK(Ms5837_GetSample(&sample) == MS5837_OK);
  CHECK(nearly_equal(sample.depth_raw_m, 0.0f, 1e-6f));
  CHECK(nearly_equal(sample.depth_filtered_m, 0.0f, 1e-6f));
  CHECK(nearly_equal(sample.surface_pressure_pa, pa1, 1.0f));

  /* 第二帧：加大 D1 模拟下潜，深度必须与 (P-P0)/(rho*g) 一致。 */
  sim.d1_value = 5100000U;
  Ms5837_ClearNewSampleFlag();
  CHECK(run_until(pred_new_sample, 400U) != 0U);
  CHECK(Ms5837_GetSample(&sample) == MS5837_OK);
  pa2 = ref_pressure_pa(MS5837_MODEL_30BA, prom_cross, 5100000U, d2_25c);
  depth_expected = (pa2 - pa1) / (MS5837_WATER_DENSITY_DEFAULT * MS5837_GRAVITY);
  CHECK(depth_expected > 7.0f);
  CHECK(depth_expected < 8.0f);
  CHECK(nearly_equal(sample.depth_raw_m, depth_expected, 1e-3f));
  /* 默认 FILTER_K=0：滤波深度等于原始深度。 */
  CHECK(nearly_equal(sample.depth_filtered_m, sample.depth_raw_m, 1e-6f));

  /* 02BA 也必须给出以 Pa 为单位的正确深度（1 LSB = 1 Pa）。 */
  sim_reset();
  sim.model = MS5837_MODEL_02BA;
  sim.d1_value = 6465444U;
  sim.d2_value = 8077636U;
  sim_set_prom_from_coefficients(prom_02ba_example);
  test_tick = 0U;
  I2c_Init(&fake_handle);
  Ms5837_RestoreDefaults();
  CHECK(Ms5837_SetModel(MS5837_MODEL_02BA) == MS5837_OK);
  CHECK(Ms5837_Init() == MS5837_OK);
  CHECK(run_until(pred_prom_valid, 200U) != 0U);
  Ms5837_ClearNewSampleFlag();
  CHECK(run_until(pred_new_sample, 400U) != 0U);
  CHECK(Ms5837_GetSample(&sample) == MS5837_OK);
  CHECK(nearly_equal(sample.pressure_pa, 110002.0f, 0.5f));
  CHECK(nearly_equal(sample.temperature_c, 20.0f, 0.001f));
  CHECK(Ms5837_Zero() == MS5837_OK);
  CHECK(Ms5837_GetSample(&sample) == MS5837_OK);
  CHECK(nearly_equal(sample.depth_raw_m, 0.0f, 1e-6f));
  TEST_END();
}

/* ---------------------------------------------------------------- 10. 滤波 */
static void Test_FilterK(void)
{
  Ms5837Sample_t sample;
  float first;
  float second;

  TEST_BEGIN("FilterK");

  harness_start(MS5837_MODEL_30BA, 5000000U, 6981794U);
  CHECK(Ms5837_Zero() == MS5837_OK);
  CHECK(Ms5837_SetFilterK(0.9f) == MS5837_OK);
  /* 改变零点后滤波重新起步：下一帧的滤波值必须等于原始值。 */
  Ms5837_ClearNewSampleFlag();
  CHECK(run_until(pred_new_sample, 400U) != 0U);
  CHECK(Ms5837_GetSample(&sample) == MS5837_OK);
  first = sample.depth_raw_m;
  CHECK(nearly_equal(sample.depth_filtered_m, first, 1e-6f));

  /* 阶跃后滤波值必须落在旧值与新值之间。 */
  sim.d1_value = 5100000U;
  Ms5837_ClearNewSampleFlag();
  CHECK(run_until(pred_new_sample, 400U) != 0U);
  CHECK(Ms5837_GetSample(&sample) == MS5837_OK);
  second = sample.depth_raw_m;
  CHECK(second > first);
  CHECK(sample.depth_filtered_m > first);
  CHECK(sample.depth_filtered_m < second);
  CHECK(nearly_equal(sample.depth_filtered_m, 0.9f * first + 0.1f * second, 1e-3f));
  TEST_END();
}

/* ---------------------------------------------------------------- 11. 参数非法 */
static void Test_ParameterValidation(void)
{
  TEST_BEGIN("ParameterValidation");

  harness_start(MS5837_MODEL_30BA, 5000000U, 6800000U);

  /* OSR 只允许 256/512/1024/2048/4096/8192。 */
  CHECK(Ms5837_SetOsr(0U) == MS5837_ERR_PARAM);
  CHECK(Ms5837_SetOsr(1000U) == MS5837_ERR_PARAM);
  CHECK(Ms5837_SetOsr(65535U) == MS5837_ERR_PARAM);
  CHECK(Ms5837_SetOsr(MS5837_OSR_1024) == MS5837_OK);
  CHECK(Ms5837_GetOsr() == MS5837_OSR_1024);

  /* 采样率范围 1~100 Hz。 */
  CHECK(Ms5837_SetOutputRateHz(0U) == MS5837_ERR_PARAM);
  CHECK(Ms5837_SetOutputRateHz(101U) == MS5837_ERR_PARAM);
  CHECK(Ms5837_SetOutputRateHz(10U) == MS5837_OK);

  /*
   * 周期预算 = 2×(最大转换时间 + 1 ms 余量) + 2 ms 事务预算，必须 ≤ 1000/rate。
   * 30BA OSR4096 → 24 ms：50 Hz(20 ms) 拒绝，40 Hz(25 ms) 允许，25 Hz 允许。
   */
  CHECK(Ms5837_SetOsr(MS5837_OSR_4096) == MS5837_OK);
  CHECK(Ms5837_SetOutputRateHz(50U) == MS5837_ERR_PARAM);
  CHECK(Ms5837_SetOutputRateHz(40U) == MS5837_OK);
  CHECK(Ms5837_SetOutputRateHz(25U) == MS5837_OK);

  /* 30BA OSR8192 → 42 ms：25 Hz(40 ms) 拒绝，20 Hz(50 ms) 允许；回到 25 Hz 又被拒。 */
  CHECK(Ms5837_SetOsr(MS5837_OSR_8192) == MS5837_ERR_PARAM);
  CHECK(Ms5837_SetOutputRateHz(20U) == MS5837_OK);
  CHECK(Ms5837_SetOsr(MS5837_OSR_8192) == MS5837_OK);
  CHECK(Ms5837_SetOutputRateHz(25U) == MS5837_ERR_PARAM);
  CHECK(Ms5837_SetOutputRateHz(20U) == MS5837_OK);

  /* 30BA OSR2048 → 14 ms：25 Hz 允许，100 Hz(10 ms) 拒绝。 */
  CHECK(Ms5837_SetOsr(MS5837_OSR_2048) == MS5837_OK);
  CHECK(Ms5837_SetOutputRateHz(25U) == MS5837_OK);
  CHECK(Ms5837_SetOutputRateHz(100U) == MS5837_ERR_PARAM);
  CHECK(Ms5837_SetOutputRateHz(50U) == MS5837_OK);
  CHECK(Ms5837_SetOutputRateHz(10U) == MS5837_OK);

  /* 30BA OSR256 → 6 ms：100 Hz 允许。 */
  CHECK(Ms5837_SetOsr(MS5837_OSR_256) == MS5837_OK);
  CHECK(Ms5837_SetOutputRateHz(100U) == MS5837_OK);

  /* 02BA 的最大转换时间更短：OSR4096 → 22 ms，25 Hz/40 Hz 允许；OSR8192 → 40 ms，25 Hz 正好允许。 */
  CHECK(Ms5837_SetModel(MS5837_MODEL_02BA) == MS5837_OK);
  CHECK(Ms5837_SetOutputRateHz(25U) == MS5837_OK);
  CHECK(Ms5837_SetOsr(MS5837_OSR_4096) == MS5837_OK);
  CHECK(Ms5837_SetOutputRateHz(40U) == MS5837_OK);
  CHECK(Ms5837_SetOutputRateHz(50U) == MS5837_ERR_PARAM); /* 20 ms < 22 ms */
  CHECK(Ms5837_SetOutputRateHz(25U) == MS5837_OK);
  CHECK(Ms5837_SetOsr(MS5837_OSR_8192) == MS5837_OK); /* 2×(18+1)+2 = 40 ms ≤ 40 ms */
  CHECK(Ms5837_SetOutputRateHz(25U) == MS5837_OK);
  CHECK(Ms5837_SetOsr(MS5837_OSR_4096) == MS5837_OK);
  CHECK(Ms5837_SetModel(MS5837_MODEL_30BA) == MS5837_OK);

  /* 型号只允许 0/2/30。 */
  CHECK(Ms5837_SetModel(1U) == MS5837_ERR_PARAM);
  CHECK(Ms5837_SetModel(5U) == MS5837_ERR_PARAM);
  CHECK(Ms5837_SetModel(99U) == MS5837_ERR_PARAM);
  CHECK(Ms5837_SetModel(MS5837_MODEL_UNKNOWN) == MS5837_OK);
  CHECK(Ms5837_SetModel(MS5837_MODEL_30BA) == MS5837_OK);

  /* 密度 900~1300 kg/m3。 */
  CHECK(Ms5837_SetWaterDensity(899.0f) == MS5837_ERR_PARAM);
  CHECK(Ms5837_SetWaterDensity(1301.0f) == MS5837_ERR_PARAM);
  CHECK(Ms5837_SetWaterDensity(1000.0f) == MS5837_OK);
  CHECK(nearly_equal(Ms5837_GetWaterDensity(), 1000.0f, 0.001f));

  /* 滤波系数 0~0.99。 */
  CHECK(Ms5837_SetFilterK(-0.01f) == MS5837_ERR_PARAM);
  CHECK(Ms5837_SetFilterK(1.0f) == MS5837_ERR_PARAM);
  CHECK(Ms5837_SetFilterK(0.99f) == MS5837_OK);
  CHECK(Ms5837_SetFilterK(0.0f) == MS5837_OK);

  /* 零点压力 10000~200000 Pa。 */
  CHECK(Ms5837_SetSurfacePressurePa(9999.0f) == MS5837_ERR_PARAM);
  CHECK(Ms5837_SetSurfacePressurePa(200001.0f) == MS5837_ERR_PARAM);
  CHECK(Ms5837_SetSurfacePressurePa(101325.0f) == MS5837_OK);

  /* 未知参数 ID。 */
  {
    uint8_t type = 0U;
    uint8_t length = 0U;
    uint8_t value[4];
    uint16_t u16 = 25U;
    CHECK(Ms5837_SetParam(0x9999U, MS5837_PARAM_TYPE_U16, &u16, 2U) == MS5837_ERR_UNSUPPORTED);
    CHECK(Ms5837_GetParam(0x9999U, &type, &length, value) == MS5837_ERR_UNSUPPORTED);
    /* 类型/长度不匹配。 */
    CHECK(Ms5837_SetParam(MS5837_PARAM_DEPTH_OSR, MS5837_PARAM_TYPE_F32, &u16, 4U) == MS5837_ERR_PARAM);
    CHECK(Ms5837_SetParam(MS5837_PARAM_DEPTH_OSR, MS5837_PARAM_TYPE_U16, &u16, 1U) == MS5837_ERR_PARAM);
    CHECK(Ms5837_SetParam(MS5837_PARAM_DEPTH_MODEL, MS5837_PARAM_TYPE_U8, 0, 1U) == MS5837_ERR_PARAM);
    CHECK(Ms5837_GetParam(MS5837_PARAM_DEPTH_OSR, 0, &length, value) == MS5837_ERR_PARAM);
  }
  TEST_END();
}

/* ---------------------------------------------------------------- 12. 参数线格式 */
static void Test_ParameterWireEncoding(void)
{
  uint8_t type = 0U;
  uint8_t length = 0U;
  uint8_t value[4];
  uint8_t density_le[4];
  uint16_t rate_le;
  float density = 1100.0f;
  float read_back;
  uint32_t bits;

  TEST_BEGIN("ParameterWireEncoding");

  harness_start(MS5837_MODEL_30BA, 5000000U, 6800000U);

  /* u16 小端。 */
  rate_le = 40U;
  CHECK(Ms5837_SetParam(MS5837_PARAM_OUTPUT_RATE_HZ, MS5837_PARAM_TYPE_U16, &rate_le, 2U) == MS5837_OK);
  CHECK(Ms5837_GetOutputRateHz() == 40U);
  CHECK(Ms5837_GetParam(MS5837_PARAM_OUTPUT_RATE_HZ, &type, &length, value) == MS5837_OK);
  CHECK(type == MS5837_PARAM_TYPE_U16);
  CHECK(length == 2U);
  CHECK(value[0] == 40U);
  CHECK(value[1] == 0U);

  /* f32 小端。 */
  memcpy(&bits, &density, sizeof(bits));
  density_le[0] = (uint8_t)(bits & 0xFFU);
  density_le[1] = (uint8_t)((bits >> 8) & 0xFFU);
  density_le[2] = (uint8_t)((bits >> 16) & 0xFFU);
  density_le[3] = (uint8_t)((bits >> 24) & 0xFFU);
  CHECK(Ms5837_SetParam(MS5837_PARAM_WATER_DENSITY, MS5837_PARAM_TYPE_F32, density_le, 4U) == MS5837_OK);
  CHECK(Ms5837_GetParam(MS5837_PARAM_WATER_DENSITY, &type, &length, value) == MS5837_OK);
  CHECK(type == MS5837_PARAM_TYPE_F32);
  CHECK(length == 4U);
  memcpy(&bits, value, sizeof(bits));
  memcpy(&read_back, &bits, sizeof(read_back));
  CHECK(nearly_equal(read_back, 1100.0f, 0.0001f));

  /* u8 型号。 */
  value[0] = MS5837_MODEL_02BA;
  CHECK(Ms5837_SetParam(MS5837_PARAM_DEPTH_MODEL, MS5837_PARAM_TYPE_U8, value, 1U) == MS5837_OK);
  CHECK(Ms5837_GetModel() == MS5837_MODEL_02BA);
  CHECK(Ms5837_GetParam(MS5837_PARAM_DEPTH_MODEL, &type, &length, value) == MS5837_OK);
  CHECK(type == MS5837_PARAM_TYPE_U8);
  CHECK(length == 1U);
  CHECK(value[0] == MS5837_MODEL_02BA);

  /* OSR 与 FILTER_K 往返。 */
  {
    uint16_t osr_le = 2048U;
    float k = 0.25f;
    CHECK(Ms5837_SetParam(MS5837_PARAM_DEPTH_OSR, MS5837_PARAM_TYPE_U16, &osr_le, 2U) == MS5837_OK);
    CHECK(Ms5837_GetOsr() == MS5837_OSR_2048);
    CHECK(Ms5837_SetParam(MS5837_PARAM_FILTER_K, MS5837_PARAM_TYPE_F32, &k, 4U) == MS5837_OK);
    CHECK(nearly_equal(Ms5837_GetFilterK(), 0.25f, 1e-6f));
  }

  /* RESTORE_DEFAULTS 语义。 */
  CHECK(Ms5837_RestoreDefaults() == MS5837_OK);
  CHECK(Ms5837_GetModel() == MS5837_MODEL_UNKNOWN);
  CHECK(Ms5837_GetOsr() == MS5837_OSR_DEFAULT);
  CHECK(Ms5837_GetOutputRateHz() == MS5837_OUTPUT_RATE_HZ_DEFAULT);
  CHECK(nearly_equal(Ms5837_GetWaterDensity(), MS5837_WATER_DENSITY_DEFAULT, 0.001f));
  CHECK(nearly_equal(Ms5837_GetFilterK(), MS5837_FILTER_K_DEFAULT, 0.0001f));
  CHECK(Ms5837_IsZeroValid() == 0U);
  TEST_END();
}

/* ---------------------------------------------------------------- 13. 离线 */
static void Test_OfflineDevice(void)
{
  Ms5837Stats_t stats;
  Ms5837Stats_t stats_before;
  uint32_t errors_before;

  TEST_BEGIN("OfflineDevice");

  sim_reset();
  test_tick = 0U;
  I2c_Init(&fake_handle);
  Ms5837_RestoreDefaults();
  CHECK(Ms5837_GetStats(&stats_before) == MS5837_OK);

  sim.online = 0U; /* 从设备不回 ACK。 */
  CHECK(Ms5837_Init() == MS5837_ERR_IO);
  CHECK((Ms5837_GetStatus() & MS5837_STATUS_ONLINE) == 0U);
  CHECK((Ms5837_GetStatus() & MS5837_STATUS_PROM_VALID) == 0U);

  {
    uint32_t step;
    for (step = 0U; step < 300U; step++)
    {
      Ms5837_Process();
      test_tick++;
    }
  }
  CHECK(Ms5837_GetStats(&stats) == MS5837_OK);
  CHECK(stats.errors > stats_before.errors);
  CHECK(stats.last_error == (uint32_t)MS5837_ERR_IO);
  CHECK(stats.crc_errors == stats_before.crc_errors);
  /* 离线期间不得产生任何样本，也不得伪造数据。 */
  CHECK(Ms5837_GetSample(0) == MS5837_ERR_PARAM);
  {
    Ms5837Sample_t sample;
    CHECK(Ms5837_GetSample(&sample) == MS5837_ERR_NO_SAMPLE);
  }
  CHECK(Ms5837_GetSampleAgeMs() == 0xFFFFFFFFUL);

  /* 恢复在线后自动重试成功并产出样本。 */
  errors_before = stats.errors;
  sim.online = 1U;
  CHECK(run_until(pred_prom_valid, 400U) != 0U);
  Ms5837_ClearNewSampleFlag();
  CHECK(run_until(pred_new_sample, 400U) != 0U);
  CHECK(Ms5837_GetStats(&stats) == MS5837_OK);
  CHECK(stats.errors == errors_before);
  CHECK(stats.good_frames > 0U);
  CHECK((Ms5837_GetStatus() & MS5837_STATUS_ONLINE) != 0U);
  TEST_END();
}

/* ---------------------------------------------------------------- 14. 超时 */
static void Test_BusTimeout(void)
{
  Ms5837Stats_t stats;
  uint32_t before;
  uint32_t elapsed;
  uint32_t step;
  uint8_t blocked_extra = 0U;

  TEST_BEGIN("BusTimeout");

  harness_start(MS5837_MODEL_30BA, 5000000U, 6800000U);
  CHECK(Ms5837_GetStats(&stats) == MS5837_OK);

  /* 单次事务超时必须落在短超时区间（默认 5 ms，远小于转换等待）。 */
  sim.stall = 1U;
  before = test_tick;
  CHECK(Ms5837_Init() == MS5837_ERR_TIMEOUT);
  elapsed = test_tick - before;
  CHECK(elapsed == (uint32_t)I2C_BUS_DEFAULT_TIMEOUT_MS);
  CHECK(sim.last_timeout_ms == (uint32_t)I2C_BUS_DEFAULT_TIMEOUT_MS);

  CHECK(Ms5837_GetStats(&stats) == MS5837_OK);
  CHECK(stats.last_error == (uint32_t)MS5837_ERR_TIMEOUT);
  CHECK(stats.bus_timeouts > 0U);

  /* 卡死期间主循环继续推进，但每个 Process 调用最多消耗一个超时窗口。 */
  {
    uint32_t max_elapsed = 0U;
    for (step = 0U; step < 50U; step++)
    {
      before = test_tick;
      Ms5837_Process();
      elapsed = test_tick - before;
      if (elapsed > max_elapsed)
      {
        max_elapsed = elapsed;
      }
      if (elapsed > (uint32_t)I2C_BUS_DEFAULT_TIMEOUT_MS)
      {
        blocked_extra = 1U;
      }
      test_tick++;
    }
    CHECK(blocked_extra == 0U);
    CHECK(max_elapsed <= (uint32_t)I2C_BUS_DEFAULT_TIMEOUT_MS);
  }
  {
    Ms5837Sample_t sample;
    CHECK(Ms5837_GetSample(&sample) == MS5837_ERR_NO_SAMPLE);
  }

  /* 总线恢复后必须能重新初始化并产出样本。 */
  sim.stall = 0U;
  CHECK(Ms5837_Init() == MS5837_OK);
  CHECK(run_until(pred_prom_valid, 400U) != 0U);
  Ms5837_ClearNewSampleFlag();
  CHECK(run_until(pred_new_sample, 400U) != 0U);
  CHECK(Ms5837_HasNewSample() != 0U);
  TEST_END();
}

/* ---------------------------------------------------------------- 15. 非阻塞 */
static void Test_NonBlockingConversionWait(void)
{
  uint32_t calls = 0U;
  uint32_t blocked = 0U;
  uint32_t tx_before;
  uint32_t rx_before;
  uint32_t max_transactions = 0U;

  TEST_BEGIN("NonBlockingConversionWait");

  harness_start(MS5837_MODEL_30BA, 5000000U, 6981794U);
  CHECK(Ms5837_GetOsr() == MS5837_OSR_4096);

  Ms5837_ClearNewSampleFlag();
  while ((Ms5837_HasNewSample() == 0U) && (calls < 500U))
  {
    uint32_t tick_before = test_tick;
    tx_before = sim.tx_count;
    rx_before = sim.rx_count;
    Ms5837_Process();
    if (test_tick != tick_before)
    {
      blocked = 1U; /* 在线设备不会主动推进时间：说明驱动内部阻塞等待了。 */
    }
    if ((sim.tx_count - tx_before) + (sim.rx_count - rx_before) > max_transactions)
    {
      max_transactions = (sim.tx_count - tx_before) + (sim.rx_count - rx_before);
    }
    test_tick++;
    calls++;
  }

  CHECK(Ms5837_HasNewSample() != 0U);
  CHECK(blocked == 0U);
  /* OSR4096 两个方向各 9.04 ms，各含 1 ms 余量 → 22 ms：样本至少跨 22 次主循环调用。 */
  CHECK(calls >= 22U);
  /* 单次 Process 最多：读一次 ADC（读事务）+ 发一条 D2 转换命令。 */
  CHECK(max_transactions <= 3U);
  /* 驱动从未提前读取 ADC（否则转换时间检查形同虚设）。 */
  CHECK(sim.early_reads == 0U);
  /* 实际等待不得少于 数据手册最大值 + 1 ms 余量。 */
  CHECK(sim.min_conv_wait_valid != 0U);
  CHECK(sim.min_conv_wait_ms >= Ms5837_MaxConversionTimeMs(MS5837_MODEL_30BA, MS5837_OSR_4096) +
                                    MS5837_CONVERSION_MARGIN_MS);
  TEST_END();
}

/* ---------------------------------------------------------------- 15b. 周期时序（25 Hz） */
/*
 * 关键回归：转换等待绝不能额外累加到采样周期上。
 * 这里让一次 HAL 事务真实占用 1 ms，逐毫秒推进主循环，检查
 *   - 相邻样本时间戳间隔 == 1000/25 = 40 ms（不是 40 + 20 ms 的 ~16 Hz）；
 *   - 每次“命令 → 读 ADC”的等待 ≥ 数据手册最大转换时间 + 1 ms；
 *   - 没有提前读 ADC、没有超期连发追赶。
 */
static void Test_CycleTimingAt25Hz(void)
{
  Ms5837Sample_t sample;
  uint32_t stamps[6];
  uint8_t count = 0U;
  uint8_t i;
  uint32_t conversions;

  TEST_BEGIN("CycleTimingAt25Hz");

  harness_start(MS5837_MODEL_30BA, 5000000U, 6981794U);
  CHECK(Ms5837_GetOsr() == MS5837_OSR_4096);
  CHECK(Ms5837_GetOutputRateHz() == 25U);

  sim.tx_time_ms = 1U; /* 之后每次 I2C 事务都推进 1 ms，模拟真实总线耗时。 */
  conversions = sim.tx_count + sim.rx_count;
  Ms5837_ClearNewSampleFlag();

  while ((count < 6U) && (test_tick < 20000U))
  {
    uint32_t tick_before = test_tick;
    Ms5837_Process();
    /* 允许事务自身推进 tick；单次调用最多 3 次事务（读 ADC + 发下一条转换命令）。 */
    if ((test_tick - tick_before) > 3U * sim.tx_time_ms)
    {
      CHECK(0);
    }
    if (Ms5837_HasNewSample() != 0U)
    {
      Ms5837_ClearNewSampleFlag();
      CHECK(Ms5837_GetSample(&sample) == MS5837_OK);
      stamps[count] = sample.timestamp_ms;
      count++;
    }
    test_tick++;
  }

  CHECK(count == 6U);
  for (i = 1U; i < count; i++)
  {
    uint32_t delta = stamps[i] - stamps[i - 1U];
    /* 25 Hz → 40 ms。允许 ±1 ms 的毫秒对齐误差，但绝不允许 55 ms 这种“周期 + 转换”叠加。 */
    CHECK(delta <= 41U);
    CHECK(delta >= 39U);
  }
  /* 实测采样率应接近 25 Hz，明显高于修复前的 ~16 Hz。 */
  CHECK((stamps[count - 1U] - stamps[0]) <= (uint32_t)((count - 1U) * 41U));
  CHECK(sim.early_reads == 0U);
  CHECK(sim.min_conv_wait_ms >= Ms5837_MaxConversionTimeMs(MS5837_MODEL_30BA, MS5837_OSR_4096) +
                                    MS5837_CONVERSION_MARGIN_MS);
  CHECK((sim.tx_count + sim.rx_count) > conversions);
  printf("        样本时间戳: %u %u %u %u %u %u ms；间隔 %u/%u/%u/%u/%u ms；"
         "最小转换等待 %u ms（要求 >= %u ms）\n",
         stamps[0], stamps[1], stamps[2], stamps[3], stamps[4], stamps[5],
         stamps[1] - stamps[0], stamps[2] - stamps[1], stamps[3] - stamps[2],
         stamps[4] - stamps[3], stamps[5] - stamps[4],
         sim.min_conv_wait_ms,
         Ms5837_MaxConversionTimeMs(MS5837_MODEL_30BA, MS5837_OSR_4096) + MS5837_CONVERSION_MARGIN_MS);
  TEST_END();
}

/* ---------------------------------------------------------------- 16. CRC 失败 */
static void Test_PromCrcFailure(void)
{
  Ms5837Stats_t stats;
  Ms5837Stats_t stats_before;
  Ms5837Sample_t sample;

  TEST_BEGIN("PromCrcFailure");

  sim_reset();
  sim.model = MS5837_MODEL_30BA;
  test_tick = 0U;
  I2c_Init(&fake_handle);
  Ms5837_RestoreDefaults();
  CHECK(Ms5837_SetModel(MS5837_MODEL_30BA) == MS5837_OK);
  CHECK(Ms5837_GetStats(&stats_before) == MS5837_OK);
  sim.prom[3] ^= 0x0004U; /* 破坏 C3，使 CRC 不符。 */
  CHECK(Ms5837_Init() == MS5837_OK);

  {
    uint32_t step;
    for (step = 0U; step < 300U; step++)
    {
      Ms5837_Process();
      test_tick++;
    }
  }
  CHECK((Ms5837_GetStatus() & MS5837_STATUS_PROM_VALID) == 0U);
  CHECK((Ms5837_GetStatus() & MS5837_STATUS_RAW_VALID) == 0U);
  CHECK(Ms5837_GetStats(&stats) == MS5837_OK);
  CHECK(stats.crc_errors > stats_before.crc_errors);
  CHECK(stats.last_error == (uint32_t)MS5837_ERR_CRC);
  CHECK(stats.good_frames == stats_before.good_frames);
  CHECK(Ms5837_GetSample(&sample) == MS5837_ERR_NO_SAMPLE);
  {
    uint16_t prom[MS5837_PROM_WORDS];
    CHECK(Ms5837_GetProm(prom) == MS5837_ERR_NOT_READY);
  }

  /* 修复 PROM 后自动恢复。 */
  sim.prom[3] ^= 0x0004U;
  CHECK(run_until(pred_prom_valid, 400U) != 0U);
  Ms5837_ClearNewSampleFlag();
  CHECK(run_until(pred_new_sample, 400U) != 0U);
  CHECK(Ms5837_GetStats(&stats) == MS5837_OK);
  CHECK(stats.good_frames == stats_before.good_frames + 1U);
  TEST_END();
}

/* ---------------------------------------------------------------- 17. 样本元数据与新鲜度 */
static void Test_SampleMetadataAndFreshness(void)
{
  Ms5837Sample_t first;
  Ms5837Sample_t second;
  Ms5837Stats_t stats;
  uint32_t step;

  TEST_BEGIN("SampleMetadataAndFreshness");

  harness_start(MS5837_MODEL_30BA, 5000000U, 6981794U);
  CHECK(Ms5837_GetSample(&first) == MS5837_OK);
  CHECK(first.sequence >= 1U);
  CHECK((first.status & MS5837_STATUS_PRESSURE_VALID) != 0U);
  CHECK(Ms5837_HasNewSample() != 0U);
  CHECK(Ms5837_GetSampleAgeMs() == (uint32_t)(test_tick - first.timestamp_ms));

  Ms5837_ClearNewSampleFlag();
  CHECK(Ms5837_HasNewSample() == 0U);
  CHECK(run_until(pred_new_sample, 400U) != 0U);
  CHECK(Ms5837_GetSample(&second) == MS5837_OK);
  CHECK(second.sequence == first.sequence + 1U);
  CHECK(second.timestamp_ms > first.timestamp_ms);

  /* 总线故障后，旧样本不得继续报告“压力有效”。 */
  sim.online = 0U;
  for (step = 0U; step < 400U; step++)
  {
    Ms5837_Process();
    test_tick++;
  }
  CHECK((Ms5837_GetStatus() & MS5837_STATUS_ONLINE) == 0U);
  CHECK(Ms5837_GetSample(&second) == MS5837_OK);
  CHECK((second.status & MS5837_STATUS_PRESSURE_VALID) == 0U);
  CHECK((second.status & MS5837_STATUS_DEPTH_VALID) == 0U);
  /* PROM 属于身份信息，仍然保留。 */
  CHECK((second.status & MS5837_STATUS_PROM_VALID) != 0U);
  CHECK(Ms5837_GetStats(&stats) == MS5837_OK);
  CHECK(stats.errors > 0U);

  sim.online = 1U;
  Ms5837_ClearNewSampleFlag();
  CHECK(run_until(pred_new_sample, 600U) != 0U);
  CHECK((Ms5837_GetStatus() & MS5837_STATUS_ONLINE) != 0U);
  CHECK((Ms5837_GetStatus() & MS5837_STATUS_PRESSURE_VALID) != 0U);
  TEST_END();
}

/* ---------------------------------------------------------------- 17b. 假 PROM（CRC 碰巧通过） */
/*
 * 全 0 / 全 0xFFFF 的 C1~C6 在 I2C 卡死或空器件时很常见，而且可能碰上 CRC 自洽。
 * 仿真里用 sim_set_prom_from_coefficients() 生成“CRC 正确”的 PROM，
 * 因此这里的拒绝只可能来自内容可信度检查，而不是 CRC 检查。
 */
static void Test_PromContentSanity(void)
{
  Ms5837Stats_t stats;
  Ms5837Stats_t stats_before;
  Ms5837Sample_t sample;
  uint16_t bogus[MS5837_PROM_WORDS];
  uint8_t index;

  TEST_BEGIN("PromContentSanity");

  /* 全 0 的 C1~C6。 */
  memset(bogus, 0, sizeof(bogus));
  sim_reset();
  sim.model = MS5837_MODEL_30BA;
  test_tick = 0U;
  I2c_Init(&fake_handle);
  Ms5837_RestoreDefaults();
  CHECK(Ms5837_SetModel(MS5837_MODEL_30BA) == MS5837_OK);
  sim_set_prom_from_coefficients(bogus);
  /* 先确认这个假 PROM 的 CRC 是自洽的（否则测不到内容检查这一层）。 */
  CHECK((sim.prom[0] >> 12) == Ms5837_Crc4(sim.prom));
  CHECK(Ms5837_GetStats(&stats_before) == MS5837_OK);
  CHECK(Ms5837_Init() == MS5837_OK);
  {
    uint32_t step;
    for (step = 0U; step < 300U; step++)
    {
      Ms5837_Process();
      test_tick++;
    }
  }
  CHECK((Ms5837_GetStatus() & MS5837_STATUS_PROM_VALID) == 0U);
  CHECK((Ms5837_GetStatus() & MS5837_STATUS_RAW_VALID) == 0U);
  CHECK(Ms5837_GetSample(&sample) == MS5837_ERR_NO_SAMPLE);
  CHECK(Ms5837_GetStats(&stats) == MS5837_OK);
  CHECK(stats.errors > stats_before.errors);
  CHECK(stats.crc_errors > stats_before.crc_errors);

  /* 全 0xFFFF 的 C1~C6。 */
  for (index = 1U; index <= 6U; index++)
  {
    bogus[index] = 0xFFFFU;
  }
  bogus[0] = 0U;
  bogus[7] = 0U;
  sim_set_prom_from_coefficients(bogus);
  CHECK((sim.prom[0] >> 12) == Ms5837_Crc4(sim.prom));
  CHECK(Ms5837_GetStats(&stats_before) == MS5837_OK);
  CHECK(Ms5837_Init() == MS5837_OK);
  {
    uint32_t step;
    for (step = 0U; step < 300U; step++)
    {
      Ms5837_Process();
      test_tick++;
    }
  }
  CHECK((Ms5837_GetStatus() & MS5837_STATUS_PROM_VALID) == 0U);
  CHECK(Ms5837_GetSample(&sample) == MS5837_ERR_NO_SAMPLE);
  CHECK(Ms5837_GetStats(&stats) == MS5837_OK);
  CHECK(stats.crc_errors > stats_before.crc_errors);

  /* 换回正常 PROM 后必须能上线出帧。 */
  sim_set_prom_from_coefficients(prom_cross);
  CHECK(Ms5837_Init() == MS5837_OK);
  CHECK(run_until(pred_prom_valid, 400U) != 0U);
  Ms5837_ClearNewSampleFlag();
  CHECK(run_until(pred_new_sample, 400U) != 0U);
  CHECK((Ms5837_GetStatus() & MS5837_STATUS_PRESSURE_VALID) != 0U);
  TEST_END();
}

/* ---------------------------------------------------------------- 17c. 无效转换码字 */
static void Test_InvalidConversionRejected(void)
{
  Ms5837Sample_t sample;
  Ms5837Stats_t stats;
  Ms5837Stats_t stats_before;
  uint32_t seq_before;

  TEST_BEGIN("InvalidConversionRejected");

  harness_start(MS5837_MODEL_30BA, 5000000U, 6981794U);
  CHECK(Ms5837_GetSample(&sample) == MS5837_OK);
  seq_before = sample.sequence;
  CHECK(Ms5837_GetStats(&stats_before) == MS5837_OK);

  /* D1 全 0。 */
  sim.d1_value = 0U;
  Ms5837_ClearNewSampleFlag();
  {
    uint32_t step;
    for (step = 0U; step < 400U; step++)
    {
      Ms5837_Process();
      test_tick++;
    }
  }
  CHECK(Ms5837_HasNewSample() == 0U); /* 不得发布新样本 */
  CHECK((Ms5837_GetStatus() & MS5837_STATUS_RAW_VALID) == 0U);
  CHECK((Ms5837_GetStatus() & MS5837_STATUS_PRESSURE_VALID) == 0U);
  CHECK((Ms5837_GetStatus() & MS5837_STATUS_DEPTH_VALID) == 0U);
  CHECK(Ms5837_GetSample(&sample) == MS5837_OK);
  CHECK(sample.sequence == seq_before); /* 序号没有推进 */
  CHECK(MS5837_IS_NAN(sample.pressure_pa));
  CHECK(Ms5837_GetStats(&stats) == MS5837_OK);
  CHECK(stats.last_error == (uint32_t)MS5837_ERR_NOT_READY);
  CHECK(stats.good_frames == stats_before.good_frames);
  /* 总线本身没坏：ONLINE 保留。 */
  CHECK((Ms5837_GetStatus() & MS5837_STATUS_ONLINE) != 0U);

  /* D1 全 1（0xFFFFFF）。 */
  sim.d1_value = 0xFFFFFFU;
  CHECK(Ms5837_GetStats(&stats_before) == MS5837_OK);
  {
    uint32_t step;
    for (step = 0U; step < 400U; step++)
    {
      Ms5837_Process();
      test_tick++;
    }
  }
  CHECK(Ms5837_HasNewSample() == 0U);
  CHECK(Ms5837_GetSample(&sample) == MS5837_OK);
  CHECK(sample.sequence == seq_before);
  CHECK(MS5837_IS_NAN(sample.pressure_pa));
  CHECK(Ms5837_GetStats(&stats) == MS5837_OK);
  CHECK(stats.good_frames == stats_before.good_frames);

  /* D1 恢复正常，但 D2 全 1。 */
  sim.d1_value = 5000000U;
  sim.d2_value = 0xFFFFFFU;
  CHECK(Ms5837_GetStats(&stats_before) == MS5837_OK);
  {
    uint32_t step;
    for (step = 0U; step < 400U; step++)
    {
      Ms5837_Process();
      test_tick++;
    }
  }
  CHECK(Ms5837_HasNewSample() == 0U);
  CHECK(Ms5837_GetStats(&stats) == MS5837_OK);
  CHECK(stats.good_frames == stats_before.good_frames);

  /* 恢复正常：必须重新出有效样本。 */
  sim.d2_value = 6981794U;
  Ms5837_ClearNewSampleFlag();
  CHECK(run_until(pred_new_sample, 600U) != 0U);
  CHECK(Ms5837_GetSample(&sample) == MS5837_OK);
  CHECK(sample.sequence == seq_before + 1U);
  CHECK((sample.status & MS5837_STATUS_PRESSURE_VALID) != 0U);
  CHECK(!MS5837_IS_NAN(sample.pressure_pa));
  TEST_END();
}

/* ---------------------------------------------------------------- 17d. 纯函数输入校验 */
static void Test_CompensateInputValidation(void)
{
  int64_t pressure_raw = 12345;
  int32_t temperature = 6789;

  TEST_BEGIN("CompensateInputValidation");

  /* 超过 24 位的输入必须拒绝，且不得写输出。 */
  CHECK(Ms5837_Compensate(MS5837_MODEL_30BA, prom_cross, 0x01000000U, 6800000U,
                          &pressure_raw, &temperature) == 0U);
  CHECK(pressure_raw == 12345);
  CHECK(temperature == 6789);
  CHECK(Ms5837_Compensate(MS5837_MODEL_30BA, prom_cross, 5000000U, 0xFFFFFFFFU,
                          &pressure_raw, &temperature) == 0U);
  CHECK(pressure_raw == 12345);
  CHECK(temperature == 6789);
  CHECK(Ms5837_Compensate(MS5837_MODEL_02BA, prom_02ba_example, 0xFFFFFF00U, 8077636U,
                          &pressure_raw, &temperature) == 0U);

  /* 0 / 0xFFFFFF 是无效转换码字，同样拒绝。 */
  CHECK(Ms5837_Compensate(MS5837_MODEL_30BA, prom_cross, 0U, 6800000U,
                          &pressure_raw, &temperature) == 0U);
  CHECK(Ms5837_Compensate(MS5837_MODEL_30BA, prom_cross, 5000000U, 0xFFFFFFU,
                          &pressure_raw, &temperature) == 0U);
  CHECK(Ms5837_Compensate(MS5837_MODEL_30BA, prom_cross, 0xFFFFFFU, 6800000U,
                          &pressure_raw, &temperature) == 0U);
  CHECK(pressure_raw == 12345);
  CHECK(temperature == 6789);

  /* 边界之内的正常输入照常工作（24 位最大值附近仍可补偿）。 */
  CHECK(Ms5837_Compensate(MS5837_MODEL_30BA, prom_cross, 0xFFFFFEU, 0xFFFFFEU,
                          &pressure_raw, &temperature) == 1U);
  /* 允许输出指针为空（只算不取）。 */
  CHECK(Ms5837_Compensate(MS5837_MODEL_30BA, prom_cross, 5000000U, 6800000U, 0, 0) == 1U);
  TEST_END();
}

/* ---------------------------------------------------------------- 18. I2C 层 */
static void Test_I2cBusLayer(void)
{
  uint8_t command = 0xA0U;
  uint8_t buffer[3];
  uint8_t big[I2C_BUS_MAX_TRANSFER + 1U];

  TEST_BEGIN("I2cBusLayer");

  memset(big, 0, sizeof(big));
  sim_reset();

  /* 未绑定句柄时必须拒绝事务。 */
  I2c_Init(0);
  CHECK(I2c_IsReady() == 0U);
  CHECK(I2c_Write(&command, 1U, 0U) == I2C_BUS_NOT_READY);
  CHECK(I2c_Read(buffer, 2U, 0U) == I2C_BUS_NOT_READY);

  I2c_Init(&fake_handle);
  CHECK(I2c_IsReady() == 1U);

  /* 默认地址就是 0x76（HAL 0xEC）。 */
  CHECK(I2c_GetDevice() == I2C_BUS_MS5837_ADDRESS7);
  CHECK(I2C_BUS_MS5837_ADDRESS7 == 0x76U);

  /* 7 位地址范围检查。 */
  CHECK(I2c_SetDevice(0x07U) == I2C_BUS_PARAM);
  CHECK(I2c_SetDevice(0x78U) == I2C_BUS_PARAM);
  CHECK(I2c_SetDevice(0x08U) == I2C_BUS_OK);
  CHECK(I2c_SetDevice(0x77U) == I2C_BUS_OK);
  CHECK(I2c_SetDevice(0x76U) == I2C_BUS_OK);

  /* 参数检查。 */
  CHECK(I2c_Write(0, 1U, 5U) == I2C_BUS_PARAM);
  CHECK(I2c_Write(&command, 0U, 5U) == I2C_BUS_PARAM);
  CHECK(I2c_Write(big, (uint16_t)sizeof(big), 5U) == I2C_BUS_PARAM);
  CHECK(I2c_Read(0, 2U, 5U) == I2C_BUS_PARAM);

  /* 超时收敛：0 → 默认值，过大 → 上限。 */
  CHECK(I2c_WriteRead(&command, 1U, buffer, 2U, 0U) == I2C_BUS_OK);
  CHECK(sim.last_timeout_ms == (uint32_t)I2C_BUS_DEFAULT_TIMEOUT_MS);
  CHECK(buffer[0] == (uint8_t)(sim.prom[0] >> 8));
  CHECK(buffer[1] == (uint8_t)(sim.prom[0] & 0xFFU));

  sim.prom[0] = 0x1234U; /* 命令 0xA0 对应 PROM 字 0。 */
  CHECK(I2c_WriteRead(&command, 1U, buffer, 2U, 1000U) == I2C_BUS_OK);
  CHECK(sim.last_timeout_ms == (uint32_t)I2C_BUS_MAX_TIMEOUT_MS);
  CHECK(buffer[0] == 0x12U);
  CHECK(buffer[1] == 0x34U);

  /* BUSY / 错误地址 / 离线映射。 */
  sim.busy = 1U;
  CHECK(I2c_Write(&command, 1U, 5U) == I2C_BUS_BUSY);
  sim.busy = 0U;
  CHECK(I2c_SetDevice(0x30U) == I2C_BUS_OK);
  CHECK(I2c_Write(&command, 1U, 5U) == I2C_BUS_ERROR);
  CHECK(I2c_SetDevice(I2C_BUS_MS5837_ADDRESS7) == I2C_BUS_OK);
  sim.online = 0U;
  CHECK(I2c_Write(&command, 1U, 5U) == I2C_BUS_ERROR);
  sim.online = 1U;
  sim.stall = 1U;
  {
    uint32_t before = test_tick;
    CHECK(I2c_Write(&command, 1U, 5U) == I2C_BUS_TIMEOUT);
    CHECK((test_tick - before) == (uint32_t)I2C_BUS_DEFAULT_TIMEOUT_MS);
  }
  sim.stall = 0U;
  TEST_END();
}

/* ---------------------------------------------------------------- main */
int main(void)
{
  Test_Crc4();
  Test_OfficialExample30ba();
  Test_OfficialExample02ba();
  Test_TemperatureBranches();
  Test_ModelDifference();
  Test_CommandSequence();
  Test_UnknownModelDefault();
  Test_NoZeroMeansNoDepth();
  Test_ZeroAndDepth();
  Test_FilterK();
  Test_ParameterValidation();
  Test_ParameterWireEncoding();
  Test_OfflineDevice();
  Test_BusTimeout();
  Test_NonBlockingConversionWait();
  Test_CycleTimingAt25Hz();
  Test_PromCrcFailure();
  Test_PromContentSanity();
  Test_InvalidConversionRejected();
  Test_CompensateInputValidation();
  Test_SampleMetadataAndFreshness();
  Test_I2cBusLayer();

  if (test_failures != 0U)
  {
    printf("ms5837_host_test: FAIL (%u 项断言失败)\n", test_failures);
    return 1;
  }
  printf("ms5837_host_test: PASS\n");
  return 0;
}
