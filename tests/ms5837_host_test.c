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

/* 由独立 Python 实现生成的参考向量表（含生成说明）。 */
#include "ms5837_reference_vectors.h"

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
  uint32_t busy_until; /* 芯片“忙”到什么时候（手册第11页：转换完成前一直 busy）。 */
  uint32_t new_conversion_while_busy; /* 转换未完成又收到新 D1/D2 的次数（必须为 0）。 */
  uint32_t read_while_busy; /* 转换未完成就读 ADC 的次数（手册：结果会是 0）。 */
  uint8_t last_d1_osr; /* 最近一次 D1 命令的 OSR 下标；0xFF 表示还没有。 */
  uint8_t d1_valid; /* 是否有一个“待配对”的 D1。 */
  uint32_t mismatched_pair; /* D1 与随后 D2 的 OSR 不一致的次数（半周期泄漏，必须为 0）。 */
  uint32_t min_conv_wait_ms; /* 观测到的最小“命令→读 ADC”间隔。 */
  uint8_t min_conv_wait_valid; /* 是否已记录过转换等待。 */
  uint32_t tx_time_ms; /* 每次 HAL I2C 事务占用总线的毫秒数（模拟真实事务耗时）。 */
  uint32_t d1_value; /* D1 目标值。 */
  uint32_t d2_value; /* D2 目标值。 */
  uint32_t early_reads; /* 提前读 ADC 的次数（期望 0）。 */
  uint32_t reset_count; /* 收到复位命令的次数。 */
  uint32_t tx_count; /* 写事务次数。 */
  uint32_t rx_count; /* 读事务次数。 */
  uint32_t abort_count; /* 软件超时后请求 Abort 的次数。 */
  uint32_t last_timeout_ms; /* 最近一次 HAL 调用收到的超时参数。 */
  uint8_t read_buffer[4]; /* 待读数据。 */
  uint8_t read_length; /* 待读字节数。 */
  uint8_t read_pending; /* 是否有待读数据。 */
  uint8_t command_log[SIM_COMMAND_LOG]; /* 命令字节日志。 */
  uint32_t command_count; /* 命令字节数量。 */
} SimDevice_t;

static SimDevice_t sim;

/* HAL 句柄替身：IT 替身与后面的测试都用这一个。 */
static I2C_HandleTypeDef fake_handle;

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
    sim.busy_until = test_tick; /* 复位后不再忙（手册未给精确等待时间，项目另有保守延时）。 */
    sim.reset_count++;
    return HAL_OK;
  }
  if (((command & 0xF0U) == 0x40U) || ((command & 0xF0U) == 0x50U))
  {
    /*
     * 手册第 11 页：转换进行中再发转换命令会得到错误结果（“stays busy until conversion is done”）。
     * 这里把“旧转换未完成就重发 D1/D2”记成违规，便于回归证明驱动没有提前重发。
     */
    if ((int32_t)(test_tick - sim.busy_until) < 0)
    {
      sim.new_conversion_while_busy++;
    }
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
    sim.busy_until = sim.conversion_ready_ms;
    sim.last_d1_osr = index;
    sim.d1_valid = 1U;
    return HAL_OK;
  }
  if ((command & 0xF0U) == 0x50U) /* D2 转换 */
  {
    uint8_t index = (uint8_t)((command >> 1) & 0x07U);
    if (index > 5U)
    {
      return HAL_ERROR;
    }
    /*
     * 配对不变量：补偿用的 D1/D2 必须来自同一 OSR 设置。
     * 若中途改过 OSR（半周期没被丢弃），这里的下标就会不一致。
     */
    if ((sim.d1_valid != 0U) && (sim.last_d1_osr != index))
    {
      sim.mismatched_pair++;
    }
    sim.conversion = 2U;
    sim.conversion_command_ms = test_tick;
    sim.conversion_ready_ms = test_tick + sim_conversion_ms(sim.model, index);
    sim.busy_until = sim.conversion_ready_ms;
    sim.d1_valid = 0U;
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
    uint8_t busy = ((int32_t)(test_tick - sim.busy_until) < 0) ? 1U : 0U;
    if ((sim.conversion == 0U) || (busy != 0U))
    {
      sim.early_reads++; /* 转换未完成就被读取：驱动等待时间不足。 */
    }
    if (busy != 0U)
    {
      /* 手册第 11 页：转换中读 ADC 结果会是 0，且最终结果错误。 */
      sim.read_while_busy++;
      value = 0U;
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

/* ------------------------------------------------ 中断模式（IT）替身
 *
 * 真实硬件上 _IT 只启动传输，完成由中断回调通知；这里两种模式都支持：
 *  - sim_it_defer = 0（默认）：_IT 调用内部立即完成并触发回调，
 *    这样既有测试在“总线比 1 ms 快得多”的前提下行为完全不变；
 *  - sim_it_defer = 1：_IT 只登记请求并返回，由测试调用 sim_it_pump()
 *    （模拟中断到来）后才触发回调，用于验证驱动确实不阻塞、由回调推进。
 * 卡死（sim.stall）时 _IT 不做任何计时推进：CPU 是空闲的，超时由 I2c_Process 负责。
 */
static uint8_t sim_it_defer = 0U;
static uint8_t sim_it_pending = 0U; /* 0 无 / 1 Tx / 2 Rx / 3 Abort */
static uint32_t sim_it_error = 0U;

void sim_it_pump(void)
{
  uint8_t pending = sim_it_pending;
  sim_it_pending = 0U;
  switch (pending)
  {
    case 1U:
      HAL_I2C_MasterTxCpltCallback(&fake_handle);
      break;
    case 2U:
      HAL_I2C_MasterRxCpltCallback(&fake_handle);
      break;
    case 3U:
      HAL_I2C_AbortCpltCallback(&fake_handle);
      break;
    default:
      break;
  }
}

HAL_StatusTypeDef HAL_I2C_Master_Transmit_IT(I2C_HandleTypeDef *hi2c,
                                             uint16_t dev_address,
                                             uint8_t *data,
                                             uint16_t size)
{
  HAL_StatusTypeDef status;

  sim.last_timeout_ms = I2C_BUS_DEFAULT_TIMEOUT_MS;
  if (sim.stall != 0U)
  {
    if (sim_it_defer != 0U)
    {
      return HAL_OK; /* 真异步：CPU 空闲、时间由测试推进，超时交给 I2c_Process + Abort。 */
    }
    /* 同步兼容路径：假 HAL 不会自己走时间，用“推进到超时 + 报错”复现阻塞版本行为。 */
    test_tick += I2C_BUS_DEFAULT_TIMEOUT_MS;
    sim_it_error = 0x00000020U; /* HAL_I2C_ERROR_TIMEOUT */
    HAL_I2C_ErrorCallback(hi2c);
    return HAL_OK;
  }
  if (sim.online == 0U)
  {
    sim_it_error = 0x00000004U; /* HAL_I2C_ERROR_AF：地址未被应答 */
    HAL_I2C_ErrorCallback(hi2c);
    return HAL_OK;
  }
  status = HAL_I2C_Master_Transmit(hi2c, dev_address, data, size, I2C_BUS_DEFAULT_TIMEOUT_MS);
  if (status == HAL_BUSY)
  {
    return HAL_BUSY;
  }
  if (status != HAL_OK)
  {
    sim_it_error = 0U;
    HAL_I2C_ErrorCallback(hi2c);
    return HAL_OK;
  }
  if (sim_it_defer != 0U)
  {
    sim_it_pending = 1U;
  }
  else
  {
    HAL_I2C_MasterTxCpltCallback(hi2c);
  }
  return HAL_OK;
}

HAL_StatusTypeDef HAL_I2C_Master_Receive_IT(I2C_HandleTypeDef *hi2c,
                                            uint16_t dev_address,
                                            uint8_t *data,
                                            uint16_t size)
{
  HAL_StatusTypeDef status;

  if (sim.stall != 0U)
  {
    if (sim_it_defer != 0U)
    {
      return HAL_OK;
    }
    test_tick += I2C_BUS_DEFAULT_TIMEOUT_MS;
    sim_it_error = 0x00000020U;
    HAL_I2C_ErrorCallback(hi2c);
    return HAL_OK;
  }
  if (sim.online == 0U)
  {
    sim_it_error = 0x00000004U;
    HAL_I2C_ErrorCallback(hi2c);
    return HAL_OK;
  }
  status = HAL_I2C_Master_Receive(hi2c, dev_address, data, size, I2C_BUS_DEFAULT_TIMEOUT_MS);
  if (status == HAL_BUSY)
  {
    return HAL_BUSY;
  }
  if (status != HAL_OK)
  {
    sim_it_error = 0U;
    HAL_I2C_ErrorCallback(hi2c);
    return HAL_OK;
  }
  if (sim_it_defer != 0U)
  {
    sim_it_pending = 2U;
  }
  else
  {
    HAL_I2C_MasterRxCpltCallback(hi2c);
  }
  return HAL_OK;
}

HAL_StatusTypeDef HAL_I2C_Master_Abort_IT(I2C_HandleTypeDef *hi2c, uint16_t dev_address)
{
  (void)dev_address;
  sim.abort_count++;
  if (sim_it_defer != 0U)
  {
    sim_it_pending = 3U;
  }
  else
  {
    HAL_I2C_AbortCpltCallback(hi2c);
  }
  return HAL_OK;
}

uint32_t HAL_I2C_GetError(I2C_HandleTypeDef *hi2c)
{
  (void)hi2c;
  return sim_it_error;
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

/* 与 CHECK 相同，但用于返回 uint8_t 的场景辅助函数。 */
#define BUSY_CHECK(cond)                                                   \
  do                                                                       \
  {                                                                        \
    if (!(cond))                                                           \
    {                                                                      \
      printf("[ FAIL ] %s: %s:%d: %s\n", test_current, __FILE__, __LINE__, \
             #cond);                                                       \
      test_failures++;                                                     \
      return 0U;                                                           \
    }                                                                      \
  } while (0)

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
 * 涉及 Ms5837_Zero() 的用例必须使用“物理合理”的原始值：
 * ZERO 复用参数 0103 的 10000~200000 Pa 校验，因此这里给出约 1 atm 的向量。
 */
#define TEST_D2_25C          6981794U /* prom_cross 下约 25 °C。 */
#define TEST_D1_SURFACE      3914352U /* prom_cross + 30BA → 101330 Pa。 */
#define TEST_D1_DIVE         4014352U /* prom_cross + 30BA → ≈176 kPa（≈7.4 m）。 */
#define TEST_D1_SURFACE_02BA 6413836U /* prom_cross + 02BA → 101325 Pa。 */

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

  /* 计算 CRC 不得破坏调用方保存的原始 PROM（含字 0 高 4 位的原 CRC 与第 8 个软件辅助字）。 */
  {
    uint16_t keep[MS5837_PROM_WORDS];
    uint16_t copy[MS5837_PROM_WORDS];
    for (index = 0U; index < MS5837_PROM_WORDS; index++)
    {
      keep[index] = prom_30ba_example[index];
    }
    keep[0] = (uint16_t)(0xA000U | (keep[0] & 0x0FFFU)); /* 高 4 位是芯片给的 CRC */
    keep[7] = 0x1234U;
    memcpy(copy, keep, sizeof(copy));
    (void)Ms5837_Crc4(keep);
    CHECK(memcmp(copy, keep, sizeof(copy)) == 0);
  }
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

  /* 第一帧：约 1 atm（101330 Pa，落在参数 0103 的 10000~200000 Pa 内）作为水面零点。 */
  sim_reset();
  sim.model = MS5837_MODEL_30BA;
  sim.d1_value = TEST_D1_SURFACE;
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
  pa1 = ref_pressure_pa(MS5837_MODEL_30BA, prom_cross, TEST_D1_SURFACE, d2_25c);
  CHECK(nearly_equal(sample.pressure_pa, pa1, 1.0f));

  /* 显式采集零点（AA5B ZERO_DEPTH 语义）。 */
  CHECK(Ms5837_Zero() == MS5837_OK);
  CHECK(Ms5837_IsZeroValid() == 1U);
  CHECK(Ms5837_GetSample(&sample) == MS5837_OK);
  CHECK(nearly_equal(sample.depth_raw_m, 0.0f, 1e-6f));
  CHECK(nearly_equal(sample.depth_filtered_m, 0.0f, 1e-6f));
  CHECK(nearly_equal(sample.surface_pressure_pa, pa1, 1.0f));

  /* 第二帧：加大 D1 模拟下潜（≈176 kPa ≈ 7.4 m），深度必须与 (P-P0)/(rho*g) 一致。 */
  sim.d1_value = TEST_D1_DIVE;
  Ms5837_ClearNewSampleFlag();
  CHECK(run_until(pred_new_sample, 400U) != 0U);
  CHECK(Ms5837_GetSample(&sample) == MS5837_OK);
  pa2 = ref_pressure_pa(MS5837_MODEL_30BA, prom_cross, TEST_D1_DIVE, d2_25c);
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

  harness_start(MS5837_MODEL_30BA, TEST_D1_SURFACE, TEST_D2_25C);
  CHECK(Ms5837_Zero() == MS5837_OK);
  CHECK(Ms5837_SetFilterK(0.9f) == MS5837_OK);
  /* 改变零点后滤波重新起步：下一帧的滤波值必须等于原始值。 */
  Ms5837_ClearNewSampleFlag();
  CHECK(run_until(pred_new_sample, 400U) != 0U);
  CHECK(Ms5837_GetSample(&sample) == MS5837_OK);
  first = sample.depth_raw_m;
  CHECK(nearly_equal(sample.depth_filtered_m, first, 1e-6f));

  /* 阶跃后滤波值必须落在旧值与新值之间。 */
  sim.d1_value = TEST_D1_DIVE;
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

/* ---------------------------------------------------------------- 17e. 型号切换清零点 */
static void Test_ModelChangeClearsZero(void)
{
  Ms5837Sample_t sample;
  uint8_t type = 0U;
  uint8_t length = 0U;
  uint8_t value[4];
  float pa = 0.0f;

  TEST_BEGIN("ModelChangeClearsZero");

  harness_start(MS5837_MODEL_30BA, TEST_D1_SURFACE, TEST_D2_25C);
  CHECK(Ms5837_Zero() == MS5837_OK);
  CHECK(Ms5837_IsZeroValid() == 1U);
  CHECK((Ms5837_GetStatus() & MS5837_STATUS_ZERO_VALID) != 0U);
  CHECK(Ms5837_GetSample(&sample) == MS5837_OK);
  CHECK((sample.status & MS5837_STATUS_DEPTH_VALID) != 0U);

  /* 同一型号重复设置：幂等，零点保留。 */
  CHECK(Ms5837_SetModel(MS5837_MODEL_30BA) == MS5837_OK);
  CHECK(Ms5837_IsZeroValid() == 1U);

  /* 换成另一种型号：旧 P0 可能是错误型号算的，必须清除零点与滤波。 */
  CHECK(Ms5837_SetModel(MS5837_MODEL_02BA) == MS5837_OK);
  CHECK(Ms5837_IsZeroValid() == 0U);
  CHECK((Ms5837_GetStatus() & MS5837_STATUS_ZERO_VALID) == 0U);
  CHECK(Ms5837_GetSurfacePressurePa(&pa) == MS5837_ERR_NO_ZERO);
  CHECK(Ms5837_GetParam(MS5837_PARAM_SURFACE_PRESSURE, &type, &length, value) == MS5837_ERR_NO_ZERO);
  CHECK(Ms5837_GetSample(&sample) == MS5837_OK);
  CHECK(MS5837_IS_NAN(sample.depth_raw_m));
  CHECK(MS5837_IS_NAN(sample.depth_filtered_m));
  CHECK(MS5837_IS_NAN(sample.surface_pressure_pa));
  CHECK((sample.status & MS5837_STATUS_DEPTH_VALID) == 0U);

  /* 重新建立零点（此时型号是 02BA，用 02BA 的合理原始值）后再换成 unknown：同样必须清除。 */
  sim.d1_value = TEST_D1_SURFACE_02BA;
  Ms5837_ClearNewSampleFlag();
  CHECK(run_until(pred_new_sample, 400U) != 0U);
  CHECK(Ms5837_Zero() == MS5837_OK);
  CHECK(Ms5837_IsZeroValid() == 1U);
  CHECK(Ms5837_SetModel(MS5837_MODEL_UNKNOWN) == MS5837_OK);
  CHECK(Ms5837_IsZeroValid() == 0U);
  CHECK(Ms5837_GetSample(&sample) == MS5837_OK);
  CHECK(MS5837_IS_NAN(sample.depth_raw_m));
  TEST_END();
}

/* ---------------------------------------------------------------- 17f. Zero 的范围校验 */
static void Test_ZeroValidatesRange(void)
{
  Ms5837Sample_t sample;
  float pa = 0.0f;
  uint8_t type = 0U;
  uint8_t length = 0U;
  uint8_t value[4];
  float p0 = 0.0f;

  TEST_BEGIN("ZeroValidatesRange");

  /* 压力低于 10000 Pa：ZERO 必须失败且不建立零点（不能留下 GET_PARAMETER 认为越界的 P0）。 */
  sim_reset();
  sim.model = MS5837_MODEL_30BA;
  sim.d1_value = 50000U; /* 远低于量程下沿 → 补偿压力为负。 */
  sim.d2_value = 6981794U;
  test_tick = 0U;
  I2c_Init(&fake_handle);
  Ms5837_RestoreDefaults();
  CHECK(Ms5837_SetModel(MS5837_MODEL_30BA) == MS5837_OK);
  CHECK(Ms5837_Init() == MS5837_OK);
  CHECK(run_until(pred_prom_valid, 200U) != 0U);
  Ms5837_ClearNewSampleFlag();
  CHECK(run_until(pred_new_sample, 400U) != 0U);
  CHECK(Ms5837_GetSample(&sample) == MS5837_OK);
  CHECK(sample.pressure_pa < MS5837_SURFACE_PRESSURE_MIN);
  CHECK(Ms5837_Zero() == MS5837_ERR_PARAM);
  CHECK(Ms5837_IsZeroValid() == 0U);
  CHECK(Ms5837_GetParam(MS5837_PARAM_SURFACE_PRESSURE, &type, &length, value) == MS5837_ERR_NO_ZERO);
  CHECK(MS5837_IS_NAN(sample.depth_raw_m));

  /* 压力高于 200000 Pa 同样被拒。 */
  sim.d1_value = 0xFFFFFEU;
  Ms5837_ClearNewSampleFlag();
  CHECK(run_until(pred_new_sample, 400U) != 0U);
  CHECK(Ms5837_GetSample(&sample) == MS5837_OK);
  CHECK(sample.pressure_pa > MS5837_SURFACE_PRESSURE_MAX);
  CHECK(Ms5837_Zero() == MS5837_ERR_PARAM);
  CHECK(Ms5837_IsZeroValid() == 0U);

  /* 回到量程内：ZERO 成功，并且读回来的 P0 与样本压力一致、不再越界。 */
  sim.d1_value = TEST_D1_SURFACE;
  Ms5837_ClearNewSampleFlag();
  CHECK(run_until(pred_new_sample, 400U) != 0U);
  CHECK(Ms5837_GetSample(&sample) == MS5837_OK);
  CHECK(Ms5837_Zero() == MS5837_OK);
  CHECK(Ms5837_IsZeroValid() == 1U);
  CHECK(Ms5837_GetSurfacePressurePa(&pa) == MS5837_OK);
  CHECK(nearly_equal(pa, sample.pressure_pa, 1e-3f));
  CHECK(Ms5837_GetParam(MS5837_PARAM_SURFACE_PRESSURE, &type, &length, value) == MS5837_OK);
  CHECK(type == MS5837_PARAM_TYPE_F32);
  CHECK(length == 4U);
  memcpy(&p0, value, sizeof(p0));
  CHECK(nearly_equal(p0, sample.pressure_pa, 1e-3f));
  CHECK((p0 >= MS5837_SURFACE_PRESSURE_MIN) && (p0 <= MS5837_SURFACE_PRESSURE_MAX));
  TEST_END();
}

/* ---------------------------------------------------------------- 17g. 转换中改配置 */
/*
 * 半周期丢弃：转换等待是按旧配置算的，改 OSR/型号后必须重新走完整周期，
 * 绝不能沿用旧 deadline（否则 256 → 8192 会在 2 ms 后就去读需要 18.08 ms 的转换）。
 */
static void Test_ConfigChangeDuringConversion(void)
{
  uint32_t index;
  uint32_t first_d1 = 0U;
  uint8_t saw_aborted_d2 = 0U;

  TEST_BEGIN("ConfigChangeDuringConversion");

  /* 起点 OSR256（等待只有 1+1 ms），保证能在 D1/D2 转换途中改配置。 */
  sim_reset();
  sim.model = MS5837_MODEL_30BA;
  sim.d1_value = 5000000U;
  sim.d2_value = 6981794U;
  test_tick = 0U;
  I2c_Init(&fake_handle);
  Ms5837_RestoreDefaults();
  CHECK(Ms5837_SetOutputRateHz(20U) == MS5837_OK);
  CHECK(Ms5837_SetModel(MS5837_MODEL_30BA) == MS5837_OK);
  CHECK(Ms5837_SetOsr(MS5837_OSR_256) == MS5837_OK);
  CHECK(Ms5837_Init() == MS5837_OK);
  CHECK(run_until(pred_prom_valid, 200U) != 0U);

  /* 启动一个周期，停在 D1 转换中（此时已经下发 0x40）。 */
  Ms5837_ClearNewSampleFlag();
  while ((ms5837.state != MS5837_STATE_CONVERT_D1) && (test_tick < 500U))
  {
    Ms5837_Process();
    test_tick++;
  }
  CHECK(ms5837.state == MS5837_STATE_CONVERT_D1);
  sim.command_count = 0U;
  sim.min_conv_wait_valid = 0U;
  sim.min_conv_wait_ms = 0U;

  /* 转换途中把 OSR 从 256 改成 8192：必须丢弃半周期，用新 OSR 重新开始。 */
  CHECK(Ms5837_SetOsr(MS5837_OSR_8192) == MS5837_OK);
  Ms5837_ClearNewSampleFlag();
  CHECK(run_until(pred_new_sample, 2000U) != 0U);

  /* 改动后第一条转换命令必须是新的 D1（0x4A），说明旧半周期被丢弃、重新从 D1 开始。 */
  for (index = 0U; index < sim.command_count; index++)
  {
    if ((sim.command_log[index] & 0xF0U) == 0x40U)
    {
      first_d1 = sim.command_log[index];
      break;
    }
    if ((sim.command_log[index] & 0xF0U) == 0x50U)
    {
      saw_aborted_d2 = 1U; /* 改动后还先出现 D2 说明旧半周期没被丢弃。 */
    }
  }
  CHECK(first_d1 == 0x4AU);
  CHECK(saw_aborted_d2 == 0U);
  /* 新的等待必须按 OSR8192 计算：>= 18.08 → 19 ms + 1 ms 余量。 */
  CHECK(sim.min_conv_wait_valid != 0U);
  CHECK(sim.min_conv_wait_ms >= Ms5837_MaxConversionTimeMs(MS5837_MODEL_30BA, MS5837_OSR_8192) +
                                    MS5837_CONVERSION_MARGIN_MS);
  CHECK(sim.early_reads == 0U);

  /* 型号切换同理：02BA/30BA 的最大转换时间不同，切换后必须重新走完整周期。 */
  CHECK(Ms5837_SetOsr(MS5837_OSR_4096) == MS5837_OK);
  Ms5837_ClearNewSampleFlag();
  while ((ms5837.state != MS5837_STATE_CONVERT_D1) && (test_tick < 20000U))
  {
    Ms5837_Process();
    test_tick++;
  }
  CHECK(ms5837.state == MS5837_STATE_CONVERT_D1);
  sim.command_count = 0U;
  sim.min_conv_wait_valid = 0U;
  sim.min_conv_wait_ms = 0U;
  CHECK(Ms5837_SetModel(MS5837_MODEL_02BA) == MS5837_OK);
  /* 手册第11页：芯片仍 busy，因此只进入 DISCARD_WAIT，绝不能立刻重发转换。 */
  CHECK(ms5837.state == MS5837_STATE_DISCARD_WAIT);
  CHECK((int32_t)(test_tick - sim.busy_until) < 0);
  CHECK(sim.new_conversion_while_busy == 0U);
  Ms5837_ClearNewSampleFlag();
  CHECK(run_until(pred_new_sample, 2000U) != 0U);
  first_d1 = 0U;
  for (index = 0U; index < sim.command_count; index++)
  {
    if ((sim.command_log[index] & 0xF0U) == 0x40U)
    {
      first_d1 = sim.command_log[index];
      break;
    }
  }
  CHECK(first_d1 == 0x48U); /* OSR4096 的 D1 */
  CHECK(sim.min_conv_wait_ms >= Ms5837_MaxConversionTimeMs(MS5837_MODEL_02BA, MS5837_OSR_4096) +
                                    MS5837_CONVERSION_MARGIN_MS);
  CHECK(sim.early_reads == 0U);
  TEST_END();
}

/* ---------------------------------------------------------------- 17h. 芯片 busy 期间不得重发转换（第11页） */
/*
 * 手册第 11 页：
 *  - “If the ADC read command is sent during conversion the result will be 0, the conversion will not stop
 *     and the final result will be wrong.”
 *  - “Conversion sequence sent during the already started conversion process will yield incorrect result.”
 *  - “When command is sent to the system it stays busy until conversion is done.”
 * 因此“丢弃软件半周期”不能等于“取消芯片正在进行的转换”：改 OSR/型号后必须先等旧转换安全结束，
 * 再发新的 D1/D2。仿真里 busy_until 表示芯片忙到什么时候，任何提前重发/提前读都会被计数。
 */
static uint8_t busy_expected_d1_cmd(uint16_t osr)
{
  switch (osr)
  {
    case MS5837_OSR_256: return 0x40U;
    case MS5837_OSR_512: return 0x42U;
    case MS5837_OSR_1024: return 0x44U;
    case MS5837_OSR_2048: return 0x46U;
    case MS5837_OSR_4096: return 0x48U;
    case MS5837_OSR_8192: return 0x4AU;
    default: return 0xFFU;
  }
}

static uint8_t busy_scenario(const char *name,
                             uint8_t start_model, uint16_t start_osr,
                             uint8_t change_model, uint16_t change_osr,
                             uint8_t in_d2_phase, uint8_t change_count,
                             uint32_t tick_base)
{
  uint32_t index;
  uint32_t first_d1 = 0U;
  uint8_t step;

  sim_reset();
  sim.model = MS5837_MODEL_30BA; /* 物理器件取较慢的一侧，busy 判定更严格。 */
  sim.d1_value = 5000000U;
  sim.d2_value = 6981794U;
  test_tick = tick_base;
  I2c_Init(&fake_handle);
  Ms5837_RestoreDefaults();
  BUSY_CHECK(Ms5837_SetOutputRateHz(10U) == MS5837_OK); /* 10 Hz 周期足够容纳所有 OSR。 */
  BUSY_CHECK(Ms5837_SetModel(start_model) == MS5837_OK);
  BUSY_CHECK(Ms5837_SetOsr(start_osr) == MS5837_OK);
  BUSY_CHECK(Ms5837_Init() == MS5837_OK);
  BUSY_CHECK(run_until(pred_prom_valid, 400U) != 0U);

  /* 走到目标转换阶段（D1 或 D2）。在推进 tick 之前就跳出，保证芯片此刻真的还在忙。 */
  Ms5837_ClearNewSampleFlag();
  step = 0U;
  while (step < 200U)
  {
    Ms5837_Process();
    if ((in_d2_phase == 0U) && (ms5837.state == MS5837_STATE_CONVERT_D1))
    {
      break;
    }
    if ((in_d2_phase != 0U) && (ms5837.state == MS5837_STATE_CONVERT_D2))
    {
      break;
    }
    test_tick++;
    step++;
  }
  BUSY_CHECK(ms5837.state == ((in_d2_phase != 0U) ? MS5837_STATE_CONVERT_D2
                                                  : MS5837_STATE_CONVERT_D1));
  /* 此刻芯片必须真的还在忙，否则这个用例没有验证价值。 */
  BUSY_CHECK((int32_t)(test_tick - sim.busy_until) < 0);

  /* 转换途中改配置（可连续多次）。 */
  sim.command_count = 0U;
  for (index = 0U; index < change_count; index++)
  {
    BUSY_CHECK(Ms5837_SetModel(change_model) == MS5837_OK);
    BUSY_CHECK(Ms5837_SetOsr(change_osr) == MS5837_OK);
  }
  /* 芯片仍忙：不得出现新的转换命令，也不得读 ADC。 */
  BUSY_CHECK((int32_t)(test_tick - sim.busy_until) < 0);
  BUSY_CHECK(sim.new_conversion_while_busy == 0U);
  BUSY_CHECK(sim.read_while_busy == 0U);
  BUSY_CHECK(sim.early_reads == 0U);

  /* 旧转换结束后必须能正常出新样本，且第一条转换命令用的是新配置。 */
  Ms5837_ClearNewSampleFlag();
  BUSY_CHECK(run_until(pred_new_sample, 4000U) != 0U);
  BUSY_CHECK(sim.new_conversion_while_busy == 0U);
  BUSY_CHECK(sim.read_while_busy == 0U);
  BUSY_CHECK(sim.early_reads == 0U);
  for (index = 0U; index < sim.command_count; index++)
  {
    if ((sim.command_log[index] & 0xF0U) == 0x40U)
    {
      first_d1 = sim.command_log[index];
      break;
    }
  }
  BUSY_CHECK(first_d1 == busy_expected_d1_cmd(change_osr));
  printf("        %-16s busy重发=%u busy读=%u 提前读=%u 新D1=0x%02X\n", name,
         sim.new_conversion_while_busy, sim.read_while_busy, sim.early_reads, first_d1);
  return 1U;
}

static void Test_BusyDeviceNoEarlyRestart(void)
{
  TEST_BEGIN("BusyDeviceNoEarlyRestart");

  if (busy_scenario("D1-8192to256", MS5837_MODEL_30BA, MS5837_OSR_8192,
                    MS5837_MODEL_30BA, MS5837_OSR_256, 0U, 1U, 0U) == 0U) { return; }
  if (busy_scenario("D2-8192to256", MS5837_MODEL_30BA, MS5837_OSR_8192,
                    MS5837_MODEL_30BA, MS5837_OSR_256, 1U, 1U, 0U) == 0U) { return; }
  if (busy_scenario("D1-256to8192", MS5837_MODEL_30BA, MS5837_OSR_256,
                    MS5837_MODEL_30BA, MS5837_OSR_8192, 0U, 1U, 0U) == 0U) { return; }
  if (busy_scenario("D2-256to8192", MS5837_MODEL_30BA, MS5837_OSR_256,
                    MS5837_MODEL_30BA, MS5837_OSR_8192, 1U, 1U, 0U) == 0U) { return; }
  if (busy_scenario("D1-30BAto02BA", MS5837_MODEL_30BA, MS5837_OSR_4096,
                    MS5837_MODEL_02BA, MS5837_OSR_4096, 0U, 1U, 0U) == 0U) { return; }
  if (busy_scenario("D1-02BAto30BA", MS5837_MODEL_02BA, MS5837_OSR_4096,
                    MS5837_MODEL_30BA, MS5837_OSR_4096, 0U, 1U, 0U) == 0U) { return; }
  if (busy_scenario("D1-change-x4", MS5837_MODEL_30BA, MS5837_OSR_8192,
                    MS5837_MODEL_30BA, MS5837_OSR_256, 0U, 4U, 0U) == 0U) { return; }
  if (busy_scenario("D1-wrap-8192to256", MS5837_MODEL_30BA, MS5837_OSR_8192,
                    MS5837_MODEL_30BA, MS5837_OSR_256, 0U, 1U, 0xFFFFFF00UL) == 0U) { return; }
  TEST_END();
}

/* ---------------------------------------------------------------- 17i. 02BA 手册第7页算例逐项 */
/*
 * 手册第 7 页算例逐项（C1..C6 = 46372/43981/29059/27842/31553/28165，D1=6465444，D2=8077636）：
 *   dT = 68；TEMP = 2000（20.00 °C）；OFF = 5764707214；SENS = 3039050829；P = 110002（1100.02 mbar）。
 * 这里用测试自己的一阶表达式复算，逐项对照手册打印值；再断言驱动输出与之一致。
 * 除法用有符号右移（= 向负无穷取整），与手册流程一致。
 */
static void Test_OfficialExample02baTerms(void)
{
  const uint16_t *c = prom_02ba_example;
  int64_t dt;
  int64_t temp;
  int64_t off;
  int64_t sens;
  int64_t pressure;
  int64_t driver_raw = 0;
  int32_t driver_temp = 0;

  TEST_BEGIN("OfficialExample02baTerms");

  dt = (int64_t)8077636 - ((int64_t)c[5] << 8);
  temp = 2000 + ((dt * (int64_t)c[6]) >> 23);
  off = ((int64_t)c[2] << 17) + (((int64_t)c[4] * dt) >> 6);
  sens = ((int64_t)c[1] << 16) + (((int64_t)c[3] * dt) >> 7);
  pressure = ((((int64_t)6465444 * sens) >> 21) - off) >> 15;

  CHECK(dt == 68);
  CHECK(temp == 2000); /* 20.00 °C */
  CHECK(off == 5764707214LL);
  CHECK(sens == 3039050829LL);
  CHECK(pressure == 110002LL); /* 0.01 mbar 整数 → 1100.02 mbar */
  /* 0.01 mbar/LSB ⇒ 1 LSB = 1 Pa，与手册第7页换算一致。 */
  CHECK(nearly_equal((float)pressure, 110002.0f, 0.5f));

  CHECK(Ms5837_Compensate(MS5837_MODEL_02BA, c, 6465444U, 8077636U, &driver_raw,
                          &driver_temp) == 1U);
  CHECK(driver_temp == 2000);
  CHECK(driver_raw == pressure);
  TEST_END();
}

/* ---------------------------------------------------------------- 17j. 20 °C 边界与冷热切换 */
static void Test_TemperatureBranchBoundary(void)
{
  const uint16_t *c = prom_30ba_example;
  int64_t raw_cold = 0;
  int64_t raw_exact = 0;
  int64_t ref_cold = 0;
  int64_t ref_exact = 0;
  int32_t t_cold = 0;
  int32_t t_exact = 0;
  uint32_t d2_cold; /* TEMP = 1999，落在低温分支 */
  uint32_t d2_exact; /* TEMP = 2000，不进低温分支 */
  uint32_t d2_warm;
  uint32_t d2_very_cold;
  int64_t ref_warm = 0;
  int32_t t_warm = 0;
  Ms5837Sample_t sample;

  TEST_BEGIN("TemperatureBranchBoundary");

  d2_exact = (uint32_t)(c[5] * 256U); /* dT = 0 → TEMP = 2000 */
  d2_cold = (uint32_t)(c[5] * 256U - 1U); /* dT = -1 → TEMP = 1999（向下取整） */
  d2_warm = (uint32_t)(c[5] * 256U + 300000U);
  d2_very_cold = (uint32_t)(c[5] * 256U - 500000U);

  /* 边界两侧：TEMP=1999 走低温修正，TEMP=2000 不修正。 */
  CHECK(Ms5837_Compensate(MS5837_MODEL_30BA, c, 4958179U, d2_cold, &raw_cold, &t_cold) == 1U);
  CHECK(Ms5837_Compensate(MS5837_MODEL_30BA, c, 4958179U, d2_exact, &raw_exact, &t_exact) == 1U);
  CHECK(t_cold == 1999);
  CHECK(t_exact == 2000);
  ref_compensate(MS5837_MODEL_30BA, c, 4958179U, d2_cold, &ref_cold, &t_cold);
  ref_compensate(MS5837_MODEL_30BA, c, 4958179U, d2_exact, &ref_exact, &t_exact);
  CHECK(raw_cold == ref_cold);
  CHECK(raw_exact == ref_exact);
  /* TEMP=2000 时 Ti/OFFi/SENSi 必须为 0：一阶压力与二阶压力完全一致。 */
  {
    int64_t dt = (int64_t)d2_exact - ((int64_t)c[5] << 8);
    int64_t off = ((int64_t)c[2] << 16) + (((int64_t)c[4] * dt) >> 7);
    int64_t sens = ((int64_t)c[1] << 15) + (((int64_t)c[3] * dt) >> 8);
    int64_t first_order = ((((int64_t)4958179 * sens) >> 21) - off) >> 13;
    CHECK(dt == 0);
    CHECK(raw_exact == first_order);
  }

  /* 02BA 非低温分支同样不得有任何修正（手册第8页：没有高温项）。 */
  {
    const uint16_t *c2 = prom_02ba_example;
    int64_t dt = 300000;
    uint32_t d2 = (uint32_t)((int64_t)c2[5] * 256 + dt);
    int64_t off = ((int64_t)c2[2] << 17) + (((int64_t)c2[4] * dt) >> 6);
    int64_t sens = ((int64_t)c2[1] << 16) + (((int64_t)c2[3] * dt) >> 7);
    int64_t first_order = ((((int64_t)6465444 * sens) >> 21) - off) >> 15;
    int64_t raw2 = 0;
    int32_t t2 = 0;
    CHECK(Ms5837_Compensate(MS5837_MODEL_02BA, c2, 6465444U, d2, &raw2, &t2) == 1U);
    CHECK(t2 > 2000);
    CHECK(raw2 == first_order);
  }

  /* 冷 → 热连续两帧：第二帧不得残留第一帧的 Ti/OFFi/SENSi。 */
  harness_start(MS5837_MODEL_30BA, 4958179U, d2_very_cold);
  CHECK(Ms5837_GetSample(&sample) == MS5837_OK);
  /* dT=-500000 → 一阶 TEMP=441（4.41 °C），Ti=87，二阶 TEMP2=354。 */
  CHECK(sample.temperature_centi_c == 354);
  CHECK(sample.pressure_raw == 89502);
  sim.d2_value = d2_warm;
  Ms5837_ClearNewSampleFlag();
  CHECK(run_until(pred_new_sample, 400U) != 0U);
  CHECK(Ms5837_GetSample(&sample) == MS5837_OK);
  /* 热的这一帧必须等于独立复算结果（Ti/OFFi/SENSi 用本帧的值，不能残留低温值）。 */
  ref_compensate(MS5837_MODEL_30BA, prom_cross, 4958179U, d2_warm, &ref_warm, &t_warm);
  CHECK(sample.temperature_centi_c == t_warm);
  CHECK(sample.pressure_raw == ref_warm);
  /* 反向再切回低温一帧，同样必须与独立复算一致。 */
  sim.d2_value = d2_very_cold;
  Ms5837_ClearNewSampleFlag();
  CHECK(run_until(pred_new_sample, 400U) != 0U);
  CHECK(Ms5837_GetSample(&sample) == MS5837_OK);
  ref_compensate(MS5837_MODEL_30BA, prom_cross, 4958179U, d2_very_cold, &ref_warm, &t_warm);
  CHECK(sample.temperature_centi_c == t_warm);
  CHECK(sample.pressure_raw == ref_warm);
  TEST_END();
}

/* ---------------------------------------------------------------- 17k. 单位、负水深与无效零点 */
static void Test_UnitsAndNegativeDepth(void)
{
  Ms5837Sample_t sample;
  float depth_below_zero;
  float depth_above_zero;

  TEST_BEGIN("UnitsAndNegativeDepth");

  /* 30BA：整数压力单位 0.1 mbar ⇒ pressure_pa = raw × 10。 */
  harness_start(MS5837_MODEL_30BA, TEST_D1_SURFACE, TEST_D2_25C);
  CHECK(Ms5837_GetSample(&sample) == MS5837_OK);
  CHECK(nearly_equal(sample.pressure_pa, (float)sample.pressure_raw * 10.0f, 0.5f));
  /* 24 位 ADC 按 MSB first 组装：仿真给 0x123456，样本里必须原样出现。 */
  sim.d1_value = 0x123456U;
  Ms5837_ClearNewSampleFlag();
  CHECK(run_until(pred_new_sample, 400U) != 0U);
  CHECK(Ms5837_GetSample(&sample) == MS5837_OK);
  CHECK(sample.d1 == 0x123456U);

  /* 02BA：整数压力单位 0.01 mbar ⇒ 1 LSB = 1 Pa，mbar = raw / 100。 */
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
  CHECK(nearly_equal(sample.pressure_pa, (float)sample.pressure_raw, 0.5f));
  CHECK(nearly_equal(sample.pressure_pa / 100.0f, 1100.02f, 0.01f)); /* 手册第7页的 mbar 值 */

  /* 负水深不得被夹到 0：先在水面建立零点，再给一个低于 P0 的压力。 */
  sim_reset();
  sim.model = MS5837_MODEL_30BA;
  sim.d1_value = TEST_D1_SURFACE;
  sim.d2_value = TEST_D2_25C;
  test_tick = 0U;
  I2c_Init(&fake_handle);
  Ms5837_RestoreDefaults();
  CHECK(Ms5837_SetModel(MS5837_MODEL_30BA) == MS5837_OK);
  CHECK(Ms5837_Init() == MS5837_OK);
  CHECK(run_until(pred_prom_valid, 200U) != 0U);
  Ms5837_ClearNewSampleFlag();
  CHECK(run_until(pred_new_sample, 400U) != 0U);
  CHECK(Ms5837_Zero() == MS5837_OK);

  sim.d1_value = TEST_D1_DIVE; /* 高于 P0 → 正深度 */
  Ms5837_ClearNewSampleFlag();
  CHECK(run_until(pred_new_sample, 400U) != 0U);
  CHECK(Ms5837_GetSample(&sample) == MS5837_OK);
  depth_above_zero = sample.depth_raw_m;
  CHECK(depth_above_zero > 0.0f);

  sim.d1_value = TEST_D1_SURFACE - 20000U; /* 低于 P0 → 负深度，必须原样保留 */
  Ms5837_ClearNewSampleFlag();
  CHECK(run_until(pred_new_sample, 400U) != 0U);
  CHECK(Ms5837_GetSample(&sample) == MS5837_OK);
  depth_below_zero = sample.depth_raw_m;
  CHECK(depth_below_zero < 0.0f);
  CHECK((sample.status & MS5837_STATUS_DEPTH_VALID) != 0U); /* 压力有效且零点有效 */
  CHECK(depth_below_zero < -0.1f);
  CHECK(depth_below_zero > -2.0f);
  /* 与 (P-P0)/(rho*g) 一致，没有被夹紧成 0。 */
  CHECK(nearly_equal(depth_below_zero,
                     (sample.pressure_pa - sample.surface_pressure_pa) /
                         (MS5837_WATER_DENSITY_DEFAULT * MS5837_GRAVITY),
                     1e-4f));
  TEST_END();
}

/* ---------------------------------------------------------------- 17h2. RESTORE_DEFAULTS 也要丢弃半周期 */
static void Test_RestoreDefaultsDuringConversion(void)
{
  Ms5837Sample_t sample;

  TEST_BEGIN("RestoreDefaultsDuringConversion");

  sim_reset();
  sim.model = MS5837_MODEL_30BA;
  sim.d1_value = 5000000U;
  sim.d2_value = 6981794U;
  test_tick = 0U;
  I2c_Init(&fake_handle);
  Ms5837_RestoreDefaults();
  CHECK(Ms5837_SetOutputRateHz(10U) == MS5837_OK);
  CHECK(Ms5837_SetModel(MS5837_MODEL_30BA) == MS5837_OK);
  CHECK(Ms5837_SetOsr(MS5837_OSR_8192) == MS5837_OK);
  CHECK(Ms5837_Init() == MS5837_OK);
  CHECK(run_until(pred_prom_valid, 400U) != 0U);

  /* 走到 D1 转换中（推进 tick 之前跳出，保证芯片仍忙）。 */
  while ((ms5837.state != MS5837_STATE_CONVERT_D1) && (test_tick < 500U))
  {
    Ms5837_Process();
    if (ms5837.state != MS5837_STATE_CONVERT_D1)
    {
      test_tick++;
    }
  }
  CHECK(ms5837.state == MS5837_STATE_CONVERT_D1);
  CHECK((int32_t)(test_tick - sim.busy_until) < 0);

  /* RestoreDefaults 同样会改 OSR/型号，必须走同一条“延迟丢弃”路径。 */
  CHECK(Ms5837_RestoreDefaults() == MS5837_OK);
  CHECK(ms5837.state == MS5837_STATE_DISCARD_WAIT);
  CHECK(sim.new_conversion_while_busy == 0U);

  Ms5837_ClearNewSampleFlag();
  CHECK(run_until(pred_new_sample, 4000U) != 0U);
  CHECK(sim.new_conversion_while_busy == 0U);
  CHECK(sim.read_while_busy == 0U);
  CHECK(sim.early_reads == 0U);
  /* D1/D2 必须配对（同一 OSR），不能把改配置前后的两半拼起来补偿。 */
  CHECK(sim.mismatched_pair == 0U);
  /* 默认型号是 unknown：样本只能给原始值，不得声称压力有效。 */
  CHECK(Ms5837_GetSample(&sample) == MS5837_OK);
  CHECK(MS5837_IS_NAN(sample.pressure_pa));
  CHECK((sample.status & MS5837_STATUS_PRESSURE_VALID) == 0U);
  CHECK(Ms5837_GetOsr() == MS5837_OSR_DEFAULT);

  /* 同一条自查：RESTORE_DEFAULTS 后不得再保留“已知型号算出的有效压力/温度”标志与数值。 */
  sim_reset();
  sim.model = MS5837_MODEL_30BA;
  sim.d1_value = TEST_D1_SURFACE;
  sim.d2_value = TEST_D2_25C;
  test_tick = 0U;
  I2c_Init(&fake_handle);
  Ms5837_RestoreDefaults();
  CHECK(Ms5837_SetModel(MS5837_MODEL_30BA) == MS5837_OK);
  CHECK(Ms5837_Init() == MS5837_OK);
  CHECK(run_until(pred_prom_valid, 400U) != 0U);
  Ms5837_ClearNewSampleFlag();
  CHECK(run_until(pred_new_sample, 400U) != 0U);
  CHECK(Ms5837_GetSample(&sample) == MS5837_OK);
  CHECK((Ms5837_GetStatus() & MS5837_STATUS_PRESSURE_VALID) != 0U);
  CHECK(!MS5837_IS_NAN(sample.pressure_pa));

  CHECK(Ms5837_RestoreDefaults() == MS5837_OK);
  CHECK(Ms5837_GetModel() == MS5837_MODEL_UNKNOWN);
  CHECK((Ms5837_GetStatus() & MS5837_STATUS_PRESSURE_VALID) == 0U);
  CHECK((Ms5837_GetStatus() & MS5837_STATUS_TEMPERATURE_VALID) == 0U);
  CHECK((Ms5837_GetStatus() & MS5837_STATUS_DEPTH_VALID) == 0U);
  CHECK(Ms5837_GetSample(&sample) == MS5837_OK);
  CHECK(MS5837_IS_NAN(sample.pressure_pa));
  CHECK(MS5837_IS_NAN(sample.temperature_c));
  TEST_END();
}

/* ---------------------------------------------------------------- 17l. 随机化不变量（自查用） */
/*
 * 用确定性 LCG 随机组合：推进主循环、改 OSR/型号/采样率、RESTORE_DEFAULTS、改零点/滤波、
 * 随机离线/卡死。每一步都检查全局不变量，用来抓手写用例没覆盖到的组合缺陷。
 */
static uint32_t fuzz_state = 0x12345678U;

static uint32_t fuzz_next(void)
{
  fuzz_state = (fuzz_state * 1103515245U) + 12345U;
  return (fuzz_state >> 8) & 0x00FFFFFFU;
}

static uint8_t fuzz_check_invariants(void)
{
  Ms5837Sample_t sample;
  uint32_t status = Ms5837_GetStatus();
  float pa = 0.0f;
  uint8_t type = 0U;
  uint8_t length = 0U;
  uint8_t value[4];

  /* 手册第 11 页：芯片忙期间不得重发转换、不得读 ADC。 */
  if (sim.new_conversion_while_busy != 0U)
  {
    return 0U;
  }
  if (sim.read_while_busy != 0U)
  {
    return 0U;
  }
  if (sim.early_reads != 0U)
  {
    return 0U;
  }
  /* D1/D2 必须来自同一 OSR（半周期不得跨配置配对）。 */
  if (sim.mismatched_pair != 0U)
  {
    return 0U;
  }
  /* 型号未确认时不得声称压力/温度/深度有效。 */
  if ((Ms5837_GetModel() == MS5837_MODEL_UNKNOWN) &&
      ((status & (MS5837_STATUS_PRESSURE_VALID | MS5837_STATUS_TEMPERATURE_VALID)) != 0U))
  {
    return 0U;
  }
  /* 零点无效时不得声称深度有效，也不得给出深度数值。 */
  if ((Ms5837_IsZeroValid() == 0U) && ((status & MS5837_STATUS_DEPTH_VALID) != 0U))
  {
    return 0U;
  }
  if (Ms5837_GetParam(MS5837_PARAM_SURFACE_PRESSURE, &type, &length, value) == MS5837_OK)
  {
    float stored;
    memcpy(&stored, value, sizeof(stored));
    if (Ms5837_IsZeroValid() == 0U)
    {
      return 0U;
    }
    if ((stored < MS5837_SURFACE_PRESSURE_MIN) || (stored > MS5837_SURFACE_PRESSURE_MAX))
    {
      return 0U; /* ZERO 与 GET_PARAMETER 的范围口径必须一致 */
    }
  }
  else if (Ms5837_IsZeroValid() != 0U)
  {
    /* 有零点却读不回参数：同样不允许。 */
    return 0U;
  }
  if (Ms5837_GetSurfacePressurePa(&pa) == MS5837_OK)
  {
    if ((pa < MS5837_SURFACE_PRESSURE_MIN) || (pa > MS5837_SURFACE_PRESSURE_MAX))
    {
      return 0U;
    }
  }

  if (Ms5837_GetSample(&sample) == MS5837_OK)
  {
    /* 绝不允许“标志有效但数值是 NaN”。 */
    if (((status & MS5837_STATUS_PRESSURE_VALID) != 0U) && MS5837_IS_NAN(sample.pressure_pa))
    {
      return 0U;
    }
    if (((status & MS5837_STATUS_TEMPERATURE_VALID) != 0U) && MS5837_IS_NAN(sample.temperature_c))
    {
      return 0U;
    }
    if ((status & MS5837_STATUS_DEPTH_VALID) != 0U)
    {
      if (MS5837_IS_NAN(sample.depth_raw_m) || MS5837_IS_NAN(sample.depth_filtered_m))
      {
        return 0U;
      }
    }
    else if (!MS5837_IS_NAN(sample.depth_raw_m) && (Ms5837_IsZeroValid() == 0U))
    {
      return 0U;
    }
  }
  return 1U;
}

static void Test_RandomizedInvariants(void)
{
  uint32_t step;
  uint32_t samples = 0U;
  uint32_t seq_seen = 0U;
  uint32_t seed_index;
  static const uint32_t seeds[5] = {0x12345678U, 0x0BADF00DU, 0x5EED1234U, 0xA5A5A5A5U, 0x0000FFFFU};
  Ms5837Sample_t sample;

  TEST_BEGIN("RandomizedInvariants");

  for (seed_index = 0U; seed_index < 5U; seed_index++)
  {
    uint32_t seed_samples = 0U;

    fuzz_state = seeds[seed_index];
    sim_reset();
    sim.model = MS5837_MODEL_30BA;
    sim.d1_value = 5000000U;
    sim.d2_value = 6981794U;
    test_tick = 0U;
    I2c_Init(&fake_handle);
    Ms5837_RestoreDefaults();
    CHECK(Ms5837_SetModel(MS5837_MODEL_30BA) == MS5837_OK);
    CHECK(Ms5837_Init() == MS5837_OK);
    CHECK(run_until(pred_prom_valid, 400U) != 0U);
    Ms5837_ClearNewSampleFlag();
    seq_seen = 0U;

    for (step = 0U; step < 3000U; step++)
    {
      uint32_t action = fuzz_next() % 16U;

      if (action < 8U)
      {
        Ms5837_Process();
      }
      else if (action == 8U)
      {
        static const uint16_t osrs[6] = {256U, 512U, 1024U, 2048U, 4096U, 8192U};
        (void)Ms5837_SetOsr(osrs[fuzz_next() % 6U]);
      }
      else if (action == 9U)
      {
        static const uint8_t models[3] = {MS5837_MODEL_UNKNOWN, MS5837_MODEL_02BA,
                                          MS5837_MODEL_30BA};
        (void)Ms5837_SetModel(models[fuzz_next() % 3U]);
      }
      else if (action == 10U)
      {
        (void)Ms5837_SetOutputRateHz((uint16_t)(1U + (fuzz_next() % 100U)));
      }
      else if (action == 11U)
      {
        (void)Ms5837_RestoreDefaults();
      }
      else if (action == 12U)
      {
        switch (fuzz_next() % 4U)
        {
          case 0U:
            (void)Ms5837_SetFilterK((float)(fuzz_next() % 100U) / 100.0f);
            break;
          case 1U:
            (void)Ms5837_SetWaterDensity(900.0f + (float)(fuzz_next() % 400U));
            break;
          case 2U:
            (void)Ms5837_Zero();
            break;
          default:
            (void)Ms5837_ClearZero();
            break;
        }
      }
      else if (action == 13U)
      {
        /* 通用参数入口（AA5B 路由走这条），含非法值。 */
        uint8_t raw[4];
        uint8_t type;
        uint8_t length;
        uint16_t id;
        uint32_t r = fuzz_next();
        switch (r % 4U)
        {
          case 0U:
            id = MS5837_PARAM_DEPTH_OSR;
            type = MS5837_PARAM_TYPE_U16;
            raw[0] = (uint8_t)r;
            raw[1] = (uint8_t)(r >> 8);
            (void)Ms5837_SetParam(id, type, raw, 2U);
            break;
          case 1U:
            id = MS5837_PARAM_DEPTH_MODEL;
            type = MS5837_PARAM_TYPE_U8;
            raw[0] = (uint8_t)(r % 5U);
            (void)Ms5837_SetParam(id, type, raw, 1U);
            break;
          case 2U:
            id = MS5837_PARAM_SURFACE_PRESSURE;
            type = MS5837_PARAM_TYPE_F32;
            {
              float v = (float)(r % 300000U);
              memcpy(raw, &v, sizeof(v));
            }
            (void)Ms5837_SetParam(id, type, raw, 4U);
            break;
          default:
            (void)Ms5837_GetParam(MS5837_PARAM_WATER_DENSITY, &type, &length, raw);
            (void)Ms5837_GetProm(0);
            break;
        }
      }
      else if (action == 14U)
      {
        /* 重新初始化：会放弃当前转换（手册第10页允许随时 Reset）。 */
        (void)Ms5837_Init();
      }
      else
      {
        /* 制造总线异常：离线/卡死会短暂持续，随后大概率恢复。 */
        sim.online = ((fuzz_next() % 4U) != 0U) ? 1U : 0U;
        sim.stall = ((fuzz_next() % 32U) == 0U) ? 1U : 0U;
      }

      if (fuzz_check_invariants() == 0U)
      {
        printf("        fuzz 违规 seed=%u step=%u action=%u (busy重发=%u busy读=%u 提前读=%u 配对错=%u)\n",
               seed_index, step, action, sim.new_conversion_while_busy, sim.read_while_busy,
               sim.early_reads, sim.mismatched_pair);
        CHECK(0);
      }
      if (Ms5837_HasNewSample() != 0U)
      {
        Ms5837_ClearNewSampleFlag();
        CHECK(Ms5837_GetSample(&sample) == MS5837_OK);
        CHECK(sample.sequence > seq_seen); /* 序号必须单调递增 */
        seq_seen = sample.sequence;
        samples++;
        seed_samples++;
      }

      /* 有时不推进 tick（命中同 tick 路径），有时大步跳（超期后不得追赶连发）。 */
      switch (fuzz_next() % 8U)
      {
        case 0U:
          break;
        case 1U:
          test_tick += 1U + (fuzz_next() % 500U);
          break;
        default:
          test_tick += fuzz_next() % 4U;
          break;
      }
    }

    printf("        fuzz seed=%u: 3000 步 / %u 个样本\n", seed_index, seed_samples);
  }

  sim.online = 1U;
  sim.stall = 0U;
  printf("        fuzz 合计: 5 seeds × 3000 步 / %u 个样本 / 最终 model=%u osr=%u rate=%uHz\n",
         samples, Ms5837_GetModel(), Ms5837_GetOsr(), Ms5837_GetOutputRateHz());
  CHECK(samples >= 20U); /* 确保随机序列真的跑出了采样 */
  CHECK(fuzz_check_invariants() != 0U);
  TEST_END();
}

/* ---------------------------------------------------------------- 17m. 跨实现参考向量 */
/*
 * 与固件无共享代码的独立 Python 实现在随机/极端输入上算出期望值，
 * 这里逐条比对 Ms5837_Compensate()，用于抓常数、取整语义与分支错误。
 */
static void Test_ReferenceVectors(void)
{
  uint32_t index;
  uint32_t checked = 0U;

  TEST_BEGIN("ReferenceVectors");

  for (index = 0U; index < MS5837_REFERENCE_VECTOR_COUNT; index++)
  {
    const Ms5837ReferenceVector_t *vector = &ms5837_reference_vectors[index];
    uint16_t prom[MS5837_PROM_WORDS];
    int64_t raw = 0;
    int32_t temp = 0;
    uint8_t k;

    for (k = 0U; k < MS5837_PROM_WORDS; k++)
    {
      prom[k] = 0U;
    }
    for (k = 1U; k <= 6U; k++)
    {
      prom[k] = vector->coefficient[k];
    }

    CHECK(Ms5837_Compensate(vector->model, prom, vector->d1, vector->d2, &raw, &temp) == 1U);
    if ((raw != vector->expected_raw) || (temp != vector->expected_temp_centi_c))
    {
      printf("        向量 %u 不一致: model=%u C1..C6=%u/%u/%u/%u/%u/%u d1=%u d2=%u"
             " 期望 raw=%lld temp=%d 实得 raw=%lld temp=%d\n",
             index, vector->model, vector->coefficient[1], vector->coefficient[2],
             vector->coefficient[3], vector->coefficient[4], vector->coefficient[5],
             vector->coefficient[6], vector->d1, vector->d2,
             (long long)vector->expected_raw, vector->expected_temp_centi_c,
             (long long)raw, temp);
      CHECK(0);
    }
    checked++;
  }
  printf("        参考向量 %u 条全部一致（30BA/02BA、物理量与极端输入、分支边界）\n", checked);
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

  /* 超时收敛：0 → 默认值（用中断模式的超时行为验证，见下面“上限”一段）。 */
  CHECK(I2c_WriteRead(&command, 1U, buffer, 2U, 0U) == I2C_BUS_OK);
  CHECK(sim.last_timeout_ms == (uint32_t)I2C_BUS_DEFAULT_TIMEOUT_MS);
  CHECK(buffer[0] == (uint8_t)(sim.prom[0] >> 8));
  CHECK(buffer[1] == (uint8_t)(sim.prom[0] & 0xFFU));

  sim.prom[0] = 0x1234U; /* 命令 0xA0 对应 PROM 字 0。 */
  CHECK(I2c_WriteRead(&command, 1U, buffer, 2U, 1000U) == I2C_BUS_OK);
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

/* ---------------------------------------------------------------- 中断模式异步核心 */
static void Test_InterruptModeAsync(void)
{
  uint8_t command = 0xA0U;
  uint8_t buffer[3] = {0U, 0U, 0U};

  TEST_BEGIN("InterruptModeAsync");

  sim_reset();
  test_tick = 0U;
  I2c_Init(&fake_handle);
  CHECK(I2c_GetPhase() == I2C_BUS_PHASE_IDLE);
  CHECK(I2c_IsBusy() == 0U);

  /* 延迟完成模式：提交后必须立刻返回，阶段为 BUSY，CPU 不被占用（tick 不推进）。 */
  sim_it_defer = 1U;
  sim.prom[0] = 0x5678U;
  {
    uint32_t before = test_tick;
    CHECK(I2c_Submit(&command, 1U, buffer, 2U, 0U) == I2C_BUS_OK);
    CHECK(I2c_GetPhase() == I2C_BUS_PHASE_BUSY);
    CHECK(I2c_IsBusy() == 1U);
    CHECK(test_tick == before); /* 异步：提交不消耗时间，也不自旋 */
  }

  /* 事务在飞时不允许复用总线（结果没取走也不允许覆盖）。 */
  CHECK(I2c_Submit(&command, 1U, buffer, 2U, 0U) == I2C_BUS_BUSY);
  CHECK(I2c_GetResult() == I2C_BUS_BUSY); /* 还没完成 */

  /* 中断到来：写阶段完成后自动发起读阶段，再完成读阶段。 */
  CHECK(sim_it_pending == 1U);
  sim_it_pump(); /* Tx 完成回调 → 内部启动 Rx */
  CHECK(I2c_GetPhase() == I2C_BUS_PHASE_BUSY);
  CHECK(sim_it_pending == 2U);
  sim_it_pump(); /* Rx 完成回调 */
  CHECK(I2c_GetPhase() == I2C_BUS_PHASE_DONE);
  CHECK(I2c_GetResult() == I2C_BUS_OK);
  CHECK(buffer[0] == 0x56U);
  CHECK(buffer[1] == 0x78U);
  CHECK(I2c_GetPhase() == I2C_BUS_PHASE_IDLE);

  /* 软件超时 + Abort：卡死时不再阻塞主循环，而是按上限超时后请求中止。 */
  sim.stall = 1U;
  CHECK(I2c_Submit(&command, 1U, buffer, 2U, 1000U) == I2C_BUS_OK);
  CHECK(I2c_GetPhase() == I2C_BUS_PHASE_BUSY);
  test_tick += (uint32_t)I2C_BUS_MAX_TIMEOUT_MS - 1U; /* 上限内：不中止 */
  I2c_Process();
  CHECK(sim.abort_count == 0U);
  CHECK(I2c_GetPhase() == I2C_BUS_PHASE_BUSY);
  test_tick += 1U; /* 到上限 50 ms：必须请求中止（1000 ms 被截断到 50 ms） */
  I2c_Process();
  CHECK(sim.abort_count == 1U);
  sim_it_pump(); /* Abort 完成回调 */
  CHECK(I2c_GetPhase() == I2C_BUS_PHASE_DONE);
  CHECK(I2c_GetResult() == I2C_BUS_TIMEOUT);
  sim.stall = 0U;

  /* 离线（地址无 ACK）在中断模式下由 ErrorCallback 结束，不阻塞。 */
  sim.online = 0U;
  CHECK(I2c_Submit(&command, 1U, 0, 0U, 0U) == I2C_BUS_OK);
  CHECK(I2c_GetPhase() == I2C_BUS_PHASE_DONE); /* 回调已同步触发（AF） */
  CHECK(I2c_GetResult() == I2C_BUS_ERROR);
  sim.online = 1U;

  /* 只读事务（无写阶段）与参数校验：回到“立即完成”模式。 */
  sim_it_defer = 0U;
  sim.read_pending = 1U;
  sim.read_length = 3U;
  sim.read_buffer[0] = 0xAAU;
  sim.read_buffer[1] = 0xBBU;
  sim.read_buffer[2] = 0xCCU;
  CHECK(I2c_Submit(0, 0U, buffer, 3U, 0U) == I2C_BUS_OK);
  CHECK(I2c_GetPhase() == I2C_BUS_PHASE_DONE);
  CHECK(I2c_GetResult() == I2C_BUS_OK);
  CHECK(buffer[0] == 0xAAU);
  CHECK(buffer[2] == 0xCCU);
  CHECK(I2c_Submit(0, 0U, 0, 0U, 0U) == I2C_BUS_PARAM); /* 既不写也不读 */
  CHECK(I2c_Submit(&command, (uint16_t)(I2C_BUS_MAX_TRANSFER + 1U), 0, 0U, 0U) == I2C_BUS_PARAM);

  /* 未绑定句柄时异步接口必须拒绝。 */
  I2c_Init(0);
  CHECK(I2c_Submit(&command, 1U, 0, 0U, 0U) == I2C_BUS_NOT_READY);
  I2c_Init(&fake_handle);

  sim_it_defer = 0U;
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
  Test_ModelChangeClearsZero();
  Test_ZeroValidatesRange();
  Test_ConfigChangeDuringConversion();
  Test_BusyDeviceNoEarlyRestart();
  Test_RestoreDefaultsDuringConversion();
  Test_OfficialExample02baTerms();
  Test_TemperatureBranchBoundary();
  Test_UnitsAndNegativeDepth();
  Test_RandomizedInvariants();
  Test_ReferenceVectors();
  Test_SampleMetadataAndFreshness();
  Test_I2cBusLayer();
  Test_InterruptModeAsync();

  if (test_failures != 0U)
  {
    printf("ms5837_host_test: FAIL (%u 项断言失败)\n", test_failures);
    return 1;
  }
  printf("ms5837_host_test: PASS\n");
  return 0;
}
