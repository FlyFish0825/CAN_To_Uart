#include "ms5837.h"

#include <string.h>

#include "sensor_i2c_bus.h"

/* ---------------------------------------------------------------- 命令字 */
#define MS5837_CMD_RESET           0x1EU /* 软复位。 */
#define MS5837_CMD_ADC_READ        0x00U /* 读取上一次转换的 24 位结果。 */
#define MS5837_CMD_PROM_READ_BASE  0xA0U /* 0xA0..0xAC，共 7 个字。 */
#define MS5837_CMD_CONVERT_D1_BASE 0x40U /* D1(压力)：0x40..0x4A。 */
#define MS5837_CMD_CONVERT_D2_BASE 0x50U /* D2(温度)：0x50..0x5A。 */

/*
 * 官方物理 PROM 只有 7 个 16 位字，读取命令 0xA0/0xA2/0xA4/0xA6/0xA8/0xAA/0xAC。
 * 驱动**只读这 7 个字**，绝不读 0xAE；CRC4 算法需要的第 8 个数组元素由软件置 0
 * （见 ms5837.prom[7] = 0），设备不需要有第 8 个字也能上线。
 */
#define MS5837_CMD_PROM_WORD_COUNT 7U /* 实读 PROM 字数（0xA0..0xAC）。 */
#define MS5837_ADC_BYTES           3U /* ADC 结果长度。 */
#define MS5837_PROM_BYTES          2U /* PROM 单字长度。 */
#define MS5837_I2C_TIMEOUT_MS      I2C_BUS_DEFAULT_TIMEOUT_MS /* 单次 I2C 事务短超时。 */
#define MS5837_NAN_BITS            0x7FC00000UL /* 标准 quiet NaN 位模式。 */
#define MS5837_SAMPLE_NEVER_MS     0xFFFFFFFFUL /* 从未采样的时间戳。 */

/* ---------------------------------------------------------------- 状态机 */
typedef enum
{
  MS5837_STATE_OFFLINE = 0, /* 未初始化或初始化失败，等待重试。 */
  MS5837_STATE_RESET_WAIT, /* 已发复位命令，等待复位延时。 */
  MS5837_STATE_PROM_READ, /* 逐字读取 PROM。 */
  MS5837_STATE_IDLE, /* 等待下一次采样周期。 */
  MS5837_STATE_CONVERT_D1, /* D1 转换进行中（非阻塞等待）。 */
  MS5837_STATE_CONVERT_D2 /* D2 转换进行中（非阻塞等待）。 */
} Ms5837State_t;

typedef struct
{
  uint8_t model; /* 型号：0/2/30。 */
  uint16_t osr; /* 过采样率。 */
  uint16_t output_rate_hz; /* 采样率。 */
  float water_density; /* 水体密度 kg/m3。 */
  float surface_pressure_pa; /* 显式零点压力；zero_valid 为唯一有效性依据。 */
  float filter_k; /* 一阶滤波系数 0~0.99。 */
} Ms5837Config_t;

typedef struct
{
  Ms5837State_t state; /* 当前状态。 */
  uint8_t init_requested; /* 是否已显式要求初始化（决定 OFFLINE 是否自动重试）。 */
  uint8_t prom_valid; /* PROM CRC 是否通过。 */
  uint8_t zero_valid; /* 零点是否有效。 */
  uint8_t sample_ready; /* 是否已有完成的样本。 */
  uint8_t new_sample; /* 自上次清除后是否有新样本。 */
  uint8_t filter_valid; /* 滤波状态是否可用。 */
  uint8_t prom_index; /* 正在读取的 PROM 字下标。 */
  uint32_t deadline_ms; /* 当前状态的到期时刻（等待或下一次动作）。 */
  uint32_t status; /* 实时状态字。 */
  uint32_t d1_raw; /* 本次周期的 D1。 */
  uint32_t d2_raw; /* 本次周期的 D2。 */
  float depth_filtered_m; /* 滤波深度状态。 */
  uint16_t prom[MS5837_PROM_WORDS]; /* PROM 原始字。 */
  Ms5837Config_t config; /* 本地参数。 */
  Ms5837Sample_t sample; /* 最近一次样本。 */
  Ms5837Stats_t stats; /* 运行统计。 */
} Ms5837Sensor_t;

/* 单实例驱动状态。上电默认：型号未确认、OSR4096、25 Hz、海水密度、无零点、不做滤波。 */
static Ms5837Sensor_t ms5837 =
{
  .state = MS5837_STATE_OFFLINE,
  .init_requested = 0U,
  .prom_valid = 0U,
  .zero_valid = 0U,
  .sample_ready = 0U,
  .new_sample = 0U,
  .filter_valid = 0U,
  .prom_index = 0U,
  .deadline_ms = 0U,
  .status = 0U,
  .d1_raw = 0U,
  .d2_raw = 0U,
  .depth_filtered_m = 0.0f,
  .prom = {0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U},
  .config =
  {
    .model = MS5837_MODEL_UNKNOWN,
    .osr = MS5837_OSR_DEFAULT,
    .output_rate_hz = MS5837_OUTPUT_RATE_HZ_DEFAULT,
    .water_density = MS5837_WATER_DENSITY_DEFAULT,
    .surface_pressure_pa = 0.0f, /* 未采集零点时不参与任何计算。 */
    .filter_k = MS5837_FILTER_K_DEFAULT
  },
  .sample = {0},
  .stats =
  {
    .osr = MS5837_OSR_DEFAULT,
    .output_rate_hz = MS5837_OUTPUT_RATE_HZ_DEFAULT
  }
};

/* 数据手册 ADC 表给出的最大转换时间（微秒）：下标对应 OSR 256/512/1024/2048/4096/8192。 */
static const uint16_t ms5837_conv_time_us_30ba[6] = {600U, 1170U, 2280U, 4540U, 9040U, 18080U};
static const uint16_t ms5837_conv_time_us_02ba[6] = {560U, 1100U, 2170U, 4320U, 8610U, 17200U};

static const uint16_t ms5837_osr_table[6] = {
    MS5837_OSR_256, MS5837_OSR_512, MS5837_OSR_1024,
    MS5837_OSR_2048, MS5837_OSR_4096, MS5837_OSR_8192};

/* ---------------------------------------------------------------- 小工具 */
static float ms5837_nan(void)
{
  float value;
  uint32_t bits = MS5837_NAN_BITS;
  memcpy(&value, &bits, sizeof(value));
  return value;
}

/* 有符号整数除法向负无穷取整：与数据手册流程图中的 >> n 语义一致（官方算例可验证）。 */
static int64_t ms5837_floor_div_pow2(int64_t value, uint8_t shift)
{
  int64_t divisor = (int64_t)1 << shift;
  if (value >= 0)
  {
    return value / divisor;
  }
  return -(((-value) + divisor - 1) / divisor);
}

/* 是否到达到期时刻；用无符号差值比较，天然处理 tick 回绕。 */
static uint8_t ms5837_deadline_reached(uint32_t now, uint32_t deadline)
{
  return ((int32_t)(now - deadline) >= 0) ? 1U : 0U;
}

static uint8_t ms5837_osr_to_index(uint16_t osr)
{
  uint8_t index;
  for (index = 0U; index < 6U; index++)
  {
    if (ms5837_osr_table[index] == osr)
    {
      return index;
    }
  }
  return 0xFFU;
}

/* 型号/OSR 对应的最大转换时间（微秒）；型号未知时取两者较大值。 */
static uint32_t ms5837_conversion_time_us(uint8_t model, uint16_t osr)
{
  uint8_t index = ms5837_osr_to_index(osr);
  if (index == 0xFFU)
  {
    return 0U;
  }
  if (model == MS5837_MODEL_02BA)
  {
    return (uint32_t)ms5837_conv_time_us_02ba[index];
  }
  if (model == MS5837_MODEL_30BA)
  {
    return (uint32_t)ms5837_conv_time_us_30ba[index];
  }
  /* 型号未确认：按较慢的一侧等待，避免提前读到未完成的 ADC 结果。 */
  return (uint32_t)(ms5837_conv_time_us_30ba[index] > ms5837_conv_time_us_02ba[index]
                        ? ms5837_conv_time_us_30ba[index]
                        : ms5837_conv_time_us_02ba[index]);
}

/* 向上取整到毫秒：宁可多等 1 ms，也不提前读 ADC。 */
static uint32_t ms5837_conversion_time_ms(uint8_t model, uint16_t osr)
{
  uint32_t us = ms5837_conversion_time_us(model, osr);
  return (us + 999U) / 1000U;
}

/* 一个 D1+D2 采样周期能否塞进当前采样率周期。 */
static uint8_t ms5837_schedule_fits(uint8_t model, uint16_t osr, uint16_t rate_hz)
{
  uint32_t period_ms;
  uint32_t needed_ms;
  if ((rate_hz < MS5837_OUTPUT_RATE_HZ_MIN) || (rate_hz > MS5837_OUTPUT_RATE_HZ_MAX))
  {
    return 0U;
  }
  period_ms = 1000U / (uint32_t)rate_hz;
  needed_ms = 2U * ms5837_conversion_time_ms(model, osr);
  return (needed_ms <= period_ms) ? 1U : 0U;
}

static Ms5837Result_t ms5837_map_i2c(I2cBusResult_t result)
{
  switch (result)
  {
    case I2C_BUS_OK:
      return MS5837_OK;
    case I2C_BUS_NOT_READY:
      return MS5837_ERR_NOT_READY;
    case I2C_BUS_PARAM:
      return MS5837_ERR_PARAM;
    case I2C_BUS_BUSY:
      return MS5837_ERR_BUSY;
    case I2C_BUS_TIMEOUT:
      return MS5837_ERR_TIMEOUT;
    default:
      return MS5837_ERR_IO;
  }
}

/* ---------------------------------------------------------------- I2C 访问 */
static I2cBusResult_t ms5837_write_command(uint8_t command)
{
  return I2c_Write(&command, 1U, MS5837_I2C_TIMEOUT_MS);
}

static I2cBusResult_t ms5837_read_adc(uint32_t *value)
{
  uint8_t command = MS5837_CMD_ADC_READ;
  uint8_t buffer[MS5837_ADC_BYTES];
  I2cBusResult_t result;

  result = I2c_WriteRead(&command, 1U, buffer, MS5837_ADC_BYTES, MS5837_I2C_TIMEOUT_MS);
  if (result != I2C_BUS_OK)
  {
    return result;
  }
  *value = ((uint32_t)buffer[0] << 16) | ((uint32_t)buffer[1] << 8) | (uint32_t)buffer[2];
  return I2C_BUS_OK;
}

static I2cBusResult_t ms5837_read_prom_word(uint8_t index, uint16_t *word)
{
  uint8_t command = (uint8_t)(MS5837_CMD_PROM_READ_BASE + (uint8_t)(index * 2U));
  uint8_t buffer[MS5837_PROM_BYTES];
  I2cBusResult_t result;

  result = I2c_WriteRead(&command, 1U, buffer, MS5837_PROM_BYTES, MS5837_I2C_TIMEOUT_MS);
  if (result != I2C_BUS_OK)
  {
    return result;
  }
  *word = (uint16_t)(((uint16_t)buffer[0] << 8) | (uint16_t)buffer[1]);
  return I2C_BUS_OK;
}

/* ---------------------------------------------------------------- 错误处理 */
static void ms5837_record_error(Ms5837Result_t error, uint32_t now)
{
  ms5837.stats.errors++;
  ms5837.stats.last_error = (uint32_t)error;
  if ((error == MS5837_ERR_TIMEOUT) || (error == MS5837_ERR_BUSY))
  {
    ms5837.stats.bus_timeouts++;
  }
  if (error == MS5837_ERR_CRC)
  {
    ms5837.stats.crc_errors++;
  }
  /* 总线或数据不可信：清掉新鲜数据类状态位；PROM/ZERO 属于配置与身份信息，保留。 */
  ms5837.status &= ~(MS5837_STATUS_ONLINE | MS5837_STATUS_RAW_VALID |
                     MS5837_STATUS_PRESSURE_VALID | MS5837_STATUS_TEMPERATURE_VALID |
                     MS5837_STATUS_DEPTH_VALID);
  ms5837.state = MS5837_STATE_IDLE;
  ms5837.deadline_ms = now + MS5837_RETRY_DELAY_MS;
}

/* ---------------------------------------------------------------- 补偿 */
uint8_t Ms5837_Crc4(const uint16_t prom[MS5837_PROM_WORDS])
{
  uint16_t local[MS5837_PROM_WORDS];
  uint16_t remainder = 0U;
  uint8_t counter;
  uint8_t bit;

  if (prom == 0)
  {
    return 0xFFU;
  }
  memcpy(local, prom, sizeof(local));
  local[0] &= 0x0FFFU; /* 字 0 高 4 位是 CRC 本身，先清零。 */
  local[7] = 0U; /* 保留字固定为 0。 */

  for (counter = 0U; counter < 16U; counter++)
  {
    if ((counter % 2U) == 1U)
    {
      remainder ^= (uint16_t)(local[counter >> 1] & 0x00FFU);
    }
    else
    {
      remainder ^= (uint16_t)(local[counter >> 1] >> 8);
    }
    for (bit = 8U; bit > 0U; bit--)
    {
      if ((remainder & 0x8000U) != 0U)
      {
        remainder = (uint16_t)((remainder << 1) ^ 0x3000U);
      }
      else
      {
        remainder = (uint16_t)(remainder << 1);
      }
    }
  }
  return (uint8_t)((remainder >> 12) & 0x000FU);
}

uint16_t Ms5837_MaxConversionTimeMs(uint8_t model, uint16_t osr)
{
  return (uint16_t)ms5837_conversion_time_ms(model, osr);
}

uint8_t Ms5837_Compensate(uint8_t model,
                          const uint16_t prom[MS5837_PROM_WORDS],
                          uint32_t d1,
                          uint32_t d2,
                          int64_t *pressure_raw,
                          int32_t *temperature_centi_c)
{
  int64_t dt;
  int64_t temp;
  int64_t off;
  int64_t sens;
  int64_t ti = 0;
  int64_t off_i = 0;
  int64_t sens_i = 0;
  int64_t off2;
  int64_t sens2;
  int64_t pressure;

  if ((prom == 0) || (model == MS5837_MODEL_UNKNOWN))
  {
    return 0U;
  }
  if ((model != MS5837_MODEL_02BA) && (model != MS5837_MODEL_30BA))
  {
    return 0U;
  }

  /* 一阶：dT / TEMP / OFF / SENS。TEMP 向负无穷取整，与数据手册 19.81 °C 算例一致。 */
  dt = (int64_t)d2 - ((int64_t)prom[5] << 8);
  temp = 2000 + ms5837_floor_div_pow2(dt * (int64_t)prom[6], 23);

  if (model == MS5837_MODEL_02BA)
  {
    off = ((int64_t)prom[2] << 17) + ms5837_floor_div_pow2((int64_t)prom[4] * dt, 6);
    sens = ((int64_t)prom[1] << 16) + ms5837_floor_div_pow2((int64_t)prom[3] * dt, 7);
  }
  else
  {
    off = ((int64_t)prom[2] << 16) + ms5837_floor_div_pow2((int64_t)prom[4] * dt, 7);
    sens = ((int64_t)prom[1] << 15) + ms5837_floor_div_pow2((int64_t)prom[3] * dt, 8);
  }

  /* 二阶温度补偿。比较用 0.01 °C 整数：TEMP/100 < 20 °C 等价 TEMP < 2000。 */
  if (model == MS5837_MODEL_02BA)
  {
    if (temp < 2000)
    {
      ti = ms5837_floor_div_pow2(11 * dt * dt, 35);
      off_i = ms5837_floor_div_pow2(31 * (temp - 2000) * (temp - 2000), 3);
      sens_i = ms5837_floor_div_pow2(63 * (temp - 2000) * (temp - 2000), 5);
    }
  }
  else
  {
    if (temp < 2000) /* 低温：< 20 °C。 */
    {
      ti = ms5837_floor_div_pow2(3 * dt * dt, 33);
      off_i = ms5837_floor_div_pow2(3 * (temp - 2000) * (temp - 2000), 1);
      sens_i = ms5837_floor_div_pow2(5 * (temp - 2000) * (temp - 2000), 3);
      if (temp < -1500) /* 极低温：< -15 °C。 */
      {
        off_i += 7 * (temp + 1500) * (temp + 1500);
        sens_i += 4 * (temp + 1500) * (temp + 1500);
      }
    }
    else /* 高温：>= 20 °C。 */
    {
      ti = ms5837_floor_div_pow2(2 * dt * dt, 37);
      off_i = ms5837_floor_div_pow2((temp - 2000) * (temp - 2000), 4);
      sens_i = 0;
    }
  }

  off2 = off - off_i;
  sens2 = sens - sens_i;
  temp -= ti;

  if (model == MS5837_MODEL_02BA)
  {
    /* 02BA 整数压力单位为 0.01 mbar，正好 1 LSB = 1 Pa。 */
    pressure = ms5837_floor_div_pow2(ms5837_floor_div_pow2((int64_t)d1 * sens2, 21) - off2, 15);
  }
  else
  {
    /* 30BA 整数压力单位为 0.1 mbar，1 LSB = 10 Pa。 */
    pressure = ms5837_floor_div_pow2(ms5837_floor_div_pow2((int64_t)d1 * sens2, 21) - off2, 13);
  }

  if (pressure_raw != 0)
  {
    *pressure_raw = pressure;
  }
  if (temperature_centi_c != 0)
  {
    *temperature_centi_c = (int32_t)temp;
  }
  return 1U;
}

/* ---------------------------------------------------------------- 深度与样本 */
static void ms5837_apply_depth(void)
{
  float depth_raw;
  float pressure_pa = ms5837.sample.pressure_pa;

  if (((ms5837.status & MS5837_STATUS_PRESSURE_VALID) == 0U) || (ms5837.zero_valid == 0U))
  {
    ms5837.status &= ~MS5837_STATUS_DEPTH_VALID;
    ms5837.sample.depth_raw_m = ms5837_nan();
    ms5837.sample.depth_filtered_m = ms5837_nan();
    ms5837.sample.surface_pressure_pa = (ms5837.zero_valid != 0U)
                                            ? ms5837.config.surface_pressure_pa
                                            : ms5837_nan();
    ms5837.filter_valid = 0U;
    return;
  }

  depth_raw = (pressure_pa - ms5837.config.surface_pressure_pa) /
              (ms5837.config.water_density * MS5837_GRAVITY);
  ms5837.sample.depth_raw_m = depth_raw;
  ms5837.sample.surface_pressure_pa = ms5837.config.surface_pressure_pa;

  if (ms5837.filter_valid == 0U)
  {
    ms5837.depth_filtered_m = depth_raw;
    ms5837.filter_valid = 1U;
  }
  else
  {
    /* y = k*y_prev + (1-k)*x，与参数 0104 FILTER_K 的定义一致。 */
    ms5837.depth_filtered_m = (ms5837.config.filter_k * ms5837.depth_filtered_m) +
                              ((1.0f - ms5837.config.filter_k) * depth_raw);
  }
  ms5837.sample.depth_filtered_m = ms5837.depth_filtered_m;
  ms5837.status |= MS5837_STATUS_DEPTH_VALID;
}

static void ms5837_finish_sample(uint32_t now)
{
  int64_t pressure_raw = 0;
  int32_t temperature_centi_c = 0;

  ms5837.sample.d1 = ms5837.d1_raw;
  ms5837.sample.d2 = ms5837.d2_raw;
  ms5837.sample.timestamp_ms = now;
  ms5837.sample.model = ms5837.config.model;
  memcpy(ms5837.sample.prom, ms5837.prom, sizeof(ms5837.sample.prom));
  ms5837.sample.pressure_raw = 0;
  ms5837.sample.temperature_centi_c = 0;
  ms5837.sample.pressure_pa = ms5837_nan();
  ms5837.sample.temperature_c = ms5837_nan();
  ms5837.sample.depth_raw_m = ms5837_nan();
  ms5837.sample.depth_filtered_m = ms5837_nan();
  ms5837.sample.surface_pressure_pa = (ms5837.zero_valid != 0U)
                                          ? ms5837.config.surface_pressure_pa
                                          : ms5837_nan();

  ms5837.status |= (MS5837_STATUS_ONLINE | MS5837_STATUS_RAW_VALID);
  ms5837.status &= ~(MS5837_STATUS_PRESSURE_VALID | MS5837_STATUS_TEMPERATURE_VALID |
                     MS5837_STATUS_DEPTH_VALID);

  if (Ms5837_Compensate(ms5837.config.model, ms5837.prom, ms5837.d1_raw, ms5837.d2_raw,
                        &pressure_raw, &temperature_centi_c) != 0U)
  {
    ms5837.sample.pressure_raw = pressure_raw;
    ms5837.sample.temperature_centi_c = temperature_centi_c;
    /* 30BA 的 0.1 mbar 换算 10 Pa/LSB；02BA 的 0.01 mbar 正好 1 Pa/LSB。 */
    ms5837.sample.pressure_pa = (ms5837.config.model == MS5837_MODEL_30BA)
                                    ? ((float)pressure_raw * 10.0f)
                                    : ((float)pressure_raw);
    ms5837.sample.temperature_c = (float)temperature_centi_c / 100.0f;
    ms5837.status |= (MS5837_STATUS_PRESSURE_VALID | MS5837_STATUS_TEMPERATURE_VALID);
    ms5837.status &= ~MS5837_STATUS_CONFIG_UNKNOWN;
  }
  else
  {
    ms5837.status |= MS5837_STATUS_CONFIG_UNKNOWN;
    ms5837.status &= ~(MS5837_STATUS_PRESSURE_VALID | MS5837_STATUS_TEMPERATURE_VALID);
  }

  ms5837.sample.sequence = ms5837.stats.sample_seq + 1U;
  ms5837.stats.sample_seq = ms5837.sample.sequence;
  ms5837.stats.good_frames++;
  ms5837.sample_ready = 1U;
  ms5837.new_sample = 1U;

  ms5837_apply_depth();
  ms5837.sample.status = ms5837.status;
}

/* ---------------------------------------------------------------- 状态机步骤 */
static void ms5837_start_cycle(uint32_t now)
{
  I2cBusResult_t result;
  uint8_t osr_index = ms5837_osr_to_index(ms5837.config.osr);

  if (osr_index == 0xFFU)
  {
    ms5837_record_error(MS5837_ERR_PARAM, now);
    return;
  }
  result = ms5837_write_command((uint8_t)(MS5837_CMD_CONVERT_D1_BASE + (uint8_t)(osr_index << 1)));
  if (result != I2C_BUS_OK)
  {
    ms5837_record_error(ms5837_map_i2c(result), now);
    return;
  }
  ms5837.state = MS5837_STATE_CONVERT_D1;
  ms5837.deadline_ms = now + ms5837_conversion_time_ms(ms5837.config.model, ms5837.config.osr);
}

static void ms5837_read_d1(uint32_t now)
{
  I2cBusResult_t result;
  uint8_t osr_index = ms5837_osr_to_index(ms5837.config.osr);

  result = ms5837_read_adc(&ms5837.d1_raw);
  if (result != I2C_BUS_OK)
  {
    ms5837_record_error(ms5837_map_i2c(result), now);
    return;
  }
  ms5837.status |= MS5837_STATUS_ONLINE;
  if (osr_index == 0xFFU)
  {
    ms5837_record_error(MS5837_ERR_PARAM, now);
    return;
  }
  result = ms5837_write_command((uint8_t)(MS5837_CMD_CONVERT_D2_BASE + (uint8_t)(osr_index << 1)));
  if (result != I2C_BUS_OK)
  {
    ms5837_record_error(ms5837_map_i2c(result), now);
    return;
  }
  ms5837.state = MS5837_STATE_CONVERT_D2;
  ms5837.deadline_ms = now + ms5837_conversion_time_ms(ms5837.config.model, ms5837.config.osr);
}

static void ms5837_read_d2(uint32_t now)
{
  I2cBusResult_t result;
  uint32_t period_ms;

  result = ms5837_read_adc(&ms5837.d2_raw);
  if (result != I2C_BUS_OK)
  {
    ms5837_record_error(ms5837_map_i2c(result), now);
    return;
  }
  ms5837.status |= MS5837_STATUS_ONLINE;
  ms5837.state = MS5837_STATE_IDLE;
  period_ms = 1000U / (uint32_t)ms5837.config.output_rate_hz;
  ms5837.deadline_ms = now + period_ms;
  ms5837_finish_sample(now);
}

static void ms5837_process_prom(uint32_t now)
{
  I2cBusResult_t result;
  uint16_t word = 0U;
  uint8_t crc_read;
  uint8_t crc_calc;

  result = ms5837_read_prom_word(ms5837.prom_index, &word);
  if (result != I2C_BUS_OK)
  {
    ms5837_record_error(ms5837_map_i2c(result), now);
    /* 总线恢复后从第一个字重新读取，避免半截 PROM。 */
    ms5837.prom_index = 0U;
    ms5837.deadline_ms = now + MS5837_RETRY_DELAY_MS;
    return;
  }

  ms5837.prom[ms5837.prom_index] = word;
  ms5837.prom_index++;
  if (ms5837.prom_index < MS5837_CMD_PROM_WORD_COUNT)
  {
    return;
  }

  ms5837.prom[7] = 0U;
  crc_read = (uint8_t)((ms5837.prom[0] >> 12) & 0x0FU);
  crc_calc = Ms5837_Crc4(ms5837.prom);
  if (crc_read != crc_calc)
  {
    ms5837.status &= ~(MS5837_STATUS_PROM_VALID | MS5837_STATUS_ONLINE |
                       MS5837_STATUS_RAW_VALID | MS5837_STATUS_PRESSURE_VALID |
                       MS5837_STATUS_TEMPERATURE_VALID | MS5837_STATUS_DEPTH_VALID);
    ms5837.prom_valid = 0U;
    ms5837.stats.prom_valid = 0U;
    ms5837.stats.errors++;
    ms5837.stats.crc_errors++;
    ms5837.stats.last_error = (uint32_t)MS5837_ERR_CRC;
    ms5837.prom_index = 0U;
    ms5837.deadline_ms = now + MS5837_RETRY_DELAY_MS;
    return;
  }

  ms5837.prom_valid = 1U;
  ms5837.stats.prom_valid = 1U;
  ms5837.status |= (MS5837_STATUS_PROM_VALID | MS5837_STATUS_ONLINE);
  if (ms5837.config.model == MS5837_MODEL_UNKNOWN)
  {
    ms5837.status |= MS5837_STATUS_CONFIG_UNKNOWN;
    ms5837.status &= ~MS5837_STATUS_MODEL_CONFIRMED;
  }
  else
  {
    ms5837.status &= ~MS5837_STATUS_CONFIG_UNKNOWN;
    ms5837.status |= MS5837_STATUS_MODEL_CONFIRMED;
  }
  ms5837.state = MS5837_STATE_IDLE;
  ms5837.deadline_ms = now; /* 立即可开始第一帧。 */
}

/* ---------------------------------------------------------------- 公共接口 */
Ms5837Result_t Ms5837_Init(void)
{
  uint32_t now;
  I2cBusResult_t result;

  if (I2c_IsReady() == 0U)
  {
    ms5837.state = MS5837_STATE_OFFLINE;
    ms5837.init_requested = 0U;
    return MS5837_ERR_NOT_READY;
  }

  (void)I2c_SetDevice(I2C_BUS_MS5837_ADDRESS7);
  now = HAL_GetTick();

  ms5837.init_requested = 1U;
  ms5837.prom_valid = 0U;
  ms5837.sample_ready = 0U;
  ms5837.new_sample = 0U;
  ms5837.filter_valid = 0U;
  ms5837.prom_index = 0U;
  ms5837.status &= ~(MS5837_STATUS_PROM_VALID | MS5837_STATUS_ONLINE |
                     MS5837_STATUS_RAW_VALID | MS5837_STATUS_PRESSURE_VALID |
                     MS5837_STATUS_TEMPERATURE_VALID | MS5837_STATUS_DEPTH_VALID);
  ms5837.stats.prom_valid = 0U;

  result = ms5837_write_command(MS5837_CMD_RESET);
  if (result != I2C_BUS_OK)
  {
    Ms5837Result_t mapped = ms5837_map_i2c(result);
    ms5837_record_error(mapped, now);
    return mapped;
  }

  ms5837.state = MS5837_STATE_RESET_WAIT;
  ms5837.deadline_ms = now + MS5837_RESET_DELAY_MS;
  return MS5837_OK;
}

void Ms5837_Process(void)
{
  uint32_t now = HAL_GetTick();

  switch (ms5837.state)
  {
    case MS5837_STATE_OFFLINE:
      if ((ms5837.init_requested != 0U) && (ms5837_deadline_reached(now, ms5837.deadline_ms) != 0U))
      {
        (void)Ms5837_Init();
      }
      break;

    case MS5837_STATE_RESET_WAIT:
      if (ms5837_deadline_reached(now, ms5837.deadline_ms) != 0U)
      {
        ms5837.state = MS5837_STATE_PROM_READ;
        ms5837.prom_index = 0U;
      }
      break;

    case MS5837_STATE_PROM_READ:
      if (ms5837_deadline_reached(now, ms5837.deadline_ms) != 0U)
      {
        ms5837_process_prom(now);
      }
      break;

    case MS5837_STATE_IDLE:
      if (ms5837.prom_valid == 0U)
      {
        /* 还没有可信 PROM：不发转换命令，按重试间隔再读 PROM。 */
        if ((ms5837.init_requested != 0U) && (ms5837_deadline_reached(now, ms5837.deadline_ms) != 0U))
        {
          ms5837.state = MS5837_STATE_PROM_READ;
          ms5837.prom_index = 0U;
        }
        break;
      }
      if (ms5837_deadline_reached(now, ms5837.deadline_ms) != 0U)
      {
        ms5837_start_cycle(now);
      }
      break;

    case MS5837_STATE_CONVERT_D1:
      if (ms5837_deadline_reached(now, ms5837.deadline_ms) != 0U)
      {
        ms5837_read_d1(now);
      }
      break;

    case MS5837_STATE_CONVERT_D2:
      if (ms5837_deadline_reached(now, ms5837.deadline_ms) != 0U)
      {
        ms5837_read_d2(now);
      }
      break;

    default:
      ms5837.state = MS5837_STATE_OFFLINE;
      ms5837.deadline_ms = now + MS5837_RETRY_DELAY_MS;
      break;
  }
}

Ms5837Result_t Ms5837_GetSample(Ms5837Sample_t *sample)
{
  if (sample == 0)
  {
    return MS5837_ERR_PARAM;
  }
  if (ms5837.sample_ready == 0U)
  {
    return MS5837_ERR_NO_SAMPLE;
  }
  *sample = ms5837.sample;
  sample->status = ms5837.status;
  return MS5837_OK;
}

uint32_t Ms5837_GetStatus(void)
{
  return ms5837.status;
}

uint32_t Ms5837_GetSampleAgeMs(void)
{
  if (ms5837.sample_ready == 0U)
  {
    return MS5837_SAMPLE_NEVER_MS;
  }
  return (uint32_t)(HAL_GetTick() - ms5837.sample.timestamp_ms);
}

uint8_t Ms5837_HasNewSample(void)
{
  return ms5837.new_sample;
}

void Ms5837_ClearNewSampleFlag(void)
{
  ms5837.new_sample = 0U;
}

Ms5837Result_t Ms5837_GetStats(Ms5837Stats_t *stats)
{
  if (stats == 0)
  {
    return MS5837_ERR_PARAM;
  }
  ms5837.stats.model = ms5837.config.model;
  ms5837.stats.osr = ms5837.config.osr;
  ms5837.stats.output_rate_hz = ms5837.config.output_rate_hz;
  ms5837.stats.prom_valid = ms5837.prom_valid;
  *stats = ms5837.stats;
  return MS5837_OK;
}

Ms5837Result_t Ms5837_GetProm(uint16_t prom[MS5837_PROM_WORDS])
{
  if (prom == 0)
  {
    return MS5837_ERR_PARAM;
  }
  if (ms5837.prom_valid == 0U)
  {
    return MS5837_ERR_NOT_READY;
  }
  memcpy(prom, ms5837.prom, sizeof(ms5837.prom));
  return MS5837_OK;
}

/* ---------------------------------------------------------------- 零点 */
Ms5837Result_t Ms5837_Zero(void)
{
  if (ms5837.sample_ready == 0U)
  {
    return MS5837_ERR_NO_SAMPLE;
  }
  if ((ms5837.status & MS5837_STATUS_PRESSURE_VALID) == 0U)
  {
    return (ms5837.config.model == MS5837_MODEL_UNKNOWN) ? MS5837_ERR_MODEL_UNKNOWN
                                                         : MS5837_ERR_NOT_READY;
  }

  ms5837.config.surface_pressure_pa = ms5837.sample.pressure_pa;
  ms5837.zero_valid = 1U;
  ms5837.status |= MS5837_STATUS_ZERO_VALID;
  ms5837.filter_valid = 0U; /* 新零点：滤波从当前原始深度重新起步。 */
  ms5837_apply_depth();
  ms5837.sample.status = ms5837.status;
  return MS5837_OK;
}

Ms5837Result_t Ms5837_ClearZero(void)
{
  ms5837.config.surface_pressure_pa = ms5837_nan();
  ms5837.zero_valid = 0U;
  ms5837.status &= ~MS5837_STATUS_ZERO_VALID;
  ms5837.filter_valid = 0U;
  ms5837_apply_depth();
  ms5837.sample.status = ms5837.status;
  return MS5837_OK;
}

uint8_t Ms5837_IsZeroValid(void)
{
  return ms5837.zero_valid;
}

Ms5837Result_t Ms5837_RestoreDefaults(void)
{
  ms5837.config.model = MS5837_MODEL_UNKNOWN;
  ms5837.config.osr = MS5837_OSR_DEFAULT;
  ms5837.config.output_rate_hz = MS5837_OUTPUT_RATE_HZ_DEFAULT;
  ms5837.config.water_density = MS5837_WATER_DENSITY_DEFAULT;
  ms5837.config.filter_k = MS5837_FILTER_K_DEFAULT;
  ms5837.stats.model = MS5837_MODEL_UNKNOWN;
  ms5837.stats.osr = MS5837_OSR_DEFAULT;
  ms5837.stats.output_rate_hz = MS5837_OUTPUT_RATE_HZ_DEFAULT;
  ms5837.status |= MS5837_STATUS_CONFIG_UNKNOWN;
  ms5837.status &= ~MS5837_STATUS_MODEL_CONFIRMED;
  (void)Ms5837_ClearZero();
  return MS5837_OK;
}

/* ---------------------------------------------------------------- 参数 */
Ms5837Result_t Ms5837_SetModel(uint8_t model)
{
  if ((model != MS5837_MODEL_UNKNOWN) && (model != MS5837_MODEL_02BA) &&
      (model != MS5837_MODEL_30BA))
  {
    return MS5837_ERR_PARAM;
  }
  if (ms5837_schedule_fits(model, ms5837.config.osr, ms5837.config.output_rate_hz) == 0U)
  {
    /* 切换型号会改变最大转换时间：当前采样率放不下时拒绝，而不是偷偷降速。 */
    return MS5837_ERR_PARAM;
  }

  ms5837.config.model = model;
  ms5837.stats.model = model;

  if (model == MS5837_MODEL_UNKNOWN)
  {
    ms5837.status |= MS5837_STATUS_CONFIG_UNKNOWN;
    ms5837.status &= ~MS5837_STATUS_MODEL_CONFIRMED;
  }
  else
  {
    ms5837.status &= ~MS5837_STATUS_CONFIG_UNKNOWN;
    ms5837.status |= MS5837_STATUS_MODEL_CONFIRMED;
  }

  /* 旧样本是按旧型号算的，立即失效，等下一帧重新补偿。 */
  ms5837.status &= ~(MS5837_STATUS_PRESSURE_VALID | MS5837_STATUS_TEMPERATURE_VALID |
                     MS5837_STATUS_DEPTH_VALID);
  ms5837.sample.pressure_pa = ms5837_nan();
  ms5837.sample.temperature_c = ms5837_nan();
  ms5837.sample.depth_raw_m = ms5837_nan();
  ms5837.sample.depth_filtered_m = ms5837_nan();
  ms5837.filter_valid = 0U;
  ms5837.sample.status = ms5837.status;
  return MS5837_OK;
}

uint8_t Ms5837_GetModel(void)
{
  return ms5837.config.model;
}

Ms5837Result_t Ms5837_SetOsr(uint16_t osr)
{
  if (ms5837_osr_to_index(osr) == 0xFFU)
  {
    return MS5837_ERR_PARAM;
  }
  if (ms5837_schedule_fits(ms5837.config.model, osr, ms5837.config.output_rate_hz) == 0U)
  {
    return MS5837_ERR_PARAM;
  }
  ms5837.config.osr = osr;
  ms5837.stats.osr = osr;
  return MS5837_OK;
}

uint16_t Ms5837_GetOsr(void)
{
  return ms5837.config.osr;
}

Ms5837Result_t Ms5837_SetOutputRateHz(uint16_t rate_hz)
{
  if ((rate_hz < MS5837_OUTPUT_RATE_HZ_MIN) || (rate_hz > MS5837_OUTPUT_RATE_HZ_MAX))
  {
    return MS5837_ERR_PARAM;
  }
  if (ms5837_schedule_fits(ms5837.config.model, ms5837.config.osr, rate_hz) == 0U)
  {
    return MS5837_ERR_PARAM;
  }
  ms5837.config.output_rate_hz = rate_hz;
  ms5837.stats.output_rate_hz = rate_hz;
  return MS5837_OK;
}

uint16_t Ms5837_GetOutputRateHz(void)
{
  return ms5837.config.output_rate_hz;
}

Ms5837Result_t Ms5837_SetWaterDensity(float kg_m3)
{
  if (MS5837_IS_NAN(kg_m3) || (kg_m3 < MS5837_WATER_DENSITY_MIN) ||
      (kg_m3 > MS5837_WATER_DENSITY_MAX))
  {
    return MS5837_ERR_PARAM;
  }
  ms5837.config.water_density = kg_m3;
  if (ms5837.sample_ready != 0U)
  {
    ms5837_apply_depth();
    ms5837.sample.status = ms5837.status;
  }
  return MS5837_OK;
}

float Ms5837_GetWaterDensity(void)
{
  return ms5837.config.water_density;
}

Ms5837Result_t Ms5837_SetSurfacePressurePa(float pa)
{
  if (MS5837_IS_NAN(pa) || (pa < MS5837_SURFACE_PRESSURE_MIN) ||
      (pa > MS5837_SURFACE_PRESSURE_MAX))
  {
    return MS5837_ERR_PARAM;
  }
  ms5837.config.surface_pressure_pa = pa;
  ms5837.zero_valid = 1U;
  ms5837.status |= MS5837_STATUS_ZERO_VALID;
  ms5837.filter_valid = 0U;
  if (ms5837.sample_ready != 0U)
  {
    ms5837_apply_depth();
  }
  ms5837.sample.status = ms5837.status;
  return MS5837_OK;
}

Ms5837Result_t Ms5837_GetSurfacePressurePa(float *pa)
{
  if (pa == 0)
  {
    return MS5837_ERR_PARAM;
  }
  if (ms5837.zero_valid == 0U)
  {
    return MS5837_ERR_NO_ZERO;
  }
  *pa = ms5837.config.surface_pressure_pa;
  return MS5837_OK;
}

Ms5837Result_t Ms5837_SetFilterK(float k)
{
  if (MS5837_IS_NAN(k) || (k < 0.0f) || (k > MS5837_FILTER_K_MAX))
  {
    return MS5837_ERR_PARAM;
  }
  ms5837.config.filter_k = k;
  return MS5837_OK;
}

float Ms5837_GetFilterK(void)
{
  return ms5837.config.filter_k;
}

static uint16_t ms5837_read_le_u16(const uint8_t *bytes)
{
  return (uint16_t)((uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8));
}

static float ms5837_read_le_f32(const uint8_t *bytes)
{
  uint32_t bits = (uint32_t)bytes[0] | ((uint32_t)bytes[1] << 8) |
                  ((uint32_t)bytes[2] << 16) | ((uint32_t)bytes[3] << 24);
  float value;
  memcpy(&value, &bits, sizeof(value));
  return value;
}

Ms5837Result_t Ms5837_SetParam(uint16_t param_id,
                               uint8_t type,
                               const void *value,
                               uint8_t length)
{
  const uint8_t *bytes = (const uint8_t *)value;

  if ((bytes == 0) || (length == 0U))
  {
    return MS5837_ERR_PARAM;
  }

  switch (param_id)
  {
    case MS5837_PARAM_OUTPUT_RATE_HZ:
      if ((type != MS5837_PARAM_TYPE_U16) || (length != 2U))
      {
        return MS5837_ERR_PARAM;
      }
      return Ms5837_SetOutputRateHz(ms5837_read_le_u16(bytes));

    case MS5837_PARAM_DEPTH_OSR:
      if ((type != MS5837_PARAM_TYPE_U16) || (length != 2U))
      {
        return MS5837_ERR_PARAM;
      }
      return Ms5837_SetOsr(ms5837_read_le_u16(bytes));

    case MS5837_PARAM_WATER_DENSITY:
      if ((type != MS5837_PARAM_TYPE_F32) || (length != 4U))
      {
        return MS5837_ERR_PARAM;
      }
      return Ms5837_SetWaterDensity(ms5837_read_le_f32(bytes));

    case MS5837_PARAM_SURFACE_PRESSURE:
      if ((type != MS5837_PARAM_TYPE_F32) || (length != 4U))
      {
        return MS5837_ERR_PARAM;
      }
      return Ms5837_SetSurfacePressurePa(ms5837_read_le_f32(bytes));

    case MS5837_PARAM_FILTER_K:
      if ((type != MS5837_PARAM_TYPE_F32) || (length != 4U))
      {
        return MS5837_ERR_PARAM;
      }
      return Ms5837_SetFilterK(ms5837_read_le_f32(bytes));

    case MS5837_PARAM_DEPTH_MODEL:
      if ((type != MS5837_PARAM_TYPE_U8) || (length != 1U))
      {
        return MS5837_ERR_PARAM;
      }
      return Ms5837_SetModel(bytes[0]);

    default:
      return MS5837_ERR_UNSUPPORTED;
  }
}

Ms5837Result_t Ms5837_GetParam(uint16_t param_id,
                               uint8_t *type,
                               uint8_t *length,
                               uint8_t value[4])
{
  if ((type == 0) || (length == 0) || (value == 0))
  {
    return MS5837_ERR_PARAM;
  }

  memset(value, 0, 4U);

  switch (param_id)
  {
    case MS5837_PARAM_OUTPUT_RATE_HZ:
    {
      uint16_t rate = ms5837.config.output_rate_hz;
      *type = MS5837_PARAM_TYPE_U16;
      *length = 2U;
      value[0] = (uint8_t)(rate & 0xFFU);
      value[1] = (uint8_t)(rate >> 8);
      return MS5837_OK;
    }

    case MS5837_PARAM_DEPTH_OSR:
    {
      uint16_t osr = ms5837.config.osr;
      *type = MS5837_PARAM_TYPE_U16;
      *length = 2U;
      value[0] = (uint8_t)(osr & 0xFFU);
      value[1] = (uint8_t)(osr >> 8);
      return MS5837_OK;
    }

    case MS5837_PARAM_WATER_DENSITY:
    case MS5837_PARAM_SURFACE_PRESSURE:
    case MS5837_PARAM_FILTER_K:
    {
      float number;
      uint32_t bits;
      if (param_id == MS5837_PARAM_WATER_DENSITY)
      {
        number = ms5837.config.water_density;
      }
      else if (param_id == MS5837_PARAM_SURFACE_PRESSURE)
      {
        if (ms5837.zero_valid == 0U)
        {
          return MS5837_ERR_NO_ZERO;
        }
        number = ms5837.config.surface_pressure_pa;
      }
      else
      {
        number = ms5837.config.filter_k;
      }
      memcpy(&bits, &number, sizeof(bits));
      *type = MS5837_PARAM_TYPE_F32;
      *length = 4U;
      value[0] = (uint8_t)(bits & 0xFFU);
      value[1] = (uint8_t)((bits >> 8) & 0xFFU);
      value[2] = (uint8_t)((bits >> 16) & 0xFFU);
      value[3] = (uint8_t)((bits >> 24) & 0xFFU);
      return MS5837_OK;
    }

    case MS5837_PARAM_DEPTH_MODEL:
      *type = MS5837_PARAM_TYPE_U8;
      *length = 1U;
      value[0] = ms5837.config.model;
      return MS5837_OK;

    default:
      return MS5837_ERR_UNSUPPORTED;
  }
}
