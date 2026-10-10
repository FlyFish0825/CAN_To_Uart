/*
 * MS5837 深度计驱动（含 I2C 事务层）—— 单文件实现：ms5837.h + ms5837.c。
 *
 * 结构：
 *  1) I2C 事务层（中断异步）：原 sensor_i2c_bus.c 的全部内容；外部只需要 I2c_Init()，
 *     提交/轮询/回调/中断服务都在本文件内部（static），不对外暴露。
 *  2) 深度计驱动：PROM+CRC4、固定 02BA 补偿、非阻塞状态机、显式零点、参数与统计。
 */
#include "ms5837.h"

/* ================================================================ 1) I2C 事务层 */
/* 异步事务阶段（原 sensor_i2c_bus.h）；BUSY 期间不得复用 tx/rx 缓冲。 */
typedef enum
{
  I2C_BUS_PHASE_IDLE = 0, /* 总线空闲，可提交新事务。 */
  I2C_BUS_PHASE_BUSY, /* 事务在飞，等中断完成。 */
  I2C_BUS_PHASE_DONE /* 事务已结束，结果待取。 */
} I2cBusPhase_t;


static I2C_HandleTypeDef *i2c_bus_handle = 0; /* 绑定的 HAL I2C 句柄。 */
static uint8_t i2c_bus_device7 = I2C_BUS_MS5837_ADDRESS7; /* 当前 7 位从地址。 */
static uint8_t i2c_bus_ready = 0U; /* 句柄是否已绑定。 */

/* ---------------------------------------------------------------- 中断模式状态 */
static volatile I2cBusPhase_t i2c_phase = I2C_BUS_PHASE_IDLE;
static volatile I2cBusResult_t i2c_result = I2C_BUS_OK;
static uint32_t i2c_deadline_ms = 0U; /* 本笔事务的软件超时时刻。 */
static uint8_t i2c_abort_sent = 0U; /* 是否已对该笔事务发过 Abort。 */
static uint8_t i2c_have_tx = 0U; /* 本笔事务是否包含写阶段。 */
static uint8_t i2c_have_rx = 0U; /* 本笔事务是否包含读阶段。 */
static uint8_t i2c_tx_done = 0U; /* 写阶段是否已完成（用于先写后读）。 */
static uint8_t *i2c_rx_ptr = 0;
static uint16_t i2c_rx_len = 0U;

/* 把调用方给出的超时收敛到“短超时”区间。 */
static uint32_t i2c_bus_clamp_timeout(uint32_t timeout_ms)
{
  if (timeout_ms == 0U)
  {
    return I2C_BUS_DEFAULT_TIMEOUT_MS;
  }
  if (timeout_ms > I2C_BUS_MAX_TIMEOUT_MS)
  {
    return I2C_BUS_MAX_TIMEOUT_MS;
  }
  return timeout_ms;
}

static I2cBusResult_t i2c_bus_map_status(HAL_StatusTypeDef status)
{
  switch (status)
  {
    case HAL_OK:
      return I2C_BUS_OK;
    case HAL_BUSY:
      return I2C_BUS_BUSY;
    case HAL_TIMEOUT:
      return I2C_BUS_TIMEOUT;
    default:
      return I2C_BUS_ERROR;
  }
}

/* 结束一笔事务：置结果并进入 DONE，等调用方取走。 */
static void i2c_bus_finish(I2cBusResult_t result)
{
  i2c_result = result;
  i2c_phase = I2C_BUS_PHASE_DONE;
  i2c_have_tx = 0U;
  i2c_have_rx = 0U;
  i2c_tx_done = 0U;
  i2c_rx_ptr = 0;
  i2c_rx_len = 0U;
  i2c_abort_sent = 0U;
}

/* 启动读阶段（先写后读的第二段，或纯读）。 */
static HAL_StatusTypeDef i2c_bus_start_rx(void)
{
  return HAL_I2C_Master_Receive_IT(i2c_bus_handle,
                                   (uint16_t)((uint16_t)i2c_bus_device7 << 1),
                                   i2c_rx_ptr,
                                   i2c_rx_len);
}

/* ---------------------------------------------------------------- HAL 回调 */
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c != i2c_bus_handle)
  {
    return;
  }
  i2c_tx_done = 1U;
  if (i2c_have_rx != 0U)
  {
    /* 写阶段完成，紧接着发起读阶段。 */
    if (i2c_bus_start_rx() != HAL_OK)
    {
      i2c_bus_finish(I2C_BUS_ERROR);
    }
    return;
  }
  i2c_bus_finish(I2C_BUS_OK);
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c != i2c_bus_handle)
  {
    return;
  }
  i2c_bus_finish(I2C_BUS_OK);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c != i2c_bus_handle)
  {
    return;
  }
  /* HAL 的错误标志：NACK/仲裁丢失/总线错误都算 IO 错误；只有 HAL 报超时才回 TIMEOUT。 */
  if ((HAL_I2C_GetError(hi2c) & 0x00000020U) != 0U) /* HAL_I2C_ERROR_TIMEOUT */
  {
    i2c_bus_finish(I2C_BUS_TIMEOUT);
  }
  else
  {
    i2c_bus_finish(I2C_BUS_ERROR);
  }
}

void HAL_I2C_AbortCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c != i2c_bus_handle)
  {
    return;
  }
  i2c_bus_finish(I2C_BUS_TIMEOUT);
}

/* ---------------------------------------------------------------- 中断转发 */
void I2c_EvIrqHandler(void)
{
  if (i2c_bus_handle != 0)
  {
    HAL_I2C_EV_IRQHandler(i2c_bus_handle);
  }
}

void I2c_ErIrqHandler(void)
{
  if (i2c_bus_handle != 0)
  {
    HAL_I2C_ER_IRQHandler(i2c_bus_handle);
  }
}

#if (I2C_BUS_DEFINE_IRQ_HANDLERS == 1)
/*
 * 集成方目前没有 CubeMX 生成的 i2c.c，这两个向量表入口由本文件提供。
 * 若以后生成，请把 I2C_BUS_DEFINE_IRQ_HANDLERS 置 0 改成调用 I2c_EvIrqHandler/I2c_ErIrqHandler。
 * 这里只处理本文件绑定的那个 I2C 实例，其它实例（若存在）直接忽略。
 */
void I2C3_EV_IRQHandler(void)
{
  I2c_EvIrqHandler();
}

void I2C3_ER_IRQHandler(void)
{
  I2c_ErIrqHandler();
}
#endif

/* ---------------------------------------------------------------- 基础接口 */
void I2c_Init(I2C_HandleTypeDef *handle)
{
  i2c_bus_handle = handle;
  i2c_bus_ready = (handle != 0) ? 1U : 0U;
  i2c_phase = I2C_BUS_PHASE_IDLE;
  i2c_result = I2C_BUS_OK;
  i2c_abort_sent = 0U;
  i2c_have_tx = 0U;
  i2c_have_rx = 0U;
  i2c_tx_done = 0U;
  i2c_rx_ptr = 0;
  i2c_rx_len = 0U;

#if (I2C_BUS_ENABLE_NVIC == 1)
  if (handle != 0)
  {
    /* 按实例选择中断号：不同 I2C 的 EV/ER 向量是分开的。 */
    if (handle->Instance == I2C1)
    {
      HAL_NVIC_SetPriority(I2C1_EV_IRQn, I2C_BUS_IRQ_PRIORITY, 0U);
      HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
      HAL_NVIC_SetPriority(I2C1_ER_IRQn, I2C_BUS_IRQ_PRIORITY, 0U);
      HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
    }
    else if (handle->Instance == I2C2)
    {
      HAL_NVIC_SetPriority(I2C2_EV_IRQn, I2C_BUS_IRQ_PRIORITY, 0U);
      HAL_NVIC_EnableIRQ(I2C2_EV_IRQn);
      HAL_NVIC_SetPriority(I2C2_ER_IRQn, I2C_BUS_IRQ_PRIORITY, 0U);
      HAL_NVIC_EnableIRQ(I2C2_ER_IRQn);
    }
    else if (handle->Instance == I2C3)
    {
      HAL_NVIC_SetPriority(I2C3_EV_IRQn, I2C_BUS_IRQ_PRIORITY, 0U);
      HAL_NVIC_EnableIRQ(I2C3_EV_IRQn);
      HAL_NVIC_SetPriority(I2C3_ER_IRQn, I2C_BUS_IRQ_PRIORITY, 0U);
      HAL_NVIC_EnableIRQ(I2C3_ER_IRQn);
    }
    else if (handle->Instance == I2C4)
    {
      HAL_NVIC_SetPriority(I2C4_EV_IRQn, I2C_BUS_IRQ_PRIORITY, 0U);
      HAL_NVIC_EnableIRQ(I2C4_EV_IRQn);
      HAL_NVIC_SetPriority(I2C4_ER_IRQn, I2C_BUS_IRQ_PRIORITY, 0U);
      HAL_NVIC_EnableIRQ(I2C4_ER_IRQn);
    }
    else
    {
      /* 未知实例：不使能中断（此时异步路径不可用，调用方会看到 BUSY/超时）。 */
    }
  }
#endif
}

uint8_t I2c_IsReady(void)
{
  return i2c_bus_ready;
}

I2cBusResult_t I2c_SetDevice(uint8_t address7)
{
  if ((address7 < 0x08U) || (address7 > 0x77U))
  {
    return I2C_BUS_PARAM;
  }
  i2c_bus_device7 = address7;
  return I2C_BUS_OK;
}

uint8_t I2c_GetDevice(void)
{
  return i2c_bus_device7;
}

/* ---------------------------------------------------------------- 中断模式实现 */
static I2cBusResult_t I2c_Submit(const uint8_t *tx,
                          uint16_t tx_length,
                          uint8_t *rx,
                          uint16_t rx_length,
                          uint32_t timeout_ms)
{
  uint8_t have_tx;
  uint8_t have_rx;
  HAL_StatusTypeDef status;

  if ((i2c_bus_ready == 0U) || (i2c_bus_handle == 0))
  {
    return I2C_BUS_NOT_READY;
  }
  if (i2c_phase == I2C_BUS_PHASE_BUSY)
  {
    return I2C_BUS_BUSY;
  }
  if (i2c_phase == I2C_BUS_PHASE_DONE)
  {
    /* 上一笔结果还没被取走：不允许直接覆盖。 */
    return I2C_BUS_BUSY;
  }

  have_tx = ((tx != 0) && (tx_length > 0U)) ? 1U : 0U;
  have_rx = ((rx != 0) && (rx_length > 0U)) ? 1U : 0U;
  if ((have_tx == 0U) && (have_rx == 0U))
  {
    return I2C_BUS_PARAM;
  }
  if ((tx_length > I2C_BUS_MAX_TRANSFER) || (rx_length > I2C_BUS_MAX_TRANSFER))
  {
    return I2C_BUS_PARAM;
  }
  if ((have_tx != 0U) && (tx == 0))
  {
    return I2C_BUS_PARAM;
  }
  if ((have_rx != 0U) && (rx == 0))
  {
    return I2C_BUS_PARAM;
  }

  i2c_have_tx = have_tx;
  i2c_have_rx = have_rx;
  i2c_tx_done = 0U;
  i2c_rx_ptr = rx;
  i2c_rx_len = rx_length;
  i2c_abort_sent = 0U;
  i2c_result = I2C_BUS_OK;
  i2c_deadline_ms = HAL_GetTick() + i2c_bus_clamp_timeout(timeout_ms);
  i2c_phase = I2C_BUS_PHASE_BUSY;

  if (have_tx != 0U)
  {
    status = HAL_I2C_Master_Transmit_IT(i2c_bus_handle,
                                        (uint16_t)((uint16_t)i2c_bus_device7 << 1),
                                        (uint8_t *)tx,
                                        tx_length);
  }
  else
  {
    status = i2c_bus_start_rx();
  }

  if (status != HAL_OK)
  {
    /*
     * HAL 在启动阶段就拒绝了（BUSY/参数/句柄问题）：不会再有完成回调，
     * 因此直接把状态收回 IDLE，不留下必须被取走的 DONE 状态，否则后续每次提交
     * 都会被“上一笔结果没取走”挡掉。
     */
    I2cBusResult_t mapped = i2c_bus_map_status(status);

    i2c_bus_finish(mapped);
    i2c_phase = I2C_BUS_PHASE_IDLE;
    return mapped;
  }
  return I2C_BUS_OK;
}

static I2cBusPhase_t I2c_GetPhase(void)
{
  return i2c_phase;
}

static I2cBusResult_t I2c_GetResult(void)
{
  I2cBusResult_t result;

  if (i2c_phase != I2C_BUS_PHASE_DONE)
  {
    return I2C_BUS_BUSY;
  }
  result = i2c_result;
  i2c_phase = I2C_BUS_PHASE_IDLE;
  return result;
}

static void I2c_Process(void)
{
  if (i2c_phase != I2C_BUS_PHASE_BUSY)
  {
    return;
  }
  if (i2c_abort_sent != 0U)
  {
    /* 已经请求过中止，等 Abort 完成回调；不再重复请求。 */
    return;
  }
  if ((int32_t)(HAL_GetTick() - i2c_deadline_ms) < 0)
  {
    return;
  }

  /*
   * 软件超时：请求 HAL 中止这笔事务。中止完成后 AbortCpltCallback 会把结果置为
   * TIMEOUT，主循环因此不会被卡死（这正是改成中断模式的主要目的：
   * 传感器进水/短接把 SCL 拉低时，阻塞式最多会把主循环卡住 5~10 ms）。
   */
  i2c_abort_sent = 1U;
  if (HAL_I2C_Master_Abort_IT(i2c_bus_handle,
                              (uint16_t)((uint16_t)i2c_bus_device7 << 1)) != HAL_OK)
  {
    i2c_bus_finish(I2C_BUS_TIMEOUT);
  }
}

/* ================================================================ 2) 深度计驱动 */
#include <string.h>


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
#define MS5837_ADC_MAX             0x00FFFFFFU /* 24 位 ADC 满量程码字。 */

/*
 * 转换等待的保守化：
 *  - deadline 必须在 I2C 命令真正发完之后重新取 HAL_GetTick()，否则命令传输占用的时间
 *    会让等待变短（毫秒相位提前，可能读到还没完成的转换）；
 *  - 再额外加 1 ms 余量，保证跨过毫秒边界时也不会提前读；
 *  - 一个采样周期至少容纳 2 次这样的保守等待，外加 4 次短事务的预算。
 */
#define MS5837_CONVERSION_MARGIN_MS 1U /* 命令完成后额外等待的毫秒数。 */
#define MS5837_SCHEDULE_TX_BUDGET_MS 2U /* 一帧内 4 次短 I2C 事务的预算。 */

/* ---------------------------------------------------------------- 状态机 */
typedef enum
{
  MS5837_STATE_OFFLINE = 0, /* 未初始化或初始化失败，等待重试。 */
  MS5837_STATE_RESET_WAIT, /* 已发复位命令，等待复位延时。 */
  MS5837_STATE_PROM_READ, /* 逐字读取 PROM。 */
  MS5837_STATE_IDLE, /* 等待下一次采样周期。 */
  MS5837_STATE_CONVERT_D1, /* D1 转换进行中（非阻塞等待）。 */
  MS5837_STATE_CONVERT_D2, /* D2 转换进行中（非阻塞等待）。 */
  MS5837_STATE_DISCARD_WAIT, /* 配置已变：等旧转换安全结束后再按新配置重新开始。 */
  MS5837_STATE_BUS_WAIT /* 已提交一笔 I2C 事务，等中断回调完成（主循环零等待）。 */
} Ms5837State_t;

/*
 * 在飞事务完成后要执行的动作。异步后“提交”和“完成”被拆开，
 * 因此必须显式记住这笔事务是干什么的。
 */
typedef enum
{
  MS5837_PENDING_NONE = 0,
  MS5837_PENDING_RESET, /* 复位命令完成 → 等复位延时 */
  MS5837_PENDING_PROM_WORD, /* PROM 某字读完 → 累计/校验 */
  MS5837_PENDING_D1_CMD, /* D1 转换命令发完 → 开始算转换等待 */
  MS5837_PENDING_D1_ADC, /* D1 结果读完 → 校验并下发 D2 命令 */
  MS5837_PENDING_D2_CMD, /* D2 转换命令发完 → 开始算转换等待 */
  MS5837_PENDING_D2_ADC /* D2 结果读完 → 完成本帧样本 */
} Ms5837Pending_t;

/* 一次异步提交的结果。 */
typedef enum
{
  MS5837_SUBMIT_STARTED = 0, /* 事务已启动，状态机会进入 BUS_WAIT。 */
  MS5837_SUBMIT_RETRY, /* 总线正忙，本次未提交，下一拍重试（不阻塞）。 */
  MS5837_SUBMIT_FAILED /* 提交即失败，错误已记录，调用方直接返回。 */
} Ms5837Submit_t;

typedef struct
{
  uint16_t osr; /* 过采样率。 */
  uint16_t output_rate_hz; /* 采样率。 */
  float water_density; /* 水体密度 kg/m3。 */
  float surface_pressure_pa; /* 固定空气参考或显式零点压力；zero_valid 为唯一有效性依据。 */
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
  uint32_t cycle_start_ms; /* 本采样周期发出 D1 命令的时刻（周期调度的锚点）。 */
  uint32_t status; /* 实时状态字。 */
  uint32_t d1_raw; /* 本次周期的 D1。 */
  uint32_t d2_raw; /* 本次周期的 D2。 */
  float depth_filtered_m; /* 滤波深度状态。 */
  uint16_t prom[MS5837_PROM_WORDS]; /* PROM 原始字。 */
  Ms5837Config_t config; /* 本地参数。 */
  Ms5837Sample_t sample; /* 最近一次样本。 */
  Ms5837Stats_t stats; /* 运行统计。 */
  /* --- 异步（中断）事务状态：这些缓冲必须在事务在飞期间保持有效 --- */
  uint8_t busy_command; /* 在飞事务的命令字节。 */
  uint8_t pending_action; /* 在飞事务完成后要执行的动作（Ms5837Pending_t）。 */
  uint8_t inflight_osr_index; /* 在飞转换命令使用的 OSR 下标。 */
  uint8_t adc_bytes[MS5837_ADC_BYTES]; /* ADC 读缓冲（D1/D2 复用，完成即解析）。 */
  uint8_t prom_bytes[MS5837_PROM_BYTES]; /* PROM 单字读缓冲。 */
} Ms5837Sensor_t;

/* 单实例驱动状态。探头固定为 02BA；上电沿用历史空气基准 P0，其余为 OSR4096、25 Hz、海水密度、不做滤波。 */
static Ms5837Sensor_t ms5837 =
{
  .state = MS5837_STATE_OFFLINE,
  .init_requested = 0U,
  .prom_valid = 0U,
  .zero_valid = 1U,
  .sample_ready = 0U,
  .new_sample = 0U,
  .filter_valid = 0U,
  .prom_index = 0U,
  .deadline_ms = 0U,
  .cycle_start_ms = 0U,
  .status = MS5837_STATUS_ZERO_VALID,
  .d1_raw = 0U,
  .d2_raw = 0U,
  .depth_filtered_m = 0.0f,
  .prom = {0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U},
  .config =
  {
    .osr = MS5837_OSR_DEFAULT,
    .output_rate_hz = MS5837_OUTPUT_RATE_HZ_DEFAULT,
    .water_density = MS5837_WATER_DENSITY_DEFAULT,
    .surface_pressure_pa = MS5837_AIR_REFERENCE_PRESSURE_PA,
    .filter_k = MS5837_FILTER_K_DEFAULT
  },
  .sample = {0},
  .stats =
  {
    .osr = MS5837_OSR_DEFAULT,
    .output_rate_hz = MS5837_OUTPUT_RATE_HZ_DEFAULT
  }
};

/* 本机固定 02BA 的最大转换时间（微秒）：下标对应 OSR 256/512/1024/2048/4096/8192。 */
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

static uint32_t ms5837_conversion_time_us(uint16_t osr)
{
  uint8_t index = ms5837_osr_to_index(osr);
  if (index == 0xFFU)
  {
    return 0U;
  }
  return (uint32_t)ms5837_conv_time_us_02ba[index];
}

/* 向上取整到毫秒：宁可多等 1 ms，也不提前读 ADC。 */
static uint32_t ms5837_conversion_time_ms(uint16_t osr)
{
  uint32_t us = ms5837_conversion_time_us(osr);
  return (us + 999U) / 1000U;
}

/* 一个 D1+D2 采样周期能否塞进当前采样率周期。 */
static uint8_t ms5837_schedule_fits(uint16_t osr, uint16_t rate_hz)
{
  uint32_t period_ms;
  uint32_t needed_ms;
  if ((rate_hz < MS5837_OUTPUT_RATE_HZ_MIN) || (rate_hz > MS5837_OUTPUT_RATE_HZ_MAX))
  {
    return 0U;
  }
  period_ms = 1000U / (uint32_t)rate_hz;
  /* 两次保守等待（每次含 1 ms 余量）+ 一帧内 4 次短事务的预算。 */
  needed_ms = (2U * (ms5837_conversion_time_ms(osr) + MS5837_CONVERSION_MARGIN_MS)) +
              MS5837_SCHEDULE_TX_BUDGET_MS;
  return (needed_ms <= period_ms) ? 1U : 0U;
}

/* 本次转换的等待时长：按“命令已发完”的时刻计算，并含 1 ms 余量。 */
static uint32_t ms5837_conversion_deadline(uint16_t osr, uint32_t command_done_ms)
{
  return command_done_ms + ms5837_conversion_time_ms(osr) + MS5837_CONVERSION_MARGIN_MS;
}

/* 24 位 ADC 结果是否可用：全 0 / 全 1 是典型“无有效转换”码字，不能当有效数据发布。 */
static uint8_t ms5837_adc_value_valid(uint32_t value)
{
  return ((value != 0U) && (value != MS5837_ADC_MAX)) ? 1U : 0U;
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

/* ---------------------------------------------------------------- I2C 访问（中断异步） */
/* 提交辅助函数比 record_error 先定义，这里前置声明（错误统计在提交失败时也要记账）。 */
static void ms5837_record_error(Ms5837Result_t error, uint32_t now);

/*
 * 提交后立即返回：真正的收发由 I2C 中断完成，主循环不再自旋等待。
 * 完成/出错由总线层回调落地，Ms5837_Process() 下一拍在 BUS_WAIT 分支里处理结果。
 */
static Ms5837Submit_t ms5837_submit_command(uint8_t command, uint8_t pending)
{
  I2cBusResult_t result;

  ms5837.busy_command = command;
  result = I2c_Submit(&ms5837.busy_command, 1U, 0, 0U, MS5837_I2C_TIMEOUT_MS);
  if (result == I2C_BUS_BUSY)
  {
    return MS5837_SUBMIT_RETRY; /* 总线正忙：下一拍再试，不阻塞。 */
  }
  if (result != I2C_BUS_OK)
  {
    ms5837_record_error(ms5837_map_i2c(result), HAL_GetTick());
    return MS5837_SUBMIT_FAILED;
  }
  ms5837.pending_action = pending;
  return MS5837_SUBMIT_STARTED;
}

static Ms5837Submit_t ms5837_submit_command_read(uint8_t command,
                                                 uint8_t *rx,
                                                 uint8_t length,
                                                 uint8_t pending)
{
  I2cBusResult_t result;

  ms5837.busy_command = command;
  result = I2c_Submit(&ms5837.busy_command, 1U, rx, length, MS5837_I2C_TIMEOUT_MS);
  if (result == I2C_BUS_BUSY)
  {
    return MS5837_SUBMIT_RETRY;
  }
  if (result != I2C_BUS_OK)
  {
    ms5837_record_error(ms5837_map_i2c(result), HAL_GetTick());
    return MS5837_SUBMIT_FAILED;
  }
  ms5837.pending_action = pending;
  return MS5837_SUBMIT_STARTED;
}

/* ADC 三字节（MSB first）→ 24 位原始值。 */
static uint32_t ms5837_adc_from_bytes(const uint8_t *bytes)
{
  return ((uint32_t)bytes[0] << 16) | ((uint32_t)bytes[1] << 8) | (uint32_t)bytes[2];
}

/* PROM 两字节（MSB first）→ 16 位字。 */
static uint16_t ms5837_word_from_bytes(const uint8_t *bytes)
{
  return (uint16_t)(((uint16_t)bytes[0] << 8) | (uint16_t)bytes[1]);
}

/* OSR 下标 → OSR 值（用于“事务在飞时配置被改”的保守等待计算）。 */
static uint16_t ms5837_osr_from_index(uint8_t index)
{
  if (index > 5U)
  {
    return MS5837_OSR_DEFAULT;
  }
  return ms5837_osr_table[index];
}

/* ---------------------------------------------------------------- 错误处理 */
static void ms5837_status_record_last_error(Ms5837Result_t error)
{
  const uint32_t diagnostic_mask = MS5837_STATUS_LAST_I2C_COMMAND_MASK |
                                   MS5837_STATUS_LAST_ERROR_MASK;
  const uint32_t command = (error == MS5837_ERR_BUSY || error == MS5837_ERR_TIMEOUT ||
                            error == MS5837_ERR_IO || error == MS5837_ERR_CRC ||
                            error == MS5837_ERR_NOT_READY)
                               ? (uint32_t)ms5837.busy_command : 0U;
  const uint32_t code = (uint32_t)error;
  ms5837.status = (ms5837.status & ~diagnostic_mask) |
                  (command << MS5837_STATUS_LAST_I2C_COMMAND_SHIFT) |
                  (code << MS5837_STATUS_LAST_ERROR_SHIFT);
}

static void ms5837_record_error(Ms5837Result_t error, uint32_t now)
{
  ms5837.stats.errors++;
  ms5837.stats.last_error = (uint32_t)error;
  ms5837_status_record_last_error(error);
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

/*
 * 转换结果无效（全 0 / 全 1）：总线本身没报错，所以保留 ONLINE，
 * 但必须清掉“新鲜数据”位并把已发布样本的测量字段置回 NaN，
 * 绝不把无效码字留下的旧值当成新测量值。
 */
static void ms5837_record_invalid_conversion(uint32_t now)
{
  ms5837.stats.errors++;
  ms5837.stats.last_error = (uint32_t)MS5837_ERR_NOT_READY;
  ms5837_status_record_last_error(MS5837_ERR_NOT_READY);
  ms5837.status &= ~(MS5837_STATUS_RAW_VALID | MS5837_STATUS_PRESSURE_VALID |
                     MS5837_STATUS_TEMPERATURE_VALID | MS5837_STATUS_DEPTH_VALID);
  ms5837.sample.pressure_raw = 0;
  ms5837.sample.temperature_centi_c = 0;
  ms5837.sample.pressure_pa = ms5837_nan();
  ms5837.sample.temperature_c = ms5837_nan();
  ms5837.sample.depth_raw_m = ms5837_nan();
  ms5837.sample.depth_filtered_m = ms5837_nan();
  ms5837.sample.status = ms5837.status;
  ms5837.filter_valid = 0U;
  ms5837.state = MS5837_STATE_IDLE;
  ms5837.deadline_ms = now + MS5837_RETRY_DELAY_MS;
}

/*
 * PROM 内容可信度检查：CRC 通过也可能碰上全 0 / 全 0xFFFF 的假 PROM
 * （典型的 I2C 卡死或空器件特征）。真实模块的 C1~C6 是工厂标定值，
 * 不可能 6 个字全 0 或全 0xFFFF。
 */
static uint8_t ms5837_prom_sane(const uint16_t prom[MS5837_PROM_WORDS])
{
  uint8_t index;
  uint8_t all_zero = 1U;
  uint8_t all_ones = 1U;

  for (index = 1U; index <= 6U; index++)
  {
    if (prom[index] != 0x0000U)
    {
      all_zero = 0U;
    }
    if (prom[index] != 0xFFFFU)
    {
      all_ones = 0U;
    }
  }
  return (uint8_t)((all_zero == 0U) && (all_ones == 0U));
}

/*
 * OSR / 型号变更时丢弃正在进行的半周期。
 *
 * 手册第 11 页明确：转换期间芯片一直 busy，
 *   - “Conversion sequence sent during the already started conversion process will yield incorrect result”，
 *   - “If the ADC read command is sent during conversion the result will be 0 ... the final result will be wrong”，
 * 而且新命令并不会取消正在进行的转换。
 *
 * 所以这里只丢弃“软件侧”的半周期：保留原 deadline（它是按旧配置算出的、不会更短的等待），
 * 把状态切到 DISCARD_WAIT；等芯片真正空闲后，再用新配置重新走一遍完整的 D1+D2。
 * 绝不在这里直接重发 D1/D2，也绝不缩短旧转换的等待。
 */
static void ms5837_abort_half_cycle(void)
{
  if ((ms5837.state == MS5837_STATE_CONVERT_D1) || (ms5837.state == MS5837_STATE_CONVERT_D2))
  {
    ms5837.d1_raw = 0U;
    ms5837.d2_raw = 0U;
    ms5837.state = MS5837_STATE_DISCARD_WAIT; /* deadline_ms 保持不动。 */
    return;
  }

  /*
   * 异步特例：转换命令“已经提交、还在飞”时配置变了。
   * 命令一旦发到芯片上，转换就已经开始，无法取消；此时必须按“已发出的那条命令”
   * 保守地把等待时间补足（未知型号取两者较大值），绝不能让 DISCARD_WAIT 沿用一个
   * 可能更短的旧 deadline 而提前读 ADC（手册第 11 页）。
   */
  if (ms5837.state == MS5837_STATE_BUS_WAIT)
  {
    if ((ms5837.pending_action == MS5837_PENDING_D1_CMD) ||
        (ms5837.pending_action == MS5837_PENDING_D2_CMD))
    {
      ms5837.d1_raw = 0U;
      ms5837.d2_raw = 0U;
      ms5837.state = MS5837_STATE_DISCARD_WAIT;
      ms5837.deadline_ms = ms5837_conversion_deadline(
          ms5837_osr_from_index(ms5837.inflight_osr_index), HAL_GetTick());
    }
    else if ((ms5837.pending_action == MS5837_PENDING_D1_ADC) ||
             (ms5837.pending_action == MS5837_PENDING_D2_ADC))
    {
      /*
       * 结果读取在飞：芯片早已完成转换（我们是等够了才读的），所以不需要再等，
       * 直接丢弃这半截并立即用新配置重新开始，避免 D1/D2 跨配置配对。
       */
      ms5837.d1_raw = 0U;
      ms5837.d2_raw = 0U;
      ms5837.state = MS5837_STATE_DISCARD_WAIT;
      ms5837.deadline_ms = HAL_GetTick();
    }
    else
    {
      /* PROM 字读取 / 复位命令在飞：与型号/OSR 无关，等它正常完成。 */
    }
  }
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

uint16_t Ms5837_MaxConversionTimeMs(uint16_t osr)
{
  return (uint16_t)ms5837_conversion_time_ms(osr);
}

uint8_t Ms5837_Compensate(const uint16_t prom[MS5837_PROM_WORDS],
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

  if (prom == 0)
  {
    return 0U;
  }
  /* 输入必须是 24 位 ADC 码字；超出范围或等于全 0/全 1 的一律不补偿、不写输出。 */
  if ((d1 > MS5837_ADC_MAX) || (d2 > MS5837_ADC_MAX))
  {
    return 0U;
  }
  if ((ms5837_adc_value_valid(d1) == 0U) || (ms5837_adc_value_valid(d2) == 0U))
  {
    return 0U;
  }

  /* 一阶：dT / TEMP / OFF / SENS。TEMP 向负无穷取整，与数据手册 19.81 °C 算例一致。 */
  dt = (int64_t)d2 - ((int64_t)prom[5] << 8);
  temp = 2000 + ms5837_floor_div_pow2(dt * (int64_t)prom[6], 23);

  off = ((int64_t)prom[2] << 17) + ms5837_floor_div_pow2((int64_t)prom[4] * dt, 6);
  sens = ((int64_t)prom[1] << 16) + ms5837_floor_div_pow2((int64_t)prom[3] * dt, 7);

  /* 二阶温度补偿。比较用 0.01 °C 整数：TEMP/100 < 20 °C 等价 TEMP < 2000。 */
  if (temp < 2000)
  {
    ti = ms5837_floor_div_pow2(11 * dt * dt, 35);
    off_i = ms5837_floor_div_pow2(31 * (temp - 2000) * (temp - 2000), 3);
    sens_i = ms5837_floor_div_pow2(63 * (temp - 2000) * (temp - 2000), 5);
  }

  off2 = off - off_i;
  sens2 = sens - sens_i;
  temp -= ti;

  /* 02BA 压力单位为 0.01 mbar，正好 1 LSB = 1 Pa。 */
  pressure = ms5837_floor_div_pow2(ms5837_floor_div_pow2((int64_t)d1 * sens2, 21) - off2, 15);

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

  if (Ms5837_Compensate(ms5837.prom, ms5837.d1_raw, ms5837.d2_raw,
                        &pressure_raw, &temperature_centi_c) != 0U)
  {
    ms5837.sample.pressure_raw = pressure_raw;
    ms5837.sample.temperature_centi_c = temperature_centi_c;
    /* 本机固定 02BA：0.01 mbar 每 LSB，pressure_raw 单位正好是 Pa。 */
    ms5837.sample.pressure_pa = (float)pressure_raw;
    ms5837.sample.temperature_c = (float)temperature_centi_c / 100.0f;
    ms5837.status |= (MS5837_STATUS_PRESSURE_VALID | MS5837_STATUS_TEMPERATURE_VALID);
  }
  else
  {
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
/* 周期起点：提交 D1 转换命令（异步，不等它发完）。 */
static void ms5837_start_cycle(uint32_t now)
{
  uint8_t osr_index = ms5837_osr_to_index(ms5837.config.osr);

  if (osr_index == 0xFFU)
  {
    ms5837_record_error(MS5837_ERR_PARAM, now);
    return;
  }
  /* 周期锚点：下一次采样基于“发起命令的时刻” + 周期，而不是“转换结束时刻 + 周期”。 */
  ms5837.cycle_start_ms = now;
  ms5837.inflight_osr_index = osr_index;
  switch (ms5837_submit_command((uint8_t)(MS5837_CMD_CONVERT_D1_BASE + (uint8_t)(osr_index << 1)),
                               MS5837_PENDING_D1_CMD))
  {
    case MS5837_SUBMIT_STARTED:
      ms5837.state = MS5837_STATE_BUS_WAIT;
      break;
    case MS5837_SUBMIT_RETRY:
      /* 总线忙：保持 IDLE 与当前 deadline，下一拍再试（不阻塞主循环）。 */
      break;
    default:
      break; /* 错误已在提交处记录。 */
  }
}

/* 事务完成：D1 转换命令已发出 → 开始非阻塞的转换等待。 */
static void ms5837_done_d1_command(void)
{
  /* 以命令“发完”的时刻为基准，再加数据手册最大转换时间 + 1 ms 余量。 */
  ms5837.state = MS5837_STATE_CONVERT_D1;
  ms5837.deadline_ms = ms5837_conversion_deadline(ms5837.config.osr,
                                                  HAL_GetTick());
}

/* 提交 D1 结果读取（命令 + 3 字节）。 */
static void ms5837_begin_d1_adc(void)
{
  switch (ms5837_submit_command_read(MS5837_CMD_ADC_READ, ms5837.adc_bytes, MS5837_ADC_BYTES,
                                     MS5837_PENDING_D1_ADC))
  {
    case MS5837_SUBMIT_STARTED:
      ms5837.state = MS5837_STATE_BUS_WAIT;
      break;
    case MS5837_SUBMIT_RETRY:
      break; /* 下一拍重试：状态仍是 CONVERT_D1，deadline 已到，不会提前读。 */
    default:
      break;
  }
}

/* 事务完成：D1 结果已读到 → 校验后下发 D2 命令。 */
static void ms5837_done_d1_adc(uint32_t now)
{
  uint8_t osr_index = ms5837_osr_to_index(ms5837.config.osr);

  ms5837.d1_raw = ms5837_adc_from_bytes(ms5837.adc_bytes);
  if (ms5837_adc_value_valid(ms5837.d1_raw) == 0U)
  {
    /* 全 0 / 全 1：转换无效，不发起 D2，也不发布任何数据。 */
    ms5837_record_invalid_conversion(HAL_GetTick());
    return;
  }
  if (osr_index == 0xFFU)
  {
    ms5837_record_error(MS5837_ERR_PARAM, now);
    return;
  }
  ms5837.inflight_osr_index = osr_index;
  switch (ms5837_submit_command((uint8_t)(MS5837_CMD_CONVERT_D2_BASE + (uint8_t)(osr_index << 1)),
                               MS5837_PENDING_D2_CMD))
  {
    case MS5837_SUBMIT_STARTED:
      ms5837.state = MS5837_STATE_BUS_WAIT;
      break;
    case MS5837_SUBMIT_RETRY:
      /*
       * 总线忙（正常单实例下只在别处也占用同一总线时出现）：
       * 丢掉这半截、下一拍用新周期重新开始，避免 D1/D2 跨周期配对。
       */
      ms5837.state = MS5837_STATE_IDLE;
      ms5837.deadline_ms = now;
      break;
    default:
      break;
  }
}

/* 事务完成：D2 转换命令已发出 → 开始非阻塞的转换等待。 */
static void ms5837_done_d2_command(void)
{
  ms5837.state = MS5837_STATE_CONVERT_D2;
  ms5837.deadline_ms = ms5837_conversion_deadline(ms5837.config.osr,
                                                  HAL_GetTick());
}

/* 提交 D2 结果读取（命令 + 3 字节）。 */
static void ms5837_begin_d2_adc(void)
{
  switch (ms5837_submit_command_read(MS5837_CMD_ADC_READ, ms5837.adc_bytes, MS5837_ADC_BYTES,
                                     MS5837_PENDING_D2_ADC))
  {
    case MS5837_SUBMIT_STARTED:
      ms5837.state = MS5837_STATE_BUS_WAIT;
      break;
    case MS5837_SUBMIT_RETRY:
      break;
    default:
      break;
  }
}

/* 事务完成：D2 结果已读到 → 完成本帧样本并安排下一周期。 */
static void ms5837_done_d2_adc(void)
{
  uint32_t period_ms;
  uint32_t completed_ms;
  uint32_t next_start_ms;

  ms5837.d2_raw = ms5837_adc_from_bytes(ms5837.adc_bytes);
  if (ms5837_adc_value_valid(ms5837.d2_raw) == 0U)
  {
    ms5837_record_invalid_conversion(HAL_GetTick());
    return;
  }

  /* 样本时间戳：D2 结果读完的那一刻（异步后与中断完成时刻一致）。 */
  completed_ms = HAL_GetTick();
  ms5837.state = MS5837_STATE_IDLE;
  /*
   * 调度锚定在周期开始时刻：本帧耗掉的转换等待不会累加到下一个周期上，
   * 因此 25 Hz/4096 的实际采样间隔仍是 40 ms，而不是 40 + 20 ms。
   * 若已经超期（例如卡了一下），直接从现在开始下一帧，不做补偿性连发追赶。
   */
  period_ms = 1000U / (uint32_t)ms5837.config.output_rate_hz;
  next_start_ms = ms5837.cycle_start_ms + period_ms;
  if (ms5837_deadline_reached(completed_ms, next_start_ms) != 0U)
  {
    next_start_ms = completed_ms;
  }
  ms5837.deadline_ms = next_start_ms;
  ms5837_finish_sample(completed_ms);
}

/* 提交 PROM 某个字的读取（命令 + 2 字节）。 */
static void ms5837_begin_prom_word(void)
{
  uint8_t command = (uint8_t)(MS5837_CMD_PROM_READ_BASE + (uint8_t)(ms5837.prom_index * 2U));

  switch (ms5837_submit_command_read(command, ms5837.prom_bytes, MS5837_PROM_BYTES,
                                     MS5837_PENDING_PROM_WORD))
  {
    case MS5837_SUBMIT_STARTED:
      ms5837.state = MS5837_STATE_BUS_WAIT;
      break;
    case MS5837_SUBMIT_RETRY:
      break; /* 下一拍重试：仍是 PROM_READ，deadline 已到。 */
    default:
      break;
  }
}

/* 事务完成：一个 PROM 字已读到 → 累计、校验。 */
static void ms5837_done_prom_word(uint32_t now)
{
  uint16_t word;
  uint8_t crc_read;
  uint8_t crc_calc;

  word = ms5837_word_from_bytes(ms5837.prom_bytes);
  ms5837.prom[ms5837.prom_index] = word;
  ms5837.prom_index++;
  if (ms5837.prom_index < MS5837_CMD_PROM_WORD_COUNT)
  {
    ms5837.state = MS5837_STATE_PROM_READ;
    ms5837.deadline_ms = now;
    return;
  }

  ms5837.prom[7] = 0U;
  crc_read = (uint8_t)((ms5837.prom[0] >> 12) & 0x0FU);
  crc_calc = Ms5837_Crc4(ms5837.prom);
  /* CRC 不符，或 CRC 碰巧通过但内容明显不可信（C1~C6 全 0 / 全 0xFFFF），
     都按“PROM 不可信”处理：不置 PROM_VALID、不发布任何测量值。 */
  if ((crc_read != crc_calc) || (ms5837_prom_sane(ms5837.prom) == 0U))
  {
    ms5837.status &= ~(MS5837_STATUS_PROM_VALID | MS5837_STATUS_ONLINE |
                       MS5837_STATUS_RAW_VALID | MS5837_STATUS_PRESSURE_VALID |
                       MS5837_STATUS_TEMPERATURE_VALID | MS5837_STATUS_DEPTH_VALID);
    ms5837.prom_valid = 0U;
    ms5837.stats.prom_valid = 0U;
    ms5837.stats.errors++;
    ms5837.stats.crc_errors++;
    ms5837.stats.last_error = (uint32_t)MS5837_ERR_CRC;
    ms5837_status_record_last_error(MS5837_ERR_CRC);
    ms5837.prom_index = 0U;
    ms5837.state = MS5837_STATE_PROM_READ; /* 回到 PROM_READ，按重试间隔重读整块 */
    ms5837.deadline_ms = now + MS5837_RETRY_DELAY_MS;
    return;
  }

  ms5837.prom_valid = 1U;
  ms5837.stats.prom_valid = 1U;
  ms5837.status |= (MS5837_STATUS_PROM_VALID | MS5837_STATUS_ONLINE);
  ms5837.state = MS5837_STATE_IDLE;
  ms5837.deadline_ms = now; /* 立即可开始第一帧。 */
}

/* ---------------------------------------------------------------- 公共接口 */
Ms5837Result_t Ms5837_Init(void)
{
  uint32_t now;

  if (I2c_IsReady() == 0U)
  {
    ms5837.state = MS5837_STATE_OFFLINE;
    ms5837.init_requested = 0U;
    ms5837.stats.last_error = (uint32_t)MS5837_ERR_NOT_READY;
    ms5837_status_record_last_error(MS5837_ERR_NOT_READY);
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

  switch (ms5837_submit_command(MS5837_CMD_RESET, MS5837_PENDING_RESET))
  {
    case MS5837_SUBMIT_STARTED:
      ms5837.state = MS5837_STATE_BUS_WAIT;
      return MS5837_OK;
    case MS5837_SUBMIT_RETRY:
      /* 总线正忙：保持 OFFLINE，按重试间隔再来（不阻塞调用方）。 */
      ms5837.state = MS5837_STATE_OFFLINE;
      ms5837.deadline_ms = now + MS5837_RETRY_DELAY_MS;
      return MS5837_ERR_BUSY;
    default:
      return MS5837_ERR_IO; /* 错误已在提交处记录，state 已回到 IDLE。 */
  }
}

/* 事务完成：复位命令已发出 → 等复位延时结束再读 PROM。 */
static void ms5837_done_reset(void)
{
  ms5837.state = MS5837_STATE_RESET_WAIT;
  ms5837.deadline_ms = HAL_GetTick() + MS5837_RESET_DELAY_MS;
}

void Ms5837_Process(void)
{
  uint32_t now = HAL_GetTick();

  /*
   * 推进中断模式的 I2C 事务：只检查超时并对卡死事务发起 Abort，不做任何阻塞。
   * 放在最前面，保证即使状态机这一拍什么都不做，卡死的事务也能被及时中止。
   */
  I2c_Process();

  /*
   * 弃单清理：事务在飞期间配置被改（SetOsr/RestoreDefaults）或重新 Init 时，
   * 状态机会离开 BUS_WAIT，那个已完成的结果就没人认领了。如果不在这里取走，
   * 总线层会一直停在 DONE，后续每次提交都被拒（死锁）。这里只丢弃结果，不解析。
   */
  if ((ms5837.state != MS5837_STATE_BUS_WAIT) &&
      (I2c_GetPhase() == I2C_BUS_PHASE_DONE))
  {
    (void)I2c_GetResult();
    ms5837.pending_action = MS5837_PENDING_NONE;
  }

  switch (ms5837.state)
  {
    case MS5837_STATE_BUS_WAIT:
      /*
       * 等中断把事务做完。这里不做任何等待：BUSY 就直接返回，
       * 这一拍主循环可以继续跑 CAN/USB/IMU。DONE 时按 pending 分派后续动作。
       */
      if (I2c_GetPhase() != I2C_BUS_PHASE_DONE)
      {
        break;
      }
      {
        I2cBusResult_t bus_result = I2c_GetResult();
        uint8_t pending = ms5837.pending_action;

        ms5837.pending_action = MS5837_PENDING_NONE;
        if (ms5837.state != MS5837_STATE_BUS_WAIT)
        {
          /*
           * 事务在飞期间配置变了（例如 SetOsr 进了 DISCARD_WAIT）：
           * 只消费结果，不解析这批属于旧配置的数据。
           */
          break;
        }
        if (bus_result != I2C_BUS_OK)
        {
          ms5837_record_error(ms5837_map_i2c(bus_result), now);
          if (pending == MS5837_PENDING_PROM_WORD)
          {
            /* 总线恢复后从第一个字重新读取，避免半截 PROM。 */
            ms5837.prom_index = 0U;
            ms5837.deadline_ms = now + MS5837_RETRY_DELAY_MS;
          }
          break;
        }
        ms5837.status |= MS5837_STATUS_ONLINE;

        switch (pending)
        {
          case MS5837_PENDING_RESET:
            ms5837_done_reset();
            break;
          case MS5837_PENDING_PROM_WORD:
            ms5837_done_prom_word(now);
            break;
          case MS5837_PENDING_D1_CMD:
            ms5837_done_d1_command();
            break;
          case MS5837_PENDING_D1_ADC:
            ms5837_done_d1_adc(now);
            break;
          case MS5837_PENDING_D2_CMD:
            ms5837_done_d2_command();
            break;
          case MS5837_PENDING_D2_ADC:
            ms5837_done_d2_adc();
            break;
          default:
            break;
        }
      }
      break;

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
        ms5837_begin_prom_word();
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
        ms5837_begin_d1_adc();
      }
      break;

    case MS5837_STATE_CONVERT_D2:
      if (ms5837_deadline_reached(now, ms5837.deadline_ms) != 0U)
      {
        ms5837_begin_d2_adc();
      }
      break;

    case MS5837_STATE_DISCARD_WAIT:
      /*
       * 旧转换的等待时间已到：芯片不再 busy，可以安全地用新配置重新开始完整周期。
       * 这一步之前既不重发转换命令，也不读 ADC。
       */
      if (ms5837_deadline_reached(now, ms5837.deadline_ms) != 0U)
      {
        ms5837.state = MS5837_STATE_IDLE;
        ms5837.deadline_ms = now;
        if (ms5837.prom_valid != 0U)
        {
          ms5837_start_cycle(now);
        }
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

/* P0 只允许 ZERO_DEPTH 调用该内部设定函数，通用参数接口保持只读。 */
static Ms5837Result_t ms5837_set_surface_pressure_pa(float pa);

/* ---------------------------------------------------------------- 零点 */
Ms5837Result_t Ms5837_Zero(void)
{
  if (ms5837.sample_ready == 0U)
  {
    return MS5837_ERR_NO_SAMPLE;
  }
  if ((ms5837.status & MS5837_STATUS_PRESSURE_VALID) == 0U) return MS5837_ERR_NOT_READY;

  /*
   * 直接复用内部 P0 写入辅助函数的校验（有限值 + 10000~200000 Pa），
   * 否则可能建立一个 GET_PARAMETER 认为越界的 P0，出现“ZERO 成功但参数读不回来”的矛盾状态。
   */
  return ms5837_set_surface_pressure_pa(ms5837.sample.pressure_pa);
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
  ms5837.config.osr = MS5837_OSR_DEFAULT;
  ms5837.config.output_rate_hz = MS5837_OUTPUT_RATE_HZ_DEFAULT;
  ms5837.config.water_density = MS5837_WATER_DENSITY_DEFAULT;
  ms5837.config.filter_k = MS5837_FILTER_K_DEFAULT;
  ms5837.stats.osr = MS5837_OSR_DEFAULT;
  ms5837.stats.output_rate_hz = MS5837_OUTPUT_RATE_HZ_DEFAULT;
  /* 参数恢复不覆盖现有空气/水面参考 P0，也不修改固定探头型号。 */
  ms5837.filter_valid = 0U;
  if (ms5837.sample_ready != 0U)
  {
    ms5837_apply_depth();
    ms5837.sample.status = ms5837.status;
  }
  /*
   * OSR 会改变转换时间：必须丢弃正在进行的半周期（延迟丢弃），否则 D1/D2 会跨配置配对。
   */
  ms5837_abort_half_cycle();
  return MS5837_OK;
}

uint8_t Ms5837_GetModel(void)
{
  return MS5837_MODEL_FIXED;
}

Ms5837Result_t Ms5837_SetOsr(uint16_t osr)
{
  if (ms5837_osr_to_index(osr) == 0xFFU)
  {
    return MS5837_ERR_PARAM;
  }
  if (ms5837_schedule_fits(osr, ms5837.config.output_rate_hz) == 0U)
  {
    return MS5837_ERR_PARAM;
  }
  ms5837.config.osr = osr;
  ms5837.stats.osr = osr;
  /* 转换时间变了：丢弃按旧 OSR 计算等待的半周期，按新 OSR 重新开始。 */
  ms5837_abort_half_cycle();
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
  if (ms5837_schedule_fits(ms5837.config.osr, rate_hz) == 0U)
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

static Ms5837Result_t ms5837_set_surface_pressure_pa(float pa)
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
      /* P0 is changed only by the explicit ZERO_DEPTH action, never by a generic SET. */
      return MS5837_ERR_UNSUPPORTED;

    case MS5837_PARAM_FILTER_K:
      if ((type != MS5837_PARAM_TYPE_F32) || (length != 4U))
      {
        return MS5837_ERR_PARAM;
      }
      return Ms5837_SetFilterK(ms5837_read_le_f32(bytes));

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

    default:
      return MS5837_ERR_UNSUPPORTED;
  }
}
