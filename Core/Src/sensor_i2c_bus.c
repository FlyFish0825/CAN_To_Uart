#include "sensor_i2c_bus.h"

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
#if !defined(I2C_HOST_TEST)
  if (i2c_bus_handle != 0)
  {
    HAL_I2C_EV_IRQHandler(i2c_bus_handle);
  }
#endif
}

void I2c_ErIrqHandler(void)
{
#if !defined(I2C_HOST_TEST)
  if (i2c_bus_handle != 0)
  {
    HAL_I2C_ER_IRQHandler(i2c_bus_handle);
  }
#endif
}

#if (I2C_BUS_DEFINE_IRQ_HANDLERS == 1) && !defined(I2C_HOST_TEST)
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

#if (I2C_BUS_ENABLE_NVIC == 1) && !defined(I2C_HOST_TEST)
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
I2cBusResult_t I2c_Submit(const uint8_t *tx,
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

I2cBusPhase_t I2c_GetPhase(void)
{
  return i2c_phase;
}

uint8_t I2c_IsBusy(void)
{
  return (i2c_phase == I2C_BUS_PHASE_BUSY) ? 1U : 0U;
}

I2cBusResult_t I2c_GetResult(void)
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

void I2c_Process(void)
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

/* ---------------------------------------------------------------- 同步兼容接口 */
/*
 * 下面三个同步接口保留给初始化等非实时路径使用：内部用“提交 + 自旋”实现，
 * 仍然是阻塞的（但每次自旋都调用 I2c_Process()，所以超时行为与中断模式一致）。
 * 正式采样路径（Ms5837_Process 的状态机）不得使用它们。
 */
I2cBusResult_t I2c_Write(const uint8_t *data,
                         uint16_t length,
                         uint32_t timeout_ms)
{
  I2cBusResult_t result;

  result = I2c_Submit(data, length, 0, 0U, timeout_ms);
  if (result != I2C_BUS_OK)
  {
    return result;
  }
  while (I2c_GetPhase() == I2C_BUS_PHASE_BUSY)
  {
    I2c_Process();
  }
  return I2c_GetResult();
}

I2cBusResult_t I2c_Read(uint8_t *data,
                        uint16_t length,
                        uint32_t timeout_ms)
{
  I2cBusResult_t result;

  result = I2c_Submit(0, 0U, data, length, timeout_ms);
  if (result != I2C_BUS_OK)
  {
    return result;
  }
  while (I2c_GetPhase() == I2C_BUS_PHASE_BUSY)
  {
    I2c_Process();
  }
  return I2c_GetResult();
}

I2cBusResult_t I2c_WriteRead(const uint8_t *tx,
                             uint16_t tx_length,
                             uint8_t *rx,
                             uint16_t rx_length,
                             uint32_t timeout_ms)
{
  I2cBusResult_t result;

  result = I2c_Submit(tx, tx_length, rx, rx_length, timeout_ms);
  if (result != I2C_BUS_OK)
  {
    return result;
  }
  while (I2c_GetPhase() == I2C_BUS_PHASE_BUSY)
  {
    I2c_Process();
  }
  return I2c_GetResult();
}
