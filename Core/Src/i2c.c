#include "i2c.h"

static I2C_HandleTypeDef *i2c_bus_handle = 0; /* 绑定的 HAL I2C 句柄。 */
static uint8_t i2c_bus_device7 = I2C_BUS_MS5837_ADDRESS7; /* 当前 7 位从地址。 */
static uint8_t i2c_bus_ready = 0U; /* 句柄是否已绑定。 */

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

void I2c_Init(I2C_HandleTypeDef *handle)
{
  i2c_bus_handle = handle;
  i2c_bus_ready = (handle != 0) ? 1U : 0U;
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

I2cBusResult_t I2c_Write(const uint8_t *data,
                         uint16_t length,
                         uint32_t timeout_ms)
{
  if ((i2c_bus_ready == 0U) || (i2c_bus_handle == 0))
  {
    return I2C_BUS_NOT_READY;
  }
  if ((data == 0) || (length == 0U) || (length > I2C_BUS_MAX_TRANSFER))
  {
    return I2C_BUS_PARAM;
  }

  return i2c_bus_map_status(HAL_I2C_Master_Transmit(
      i2c_bus_handle,
      (uint16_t)((uint16_t)i2c_bus_device7 << 1),
      (uint8_t *)data,
      length,
      i2c_bus_clamp_timeout(timeout_ms)));
}

I2cBusResult_t I2c_Read(uint8_t *data,
                        uint16_t length,
                        uint32_t timeout_ms)
{
  if ((i2c_bus_ready == 0U) || (i2c_bus_handle == 0))
  {
    return I2C_BUS_NOT_READY;
  }
  if ((data == 0) || (length == 0U) || (length > I2C_BUS_MAX_TRANSFER))
  {
    return I2C_BUS_PARAM;
  }

  return i2c_bus_map_status(HAL_I2C_Master_Receive(
      i2c_bus_handle,
      (uint16_t)((uint16_t)i2c_bus_device7 << 1),
      data,
      length,
      i2c_bus_clamp_timeout(timeout_ms)));
}

I2cBusResult_t I2c_WriteRead(const uint8_t *tx,
                             uint16_t tx_length,
                             uint8_t *rx,
                             uint16_t rx_length,
                             uint32_t timeout_ms)
{
  I2cBusResult_t result;

  result = I2c_Write(tx, tx_length, timeout_ms);
  if (result != I2C_BUS_OK)
  {
    return result;
  }

  return I2c_Read(rx, rx_length, timeout_ms);
}
