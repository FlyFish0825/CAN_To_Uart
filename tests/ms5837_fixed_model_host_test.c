/* Host regression for the single fixed MS5837-02BA runtime policy. */
#define I2C_BUS_ENABLE_NVIC 0
#include "ms5837.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static unsigned checks;
#define CHECK(condition) do { \
  ++checks; \
  if (!(condition)) { \
    fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #condition); \
    return 1; \
  } \
} while (0)

static uint32_t test_tick;
static HAL_StatusTypeDef test_tx_status = HAL_BUSY, test_rx_status = HAL_BUSY;
static unsigned test_rx_calls, test_abort_calls;
uint32_t HAL_GetTick(void) { return test_tick; }
HAL_StatusTypeDef HAL_I2C_Master_Transmit_IT(I2C_HandleTypeDef *handle,
                                             uint16_t address,
                                             uint8_t *data,
                                             uint16_t size) {
  (void)handle; (void)address; (void)data; (void)size; return test_tx_status;
}
HAL_StatusTypeDef HAL_I2C_Master_Receive_IT(I2C_HandleTypeDef *handle,
                                            uint16_t address,
                                            uint8_t *data,
                                            uint16_t size) {
  (void)handle; (void)address; (void)data; (void)size; ++test_rx_calls; return test_rx_status;
}
HAL_StatusTypeDef HAL_I2C_Master_Abort_IT(I2C_HandleTypeDef *handle, uint16_t address) {
  (void)handle; (void)address; ++test_abort_calls; return HAL_OK;
}
uint32_t HAL_I2C_GetError(const I2C_HandleTypeDef *handle) { (void)handle; return 0U; }
void HAL_I2C_EV_IRQHandler(I2C_HandleTypeDef *handle) { (void)handle; }
void HAL_I2C_ER_IRQHandler(I2C_HandleTypeDef *handle) { (void)handle; }

/* White-box host regression: exercise the real driver with the interrupt bus stubbed. */
#include "../Core/Src/ms5837.c"

int main(void) {
  Ms5837Stats_t stats;
  uint8_t type = 0U, length = 0U, value[4] = {0U, 0U, 0U, 0U};
  uint16_t prom[MS5837_PROM_WORDS] = {0U, 46372U, 43981U, 29059U, 27842U, 31553U, 28165U, 0U};
  int64_t pressure_raw = 0;
  int32_t temperature_centi_c = 0;

  CHECK(Ms5837_GetModel() == MS5837_MODEL_02BA);
  memset(&stats, 0, sizeof(stats));
  CHECK(Ms5837_GetStats(&stats) == MS5837_OK);
  CHECK((Ms5837_GetStatus() & MS5837_STATUS_DIAGNOSTICS_VALID) != 0U);
  CHECK(((Ms5837_GetStatus() & MS5837_STATUS_DRIVER_STATE_MASK) >>
         MS5837_STATUS_DRIVER_STATE_SHIFT) == MS5837_STATE_OFFLINE);
  CHECK(((Ms5837_GetStatus() & MS5837_STATUS_I2C_PHASE_MASK) >>
         MS5837_STATUS_I2C_PHASE_SHIFT) == I2C_BUS_PHASE_IDLE);
  CHECK(stats.osr == MS5837_OSR_DEFAULT && stats.output_rate_hz == MS5837_OUTPUT_RATE_HZ_DEFAULT);
  CHECK((Ms5837_GetStatus() & ((1UL << 10U) | (1UL << 11U))) == 0U);
  CHECK(Ms5837_IsZeroValid() == 1U);
  CHECK((Ms5837_GetStatus() & MS5837_STATUS_ZERO_VALID) != 0U);
  CHECK(Ms5837_GetParam(MS5837_PARAM_SURFACE_PRESSURE, &type, &length, value) == MS5837_OK);
  float default_p0 = 0.0f;
  memcpy(&default_p0, value, sizeof(default_p0));
  CHECK(type == MS5837_PARAM_TYPE_F32 && length == 4U);
  CHECK(default_p0 == MS5837_AIR_REFERENCE_PRESSURE_PA);

  /* The first valid 02BA sample can produce depth against the fixed air baseline. */
  memcpy(ms5837.prom, prom, sizeof(prom));
  ms5837.prom_valid = 1U;
  ms5837.status |= MS5837_STATUS_PROM_VALID;
  ms5837.d1_raw = 6465444U;
  ms5837.d2_raw = 8077636U;
  ms5837_finish_sample(1000U);
  CHECK((ms5837.status & (MS5837_STATUS_PRESSURE_VALID | MS5837_STATUS_TEMPERATURE_VALID |
                          MS5837_STATUS_DEPTH_VALID | MS5837_STATUS_ZERO_VALID)) ==
        (MS5837_STATUS_PRESSURE_VALID | MS5837_STATUS_TEMPERATURE_VALID |
         MS5837_STATUS_DEPTH_VALID | MS5837_STATUS_ZERO_VALID));
  CHECK(ms5837.sample.pressure_pa == 110002.0f);
  CHECK(isfinite(ms5837.sample.depth_filtered_m));
  CHECK(fabsf(ms5837.sample.surface_pressure_pa - MS5837_AIR_REFERENCE_PRESSURE_PA) < 0.01f);
  const float expected_depth_m = (110002.0f - MS5837_AIR_REFERENCE_PRESSURE_PA) /
                                (MS5837_WATER_DENSITY_DEFAULT * MS5837_GRAVITY);
  CHECK(fabsf(ms5837.sample.depth_raw_m - expected_depth_m) < 0.000001f);

  value[0] = MS5837_MODEL_02BA;
  CHECK(Ms5837_SetParam(0x0105U, MS5837_PARAM_TYPE_U8, value, 1U) == MS5837_ERR_UNSUPPORTED);
  CHECK(Ms5837_GetParam(0x0105U, &type, &length, value) == MS5837_ERR_UNSUPPORTED);
  CHECK(Ms5837_SetParam(MS5837_PARAM_SURFACE_PRESSURE, MS5837_PARAM_TYPE_F32,
                        value, 4U) == MS5837_ERR_UNSUPPORTED);
  CHECK(Ms5837_Zero() == MS5837_OK);

  /* Only explicit ZERO_DEPTH can replace the startup baseline; generic SET was rejected above. */
  ms5837.sample_ready = 1U;
  ms5837.status |= MS5837_STATUS_PRESSURE_VALID;
  ms5837.sample.pressure_pa = 100123.5f;
  CHECK(Ms5837_Zero() == MS5837_OK);
  CHECK(ms5837.zero_valid == 1U && ms5837.config.surface_pressure_pa == 100123.5f);
  CHECK((Ms5837_GetStatus() & MS5837_STATUS_ZERO_VALID) != 0U);
  CHECK((Ms5837_GetStatus() & ((1UL << 10U) | (1UL << 11U))) == 0U);
  CHECK(Ms5837_GetParam(MS5837_PARAM_SURFACE_PRESSURE, &type, &length, value) == MS5837_OK);
  CHECK(type == MS5837_PARAM_TYPE_F32 && length == 4U);

  CHECK(Ms5837_Compensate(prom, 6465444U, 8077636U,
                          &pressure_raw, &temperature_centi_c) == 1U);
  CHECK(pressure_raw == 110002 && temperature_centi_c == 2000);

  /* 0x83 状态帧长度不变，高位可携带最近总线错误及命令上下文。 */
  ms5837.status = MS5837_STATUS_ZERO_VALID;
  ms5837.busy_command = MS5837_CMD_RESET;
  ms5837_record_error(MS5837_ERR_IO, 2000U);
  CHECK(((Ms5837_GetStatus() & MS5837_STATUS_LAST_I2C_COMMAND_MASK) >>
         MS5837_STATUS_LAST_I2C_COMMAND_SHIFT) == MS5837_CMD_RESET);
  CHECK(((Ms5837_GetStatus() & MS5837_STATUS_LAST_ERROR_MASK) >>
         MS5837_STATUS_LAST_ERROR_SHIFT) == MS5837_ERR_IO);
  ms5837.status = MS5837_STATUS_ZERO_VALID;
  ms5837.busy_command = 0xACU;
  ms5837_record_error(MS5837_ERR_CRC, 3000U);
  CHECK(((Ms5837_GetStatus() & MS5837_STATUS_LAST_I2C_COMMAND_MASK) >>
         MS5837_STATUS_LAST_I2C_COMMAND_SHIFT) == 0xACU);
  CHECK(((Ms5837_GetStatus() & MS5837_STATUS_LAST_ERROR_MASK) >>
         MS5837_STATUS_LAST_ERROR_SHIFT) == MS5837_ERR_CRC);
  ms5837.status = MS5837_STATUS_ZERO_VALID;
  ms5837.busy_command = 0xACU;
  ms5837_record_error(MS5837_ERR_PARAM, 4000U);
  CHECK(((Ms5837_GetStatus() & MS5837_STATUS_LAST_I2C_COMMAND_MASK) >>
         MS5837_STATUS_LAST_I2C_COMMAND_SHIFT) == 0U);
  CHECK(((Ms5837_GetStatus() & MS5837_STATUS_LAST_ERROR_MASK) >>
         MS5837_STATUS_LAST_ERROR_SHIFT) == MS5837_ERR_PARAM);
  i2c_phase = I2C_BUS_PHASE_BUSY;
  CHECK(((Ms5837_GetStatus() & MS5837_STATUS_I2C_PHASE_MASK) >>
         MS5837_STATUS_I2C_PHASE_SHIFT) == I2C_BUS_PHASE_BUSY);
  i2c_phase = I2C_BUS_PHASE_IDLE;

  CHECK(Ms5837_RestoreDefaults() == MS5837_OK);
  CHECK(Ms5837_GetModel() == MS5837_MODEL_02BA);
  CHECK(ms5837.zero_valid == 1U && ms5837.config.surface_pressure_pa == 100123.5f);
  CHECK(Ms5837_GetParam(MS5837_PARAM_SURFACE_PRESSURE, &type, &length, value) == MS5837_OK);
  /* 写完成中断之后 HAL_BUSY 只延迟读取，不能误报器件离线；读阶段不能重复提交。 */
  I2C_HandleTypeDef handle = {0};
  uint8_t command = 0xA0U, rx[2] = {0U, 0U};
  I2c_Init(&handle);
  test_tx_status = HAL_OK;
  test_rx_status = HAL_BUSY;
  test_rx_calls = 0U;
  CHECK(I2c_Submit(&command, 1U, rx, 2U, 5U) == I2C_BUS_OK);
  HAL_I2C_MasterTxCpltCallback(&handle);
  CHECK(test_rx_calls == 0U && I2c_GetPhase() == I2C_BUS_PHASE_BUSY);
  I2c_Process();
  CHECK(test_rx_calls == 1U && I2c_GetPhase() == I2C_BUS_PHASE_BUSY);
  test_rx_status = HAL_OK;
  I2c_Process();
  CHECK(test_rx_calls == 2U && I2c_GetPhase() == I2C_BUS_PHASE_BUSY);
  I2c_Process();
  CHECK(test_rx_calls == 2U);
  HAL_I2C_MasterRxCpltCallback(&handle);
  CHECK(I2c_GetResult() == I2C_BUS_OK);

  /* 持续忙仍受原事务超时约束；非 BUSY 的失败按真实 HAL 类型返回。 */
  test_tick = 100U;
  test_rx_status = HAL_BUSY;
  CHECK(I2c_Submit(&command, 1U, rx, 2U, 5U) == I2C_BUS_OK);
  HAL_I2C_MasterTxCpltCallback(&handle);
  test_tick = 106U;
  I2c_Process();
  CHECK(test_abort_calls == 1U);
  HAL_I2C_AbortCpltCallback(&handle);
  CHECK(I2c_GetResult() == I2C_BUS_TIMEOUT);
  test_rx_status = HAL_ERROR;
  CHECK(I2c_Submit(&command, 1U, rx, 2U, 5U) == I2C_BUS_OK);
  HAL_I2C_MasterTxCpltCallback(&handle);
  I2c_Process();
  CHECK(I2c_GetResult() == I2C_BUS_ERROR);
  printf("PASS %u fixed-model, read-only-P0 and deferred-I2C checks\n", checks);
  return 0;
}
