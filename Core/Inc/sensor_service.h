#ifndef SENSOR_SERVICE_H
#define SENSOR_SERVICE_H
#include "sensor_protocol.h"
#include "imu_sensor.h"
#ifdef __cplusplus
extern "C" {
#endif

/* Both functions run only in main-loop context. Ms5837_Init must have been
 * scheduled after I2c_Init. IMU remains PIN_BLOCKED until the board adapter
 * explicitly enables it after a confirmed physical wiring change. */
void SensorService_Init(SensorSendFn send, ImuSensor_TxFn imu_tx);
void SensorService_Process(uint32_t now_ms);
void SensorService_SetLink(uint8_t connected);
uint64_t SensorService_NowUs(void);

#ifdef __cplusplus
}
#endif
#endif
