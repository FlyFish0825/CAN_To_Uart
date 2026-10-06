#ifndef SENSOR_BOARD_H
#define SENSOR_BOARD_H
#include "main.h"
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

/* Default is passive PA10 receive only. PA9 is never configured/driven unless
 * TX is separately enabled after the owner confirms the wiring is free.
 * UART_ENABLED=0 disables even passive reception. */
#ifndef SENSOR_IMU_UART_ENABLED
#define SENSOR_IMU_UART_ENABLED 1
#endif
#ifndef SENSOR_IMU_UART_TX_ENABLED
#define SENSOR_IMU_UART_TX_ENABLED 0
#endif

HAL_StatusTypeDef SensorBoard_Init(void);
void SensorBoard_Process(void);
int SensorBoard_ImuTransmit(void *context, const uint8_t *data, uint16_t length);
int SensorBoard_UsbSend(const uint8_t *packet, uint16_t length);

#ifdef __cplusplus
}
#endif
#endif
