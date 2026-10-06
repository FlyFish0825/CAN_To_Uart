#ifndef SENSOR_PROTOCOL_H
#define SENSOR_PROTOCOL_H

#include <stdint.h>
#include <stddef.h>
#ifdef __cplusplus
extern "C" {
#endif

#define SENSOR_MAX_PAYLOAD 58U
#define SENSOR_MAX_PACKET 78U
#define SENSOR_REPLY_SLOTS 8U

enum { SENSOR_OK=0, SENSOR_UNSUPPORTED=1, SENSOR_BAD_VALUE=2, SENSOR_BUSY=3,
       SENSOR_TIMEOUT=4, SENSOR_OFFLINE=5, SENSOR_IO_ERROR=6, SENSOR_UNCONFIRMED=7,
       SENSOR_NOT_READY=8, SENSOR_PIN_BLOCKED=9 };
enum { SENSOR_GET_INFO=1, SENSOR_GET_STATUS=2, SENSOR_GET_PARAMETER=3,
       SENSOR_SET_PARAMETER=4, SENSOR_SAVE_CONFIG=5, SENSOR_RESTORE_DEFAULTS=6,
       SENSOR_START_STREAM=7, SENSOR_STOP_STREAM=8, SENSOR_CALIBRATE=9,
       SENSOR_SELF_TEST=10, SENSOR_REBOOT=11, SENSOR_ZERO_DEPTH=12 };

typedef struct {
    uint8_t command, flags, target;
    uint32_t sequence, timestamp_us;
    uint16_t length;
    uint8_t payload[SENSOR_MAX_PAYLOAD];
} SensorFrame;

typedef struct {
    uint32_t received, bad_frames, rejected_full, duplicates, transmitted, send_errors;
} SensorProtocolStats;

/* Callbacks are invoked in main-loop context. send must COPY bytes before returning;
 * 0=queued, nonzero=busy/error (ready replies are retained and retried). */
typedef int (*SensorSendFn)(const uint8_t *packet, uint16_t length);
typedef void (*SensorCommandFn)(const SensorFrame *request, uint8_t reply_token);

uint16_t SensorProtocol_Crc16(const uint8_t *data, uint16_t length);
uint16_t SensorProtocol_Encode(const SensorFrame *frame, uint8_t out[SENSOR_MAX_PACKET]);
int SensorProtocol_Decode(const uint8_t *packet, uint16_t length, SensorFrame *out);
void SensorProtocol_Init(SensorSendFn send, SensorCommandFn command);
void SensorProtocol_RxFeed(const uint8_t *bytes, uint16_t length, uint32_t now_ms);
void SensorProtocol_Process(uint32_t now_ms);
/* Body excludes the RESULT byte; completion never executes a request again. */
int SensorProtocol_Complete(uint8_t token, uint8_t result, const uint8_t *body, uint16_t length);
int SensorProtocol_SendTelemetry(const SensorFrame *frame);
void SensorProtocol_ResetLink(void);
void SensorProtocol_GetStats(SensorProtocolStats *stats);

uint16_t Sensor_Read16(const uint8_t *p);
uint32_t Sensor_Read32(const uint8_t *p);
float Sensor_ReadFloat(const uint8_t *p);
void Sensor_Write16(uint8_t *p, uint16_t value);
void Sensor_Write32(uint8_t *p, uint32_t value);
void Sensor_WriteFloat(uint8_t *p, float value);

#ifdef __cplusplus
}
#endif
#endif
