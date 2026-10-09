#ifndef SENSOR_IMU_BARO_H
#define SENSOR_IMU_BARO_H
#include "imu_sensor.h"
#include "sensor_protocol.h"
#include <math.h>
#include <string.h>

/* AA5B TARGET=1 CMD=0x85. Native 0x32 order: height, temperature, pressure, reference.
 * Pure packing helper: no UART writes, calibration, or modifications to the sample.
 * Advance last_sent only after SensorProtocol_SendTelemetry accepted the frame. */
static inline int SensorImuBaro_MakeFrame(const ImuSensor_Sample *sample,
                                         uint32_t last_sent, SensorFrame *frame)
{
    if(!sample || !frame || !sample->baro_seq || sample->baro_seq==last_sent) return 0;
    memset(frame,0,sizeof(*frame));
    frame->target=1U; frame->command=0x85U; frame->flags=8U; frame->length=20U;
    frame->timestamp_us=(uint32_t)sample->baro_time_us;
    uint32_t status=sample->status;
    if(!isfinite(sample->baro_height_m) || !isfinite(sample->baro_pressure_pa) ||
       !isfinite(sample->baro_ref_pa)) status&=~IMU_SENSOR_STATUS_PRESSURE_VALID;
    if(!isfinite(sample->baro_temp_c)) status&=~IMU_SENSOR_STATUS_TEMPERATURE_VALID;
    Sensor_Write32(frame->payload,status);
    Sensor_WriteFloat(frame->payload+4,isfinite(sample->baro_height_m)?sample->baro_height_m:0.0f);
    Sensor_WriteFloat(frame->payload+8,isfinite(sample->baro_temp_c)?sample->baro_temp_c:0.0f);
    Sensor_WriteFloat(frame->payload+12,isfinite(sample->baro_pressure_pa)?sample->baro_pressure_pa:0.0f);
    Sensor_WriteFloat(frame->payload+16,isfinite(sample->baro_ref_pa)?sample->baro_ref_pa:0.0f);
    return 1;
}
#endif
