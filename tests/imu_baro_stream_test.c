#include "sensor_imu_baro.h"
#include <assert.h>
#include <stdio.h>
int main(void)
{
    ImuSensor_Sample s; SensorFrame f,decoded;
    uint8_t packet[SENSOR_MAX_PACKET]; memset(&s,0,sizeof(s));
    assert(!SensorImuBaro_MakeFrame(&s,0,&f));
    assert(!SensorImuBaro_MakeFrame(NULL,0,&f));
    s.baro_seq=1; s.baro_time_us=(1ULL<<32)+1234;
    s.status=IMU_SENSOR_STATUS_PRESSURE_VALID|IMU_SENSOR_STATUS_TEMPERATURE_VALID;
    s.baro_height_m=12.5f; s.baro_temp_c=22.5f; s.baro_pressure_pa=101325.0f; s.baro_ref_pa=101500.0f;
    assert(SensorImuBaro_MakeFrame(&s,0,&f));
    assert(f.target==1 && f.command==0x85 && f.flags==8 && f.length==20 && f.timestamp_us==1234);
    const uint8_t expected[20]={0x30,0,0,0,0,0,0x48,0x41,0,0,0xb4,0x41,0x80,0xe6,0xc5,0x47,0,0x3e,0xc6,0x47};
    assert(memcmp(f.payload,expected,sizeof(expected))==0);
    f.sequence=7;
    assert(SensorProtocol_Encode(&f,packet)==40);
    assert(SensorProtocol_Decode(packet,40,&decoded));
    assert(decoded.command==0x85 && decoded.sequence==7 && memcmp(decoded.payload,expected,20)==0);
    assert(!SensorImuBaro_MakeFrame(&s,1,&f));
    assert(SensorImuBaro_MakeFrame(&s,0,&f)); /* Unsent sample remains retryable. */
    s.baro_seq=2; s.status=0; s.baro_height_m=-12.5f;
    assert(SensorImuBaro_MakeFrame(&s,1,&f));
    assert(Sensor_Read32(f.payload)==0 && Sensor_ReadFloat(f.payload+4)==-12.5f);
    s.status=0x30; s.baro_pressure_pa=NAN; s.baro_temp_c=INFINITY;
    assert(SensorImuBaro_MakeFrame(&s,1,&f));
    assert((Sensor_Read32(f.payload)&0x30)==0);
    assert(Sensor_ReadFloat(f.payload+8)==0 && Sensor_ReadFloat(f.payload+12)==0);
    puts("imu_baro_stream_test: PASS"); return 0;
}
