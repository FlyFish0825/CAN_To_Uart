#include "sensor_service.h"
#include "sensor_imu_baro.h"
#include "ms5837.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

static uint8_t link_up, streaming[2];
static uint32_t last_tick, status_tick[2], stream_seq[2];
static uint64_t tick_high, now_us;
static uint32_t raw_sent, quat_sent, euler_sent, baro_sent, depth_sent;
static struct {
    uint8_t active, token, command;
    uint32_t backend_sequence;
    uint16_t length;
    uint8_t tuple[8];
} imu_pending;

static uint8_t depth_result(Ms5837Result_t r)
{
    switch(r) {
    case MS5837_OK: return SENSOR_OK;
    case MS5837_ERR_PARAM: return SENSOR_BAD_VALUE;
    case MS5837_ERR_BUSY: return SENSOR_BUSY;
    case MS5837_ERR_TIMEOUT: return SENSOR_TIMEOUT;
    case MS5837_ERR_IO: return SENSOR_OFFLINE;
    case MS5837_ERR_CRC: return SENSOR_IO_ERROR;
    case MS5837_ERR_UNSUPPORTED: return SENSOR_UNSUPPORTED;
    default: return SENSOR_NOT_READY;
    }
}
static void complete(uint8_t token,uint8_t result,const uint8_t *body,uint16_t n)
{
    (void)SensorProtocol_Complete(token,result,body,n);
}
static uint32_t imu_error_count(const ImuSensor_Sample *s)
{
    return s->bad_checksum_frames+s->bad_length_frames+s->nonfinite_frames+s->rx_overflow_chunks+s->timeouts_total;
}
static void status_body(uint8_t target,uint8_t body[20])
{
    uint32_t status, sequence, age, good, errors;
    if(target==1U) {
        ImuSensor_Sample s; memset(&s,0,sizeof(s)); (void)ImuSensor_GetSample(&s,now_us);
        uint64_t latest=s.raw_time_us;
        if(s.quat_time_us>latest) latest=s.quat_time_us;
        if(s.euler_time_us>latest) latest=s.euler_time_us;
        if(s.baro_time_us>latest) latest=s.baro_time_us;
        uint8_t has_data=(s.raw_seq||s.quat_seq||s.euler_seq||s.baro_seq)?1U:0U;
        status=s.status; sequence=s.good_frames; good=s.good_frames; errors=imu_error_count(&s);
        age=has_data?(uint32_t)((now_us-latest)/1000ULL):UINT32_MAX;
    } else {
        Ms5837Stats_t s; memset(&s,0,sizeof(s)); (void)Ms5837_GetStats(&s);
        status=Ms5837_GetStatus(); sequence=s.sample_seq; good=s.good_frames; errors=s.errors;
        age=Ms5837_GetSampleAgeMs();
    }
    Sensor_Write32(body,status); Sensor_Write32(body+4,sequence); Sensor_Write32(body+8,age);
    Sensor_Write32(body+12,good); Sensor_Write32(body+16,errors);
}
static void info_reply(uint8_t target,uint8_t token)
{
    uint8_t body[30]; memset(body,0,sizeof(body)); body[0]=target;
    if(target==1U) {
        ImuSensor_Sample s; memset(&s,0,sizeof(s)); (void)ImuSensor_GetSample(&s,now_us);
        /* Model cannot be queried with this IMU protocol. Do not infer six/nine axes
         * merely because a float or a nonzero magnetic value has arrived. */
        body[1]=0U;
        uint32_t capabilities=(1UL<<0)|(1UL<<1)|(1UL<<2)|(1UL<<5)|(1UL<<6)|(1UL<<7)|(1UL<<8);
        if(s.baro_seq) capabilities|=(1UL<<3)|(1UL<<4); /* Observed barometer support, not a guessed model. */
        Sensor_Write32(body+2,capabilities);
        memcpy(body+6,"7E23 IMU",8U);
        if(s.version_valid) {
            char version[16];
            (void)snprintf(version,sizeof(version),"%u.%u.%u",s.version[0],s.version[1],s.version[2]);
            size_t n=strlen(version); if(n>8U) n=8U; memcpy(body+22,version,n);
        } else memcpy(body+22,"unknown",7U);
    } else {
        body[1]=Ms5837_GetModel();
        Sensor_Write32(body+2,(1UL<<3)|(1UL<<4)|(1UL<<5)|(1UL<<10)|(1UL<<11));
        memcpy(body+6,"MS5837",6U); memcpy(body+22,"host-v1",7U);
    }
    complete(token,SENSOR_OK,body,sizeof(body));
}
static void start_imu(uint8_t token,const SensorFrame *request,uint8_t operation,uint32_t arg)
{
    if(imu_pending.active) { complete(token,SENSOR_BUSY,NULL,0U); return; }
    uint32_t sequence=0U;
    int result=ImuSensor_Request(operation,arg,now_us,&sequence);
    if(result!=SENSOR_OK) { complete(token,(uint8_t)result,NULL,0U); return; }
    imu_pending.active=1U; imu_pending.token=token; imu_pending.command=request->command;
    imu_pending.backend_sequence=sequence; imu_pending.length=request->length;
    if(request->length<=sizeof(imu_pending.tuple)) memcpy(imu_pending.tuple,request->payload,request->length);
}
static void parameter_command(const SensorFrame *f,uint8_t token)
{
    const uint8_t set=f->command==SENSOR_SET_PARAMETER;
    if((!set && f->length!=2U) || (set && f->length<5U)) { complete(token,SENSOR_BAD_VALUE,NULL,0U); return; }
    const uint16_t id=Sensor_Read16(f->payload);
    uint8_t tuple[8]={0}; Sensor_Write16(tuple,id);
    if(set && ((uint16_t)f->payload[3]+4U!=f->length || f->payload[3]>4U)) {
        complete(token,SENSOR_BAD_VALUE,NULL,0U); return;
    }
    if(f->target==2U) {
        Ms5837Result_t result;
        uint8_t type=0U,n=0U;
        if(set) {
            /* 不要在服务层硬编码速率上限：驱动会按“型号/OSR 的转换预算”自己判断并返回
             BAD_VALUE。各 OSR 下的真实上限不同（例 02BA：OSR512/1024 可 100 Hz，
             OSR2048 约 71 Hz，OSR4096 约 45 Hz，OSR8192 约 25 Hz）。 */
            if(id==MS5837_PARAM_OUTPUT_RATE_HZ && (f->payload[2]!=4U || f->payload[3]!=2U || Sensor_Read16(f->payload+4)>MS5837_OUTPUT_RATE_HZ_MAX)) {
                complete(token,SENSOR_BAD_VALUE,NULL,0U); return;
            }
            result=Ms5837_SetParam(id,f->payload[2],f->payload+4,f->payload[3]);
            if(result!=MS5837_OK) { complete(token,depth_result(result),NULL,0U); return; }
        }
        result=Ms5837_GetParam(id,&type,&n,tuple+4);
        if(result!=MS5837_OK) { complete(token,depth_result(result),NULL,0U); return; }
        tuple[2]=type; tuple[3]=n;
        complete(token,SENSOR_OK,tuple,(uint16_t)(n+4U));
        return;
    }
    if(id!=IMU_SENSOR_PARAM_OUTPUT_RATE_HZ && id!=IMU_SENSOR_PARAM_ALGORITHM_MODE) {
        complete(token,SENSOR_UNSUPPORTED,NULL,0U); return;
    }
    const uint8_t type=id==IMU_SENSOR_PARAM_OUTPUT_RATE_HZ?4U:2U;
    const uint8_t n=type==4U?2U:1U;
    if(set) {
        if(f->payload[2]!=type || f->payload[3]!=n) { complete(token,SENSOR_BAD_VALUE,NULL,0U); return; }
        uint32_t value=n==2U?Sensor_Read16(f->payload+4):f->payload[4];
        start_imu(token,f,id==IMU_SENSOR_PARAM_OUTPUT_RATE_HZ?IMU_SENSOR_REQ_SET_RATE:IMU_SENSOR_REQ_SET_MODE,value);
    } else {
        uint32_t value=0U; int result=ImuSensor_GetParameter(id,&value);
        if(result!=1) { complete(token,result<0?SENSOR_UNSUPPORTED:SENSOR_NOT_READY,NULL,0U); return; }
        tuple[2]=type; tuple[3]=n;
        if(n==2U) Sensor_Write16(tuple+4,(uint16_t)value); else tuple[4]=(uint8_t)value;
        complete(token,SENSOR_UNCONFIRMED,tuple,(uint16_t)(n+4U));
    }
}
static void command_received(const SensorFrame *f,uint8_t token)
{
    if(!link_up) { complete(token,SENSOR_OFFLINE,NULL,0U); return; }
    if(f->command==SENSOR_GET_PARAMETER || f->command==SENSOR_SET_PARAMETER) {
        parameter_command(f,token); return;
    }
    if(f->command==SENSOR_CALIBRATE) {
        if(f->target!=1U) { complete(token,SENSOR_UNSUPPORTED,NULL,0U); return; }
        if(f->length!=4U || f->payload[1]>1U || f->payload[0]<1U || f->payload[0]>3U) {
            complete(token,SENSOR_BAD_VALUE,NULL,0U); return;
        }
        uint8_t op;
        if(f->payload[0]==1U) op=f->payload[1]?IMU_SENSOR_REQ_CAL_ACCEL_GYRO_START:IMU_SENSOR_REQ_CAL_ACCEL_GYRO_CLEAR;
        else if(f->payload[0]==2U) op=f->payload[1]?IMU_SENSOR_REQ_CAL_MAG_START:IMU_SENSOR_REQ_CAL_MAG_CLEAR;
        else op=IMU_SENSOR_REQ_CAL_TEMP;
        start_imu(token,f,op,0U); return;
    }
    if(f->length!=0U) { complete(token,SENSOR_BAD_VALUE,NULL,0U); return; }
    switch(f->command) {
    case SENSOR_GET_INFO:
        if(f->target==1U && !ImuSensor_IsPinsBlocked()) {
            ImuSensor_Sample sample; memset(&sample,0,sizeof(sample)); (void)ImuSensor_GetSample(&sample,now_us);
            if(!sample.version_valid) { start_imu(token,f,IMU_SENSOR_REQ_GET_VERSION,0U); return; }
        }
        info_reply(f->target,token); break;
    case SENSOR_GET_STATUS: {
        uint8_t body[20]; status_body(f->target,body); complete(token,SENSOR_OK,body,sizeof(body)); break;
    }
    case SENSOR_START_STREAM: case SENSOR_STOP_STREAM:
        streaming[f->target-1U]=f->command==SENSOR_START_STREAM;
        complete(token,SENSOR_OK,NULL,0U); break;
    case SENSOR_ZERO_DEPTH:
        complete(token,f->target==2U?depth_result(Ms5837_Zero()):SENSOR_UNSUPPORTED,NULL,0U); break;
    default:
        // No SAVE/restore/reboot implementation is advertised. Unsupported is explicit,
        // not a RAM write disguised as persistence and not an invented native opcode.
        complete(token,SENSOR_UNSUPPORTED,NULL,0U); break;
    }
}
void SensorService_Init(SensorSendFn send,ImuSensor_TxFn imu_tx)
{
    link_up=0U; memset(streaming,0,sizeof(streaming)); memset(stream_seq,0,sizeof(stream_seq));
    memset(&imu_pending,0,sizeof(imu_pending));
    raw_sent=quat_sent=euler_sent=baro_sent=depth_sent=0U;
    tick_high=now_us=0ULL; last_tick=0U; status_tick[0]=status_tick[1]=0U;
    /* Backend defaults: version query 1 s; calibration 30 s. */
    ImuSensor_Config config={imu_tx,NULL,0ULL};
    ImuSensor_Init(&config);
    SensorProtocol_Init(send,command_received);
}
void SensorService_SetLink(uint8_t connected)
{
    connected=connected?1U:0U;
    if(connected==link_up) return;
    link_up=connected;
    SensorProtocol_ResetLink(); memset(&imu_pending,0,sizeof(imu_pending));
    streaming[0]=streaming[1]=connected;
    raw_sent=quat_sent=euler_sent=baro_sent=depth_sent=0U;
    status_tick[0]=status_tick[1]=last_tick-1000U; // Status promptly on connection.
}
uint64_t SensorService_NowUs(void) { return now_us; }
static int send_stream(SensorFrame *frame)
{
    frame->flags=8U; frame->sequence=stream_seq[frame->target-1U]+1U;
    if(!SensorProtocol_SendTelemetry(frame)) return 0;
    stream_seq[frame->target-1U]++; return 1;
}
static float value_or_zero(float value) { return isfinite(value)?value:0.0f; }
void SensorService_Process(uint32_t now_ms)
{
    if(now_ms<last_tick) tick_high+=(1ULL<<32U);
    last_tick=now_ms; now_us=(tick_high+now_ms)*1000ULL;
    ImuSensor_Process(now_us); Ms5837_Process();
    ImuSensor_Result result;
    while(ImuSensor_PopResult(&result)) {
        if(!imu_pending.active || result.host_seq!=imu_pending.backend_sequence) continue;
        if(imu_pending.command==SENSOR_GET_INFO && result.result==SENSOR_OK) info_reply(1U,imu_pending.token);
        else if(imu_pending.command==SENSOR_SET_PARAMETER && (result.result==SENSOR_OK || result.result==SENSOR_UNCONFIRMED))
            complete(imu_pending.token,result.result,imu_pending.tuple,imu_pending.length);
        else complete(imu_pending.token,result.result,NULL,0U);
        imu_pending.active=0U;
    }
    SensorProtocol_Process(now_ms);
    if(!link_up) return;
    for(uint8_t target=1U;target<=2U;++target) {
        if((uint32_t)(now_ms-status_tick[target-1U])>=1000U) {
            SensorFrame f; memset(&f,0,sizeof(f)); f.command=0x83U; f.target=target;
            f.length=20U; f.timestamp_us=(uint32_t)now_us; status_body(target,f.payload);
            if(send_stream(&f)) status_tick[target-1U]=now_ms;
            return; // Per-target schedule avoids resending IMU status and starving depth.
        }
    }
    if(streaming[0]) {
        ImuSensor_Sample s; memset(&s,0,sizeof(s)); (void)ImuSensor_GetSample(&s,now_us);
        SensorFrame f; memset(&f,0,sizeof(f)); f.target=1U; Sensor_Write32(f.payload,s.status);
        if(s.raw_seq && s.raw_seq!=raw_sent) {
            f.command=0x80U; f.length=40U; f.timestamp_us=(uint32_t)s.raw_time_us;
            for(uint8_t i=0U;i<3U;++i) { Sensor_WriteFloat(f.payload+4U+4U*i,s.accel_g[i]); Sensor_WriteFloat(f.payload+16U+4U*i,s.gyro_rad_s[i]); Sensor_WriteFloat(f.payload+28U+4U*i,s.mag_units[i]); }
            if(send_stream(&f)) raw_sent=s.raw_seq;
        }
        if((s.quat_seq && s.quat_seq!=quat_sent) || (s.euler_seq && s.euler_seq!=euler_sent)) {
            f.command=0x81U; f.length=32U; f.timestamp_us=(uint32_t)(s.quat_time_us>s.euler_time_us?s.quat_time_us:s.euler_time_us);
            for(uint8_t i=0U;i<4U;++i) Sensor_WriteFloat(f.payload+4U+4U*i,value_or_zero(s.quat_wxyz[i]));
            for(uint8_t i=0U;i<3U;++i) Sensor_WriteFloat(f.payload+20U+4U*i,value_or_zero(s.euler_rpy_rad[i]));
            if(send_stream(&f)) { quat_sent=s.quat_seq; euler_sent=s.euler_seq; }
        }
        if(SensorImuBaro_MakeFrame(&s,baro_sent,&f) && send_stream(&f)) baro_sent=s.baro_seq;
    }
    if(streaming[1]) {
        Ms5837Sample_t s;
        if(Ms5837_GetSample(&s)==MS5837_OK && s.sequence!=depth_sent) {
            SensorFrame f; memset(&f,0,sizeof(f)); f.target=2U; f.command=0x82U; f.length=32U; f.timestamp_us=s.timestamp_ms*1000U;
            Sensor_Write32(f.payload,s.status);
            // Internal NaNs represent absence; wire uses finite placeholders and validity
            // flags. An absent value must never acquire a VALID bit by serialization.
            Sensor_WriteFloat(f.payload+4,value_or_zero(s.pressure_pa)); Sensor_WriteFloat(f.payload+8,value_or_zero(s.temperature_c));
            Sensor_WriteFloat(f.payload+12,value_or_zero(s.depth_raw_m)); Sensor_WriteFloat(f.payload+16,value_or_zero(s.depth_filtered_m));
            Sensor_WriteFloat(f.payload+20,value_or_zero(s.surface_pressure_pa));
            Sensor_Write32(f.payload+24,s.d1); Sensor_Write32(f.payload+28,s.d2);
            if(send_stream(&f)) depth_sent=s.sequence;
        }
    }
}
