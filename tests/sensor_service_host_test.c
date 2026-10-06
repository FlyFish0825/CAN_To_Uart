#include "sensor_service.h"
#include "ms5837.h"
#include "sensor_i2c_bus.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

static uint32_t tick, host_sequence;
static unsigned checks, failures, uart_count, frame_count;
static uint8_t native_tx[16], last_command, conversion;
static uint16_t prom[8];
static uint8_t bus_failure;
static SensorFrame frames[4096];
#define CHECK(x) do {checks++;if(!(x)){failures++;printf("line %d: %s\n",__LINE__,#x);}}while(0)
uint32_t HAL_GetTick(void) {return tick;}
HAL_StatusTypeDef HAL_I2C_Master_Transmit(I2C_HandleTypeDef *h,uint16_t addr,uint8_t *d,uint16_t n,uint32_t timeout)
{
    (void)h; CHECK(addr==0xECU && timeout<=5U && n==1U);
    if(bus_failure) return HAL_TIMEOUT;
    last_command=d[0];
    if((d[0]&0xF0U)==0x40U) conversion=1U;
    if((d[0]&0xF0U)==0x50U) conversion=2U;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_I2C_Master_Receive(I2C_HandleTypeDef *h,uint16_t addr,uint8_t *d,uint16_t n,uint32_t timeout)
{
    (void)h; CHECK(addr==0xECU && timeout<=5U);
    if(bus_failure) return HAL_TIMEOUT;
    if(last_command>=0xA0U && last_command<=0xACU && n==2U) {
        uint16_t v=prom[(last_command-0xA0U)/2U]; d[0]=(uint8_t)(v>>8);d[1]=(uint8_t)v;return HAL_OK;
    }
    if(last_command==0 && n==3U) {
        uint32_t v=conversion==1U?4958179U:6815414U;
        d[0]=(uint8_t)(v>>16);d[1]=(uint8_t)(v>>8);d[2]=(uint8_t)v;return HAL_OK;
    }
    return HAL_ERROR;
}
static int tx_uart(void *ctx,const uint8_t *p,uint16_t n)
{
    (void)ctx;CHECK(n<=sizeof(native_tx)); if(n<=sizeof(native_tx))memcpy(native_tx,p,n); uart_count++;return 0;
}
static int tx_host(const uint8_t *p,uint16_t n)
{
    CHECK(n<=78U && frame_count<4096U);
    if(frame_count>=4096U)return -1;
    CHECK(SensorProtocol_Decode(p,n,&frames[frame_count]));frame_count++;return 0;
}
static void run(unsigned ms)
{
    while(ms--) {tick++;SensorService_Process(tick);}
}
static unsigned command(uint8_t target,uint8_t cmd,const uint8_t *payload,uint16_t n)
{
    SensorFrame f;memset(&f,0,sizeof(f));f.target=target;f.command=cmd;f.flags=1;f.sequence=++host_sequence;f.length=n;
    if(n)memcpy(f.payload,payload,n);
    uint8_t packet[SENSOR_MAX_PACKET];uint16_t len=SensorProtocol_Encode(&f,packet);
    unsigned first=frame_count;
    SensorProtocol_RxFeed(packet,len,tick);run(2);
    return first;
}
static SensorFrame *find_reply(unsigned first,uint8_t cmd)
{
    for(unsigned i=first;i<frame_count;i++)
        if(frames[i].command==(uint8_t)(cmd+0x40U) && frames[i].sequence==host_sequence) return &frames[i];
    return NULL;
}
static uint8_t result(unsigned first,uint8_t cmd)
{
    SensorFrame *f=find_reply(first,cmd); CHECK(f!=NULL);return f?f->payload[0]:255U;
}
static unsigned set_u16(uint8_t target,uint16_t id,uint16_t value)
{
    uint8_t p[6];Sensor_Write16(p,id);p[2]=4;p[3]=2;Sensor_Write16(p+4,value);
    return command(target,SENSOR_SET_PARAMETER,p,6);
}
static unsigned set_float(uint16_t id,float value)
{
    uint8_t p[8];Sensor_Write16(p,id);p[2]=7;p[3]=4;Sensor_WriteFloat(p+4,value);
    return command(2,SENSOR_SET_PARAMETER,p,8);
}
static unsigned stream_count(uint8_t target,uint8_t cmd,unsigned first)
{
    unsigned n=0;for(unsigned i=first;i<frame_count;i++)if(frames[i].flags==8&&frames[i].target==target&&frames[i].command==cmd)n++;return n;
}
static void native_feed(uint8_t cmd,const uint8_t *payload,uint8_t n)
{
    uint8_t p[32]={0x7E,0x23,0,0};p[2]=(uint8_t)(n+5U);p[3]=cmd;
    memcpy(p+4,payload,n);uint8_t sum=0;for(unsigned i=0;i<n+4U;i++)sum=(uint8_t)(sum+p[i]);p[n+4U]=sum;
    CHECK(ImuSensor_Feed(p,(uint16_t)(n+5U),SensorService_NowUs())==(int)(n+5U));run(2);
}
int main(void)
{
    I2C_HandleTypeDef handle={0};
    uint16_t coefficients[8]={0,34982,36352,20328,22354,26646,26146,0};memcpy(prom,coefficients,sizeof(prom));
    prom[0]=(uint16_t)(Ms5837_Crc4(prom)<<12U);
    SensorService_Init(tx_host,tx_uart);I2c_Init(&handle);CHECK(Ms5837_Init()==MS5837_OK);SensorService_SetLink(1);
    unsigned first=command(1,SENSOR_GET_INFO,NULL,0);CHECK(result(first,SENSOR_GET_INFO)==SENSOR_OK);
    CHECK(find_reply(first,SENSOR_GET_INFO)->length==31U);
    first=command(1,SENSOR_GET_STATUS,NULL,0);CHECK(result(first,SENSOR_GET_STATUS)==SENSOR_OK);
    CHECK(Sensor_Read32(find_reply(first,SENSOR_GET_STATUS)->payload+1)&IMU_SENSOR_STATUS_PIN_BLOCKED);
    first=set_u16(1,1,25);CHECK(result(first,SENSOR_SET_PARAMETER)==SENSOR_PIN_BLOCKED&&uart_count==0U);
    first=command(1,SENSOR_SAVE_CONFIG,NULL,0);CHECK(result(first,SENSOR_SAVE_CONFIG)==SENSOR_UNSUPPORTED);
    run(250);Ms5837Sample_t sample;CHECK(Ms5837_GetSample(&sample)==MS5837_OK);
    CHECK(sample.d1==4958179U&&sample.d2==6815414U);
    CHECK((sample.status&MS5837_STATUS_PROM_VALID)!=0U);
    CHECK(!(sample.status&(MS5837_STATUS_PRESSURE_VALID|MS5837_STATUS_DEPTH_VALID)));
    CHECK(stream_count(2,0x82,0)>0);
    uint8_t param[2]={1,0};first=command(2,SENSOR_GET_PARAMETER,param,2);CHECK(result(first,SENSOR_GET_PARAMETER)==SENSOR_OK);
    CHECK(Sensor_Read16(find_reply(first,SENSOR_GET_PARAMETER)->payload+5)==25U);
    first=command(2,SENSOR_ZERO_DEPTH,NULL,0);CHECK(result(first,SENSOR_ZERO_DEPTH)==SENSOR_NOT_READY);
    uint8_t model[5]={5,1,2,1,30};first=command(2,SENSOR_SET_PARAMETER,model,5);CHECK(result(first,SENSOR_SET_PARAMETER)==SENSOR_OK);
    run(100);CHECK(Ms5837_GetSample(&sample)==MS5837_OK);CHECK(sample.status&MS5837_STATUS_PRESSURE_VALID);
    CHECK(!(sample.status&MS5837_STATUS_DEPTH_VALID)); CHECK(fabsf(sample.pressure_pa-399980.0f)<100.0f);
    first=set_float(0x0103,101325.0f);CHECK(result(first,SENSOR_SET_PARAMETER)==SENSOR_OK);run(100);
    CHECK(Ms5837_GetSample(&sample)==MS5837_OK);CHECK(sample.status&MS5837_STATUS_DEPTH_VALID);
    CHECK(fabsf(sample.depth_raw_m-(sample.pressure_pa-101325.0f)/(Ms5837_GetWaterDensity()*9.80665f))<0.001f);
    first=set_float(0x0102,1000.0f);CHECK(result(first,SENSOR_SET_PARAMETER)==SENSOR_OK);
    first=set_float(0x0104,0.99f);CHECK(result(first,SENSOR_SET_PARAMETER)==SENSOR_OK);
    first=set_float(0x0104,NAN);CHECK(result(first,SENSOR_SET_PARAMETER)==SENSOR_BAD_VALUE);
    first=set_u16(2,1,51);CHECK(result(first,SENSOR_SET_PARAMETER)==SENSOR_BAD_VALUE);
    first=set_u16(2,1,50); /* May be rejected by conservative conversion budget. */
    uint8_t res=result(first,SENSOR_SET_PARAMETER);CHECK(res==SENSOR_OK||res==SENSOR_BAD_VALUE);
    first=set_u16(2,0x0101,8192);CHECK(result(first,SENSOR_SET_PARAMETER)==SENSOR_BAD_VALUE);
    first=command(2,SENSOR_STOP_STREAM,NULL,0);CHECK(result(first,SENSOR_STOP_STREAM)==SENSOR_OK);
    first=frame_count;uint32_t before=sample.sequence;run(150);CHECK(stream_count(2,0x82,first)==0U);
    CHECK(Ms5837_GetSample(&sample)==MS5837_OK&&sample.sequence>before);
    first=command(2,SENSOR_START_STREAM,NULL,0);CHECK(result(first,SENSOR_START_STREAM)==SENSOR_OK);run(100);
    CHECK(stream_count(2,0x82,first)>0U);
    ImuSensor_SetPinsBlocked(0); /* HOST MOCK ONLY: no hardware pins exist in this test. */
    first=set_u16(1,1,100);CHECK(result(first,SENSOR_SET_PARAMETER)==SENSOR_UNCONFIRMED&&uart_count==1U);
    CHECK(native_tx[3]==0x60U&&native_tx[4]==100U);
    first=command(1,SENSOR_GET_PARAMETER,param,2);CHECK(result(first,SENSOR_GET_PARAMETER)==SENSOR_UNCONFIRMED);
    first=command(1,SENSOR_GET_INFO,NULL,0);CHECK(find_reply(first,SENSOR_GET_INFO)==NULL&&native_tx[3]==0x80U);
    uint8_t ver[3]={1,2,3};native_feed(1,ver,3);CHECK(result(first,SENSOR_GET_INFO)==SENSOR_OK);
    CHECK(memcmp(find_reply(first,SENSOR_GET_INFO)->payload+23,"1.2.3",5)==0);
    uint8_t calibration[4]={1,1,0,0};first=command(1,SENSOR_CALIBRATE,calibration,4);
    CHECK(find_reply(first,SENSOR_CALIBRATE)==NULL&&native_tx[3]==0x70U);
    uint8_t ack[2]={0x70,1};native_feed(0x81,ack,2);CHECK(result(first,SENSOR_CALIBRATE)==SENSOR_OK);
    calibration[0]=2;first=command(1,SENSOR_CALIBRATE,calibration,4);run(1100);
    CHECK(find_reply(first,SENSOR_CALIBRATE)==NULL); /* Calibration gets 30 s, not the 1 s version timeout. */
    run(30000);
    CHECK(result(first,SENSOR_CALIBRATE)==SENSOR_TIMEOUT);
    bus_failure=1;run(200);CHECK(!(Ms5837_GetStatus()&MS5837_STATUS_ONLINE));
    first=command(2,SENSOR_ZERO_DEPTH,NULL,0);CHECK(result(first,SENSOR_ZERO_DEPTH)!=SENSOR_OK);
    printf("AA5B integrated backends: %u checks, %u failures\n",checks,failures);
    return failures?1:0;
}
