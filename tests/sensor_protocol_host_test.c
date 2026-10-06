#include "sensor_protocol.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
static unsigned checks, failures, commands, output_count;
static int transport_busy, hold_command;
static SensorFrame output[64];
#define CHECK(x) do { checks++; if(!(x)){failures++;printf("line %d: %s\n",__LINE__,#x);}} while(0)
static int send_packet(const uint8_t *data,uint16_t length)
{
    if(transport_busy) return 1;
    CHECK(output_count<64U);
    if(output_count<64U) CHECK(SensorProtocol_Decode(data,length,&output[output_count++]));
    return 0;
}
static void command(const SensorFrame *f,uint8_t token)
{
    commands++;
    if(!hold_command) CHECK(SensorProtocol_Complete(token,SENSOR_OK,f->payload,f->length));
}
static SensorFrame request(uint32_t sequence)
{
    SensorFrame f; memset(&f,0,sizeof(f));
    f.command=SENSOR_GET_INFO; f.target=1U; f.flags=1U; f.sequence=sequence;
    return f;
}
static void feed(const SensorFrame *f,uint32_t now)
{
    uint8_t packet[SENSOR_MAX_PACKET]; uint16_t n=SensorProtocol_Encode(f,packet);
    CHECK(n!=0U); SensorProtocol_RxFeed(packet,n,now);
}
static void reset(void)
{
    commands=output_count=0U; transport_busy=hold_command=0;
    memset(output,0,sizeof(output)); SensorProtocol_Init(send_packet,command);
}
int main(void)
{
    uint8_t packet[SENSOR_MAX_PACKET]; SensorFrame f=request(0x12345678U), decoded;
    const uint8_t golden[]={0xAA,0x5B,1,1,1,1,0x78,0x56,0x34,0x12,0,0,0,0,0,0,0x52,0xD6,0x5B,0xAA};
    CHECK(SensorProtocol_Crc16((const uint8_t *)"123456789",9U)==0x29B1U);
    CHECK(SensorProtocol_Encode(&f,packet)==sizeof(golden)); CHECK(memcmp(packet,golden,sizeof(golden))==0);
    CHECK(SensorProtocol_Decode(golden,sizeof(golden),&decoded)); CHECK(decoded.sequence==f.sequence);
    for(uint16_t split=0;split<=sizeof(golden);split++) {
        reset(); SensorProtocol_RxFeed(golden,split,0U); SensorProtocol_RxFeed(golden+split,(uint16_t)(sizeof(golden)-split),1U);
        SensorProtocol_Process(1U); CHECK(commands==1U&&output_count==1U);
        CHECK(output[0].sequence==0x12345678U&&output[0].command==0x41U&&output[0].flags==2U);
    }
    reset();
    for(uint16_t i=0;i<sizeof(golden);i++) SensorProtocol_RxFeed(golden+i,1U,10U);
    SensorProtocol_Process(10U); CHECK(commands==1U&&output_count==1U);
    SensorProtocol_RxFeed(golden,sizeof(golden),11U); SensorProtocol_Process(11U);
    CHECK(commands==1U&&output_count==2U); /* duplicate receives cached reply, no side effect */
    reset(); transport_busy=1; feed(&f,10); SensorProtocol_Process(10);
    CHECK(commands==1U&&output_count==0U);
    feed(&f,11); CHECK(commands==1U);
    transport_busy=0; SensorProtocol_Process(12); CHECK(output_count==1U);
    reset(); hold_command=1;
    for(uint32_t i=0;i<SENSOR_REPLY_SLOTS+2U;i++){ f=request(i); feed(&f,1); }
    CHECK(commands==SENSOR_REPLY_SLOTS);
    SensorProtocolStats stats; SensorProtocol_GetStats(&stats); CHECK(stats.rejected_full==2U);
    SensorProtocol_Process(44001U); CHECK(output_count==2U&&output[0].payload[0]==SENSOR_TIMEOUT&&output[0].flags==6U);
    reset(); f=request(3); feed(&f,0); SensorProtocol_ResetLink(); SensorProtocol_Process(1);
    CHECK(output_count==0U); feed(&f,2); SensorProtocol_Process(2); CHECK(commands==2U&&output_count==1U);
    reset();
    memcpy(packet,golden,sizeof(golden)); packet[16]^=1U;
    SensorProtocol_RxFeed(packet,sizeof(golden),0); SensorProtocol_RxFeed(golden,sizeof(golden),0); SensorProtocol_Process(0);
    CHECK(commands==1U&&output_count==1U); SensorProtocol_GetStats(&stats); CHECK(stats.bad_frames>0U);
    reset(); SensorProtocol_RxFeed(golden,10U,0); SensorProtocol_RxFeed(golden,sizeof(golden),1001U); SensorProtocol_Process(1001U);
    CHECK(commands==1U&&output_count==1U);
    reset(); f=request(7); f.length=58U; memset(f.payload,0xAA,58U);
    CHECK(SensorProtocol_Encode(&f,packet)==78U); CHECK(SensorProtocol_Decode(packet,78U,&decoded));
    f.length=59U; CHECK(SensorProtocol_Encode(&f,packet)==0U);
    f.length=0; f.target=255U; CHECK(SensorProtocol_Encode(&f,packet)==0U);
    Sensor_WriteFloat(packet,-3.125f); CHECK(fabsf(Sensor_ReadFloat(packet)+3.125f)<1e-6f);
    reset(); f=request(8); f.flags=8U; f.command=0x83U; f.length=20U;
    CHECK(SensorProtocol_SendTelemetry(&f)); CHECK(output_count==1U);
    printf("AA5B firmware protocol: %u checks, %u failures\n",checks,failures);
    return failures?1:0;
}
