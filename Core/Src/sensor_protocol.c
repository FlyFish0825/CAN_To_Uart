#include "sensor_protocol.h"
#include <string.h>

_Static_assert(sizeof(float)==4U, "AA5B requires binary32 float");

typedef struct {
    uint8_t state; /* 0 free, 1 pending backend, 2 complete/retry send */
    uint32_t created_ms;
    SensorFrame request, response;
} ReplySlot;
typedef struct { uint8_t valid; SensorFrame request, response; } ReplyCache;
static ReplySlot slots[SENSOR_REPLY_SLOTS];
static ReplyCache cache[2];
static uint8_t rx[SENSOR_MAX_PACKET];
static uint16_t rx_length;
static uint32_t last_rx_ms, clock_ms;
static SensorSendFn send_callback;
static SensorCommandFn command_callback;
static SensorProtocolStats statistics;

uint16_t Sensor_Read16(const uint8_t *p) { return (uint16_t)p[0] | ((uint16_t)p[1]<<8U); }
uint32_t Sensor_Read32(const uint8_t *p) { return (uint32_t)Sensor_Read16(p) | ((uint32_t)Sensor_Read16(p+2)<<16U); }
void Sensor_Write16(uint8_t *p,uint16_t v) { p[0]=(uint8_t)v; p[1]=(uint8_t)(v>>8U); }
void Sensor_Write32(uint8_t *p,uint32_t v) { Sensor_Write16(p,(uint16_t)v); Sensor_Write16(p+2,(uint16_t)(v>>16U)); }
float Sensor_ReadFloat(const uint8_t *p) { uint32_t bits=Sensor_Read32(p); float v; memcpy(&v,&bits,4U); return v; }
void Sensor_WriteFloat(uint8_t *p,float v) { uint32_t bits; memcpy(&bits,&v,4U); Sensor_Write32(p,bits); }
uint16_t SensorProtocol_Crc16(const uint8_t *p,uint16_t length)
{
    uint16_t crc=0xFFFFU;
    for(uint16_t i=0;i<length;++i) {
        crc^=(uint16_t)p[i]<<8U;
        for(uint8_t bit=0;bit<8U;++bit)
            crc=(uint16_t)((crc&0x8000U)?((uint32_t)crc<<1U)^0x1021U:((uint32_t)crc<<1U));
    }
    return crc;
}
static int valid_header(const SensorFrame *f)
{
    return f && (f->target==1U || f->target==2U) && f->length<=SENSOR_MAX_PAYLOAD &&
           (f->flags==1U || f->flags==2U || f->flags==6U || f->flags==8U);
}
uint16_t SensorProtocol_Encode(const SensorFrame *f,uint8_t out[SENSOR_MAX_PACKET])
{
    if(!out || !valid_header(f)) return 0U;
    out[0]=0xAAU; out[1]=0x5BU; out[2]=1U; out[3]=f->command; out[4]=f->flags; out[5]=f->target;
    Sensor_Write32(out+6,f->sequence); Sensor_Write16(out+10,f->length);
    Sensor_Write32(out+12,f->timestamp_us);
    memcpy(out+16,f->payload,f->length);
    Sensor_Write16(out+16+f->length,SensorProtocol_Crc16(out+1,(uint16_t)(15U+f->length)));
    out[18+f->length]=0x5BU; out[19+f->length]=0xAAU;
    return (uint16_t)(20U+f->length);
}
int SensorProtocol_Decode(const uint8_t *p,uint16_t length,SensorFrame *out)
{
    if(!p || !out || length<20U || p[0]!=0xAAU || p[1]!=0x5BU || p[2]!=1U) return 0;
    uint16_t n=Sensor_Read16(p+10);
    if(n>SENSOR_MAX_PAYLOAD || length!=(uint16_t)(20U+n) || p[18+n]!=0x5BU || p[19+n]!=0xAAU) return 0;
    if((p[5]!=1U && p[5]!=2U) || (p[4]!=1U && p[4]!=2U && p[4]!=6U && p[4]!=8U)) return 0;
    if(Sensor_Read16(p+16+n)!=SensorProtocol_Crc16(p+1,(uint16_t)(15U+n))) return 0;
    memset(out,0,sizeof(*out));
    out->command=p[3]; out->flags=p[4]; out->target=p[5]; out->sequence=Sensor_Read32(p+6);
    out->timestamp_us=Sensor_Read32(p+12); out->length=n; memcpy(out->payload,p+16,n);
    return 1;
}
void SensorProtocol_ResetLink(void)
{
    memset(slots,0,sizeof(slots)); memset(cache,0,sizeof(cache)); rx_length=0U;
}
void SensorProtocol_Init(SensorSendFn send,SensorCommandFn command)
{
    send_callback=send; command_callback=command; clock_ms=0U; last_rx_ms=0U;
    memset(&statistics,0,sizeof(statistics)); SensorProtocol_ResetLink();
}
static int same_request(const SensorFrame *a,const SensorFrame *b)
{
    return a->target==b->target && a->sequence==b->sequence && a->command==b->command &&
           a->length==b->length && memcmp(a->payload,b->payload,a->length)==0;
}
static void dispatch(const SensorFrame *request)
{
    if(request->flags!=1U) return;
    statistics.received++;
    int free_slot=-1;
    for(uint8_t i=0;i<SENSOR_REPLY_SLOTS;++i) {
        if(slots[i].state==0U) { if(free_slot<0) free_slot=(int)i; }
        else if(same_request(&slots[i].request,request)) { statistics.duplicates++; return; }
    }
    // No command is executed unless its response has a reserved slot.
    if(free_slot<0) { statistics.rejected_full++; return; }
    ReplySlot *s=&slots[free_slot];
    memset(s,0,sizeof(*s)); s->state=1U; s->created_ms=clock_ms; s->request=*request;
    s->response.command=(uint8_t)(request->command+0x40U); s->response.target=request->target;
    s->response.sequence=request->sequence; s->response.timestamp_us=clock_ms*1000U;
    ReplyCache *c=&cache[request->target-1U];
    if(c->valid && same_request(&c->request,request)) {
        s->response=c->response; s->state=2U; statistics.duplicates++; return;
    }
    if(command_callback) command_callback(request,(uint8_t)free_slot);
    else (void)SensorProtocol_Complete((uint8_t)free_slot,SENSOR_UNSUPPORTED,NULL,0U);
}
static void discard_first(void)
{
    if(rx_length>0U) { rx_length--; memmove(rx,rx+1,rx_length); }
}
void SensorProtocol_RxFeed(const uint8_t *bytes,uint16_t length,uint32_t now_ms)
{
    if(!bytes) return;
    clock_ms=now_ms;
    if(rx_length && (uint32_t)(now_ms-last_rx_ms)>=1000U) { rx_length=0U; statistics.bad_frames++; }
    last_rx_ms=now_ms;
    for(uint16_t i=0;i<length;++i) {
        if(rx_length>=SENSOR_MAX_PACKET) discard_first();
        rx[rx_length++]=bytes[i];
        while(rx_length) {
            if(rx[0]!=0xAAU) { discard_first(); continue; }
            if(rx_length<2U) break;
            if(rx[1]!=0x5BU) { discard_first(); continue; }
            if(rx_length<12U) break;
            uint16_t n=Sensor_Read16(rx+10);
            if(n>SENSOR_MAX_PAYLOAD || rx[2]!=1U) { statistics.bad_frames++; discard_first(); continue; }
            if(rx_length<(uint16_t)(20U+n)) break;
            SensorFrame f;
            if(SensorProtocol_Decode(rx,rx_length,&f)) { rx_length=0U; dispatch(&f); }
            else { statistics.bad_frames++; discard_first(); }
        }
    }
}
int SensorProtocol_Complete(uint8_t token,uint8_t result,const uint8_t *body,uint16_t length)
{
    if(token>=SENSOR_REPLY_SLOTS || slots[token].state!=1U || result>SENSOR_PIN_BLOCKED ||
       length>=SENSOR_MAX_PAYLOAD || (length && !body)) return 0;
    ReplySlot *s=&slots[token];
    s->response.flags=(result==SENSOR_OK || result==SENSOR_UNCONFIRMED)?2U:6U;
    s->response.timestamp_us=clock_ms*1000U;
    s->response.length=(uint16_t)(length+1U); s->response.payload[0]=result;
    if(length) memcpy(s->response.payload+1,body,length);
    s->state=2U;
    return 1;
}
void SensorProtocol_Process(uint32_t now_ms)
{
    clock_ms=now_ms;
    uint8_t sent=0U;
    for(uint8_t i=0;i<SENSOR_REPLY_SLOTS;++i) {
        if(slots[i].state==1U && (uint32_t)(now_ms-slots[i].created_ms)>=44000U)
            (void)SensorProtocol_Complete(i,SENSOR_TIMEOUT,NULL,0U);
        if(slots[i].state!=2U || !send_callback) continue;
        uint8_t packet[SENSOR_MAX_PACKET];
        uint16_t n=SensorProtocol_Encode(&slots[i].response,packet);
        if(!n || send_callback(packet,n)!=0) { statistics.send_errors++; break; }
        ReplyCache *c=&cache[slots[i].request.target-1U];
        c->valid=1U; c->request=slots[i].request; c->response=slots[i].response;
        slots[i].state=0U; statistics.transmitted++;
        if(++sent>=2U) break;
    }
}
int SensorProtocol_SendTelemetry(const SensorFrame *frame)
{
    if(!frame || frame->flags!=8U || !send_callback) return 0;
    for(uint8_t i=0;i<SENSOR_REPLY_SLOTS;++i) if(slots[i].state==2U) return 0;
    uint8_t packet[SENSOR_MAX_PACKET]; uint16_t n=SensorProtocol_Encode(frame,packet);
    return n && send_callback(packet,n)==0;
}
void SensorProtocol_GetStats(SensorProtocolStats *stats) { if(stats) *stats=statistics; }
