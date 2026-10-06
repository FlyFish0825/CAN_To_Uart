/* Host-only test of the production shared USB router: no MCU registers or devices. */
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#define __MAIN_H
#define __USBD_CDC_IF_H__
typedef int HAL_StatusTypeDef;
typedef struct {int unused;} FDCAN_HandleTypeDef;
#define HAL_OK 0
#define HAL_ERROR 1
#define HAL_BUSY 2
#define USBD_OK 0
#define USBD_BUSY 1
#define UNUSED(x) ((void)(x))
#define __DMB() ((void)0)
static uint32_t tick;
uint32_t HAL_GetTick(void){return tick;}
uint8_t CDC_Transmit_FS(uint8_t *p,uint16_t n){(void)p;(void)n;return USBD_OK;}
uint8_t CDC_ResumeReceive_FS(void){return USBD_OK;}
void CDC_ResetTransmitState_FS(void){}
#include "../Core/Src/usb_can_gateway.c"

static uint8_t can_bytes[1024],flow_bytes[1024],sensor_bytes[1024];
static uint16_t can_n,flow_n,sensor_n;
static uint8_t can_ready=1U;
static unsigned checks,failures;
#define CHECK(x) do{++checks;if(!(x)){++failures;printf("line %d: %s\n",__LINE__,#x);}}while(0)
void CanGateway_RxFeed(const uint8_t *d,uint16_t n){CHECK(can_n+n<=sizeof(can_bytes));if(can_n+n<=sizeof(can_bytes)){memcpy(can_bytes+can_n,d,n);can_n=(uint16_t)(can_n+n);}}
void FirmwareFlow_RxFeed(const uint8_t *d,uint16_t n){CHECK(flow_n+n<=sizeof(flow_bytes));if(flow_n+n<=sizeof(flow_bytes)){memcpy(flow_bytes+flow_n,d,n);flow_n=(uint16_t)(flow_n+n);}}
void SensorProtocol_RxFeed(const uint8_t *d,uint16_t n,uint32_t now){(void)now;CHECK(sensor_n+n<=sizeof(sensor_bytes));if(sensor_n+n<=sizeof(sensor_bytes)){memcpy(sensor_bytes+sensor_n,d,n);sensor_n=(uint16_t)(sensor_n+n);}}
uint8_t CanGateway_CanTxReady(void){return can_ready;}
void CanGateway_GetQueueUsage(uint8_t *rx,uint8_t *tx){*rx=0;*tx=0;}
uint8_t FirmwareFlow_GetQueueUsage(void){return 0;}
static void reset(void){
    UsbCanGateway_RouteReset();usb_protocol_route_last_tick=tick;
    usb_can_rx_head=usb_can_rx_tail=0;usb_can_tx_head=usb_can_tx_tail=0;
    usb_can_tx_busy=usb_can_rx_paused=0;can_ready=1;can_n=flow_n=sensor_n=0;
}
static void feed(const uint8_t *p,unsigned n){
    for(unsigned i=0;i<n;i++){UsbCanGateway_RxPush(p+i,1);UsbCanGateway_Process();}
}
int main(void){
    /* The router does not check CRC; individual protocol tests do. The embedded
     * AA55 frame is never allowed to reach CAN parsing from a valid-sized AA5B. */
    const uint8_t can[22]={0xAA,0x55,0x10,1,0,0x23,1,0,0,0,8,1,2,3,4,5,6,7,8,0,0x55,0xAA};
    uint8_t sensor[42]={0xAA,0x5B,1,4,1,2};sensor[10]=22;memcpy(sensor+16,can,22);sensor[40]=0x5B;sensor[41]=0xAA;
    uint8_t flow[42];memcpy(flow,sensor,sizeof(flow));flow[1]=0x59;flow[40]=0x59;
    reset();feed(sensor,sizeof(sensor));
    CHECK(sensor_n==sizeof(sensor)&&memcmp(sensor_bytes,sensor,sizeof(sensor))==0);
    CHECK(can_n==0&&flow_n==0);
    reset();feed(flow,sizeof(flow));CHECK(flow_n==sizeof(flow)&&sensor_n==0&&can_n==0);
    reset();feed(can,sizeof(can));CHECK(can_n==sizeof(can)&&flow_n==0&&sensor_n==0);
    reset();feed(sensor,sizeof(sensor));feed(can,sizeof(can));feed(flow,sizeof(flow));
    CHECK(sensor_n==sizeof(sensor)&&can_n==sizeof(can)&&flow_n==sizeof(flow));
    CHECK(memcmp(can_bytes,can,sizeof(can))==0);
    reset();UsbCanGateway_RxPush(sensor,7);UsbCanGateway_Process();CHECK(sensor_n==7);
    UsbCanGateway_RxPush(sensor+7,(uint16_t)(sizeof(sensor)-7));UsbCanGateway_Process();CHECK(sensor_n==sizeof(sensor));
    reset();feed(sensor,10);tick+=1001;feed(can,sizeof(can));CHECK(can_n==sizeof(can));
    CHECK(usb_protocol_route_state==USB_PROTOCOL_WAIT_START);
    reset();uint8_t invalid[16]={0xAA,0x5B,1,1,1,2};invalid[10]=59;
    feed(invalid,sizeof(invalid));CHECK(usb_protocol_route_state==USB_PROTOCOL_WAIT_START);
    feed(can,sizeof(can));CHECK(can_n==sizeof(can));
    reset();uint8_t noise[3]={0x19,0xAA,0xAA};feed(noise,sizeof(noise));
    /* Final AA can begin a new frame; feed remaining bytes without duplicating it. */
    feed(sensor+1,sizeof(sensor)-1);CHECK(sensor_n==sizeof(sensor));
    CHECK(memcmp(sensor_bytes,sensor,sizeof(sensor))==0);
    reset();can_ready=0;UsbCanGateway_RxPush(sensor,sizeof(sensor));UsbCanGateway_Process();CHECK(sensor_n==0);
    can_ready=1;UsbCanGateway_Process();CHECK(sensor_n==sizeof(sensor));
    uint32_t epoch=UsbCanGateway_GetConnectionEpoch();UsbCanGateway_OnDeconfigured();
    CHECK(UsbCanGateway_GetConnectionEpoch()!=epoch);
    printf("USB AA55/AA59/AA5B route: %u checks, %u failures\n",checks,failures);
    return failures?1:0;
}
