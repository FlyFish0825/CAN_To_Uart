#include "sensor_board.h"
#include "sensor_service.h"
#include "sensor_i2c_bus.h"
#include "ms5837.h"
#include "usb_can_gateway.h"
#include "usb_device.h"
#include "dma.h"
#include "usart.h"
#include <string.h>

/* CubeMX declares this handle in usb_device.c, not in its public header. */
extern USBD_HandleTypeDef hUsbDeviceFS;
static I2C_HandleTypeDef depth_i2c;
static uint32_t usb_epoch;
#if SENSOR_IMU_UART_ENABLED
#define IMU_DMA_SIZE 512U
static uint8_t imu_dma[IMU_DMA_SIZE] __attribute__((section(".dma_buffer"),aligned(32)));
static uint16_t imu_tail;
static uint32_t imu_last_poll;
static uint8_t imu_started;
static volatile uint32_t imu_rx_restart_count;

static HAL_StatusTypeDef start_imu_rx(void)
{
    /* D2 SRAM is accessible by DMA1. CPU never writes this buffer while DMA owns it. */
    memset(imu_dma,0,sizeof(imu_dma));
    if((SCB->CCR & SCB_CCR_DC_Msk)!=0U)
        SCB_CleanInvalidateDCache_by_Addr((uint32_t *)imu_dma,(int32_t)sizeof(imu_dma));
    imu_tail=0U; imu_last_poll=HAL_GetTick();
    return HAL_UART_Receive_DMA(&huart1,imu_dma,(uint16_t)sizeof(imu_dma));
}
#endif

static HAL_StatusTypeDef init_depth_i2c(void)
{
    RCC_OscInitTypeDef oscillator={0};
    RCC_PeriphCLKInitTypeDef clock={0};
    GPIO_InitTypeDef gpio={0};
    oscillator.OscillatorType=RCC_OSCILLATORTYPE_HSI;
    oscillator.HSIState=RCC_HSI_ON;
    oscillator.HSICalibrationValue=RCC_HSICALIBRATION_DEFAULT;
    oscillator.PLL.PLLState=RCC_PLL_NONE;
    if(HAL_RCC_OscConfig(&oscillator)!=HAL_OK) return HAL_ERROR;
    clock.PeriphClockSelection=RCC_PERIPHCLK_I2C3;
    clock.I2c123ClockSelection=RCC_I2C123CLKSOURCE_HSI;
    if(HAL_RCCEx_PeriphCLKConfig(&clock)!=HAL_OK) return HAL_ERROR;
    __HAL_RCC_GPIOA_CLK_ENABLE(); __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_I2C3_CLK_ENABLE();
    __HAL_RCC_I2C3_FORCE_RESET(); __HAL_RCC_I2C3_RELEASE_RESET();
    gpio.Pin=GPIO_PIN_8; gpio.Mode=GPIO_MODE_AF_OD; gpio.Pull=GPIO_PULLUP;
    gpio.Speed=GPIO_SPEED_FREQ_LOW; gpio.Alternate=GPIO_AF4_I2C3;
    HAL_GPIO_Init(GPIOA,&gpio);
    gpio.Pin=GPIO_PIN_9; HAL_GPIO_Init(GPIOC,&gpio);
    depth_i2c.Instance=I2C3;
    /* HSI64MHz, PRESC15 => 250ns. SCLL=24 ticks (6us), SCLH=20 (5us),
     * SCLDEL=6 ticks (1.5us), SDADEL=2 (0.5us), analog filter ON.
     * Conservative standard-mode (<100kHz, roughly 80-90kHz including edges),
     * not a claim of exact measured SCL. External 3.3V pull-ups are required. */
    depth_i2c.Init.Timing=0xF0521317U;
    depth_i2c.Init.OwnAddress1=0U;
    depth_i2c.Init.AddressingMode=I2C_ADDRESSINGMODE_7BIT;
    depth_i2c.Init.DualAddressMode=I2C_DUALADDRESS_DISABLE;
    depth_i2c.Init.OwnAddress2=0U;
    depth_i2c.Init.OwnAddress2Masks=I2C_OA2_NOMASK;
    depth_i2c.Init.GeneralCallMode=I2C_GENERALCALL_DISABLE;
    depth_i2c.Init.NoStretchMode=I2C_NOSTRETCH_DISABLE;
    if(HAL_I2C_Init(&depth_i2c)!=HAL_OK) return HAL_ERROR;
    if(HAL_I2CEx_ConfigAnalogFilter(&depth_i2c,I2C_ANALOGFILTER_ENABLE)!=HAL_OK) return HAL_ERROR;
    return HAL_I2CEx_ConfigDigitalFilter(&depth_i2c,0U);
}

HAL_StatusTypeDef SensorBoard_Init(void)
{
    HAL_StatusTypeDef depth_status=init_depth_i2c();
    I2c_Init(depth_status==HAL_OK?&depth_i2c:NULL);
    (void)Ms5837_Init(); /* Missing sensor is an observable offline state, not a fatal error. */
    usb_epoch=UsbCanGateway_GetConnectionEpoch();
#if SENSOR_IMU_UART_ENABLED
    MX_DMA_Init();
    MX_USART1_UART_Init();
    imu_started=start_imu_rx()==HAL_OK;
    ImuSensor_SetPinsBlocked((imu_started && SENSOR_IMU_UART_TX_ENABLED)?0U:1U);
#endif
    return depth_status;
}
int SensorBoard_ImuTransmit(void *context,const uint8_t *data,uint16_t length)
{
    (void)context;
#if SENSOR_IMU_UART_ENABLED && SENSOR_IMU_UART_TX_ENABLED
    if(!imu_started || ImuSensor_IsPinsBlocked() || !data || !length || length>16U) return -1;
    /* Command <=16B, <=1.4ms at115200. Return success only after actual TX completes,
     * not when merely queued for DMA. Native no-ACK commands still yield UNCONFIRMED. */
    return HAL_UART_Transmit(&huart1,(uint8_t *)data,length,5U)==HAL_OK?0:-1;
#else
    (void)data; (void)length;
    return -1;
#endif
}
int SensorBoard_UsbSend(const uint8_t *packet,uint16_t length)
{
    if(hUsbDeviceFS.dev_state!=USBD_STATE_CONFIGURED) return -1;
    uint8_t tx_used=0U;
    UsbCanGateway_GetBufferUsage(NULL,&tx_used);
    /* Bound sensor-induced queue latency while leaving room for reliable CAN/upgrade traffic. */
    if(tx_used>=25U) return -1;
    return UsbCanGateway_TxEnqueue(packet,length)==HAL_OK?0:-1;
}
void SensorBoard_Process(void)
{
    uint32_t epoch=UsbCanGateway_GetConnectionEpoch();
    if(epoch!=usb_epoch) { SensorService_SetLink(0U); usb_epoch=epoch; }
    SensorService_SetLink(hUsbDeviceFS.dev_state==USBD_STATE_CONFIGURED?1U:0U);
#if SENSOR_IMU_UART_ENABLED
    if(!imu_started) return;
    uint32_t now=HAL_GetTick();
    /* A main-loop pause approaching a complete DMA lap cannot be reconstructed.
     * Restart and report a gap rather than splice overwritten bytes into a frame. */
    if(huart1.ErrorCode!=HAL_UART_ERROR_NONE || (uint32_t)(now-imu_last_poll)>=40U) {
        (void)HAL_UART_AbortReceive(&huart1); imu_rx_restart_count++;
        imu_started=start_imu_rx()==HAL_OK;
        if(!imu_started) ImuSensor_SetPinsBlocked(1U);
        return;
    }
    imu_last_poll=now;
    uint16_t head=(uint16_t)((IMU_DMA_SIZE-__HAL_DMA_GET_COUNTER(huart1.hdmarx))%IMU_DMA_SIZE);
    if(head==imu_tail) return;
    if((SCB->CCR & SCB_CCR_DC_Msk)!=0U)
        SCB_InvalidateDCache_by_Addr((uint32_t *)imu_dma,(int32_t)sizeof(imu_dma));
    __DMB();
    uint64_t timestamp=SensorService_NowUs();
    if(head<imu_tail) {
        (void)ImuSensor_Feed(imu_dma+imu_tail,(uint16_t)(IMU_DMA_SIZE-imu_tail),timestamp);
        imu_tail=0U;
    }
    if(head>imu_tail) (void)ImuSensor_Feed(imu_dma+imu_tail,(uint16_t)(head-imu_tail),timestamp);
    imu_tail=head;
#endif
}
