#ifndef __CAN_GATEWAY_CORE_H__
#define __CAN_GATEWAY_CORE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/**
 * @brief 外部接口向 CAN 网关核心返回的发送结果。
 *
 * 该枚举不绑定任何具体传输实现，协议核心只关心三种结果：
 * - CAN_GATEWAY_IO_OK：数据已经被传输驱动复制到自己的发送队列，调用者
 *   可以立即复用传入的 data 缓冲区；这不表示物理链路已经发送完成。
 * - CAN_GATEWAY_IO_BUSY：驱动当前没有空闲队列槽位，数据没有被接收，
 *   上层应记录一次未发送或稍后重试，不能假设数据已经保存。
 * - CAN_GATEWAY_IO_ERROR：参数错误、设备未配置或驱动发生不可恢复错误，
 *   数据同样没有被接收。
 *
 * 发送函数必须是非阻塞函数，不能在中断/主循环中等待物理传输完成。
 */
typedef enum
{
  CAN_GATEWAY_IO_OK = 0,
  CAN_GATEWAY_IO_BUSY,
  CAN_GATEWAY_IO_ERROR
} CanGatewayIoResult_t;

/**
 * @brief 应用/协议核心调用的“向外部设备发送一整个协议包”函数类型。
 *
 * @param context 传输驱动的私有上下文指针。不需要上下文时传入 NULL；
 *                如果系统存在多个传输实例，可在这里传入对应实例对象，
 *                而不必修改协议核心。
 * @param data    待发送数据首地址。数据只在本次调用期间保证有效，驱动
 *                若要异步发送，必须在函数返回前把数据复制到自己的队列。
 * @param length  data 中有效字节数，不能为 0，最大值由具体传输层约束。
 * @return        CAN_GATEWAY_IO_OK/BUSY/ERROR，含义见 CanGatewayIoResult_t。
 *
 * 这是协议核心与外部设备之间的唯一发送依赖。核心不包含任何具体传输
 * 驱动头文件，因此更换传输实现时只需重新实现此函数。
 */
typedef CanGatewayIoResult_t (*CanGatewaySendPacketFn)(
    void *context,
    const uint8_t *data,
    uint16_t length);

/**
 * @brief 一组外部传输接口操作。
 *
 * 初始化时由具体驱动填写此结构体并传给 CanGateway_Init()。初始化函数会
 * 复制结构体内容，所以调用方传入的结构体不要求一直保持在栈上；但其中
 * 的 context 指针所指向的对象必须在网关运行期间持续有效。
 */
typedef struct
{
  CanGatewaySendPacketFn send_packet;
  void *context;
} CanGatewayTransportOps_t;

/*
 * ========================= 最小使用示例 =========================
 * 任何新的外部接口只需完成下面 4 步即可接入核心：
 *
 * 1. 实现一个发送函数。函数必须快速返回，成功时把 data[0..length-1]
 *    复制到自己的发送缓存，并返回 CAN_GATEWAY_IO_OK。
 * 2. 填写 CanGatewayTransportOps_t；不需要私有对象时 context 填 NULL。
 * 3. 在底层外设初始化完成后调用一次 CanGateway_Init(&ops)。
 * 4. 主循环持续调用 CanGateway_Process()；收到外部字节时调用
 *    CanGateway_RxFeed(receive_buffer, receive_length)。
 *
 * 示例（发送函数名称和缓存实现由接入方自行决定）：
 *
 *   static CanGatewayIoResult_t App_Send(void *context,
 *                                        const uint8_t *data,
 *                                        uint16_t length)
 *   {
 *     (void)context;
 *     if (App_TxQueuePut(data, length) == 0) {
 *       return CAN_GATEWAY_IO_BUSY;
 *     }
 *     return CAN_GATEWAY_IO_OK;
 *   }
 *
 *   static const CanGatewayTransportOps_t app_transport = {
 *     App_Send,
 *     NULL
 *   };
 *
 *   // 底层初始化完成后执行一次
 *   CanGateway_Init(&app_transport);
 *
 *   // 主循环中持续执行
 *   CanGateway_RxFeed(rx_data, rx_length); // 有数据时调用
 *   CanGateway_Process();                  // 每轮都调用
 *
 * 核心会复制传输操作表，但不会复制 context 指向的对象；接入方必须保证
 * context 在整个网关运行期间有效。send_packet 返回 OK 后，核心立即允许
 * 复用传入缓冲区，因此异步驱动必须在返回前完成数据复制。
 */

/**
 * @brief 注册外部传输接口并初始化 CAN 网关核心。
 *
 * @param transport 已实现的传输操作表，至少要提供 send_packet 函数。
 * @return HAL_OK 表示接口已注册且 CRC/FDCAN/协议状态初始化成功；
 *         HAL_ERROR 表示参数为空或底层 FDCAN 初始化失败，此时不应进入
 *         正常网关循环。
 *
 * 调用时机：必须在 FDCAN 及所选传输接口的底层初始化完成后调用一次，
 * 通常放在 main() 的 USER CODE BEGIN 2 区域。
 */
HAL_StatusTypeDef CanGateway_Init(const CanGatewayTransportOps_t *transport);

/**
 * @brief 执行一次 CAN 网关核心轮询。
 *
 * 主循环应持续调用此函数。它负责处理外部接口已经交给核心的协议帧、
 * 将合法命令放入 CAN 软件队列、向 FDCAN TX FIFO 提交报文、处理波特率
 * 配置以及把 CAN RX 队列封装成协议包。函数不主动读取外部接口字节，
 * 接收驱动应先取出数据，再通过 CanGateway_RxFeed() 输入。
 */
void CanGateway_Process(void);

/**
 * @brief 把电脑侧收到的字节流喂给 AA55 协议解析器。
 *
 * @param data    输入字节数组，可以是半帧、完整帧或多帧粘包。
 * @param length  输入字节数。解析器会保留跨调用的状态，因此不要求一次
 *                调用恰好对应一帧；接收驱动可以按 DMA 或回调的实际分包
 *                大小多次调用。
 *
 * 此函数只负责协议解析和入队，不直接访问具体传输硬件。data 在本次调用
 * 返回后即可释放，解析器只会把需要的数据复制到内部缓冲区。
 */
void CanGateway_RxFeed(const uint8_t *data, uint16_t length);

/**
 * @brief 查询外部输入是否还可以安全进入 CAN 软件发送队列。
 *
 * 返回 0 时，传输层应暂缓继续接收新的完整外部报文，等待主循环把
 * 已排队报文提交给 FDCAN。该查询只用于流控，不改变 AA55 协议格式。
 */
uint8_t CanGateway_CanTxReady(void);

#ifdef __cplusplus
}
#endif

#endif /* __CAN_GATEWAY_CORE_H__ */
