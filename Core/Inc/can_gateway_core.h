#ifndef __CAN_GATEWAY_CORE_H__
#define __CAN_GATEWAY_CORE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/**
 * @brief 电脑侧传输驱动向 CAN 网关核心返回的发送结果。
 *
 * 该枚举故意不使用 USB/UART 的具体返回值，协议核心只关心三种结果：
 * - CAN_GATEWAY_IO_OK：数据已经被传输驱动复制到自己的发送队列，调用者
 *   可以立即复用传入的 data 缓冲区；这不表示 USB 线缆已经发送完成。
 * - CAN_GATEWAY_IO_BUSY：驱动当前没有空闲队列槽位，数据没有被接收，
 *   上层应记录一次未发送或稍后重试，不能假设数据已经保存。
 * - CAN_GATEWAY_IO_ERROR：参数错误、设备未配置或驱动发生不可恢复错误，
 *   数据同样没有被接收。
 *
 * 发送函数必须是非阻塞函数，不能在中断/主循环中等待 USB 传输完成。
 */
typedef enum
{
  CAN_GATEWAY_IO_OK = 0,
  CAN_GATEWAY_IO_BUSY,
  CAN_GATEWAY_IO_ERROR
} CanGatewayIoResult_t;

/**
 * @brief 应用/协议核心调用的“向电脑发送一整个协议包”函数类型。
 *
 * @param context 传输驱动的私有上下文指针。当前 USB CDC 不需要上下文，
 *                因此传入 NULL；若以后增加多个串口/网络实例，可在这里
 *                传入对应实例对象，而不必修改协议核心。
 * @param data    待发送数据首地址。数据只在本次调用期间保证有效，驱动
 *                若要异步发送，必须在函数返回前把数据复制到自己的队列。
 * @param length  data 中有效字节数，不能为 0，最大值由具体传输层约束。
 * @return        CAN_GATEWAY_IO_OK/BUSY/ERROR，含义见 CanGatewayIoResult_t。
 *
 * 这是协议核心与电脑物理接口之间的唯一发送依赖。核心不直接包含 USB
 * 或 UART 驱动头文件，因此替换为 UART、TCP 或测试桩时只需实现此函数。
 */
typedef CanGatewayIoResult_t (*CanGatewaySendPacketFn)(
    void *context,
    const uint8_t *data,
    uint16_t length);

/**
 * @brief 一组电脑侧传输适配器操作。
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

/**
 * @brief 注册电脑传输适配器并初始化 CAN 网关核心。
 *
 * @param transport 已实现的传输操作表，至少要提供 send_packet 函数。
 * @return HAL_OK 表示接口已注册且 CRC/FDCAN/协议状态初始化成功；
 *         HAL_ERROR 表示参数为空或底层 FDCAN 初始化失败，此时不应进入
 *         正常网关循环。
 *
 * 调用时机：必须在 MX_FDCAN1_Init()、MX_USB_DEVICE_Init() 等底层初始化
 * 完成后调用一次，通常放在 main() 的 USER CODE BEGIN 2 区域。
 */
HAL_StatusTypeDef CanGateway_Init(const CanGatewayTransportOps_t *transport);

/**
 * @brief 执行一次 CAN 网关核心轮询。
 *
 * 主循环应持续调用此函数。它负责处理 USB 输入已经交给核心的协议帧、
 * 将合法电脑命令放入 CAN 软件队列、向 FDCAN TX FIFO 提交报文、处理波特
 * 率配置以及把 CAN RX 队列封装成电脑协议包。函数不主动读取 USB 字节，
 * USB 字节由 UsbCanGateway_Process() 取出后通过 CanGateway_RxFeed() 输入。
 */
void CanGateway_Process(void);

/**
 * @brief 把电脑侧收到的字节流喂给 AA55 协议解析器。
 *
 * @param data    输入字节数组，可以是半帧、完整帧或多帧粘包。
 * @param length  输入字节数。解析器会保留跨调用的状态，因此不要求一次
 *                调用恰好对应一帧；传输层可以按 DMA/USB 回调的实际分包
 *                大小多次调用。
 *
 * 此函数只负责协议解析和入队，不直接访问 USB 或 FDCAN 硬件。data 在本次
 * 调用返回后即可释放，解析器只会把需要的数据复制到内部缓冲区。
 */
void CanGateway_RxFeed(const uint8_t *data, uint16_t length);

#ifdef __cplusplus
}
#endif

#endif /* __CAN_GATEWAY_CORE_H__ */
