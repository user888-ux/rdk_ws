#pragma once

/**
 * @file serial_communicator.hpp
 * @brief ROS2串口通信封装类
 * 
 * 基于Boost.Asio实现的异步串口通信类，提供与STM32等嵌入式设备的串口通信能力。
 * 支持异步数据接收和同步数据发送，与ROS2节点深度集成。
 */

#include <boost/asio.hpp>
#include <memory>
#include <string>
#include <functional>
#include <thread>
#include <rclcpp/rclcpp.hpp>

/**
 * @class SerialCommunicator
 * @brief 串口通信封装类，提供ROS2与串口设备的通信接口
 * 
 * 使用Boost.Asio实现异步串口通信，支持：
 * - 异步数据接收（非阻塞）
 * - 同步数据发送
 * - 自动重连机制
 * - ROS2日志集成
 * 
 * @note 线程安全：发送操作是线程安全的，接收回调在io_service线程中执行
 * @warning 析构时会自动断开连接，确保在ROS2节点销毁前析构本对象
 * 
 * @example
 * // 在ROS2节点中使用示例：
 * SerialCommunicator serial(node_ptr);
 * serial.setReceiveCallback([](const std::string& data) {
 *     // 处理接收到的数据
 * });
 * serial.connect("/dev/ttyUSB0", 115200);
 * serial.send("Hello STM32\n");
 */
using BinaryReceiveCallback = std::function<void(const uint8_t* data, size_t length)>;

class SerialCommunicator {
public:
    /// @brief 数据接收回调函数类型定义
    using ReceiveCallback = std::function<void(const std::string&)>;
    
    /**
     * @brief 构造函数
     * @param node 关联的ROS2节点指针，用于日志输出
     * @warning 必须传入有效的节点指针，生命周期需长于本对象
     */
    explicit SerialCommunicator(rclcpp::Node* node);
    
    /**
     * @brief 析构函数
     * @note 会自动调用disconnect()断开连接
     */
    ~SerialCommunicator();
    
    // 禁止拷贝和赋值
    SerialCommunicator(const SerialCommunicator&) = delete;
    SerialCommunicator& operator=(const SerialCommunicator&) = delete;
    
    /**
     * @brief 连接串口设备
     * @param port 串口设备路径，如 "/dev/ttyUSB0" 或 "COM3"
     * @param baud_rate 波特率，默认115200
     * @return true-连接成功, false-连接失败
     * @note 连接成功后会自动启动异步读取循环
     * @warning 如果串口设备不存在或无权限，会返回false并记录错误日志
     */
    bool connect(const std::string& port, unsigned int baud_rate = 115200);
    
    /**
     * @brief 断开串口连接
     * @note 线程安全，可多次调用
     */
    void disconnect();
    
    // 发送二进制数据
    bool send(const uint8_t* data, size_t size);
    bool send(const std::vector<uint8_t>& data);
    
    /**
     * @brief 检查串口连接状态
     * @return true-已连接, false-未连接
     */
    bool isConnected() const { return serial_port_ && serial_port_->is_open(); }

    // 新增：设置二进制接收回调
    void setBinaryCallback(BinaryReceiveCallback callback){binary_callback_ = std::move(callback);}
    // 新增：发送二进制数据
    bool sendBinary(const uint8_t* data, size_t size);
    // 便捷重载
    bool sendBinary(const std::vector<uint8_t>& data);
    
private:
    /**
     * @brief 启动异步读取操作
     * @note 内部使用，在连接成功后自动调用
     */
    void startAsyncRead();
    
    /**
     * @brief 异步读取完成回调
     * @param error 错误码
     * @param bytes_transferred 实际读取的字节数
     * @note 内部使用，处理接收到的数据并重新启动读取
     */
    void handleRead(const boost::system::error_code& error, size_t bytes_transferred);
    
private:
    rclcpp::Node* node_;                                ///< 关联的ROS2节点，用于日志输出
    std::unique_ptr<boost::asio::io_service> io_service_; ///< ASIO IO服务，管理异步操作
    std::unique_ptr<boost::asio::serial_port> serial_port_; ///< 串口对象
    std::thread io_thread_;                             ///< IO服务运行线程
    std::array<char, 1024> read_buffer_;                ///< 数据读取缓冲区
    ReceiveCallback receive_callback_;                  ///< 数据接收回调函数
    BinaryReceiveCallback binary_callback_;             ///< 二进制回调函数对象
};

// 编码规范提示：
// 1. 使用RAII模式管理资源，构造函数获取资源，析构函数释放
// 2. 使用智能指针管理动态资源，避免内存泄漏
// 3. 回调函数使用std::function，提供灵活的接口
// 4. 异步操作使用Boost.Asio，避免阻塞ROS2主线程
// 5. 充分使用ROS2日志系统，便于调试和问题追踪
// 6. 提供完整的错误处理机制