#include "dh_robot_pkg/serial_communicator.hpp"
#include <iostream>

SerialCommunicator::SerialCommunicator(rclcpp::Node* node) 
    : node_(node), io_service_(std::make_unique<boost::asio::io_service>()) {
}

SerialCommunicator::~SerialCommunicator() {
    disconnect();
}

bool SerialCommunicator::connect(const std::string& port, unsigned int baud_rate) {
    try {
        serial_port_ = std::make_unique<boost::asio::serial_port>(*io_service_, port);
        
        // 配置串口参数
        serial_port_->set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
        serial_port_->set_option(boost::asio::serial_port_base::character_size(8));
        serial_port_->set_option(boost::asio::serial_port_base::stop_bits(
            boost::asio::serial_port_base::stop_bits::one));
        serial_port_->set_option(boost::asio::serial_port_base::parity(
            boost::asio::serial_port_base::parity::none));
        serial_port_->set_option(boost::asio::serial_port_base::flow_control(
            boost::asio::serial_port_base::flow_control::none));
        
        RCLCPP_INFO(node_->get_logger(), "Successfully connected to serial port: %s", port.c_str());
        
        // 启动异步读取
        startAsyncRead();
        
        // 在后台线程中运行 io_service，一直等待数据到达，一到就调用回调函数 handleRead
        io_thread_ = std::thread([this]() { 
            io_service_->run(); 
        });
        
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to connect to serial port %s: %s", 
                    port.c_str(), e.what());
        return false;
    }
}

//析构时断开串口
void SerialCommunicator::disconnect() {
    if (io_service_) {
        io_service_->stop();
    }
    
    if (serial_port_ && serial_port_->is_open()) {
        serial_port_->close();
    }
    
    //线程也要安全关闭
    if (io_thread_.joinable()) {
        io_thread_.join();
    }
}

bool SerialCommunicator::send(const uint8_t* data, size_t size) {
    if (!isConnected()) {
        RCLCPP_WARN(node_->get_logger(), "Cannot send: port not connected");
        return false;
    }
    try {
        size_t written = boost::asio::write(*serial_port_, boost::asio::buffer(data, size));
        return written == size;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Send error: %s", e.what());
        return false;
    }
}

bool SerialCommunicator::send(const std::vector<uint8_t>& data) {
    return send(data.data(), data.size());
}

//startAsyncRead()和 handleRead()的相互调用称为 "链式异步读取" 的设计模式(类似于回调设计)，
//这是一种事件驱动架构
void SerialCommunicator::startAsyncRead() {
    if (!isConnected()) return;
    
    serial_port_->async_read_some(
        boost::asio::buffer(read_buffer_),//将 read_buffer_ 包装成Asio可用的缓冲区
        [this](const boost::system::error_code& error, size_t bytes_transferred) {
            //io_service等待得到响应就会给回调函数传入这2个参数
            handleRead(error, bytes_transferred);//回调函数：定义读取完成后的处理逻辑
        }
    );
}

void SerialCommunicator::handleRead(const boost::system::error_code& error, size_t bytes_transferred) {
    if (!error && bytes_transferred > 0) {
        //  触发二进制回调（如果已设置）
        if (binary_callback_) {
            binary_callback_(reinterpret_cast<const uint8_t*>(read_buffer_.data()), bytes_transferred);
        }
        // 日志（可保留，但二进制数据打印字符串可能乱码，建议仅打印长度）
        RCLCPP_DEBUG(node_->get_logger(), "Received %zu bytes from serial", bytes_transferred);
    } else if (error) {
        RCLCPP_ERROR(node_->get_logger(), "Serial read error: %s", error.message().c_str());
    }
    
    // 继续读取
    if (isConnected()) {
        startAsyncRead();
    }
}

bool SerialCommunicator::sendBinary(const uint8_t* data, size_t size) {
    if (!isConnected()) {
        RCLCPP_WARN(node_->get_logger(), "Cannot send binary data: serial port not connected");
        return false;
    }
    try {
        // 使用同步写入，确保数据完整发送
        size_t written = boost::asio::write(*serial_port_, boost::asio::buffer(data, size));
        return written == size;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Error sending binary data: %s", e.what());
        return false;
    }
}

bool SerialCommunicator::sendBinary(const std::vector<uint8_t>& data) {
    return sendBinary(data.data(), data.size());
}