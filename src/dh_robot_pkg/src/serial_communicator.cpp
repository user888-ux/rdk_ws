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

//向STM32发送数据(暂未启用)
bool SerialCommunicator::send(const std::string& data) {
    if (!isConnected()) {
        RCLCPP_WARN(node_->get_logger(), "Cannot send data: serial port not connected");
        return false;
    }
    
    try {
        size_t bytes_written = serial_port_->write_some(boost::asio::buffer(data));
        RCLCPP_DEBUG(node_->get_logger(), "Sent %zu bytes to serial", bytes_written);
        return bytes_written == data.size();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Error sending data to serial: %s", e.what());
        return false;
    }
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
        std::string received_data(read_buffer_.data(), bytes_transferred);//数据转换：将缓冲区中的原始字节数据转换为字符串
        RCLCPP_DEBUG(node_->get_logger(), "Received %zu bytes from serial: %s", 
                    bytes_transferred, received_data.c_str());
        
        if (receive_callback_) {//调用(在 robot_controller.cpp)通过 setReceiveCallback(在.hpp文件里) 注册的回调函数
            receive_callback_(received_data);
        }
    } else if (error) {
        RCLCPP_ERROR(node_->get_logger(), "Serial read error: %s", error.message().c_str());
    }
    
    // 即使是错误，只要连接还在就继续读取
    if (isConnected()) {
        startAsyncRead();// 重新注册，等待下一批数据
    }
}