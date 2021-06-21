/*
 * Copyright (C) 2021, LingAo Robotics, INC.
 * @Version: V1.0
 * @Author: owen
 * @Date: 2021-05-31 10:24:26
 * @LastEditTime: 2021-06-13 20:19:32
 * @LastEditors: owen
 * @Description: 
 * @FilePath: /lingao_ws/src/lingaoRobot/lingao_bringup/src/TCP_Async.cpp
 */

#include <iostream>
#include <mutex>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/smart_ptr.hpp>

#include <Serial_Async.h>
#include<TCP_Async.h>

TCP_Async::TCP_Async() : m_endpoint(boost::asio::ip::address::from_string("192.168.10.100"), 38000)
{
#if BOOST_VERSION >= 107000
    io_sev_ = boost::make_shared<boost::asio::io_context>();
#else
    io_sev_ = boost::make_shared<boost::asio::io_service>();
#endif
}

TCP_Async::~TCP_Async()
{
    if(isOpen_)
    {
        try {
            doClose();
        } catch (...) {

        }
    }
}

bool TCP_Async::init(std::string ip_, int port_)
{
    m_endpoint.address(boost::asio::ip::address::from_string(ip_));
    m_endpoint.port(port_);
    
    return init();
}


bool TCP_Async::init()
{
    boost::system::error_code ec;
    try
    {

#if BOOST_VERSION >= 107000
        socket_ = boost::make_shared<boost::asio::ip::tcp::socket>(*io_sev_);
#else
        socket_ = boost::make_shared<boost::asio::ip::tcp::socket>(boost::ref(*io_sev_));
#endif

        socket_->connect(m_endpoint, ec);

        if (ec)
        {
            std::cout << "<Error> TCP Socket open failed... ip_addr=" 
                << m_endpoint.address()<< ":" << m_endpoint.port() << ", e=" << ec.message().c_str() << std::endl;
            return false;
        }
        
        socket_->set_option(boost::asio::ip::tcp::no_delay(true), ec);  //设置为数据立即发送, 不停留
        socket_->set_option(boost::asio::socket_base::linger(true, 0), ec);  //设置socket关闭时，立即关闭
        socket_->set_option(boost::asio::socket_base::keep_alive(true), ec);    //保持常连接

    }
    catch(const std::exception& e)
    {
        std::cerr << "<Error> info: " << e.what() << std::endl;

        return false;
    }

      async_read_buffer_.resize(1024, 0);
  
    try
    {
        thread_ = boost::thread(boost::bind(&TCP_Async::AsyncTask, this));
    }
    catch(const std::exception& e)
    {
        std::cerr << "<Error> Thread Care Fail!" << std::endl;
        std::cerr << "<Error> Error Msg: " << e.what() << std::endl;
        return false;
    }

    isOpen_ = true;

    return true;
}

void TCP_Async::AsyncTask()
{
  std::cout << "[ Info ] TCP Socket Async Thread started" << std::endl;
  doRead();
  io_sev_->run();
}

void TCP_Async::doRead()
{

    socket_->async_read_some(
        boost::asio::buffer(async_read_buffer_),
        boost::bind(&TCP_Async::handle_read, this,
        boost::asio::placeholders::error,
        boost::asio::placeholders::bytes_transferred));

}

void TCP_Async::handle_read(const boost::system::error_code& error,  size_t bytes_transferred)
{
    if(error)
    {
        if(isOpen_)
        {
            doClose();
            std::cerr << "[ Error ] Serial closes unexpectedly,Error while reading data!" << "An error occurred:"<<error.message() << std::endl;
            return;
        }
    }
    else
    {
        std::lock_guard<std::mutex> lock(read_mutex_);

        vecBuff data(async_read_buffer_.begin(), async_read_buffer_.begin() + bytes_transferred);
        queue_read_buffer_.push(data);

        //DEBUG
        // printf("rece:  ");
        // for(auto aa : data)
        //         printf("%02x ", (unsigned char)aa);
        // std::cout << std::endl;

        if(callback)
        {
            read_mutex_.unlock();
            callback(bytes_transferred);
        }

        doRead();
    }
}

vecBuff TCP_Async::readData()
{
    std::lock_guard<std::mutex> lock(read_mutex_);

    if(queue_read_buffer_.empty() == false)
    {
        vecBuff data(queue_read_buffer_.front());
        queue_read_buffer_.pop();
        return data;
    }
    vecBuff data;
    return data;
}

void TCP_Async::writeData(vecBuff& data)
{
    queue_write_buffer_.push(data);
    io_sev_->post(boost::bind(&TCP_Async::doWrite, this));
}

void TCP_Async::writeData(const std::string& s)
{
    vecBuff data(s.size());
    copy(s.begin(),s.end(),data.begin());
    queue_write_buffer_.push(data);

    io_sev_->post(boost::bind(&TCP_Async::doWrite, this));
}

void TCP_Async::writeData(const char &wdata, size_t size)
{
    vecBuff data;
    data.insert(data.end(), wdata, wdata+size);
    queue_write_buffer_.push(data);

    io_sev_->post(boost::bind(&TCP_Async::doWrite, this));
}

void TCP_Async::doWrite()
{

    if(queue_write_buffer_.empty() == false)
    {
        boost::asio::async_write(*socket_, boost::asio::buffer(queue_write_buffer_.front()),
                                 boost::bind(&TCP_Async::handle_write, this, boost::asio::placeholders::error));
        queue_write_buffer_.pop();
    }
}

void TCP_Async::handle_write(const boost::system::error_code& error)
{
    if(!error)
    {
        if(queue_write_buffer_.empty() == false)
        {
            doWrite();
        }
    }
    else {
        if(isOpen_)
        {
            std::cerr << "[ Error ] Serial closes unexpectedly,Error while writing data!" << std::endl;
            std::cerr << "[ Error ] Error Msg: " << error.message().c_str() << std::endl;
        }
        doClose();
    }
}

void TCP_Async::doClose()
{
    if(isOpen_)
    {
        isOpen_ = false;
//        boost::system::error_code ec;
        socket_->cancel();
        socket_->close();
    }
}

bool TCP_Async::isOpen()
{
    return isOpen_;
}

void TCP_Async::setReadCallback(const std::function<void (size_t)>& _callback)
{
    callback = _callback;
}
