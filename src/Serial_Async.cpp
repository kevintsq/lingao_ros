/*
 *  Copyright (C) 2020, LingAo Robotics, INC.
 *  author: owen <keaa@keaa.net>
 *  maintainer: owen <keaa@keaa.net>
 *
 *  Serial Port Async server
 */


#include "Serial_Async.h"
#include <sys/ioctl.h> 

#if defined(__linux__)
#include <linux/serial.h>
#endif

Serial_Async::Serial_Async():  pimpl(new AsyncSerialImpl)
{
  io_sev_ = boost::make_shared<boost::asio::io_service>();
//  serial_port_ = "/dev/lingao";
//  serial_baud_rate = 115200;
}

Serial_Async::~Serial_Async()
{
    if(isOpen_)
    {
        try {
            doClose();
        } catch (...) {

        }
    }
}

bool Serial_Async::init(std::string port_, int baudrate_)
{
 serial_port_ = port_;
 serial_baud_rate = baudrate_;
 return init();
}

bool Serial_Async::init()
{
  boost::system::error_code ec;
  try
  {
    port_ = boost::make_shared<boost::asio::serial_port>(boost::ref(*io_sev_));

    port_->open(serial_port_, ec);
    if (ec) {
        std::cout << "<Error> Serial Port open failed... com_port_name="
          << serial_port_ << ", e=" << ec.message().c_str() << std::endl;
        return false;
      }

#if defined(__linux__)
        // Enable low latency mode on Linux
        {
                int fd = port_->native_handle();

                struct serial_struct ser_info;
                ioctl(fd, TIOCGSERIAL, &ser_info);

                ser_info.flags |= ASYNC_LOW_LATENCY;

                ioctl(fd, TIOCSSERIAL, &ser_info);
        }
 #endif

    port_->set_option(boost::asio::serial_port::baud_rate(serial_baud_rate)); //设置波特率
    port_->set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));  //设置流控制
    port_->set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));  //设置奇偶校验
    port_->set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one)); //设置停止位
    port_->set_option(boost::asio::serial_port::character_size(8)); //设置数据位

  }
  catch(const std::exception& e)
  {
    std::cerr << "<Error> info: " << e.what() << std::endl;

    return false;
  }
  
  async_read_buffer_.resize(1024, 0);
  try
  {
    thread_ = boost::thread(boost::bind(&Serial_Async::AsyncTask, this));
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

void Serial_Async::AsyncTask()
{
  std::cout << "[ Info ] Serial Port Async Thread started" << std::endl;
  doRead();
  io_sev_->run();
}

void Serial_Async::doRead()
{

  port_->async_read_some(
        boost::asio::buffer(async_read_buffer_),
        boost::bind(&Serial_Async::handle_read, this,
        boost::asio::placeholders::error,
        boost::asio::placeholders::bytes_transferred));

  // async_read(port_->port,
  //       boost::asio::buffer(async_read_buffer_, 50),
  //       boost::bind(&Serial_Async::handle_read, this,
  //       boost::asio::placeholders::error,
  //       boost::asio::placeholders::bytes_transferred));
}

void Serial_Async::handle_read(const boost::system::error_code& error,  size_t bytes_transferred)
{
  if(error)
    {
        #ifdef __APPLE__
        if(error.value()==45)
        {
            //Bug on OS X, it might be necessary to repeat the setup
            //http://osdir.com/ml/lib.boost.asio.user/2008-08/msg00004.html
            doRead();
            return;
        }
        #endif //__APPLE__
        //error can be true even because the serial port was closed.
        //In this case it is not a real error, so ignore
        if(isOpen_)
        {
            doClose();
            std::cerr << "[ Error ] Serial closes unexpectedly,Error while reading data!" << "An error occurred:"<<error.message() << std::endl;
            return;
        }
    }
  else {
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

vecBuff Serial_Async::readData()
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

void Serial_Async::writeData(vecBuff& data)
{
    queue_write_buffer_.push(data);
    io_sev_->post(boost::bind(&Serial_Async::doWrite, this));
}

void Serial_Async::writeData(const std::string& s)
{
    vecBuff data(s.size());
    copy(s.begin(),s.end(),data.begin());
    queue_write_buffer_.push(data);

    io_sev_->post(boost::bind(&Serial_Async::doWrite, this));
}

void Serial_Async::writeData(const char &wdata, size_t size)
{
    vecBuff data;
    data.insert(data.end(), wdata, wdata+size);
    queue_write_buffer_.push(data);

    io_sev_->post(boost::bind(&Serial_Async::doWrite, this));
}

void Serial_Async::doWrite()
{

    if(queue_write_buffer_.empty() == false)
    {
        boost::asio::async_write(*port_, boost::asio::buffer(queue_write_buffer_.front()),
                                 boost::bind(&Serial_Async::handle_write, this, boost::asio::placeholders::error));
        queue_write_buffer_.pop();
    }
}

void Serial_Async::handle_write(const boost::system::error_code& error)
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


void Serial_Async::doClose()
{
    if(isOpen_)
    {
        isOpen_ = false;
//        boost::system::error_code ec;
        port_->cancel();
        port_->close();
    }
}

bool Serial_Async::isOpen()
{
    return isOpen_;
}

void Serial_Async::setReadCallback(const std::function<void (size_t)>& _callback)
{
    callback = _callback;
}
