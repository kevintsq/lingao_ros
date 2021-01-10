#ifndef LINGAO_SERIAL_ASYNC_H
#define LINGAO_SERIAL_ASYNC_H

#include <boost/asio.hpp>
#include <boost/function.hpp>
#include <boost/smart_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/shared_array.hpp>

#include <iostream>
#include <queue>
#include <deque>
#include <inttypes.h>
#include <vector>
#include <mutex>

typedef std::vector<unsigned char> vecBuff;

static const int readBufferSize = 1024;

class AsyncSerialImpl : private boost::noncopyable
{
  public:
    AsyncSerialImpl() : backgroundThread(), open(false), error(false)
    {
    }

    boost::thread backgroundThread;   ///< Thread that runs read operations
    bool open;                        ///< True if port open
    bool error;                       ///< Error flag
    mutable boost::mutex errorMutex;  ///< Mutex for access to error

    int fd;  ///< File descriptor for serial port

    char readBuffer[1024];  ///< data being read

    /// Read complete callback
    boost::function<void(const char*, size_t)> callback;
};

class Transmission {
public:
	  virtual bool init()=0;
    virtual bool init(std::string port_, int baudrate_) = 0;

	  virtual vecBuff readData() = 0;
	  virtual void writeData(vecBuff &data) = 0;
    virtual void writeData(const std::string& s) = 0;
    virtual void writeData(const char &wdata, size_t size) = 0;

    virtual bool isOpen()=0;
    virtual void setReadCallback(const std::function<void (size_t)>& _callback) = 0;
};


class Serial_Async : public Transmission
{
  typedef std::function<void(size_t)> callback_t;

public:
  Serial_Async();
  ~Serial_Async();
  bool init();
  bool init(std::string port_, int baudrate_);

  void writeData(vecBuff& data);
  void writeData(const std::string& s);
  void writeData(const char &wdata, size_t size);

  vecBuff readData();

  /// Read complete callback
  std::function<void (size_t)> callback;
  void setReadCallback(const std::function<void (size_t)>& _callback);

  bool isOpen();



private:
  boost::shared_ptr<boost::asio::serial_port> port_;
  boost::shared_ptr<boost::asio::io_service> io_sev_;
  //boost::asio::io_service  io_service_;
  boost::shared_ptr<AsyncSerialImpl> pimpl;

  //serial port
  std::string serial_port_;
  int serial_baud_rate;

  //Async Read Buff
  vecBuff async_read_buffer_;

  boost::thread thread_;

  // mutex locks
  std::mutex port_mutex_;
  std::mutex read_mutex_;

  bool isOpen_;

  std::queue<vecBuff> queue_read_buffer_;

  /// Data are queued here before they go in writeBuffer
  std::queue<vecBuff> queue_write_buffer_;   ///< Data being written

  void AsyncTask();

  void doRead();
  void handle_read(const boost::system::error_code& error,  size_t bytes_transferred);

  void doWrite();
  void handle_write(const boost::system::error_code& error);

  void doClose();

};


#endif // LINGAO_SERIAL_ASYNC_H
