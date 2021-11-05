#ifndef LINGAO_UDP_ASYNC_H
#define LINGAO_UDP_ASYNC_H


#include <Serial_Async.h>

typedef std::vector<unsigned char> vecBuff;

class UDP_Async : public Transmission
{
  typedef std::function<void(size_t)> callback_t;

public:
  UDP_Async();
  ~UDP_Async();
  bool init();
  bool init(std::string ip_, int port_);

  void writeData(vecBuff& data);
  void writeData(const std::string& s);
  void writeData(const char &wdata, size_t size);

  vecBuff readData();

  /// Read complete callback
  std::function<void (size_t)> callback;
  void setReadCallback(const std::function<void (size_t)>& _callback);

  bool isOpen();


private:
    boost::shared_ptr<boost::asio::ip::udp::socket> socket_;
    boost::asio::ip::udp::endpoint m_endpoint;

#if BOOST_VERSION >= 107000
  boost::shared_ptr<boost::asio::io_context> io_sev_;
#else
  boost::shared_ptr<boost::asio::io_service> io_sev_;
#endif

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


#endif // LINGAO_UDP_ASYNC_H
