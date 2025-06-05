#include "../include/serial_port.hpp"
#include <fcntl.h>
#include <termios.h>

// STL
#include <iostream>
#include <stdexcept>
#include <string>

Serial_port::Serial_port(const std::string config_path)
    : port_name_(""),
      baudrate_(9600),
      data_bits_(8),
      stop_bits_(1),
      parity_("N"),
      fd_(-1),
      file_reader_(config_path) {}

Serial_port::~Serial_port() {
  if (IsOpen())
    ::close(fd_);
}

bool Serial_port::Open() {
  if (IsOpen()) {
    std::cerr << "Port already open" << std::endl;
    return false;
  }

  if (port_name_.empty()) {
    throw std::runtime_error("Port name not configured");
  }

  std::string port_path = "/dev/" + port_name_;
  fd_ = ::open(port_path.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);

  if (fd_ == -1) {
    throw std::runtime_error("Failed to open " + port_path + ": " +
                             strerror(errno));
  }

  // Ensure that the port is in non blocking mode
  fcntl(fd_, F_GETFL, 0); //如何判断数据包完整程度？？？
  return true;
}

void Serial_port::Close() {
  if (fd_ >= 0)
    ::close(fd_);
  fd_ = -1;
}

bool Serial_port::IsOpen() const {
  return fd_ >= 0;
}

bool Serial_port::ConfigurePortParameter() {
  if (!IsOpen()) {
    throw std::runtime_error("Port not open");
  }

  struct termios options;
  if (tcgetattr(fd_, &options) != 0) {
    throw std::runtime_error("tcgetattr: " + std::string(strerror(errno)));
  }

  // 设置原始模式 (关键修复)
  options.c_iflag &= ~(INLCR | IGNCR | ICRNL | IXON | IXOFF);
  options.c_oflag &= ~(OPOST | ONLCR | OCRNL);
  options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
  options.c_cflag &= ~(CSIZE | PARENB | CSTOPB);


  // get parameters from config file
  port_name_ = file_reader_.Read<std::string>("port_name");
  baudrate_ = file_reader_.Read<int>("baudrate");
  data_bits_ = file_reader_.Read<int>("data_bits");
  stop_bits_ = file_reader_.Read<int>("stop_bits");
  parity_ = file_reader_.Read<std::string>("parity");

  // set port parameter
  tcgetattr(fd_, &options);

  // check tcgetattr
  if (tcgetattr(fd_, &options) != 0) {
    throw std::runtime_error("tcgetattr failed");
  }

  // set bundrate
  speed_t br;
  switch (baudrate_) {
    case 115200:
      br = B115200;
      break;
    case 57600:
      br = B57600;
      break;
    case 38400:
      br = B38400;
      break;
    case 19200:
      br = B19200;
      break;
    case 9600:
      br = B9600;
      break;
    case 4800:
      br = B4800;
      break;
    case 2400:
      br = B2400;
      break;
    case 1800:
      br = B1800;
      break;
    case 1200:
      br = B1200;
      break;
    default:
      throw std::runtime_error("Unsupported baudrate: " +
                               std::to_string(baudrate_));
  }

  cfsetispeed(&options, br);
  cfsetospeed(&options, br);

  // set data_bit
  options.c_cflag &= ~CSIZE;  // clear databit config
  switch (data_bits_) {
    case 5:
      options.c_cflag |= CS5;
      break;
    case 6:
      options.c_cflag |= CS6;
      break;
    case 7:
      options.c_cflag |= CS7;
      break;
    case 8:
      options.c_cflag |= CS8;
      break;
    default:
      throw std::runtime_error("Invalid data bits: " +
                               std::to_string(data_bits_));
  }

  // set stop_bit
  if (stop_bits_ == 1) {
    options.c_cflag &= ~CSTOPB;  // 1 stop bit
  } else if (stop_bits_ == 2) {
    options.c_cflag |= CSTOPB;  // 2 stop bit
  } else {
    throw std::runtime_error("Invalid stop bits: " +
                             std::to_string(stop_bits_));
  }

  // set parity
  if (parity_ == "N") {
    options.c_cflag &= ~PARENB;  // No verification
  } else if (parity_ == "E") {
    options.c_cflag |= PARENB;  // Even verification
    options.c_cflag &= ~PARODD;
  } else if (parity_ == "O") {
    options.c_cflag |= PARENB;  // Odd Parity
    options.c_cflag |= PARODD;
  } else {
    throw std::runtime_error("Invalid parity: " + parity_);
  }

  // Set timeout (important)
  options.c_cc[VMIN] = 0;   // Minimum Character Count
  options.c_cc[VTIME] = 10; // timeout (1s)

  // Applicate configuration
  if (tcsetattr(fd_, TCSANOW, &options) != 0) {
    throw std::runtime_error("tcsetattr failed");
  }

  return true;
}
