#include "serial_port.hpp"
// POSIX
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

// C
#include <cerrno>
#include <cstddef>
#include <cstring>

// STL
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

Serial_port::Serial_port(const std::string config_path)
    : port_name_(""),
      baudrate_(9600),
      start_bits_(1),
      data_bits_(8),
      stop_bits_(1),
      parity_("N"),
      timeout_deciseconds_(10),
      fd_(-1),
      file_reader_(config_path) {
  try {
    // Try to read config
    GetConfiguration();
  } catch (const std::exception& e) {
    std::cerr << "Error reading configuration: " << e.what() << std::endl;
  }
}

Serial_port::~Serial_port() noexcept {
  Close();
}

bool Serial_port::Open() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (IsOpen()) {
    std::cerr << "Port already open" << std::endl;
    return false;
  }

  if (port_name_.empty()) {
    throw std::runtime_error("Port name not configured");
  }

  std::string port_path = "/dev/" + port_name_;
  int fd = ::open(port_path.c_str(), O_RDWR | O_NOCTTY);  // block mode

  if (fd == -1) {
    throw std::runtime_error("Failed to open " + port_path + ": " +
                             strerror(errno));
  }
  fd_ = fd;  // ensure fd open successfully

  // Try to configure port
  try {
    ConfigurePortParameter();
  } catch (const std::exception& e) {
    ::close(fd_);
    fd_ = -1;
    throw;
  }

  return true;
}

void Serial_port::Close() noexcept {
  std::lock_guard<std::mutex> lock(mutex_);
  if (fd_ >= 0)
    ::close(fd_);
  fd_ = -1;
}

bool Serial_port::IsOpen() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return fd_ >= 0;
}

void Serial_port::GetConfiguration() {
  // Read config
  port_name_ = file_reader_.Read<std::string>("port_name");
  baudrate_ = file_reader_.Read<int>("baudrate");
  start_bits_ = file_reader_.Read<int>("start_bits");
  data_bits_ = file_reader_.Read<int>("data_bits");
  stop_bits_ = file_reader_.Read<int>("stop_bits");
  parity_ = file_reader_.Read<std::string>("parity");
  timeout_deciseconds_ = file_reader_.Read<int>("timeout_deciseconds");
  max_buffer_size_ = file_reader_.Read<int>("max_buffer_size");

  // Verify port name
  if (port_name_.empty()) {
    throw std::runtime_error("Port name cannot be empty");
  }
}

bool Serial_port::ConfigurePortParameter() {
  std::lock_guard<std::mutex> lock(mutex_);

  if (!IsOpen()) {
    throw std::runtime_error("Port not open");
  }

  struct termios options;
  if (tcgetattr(fd_, &options) != 0) {
    throw std::runtime_error("tcgetattr: " + std::string(strerror(errno)));
  }

  // Set origin mode
  options.c_iflag &= ~(INLCR | IGNCR | ICRNL | IXON | IXOFF);
  options.c_oflag &= ~(OPOST | ONLCR | OCRNL);
  options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
  options.c_cflag &= ~(CSIZE | PARENB | CSTOPB);

  // Set bundrate
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

  // Set data_bit
  options.c_cflag &= ~CSIZE;  // Clear databit config
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

  // Set parity
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

  options.c_cc[VMIN] = 0;                      // Minimum Character Count
  options.c_cc[VTIME] = timeout_deciseconds_;  // Timeout (1s)

  // Applicate configuration
  if (tcsetattr(fd_, TCSANOW, &options) != 0) {
    throw std::runtime_error("tcsetattr failed");
  }

  return true;
}

size_t Serial_port::Write(const uint8_t* data, size_t size) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!IsOpen()) {
    throw std::runtime_error("Port not open");
  }

  if (data == nullptr || size == 0) {
    return 0;
  }

  // Limit buffer size
  if (size > max_buffer_size_) {
    throw std::runtime_error("Write buffer too large, max size is " +
                             std::to_string(max_buffer_size_));
  }

  // Write data
  ssize_t bytes_written = ::write(fd_, data, size);
  if (bytes_written < 0) {
    throw std::runtime_error("Write failed: " + std::string(strerror(errno)));
  }

  return static_cast<size_t>(bytes_written);
}

std::string Serial_port::Read(size_t max_size) {
  std::lock_guard<std::mutex> lock(mutex_);  // thread secure

  if (!IsOpen()) {
    throw std::runtime_error("Port not open");
  }

  // Limit buffer size
  if (max_size > max_buffer_size_) {
    throw std::runtime_error("Read buffer too large, max size is " +
                             std::to_string(max_buffer_size_));
  }

  // Create temp buffer
  std::vector<char> buffer(max_size);

  // Read data
  ssize_t bytes_read = ::read(fd_, buffer.data(), max_size);

  if (bytes_read < 0) {
    // Check timeout
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      return std::string();  // Timeout,return empty string
    }
    throw std::runtime_error("Read failed: " + std::string(strerror(errno)));
  }

  return std::string(buffer.data(), static_cast<size_t>(bytes_read));
}
