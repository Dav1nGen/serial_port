#pragma once
// System
#include <fcntl.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

// STL
#include <mutex>
#include <string>
#include <vector>

// Other
#include "file_reader.hpp"

class SerialPort {
 private:
  std::string port_name_;    // port_name
  int baudrate_;             // port baudrate
  int start_bits_;           // start_bits
  int data_bits_;            // data_bits
  int stop_bits_;            // stop_bits
  std::string parity_;       // parity
  int timeout_deciseconds_;  // timeout
  int max_buffer_size_;      // max buffer size

  int fd_;                  // file describe symble
  FileReader file_reader_;  // file reader tool class

  mutable std::mutex mutex_;  // ensure thread secure

 public:
  /**
  * @brief Construct a new SerialPort object
  * 
  */
  explicit SerialPort(const std::string& config_path);

  /**
	 * @brief Destroy the SerialPort object
	 * 
	 */
  ~SerialPort() noexcept;

  /**
	 * @brief Open serial port
	 * 
	 * @return true 
	 * @return false 
	 */
  bool Open();

  /**
	 * @brief Close port
	 * 
	 */
  void Close() noexcept;

  /**
	 * @brief Judge port whether opened
	 * 
	 * @return true 
	 * @return false 
	 */
  bool IsOpen() const;

  /**
   * @brief Get config from config file
   * 
   */
  void GetConfiguration();

  /**
   * @brief Configure port parameter
   * 
   * @return true 
   * @return false 
   */
  bool ConfigurePortParameter();

  /**
   * @brief Write data to port,return write size
   * 
   * @param data 
   * @param size 
   * @return size_t 
   */
  size_t Write(const uint8_t* data, size_t size) const;

  /**
   * @brief CRC16 calculate
   * 
   * @param data 
   * @param length 
   * @return uint16_t 
   */
  uint16_t CalculateCRC16(const uint8_t* data, size_t length);
};
