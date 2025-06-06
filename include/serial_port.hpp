#pragma once
// System
#include <fcntl.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

// STL
#include <mutex>
#include <string>

// Other
#include "file_reader.hpp"

class Serial_port {
 private:
  std::string port_name_;    // port_name
  int baudrate_;             // port baudrate
  int start_bits_;           // start_bits
  int data_bits_;            // data_bits
  int stop_bits_;            // stop_bits
  std::string parity_;       // parity
  int timeout_deciseconds_;  // timeout
  int max_buffer_size_;      // max buffer size

  uint fd_;                 // file describe symble
  FileReader file_reader_;  // file reader tool class

  mutable std::mutex mutex_;  // ensure thread secure

 public:
  /**
  * @brief Construct a new Serial_port object
  * 
  */
  explicit Serial_port(const std::string config_path);

  /**
	 *s @brief Destroy the Serial_port object
	 * 
	 */
  ~Serial_port() noexcept;

  /**
	 * @brief Open prot
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
  size_t Write(const uint8_t* data, size_t size);

  /**
   * @brief Read data from port,return read size
   * 
   * @param buffer 
   * @param max_size 
   * @return size_t 
   */
  std::string Read(size_t max_size);
};
