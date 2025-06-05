#pragma once

// System
#include <fcntl.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

// STL
#include <string>

// Other
#include "file_reader.hpp"

class Serial_port {
 private:
  std::string port_name_;  // port_name
  int baudrate_;           // port baudrate
  int data_bits_;          // data_bits
  int stop_bits_;          // stop_bits
  std::string parity_;     // parity

  uint fd_;                 // file describe symble
  FileReader file_reader_;  // file reader tool class

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
  ~Serial_port();

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
  void Close();

  /**
	 * @brief Judge port whether opened
	 * 
	 * @return true 
	 * @return false 
	 */
  bool IsOpen() const;

  /**
   * @brief Configure port parameter
   * 
   * @return true 
   * @return false 
   */
  bool ConfigurePortParameter();

  /**
   * @brief Write data to port
   * 
   * @param data 
   * @param size 
   * @return size_t 
   */
  size_t Write(const uint8_t* data, size_t size);

  /**
   * @brief Read data from port
   * 
   * @param buffer 
   * @param max_size 
   * @return size_t 
   */
  size_t Read(uint8_t* buffer, size_t max_size);
};
