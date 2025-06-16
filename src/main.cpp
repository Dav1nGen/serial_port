#include <cstdint>
#include "serial_port.hpp"

int main(int argc, char** argv) {
#define DEBUG
#ifndef DEBUG
  assert(argc >= 2 && "USAGE: ./serial_port <config_path>");
  auto config_path = std::string(argv[1]);
#endif

  std::string config_path = "../config/port_config.yaml";
  SerialPort sp(config_path);
  sp.Open();
  sp.ConfigurePortParameter();

  uint8_t request[8] = {0x01, 0x03, 0x01, 0x06, 0x00, 0x08};
  uint16_t crc = sp.CalculateCRC16(request, 6);
  request[6] = crc & 0xFF;
  request[7] = (crc >> 8) & 0xFF;
  return 0;
}
