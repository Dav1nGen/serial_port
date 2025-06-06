#include "serial_port.hpp"

int main(int argc, char** argv) {

  assert(argc >= 2 && "USAGE: ./serial_port <config_path>");

  auto config_path = std::string(argv[1]);
  Serial_port sp(config_path);
  sp.Open();
  sp.ConfigurePortParameter();
  return 0;
}
