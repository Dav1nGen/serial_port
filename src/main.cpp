#include "../include/serial_port.hpp"

int main() {
  Serial_port sp("../config/port_config.yaml");
  sp.Open();
  sp.ConfigurePortParameter();
  return 0;
}
