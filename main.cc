#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <utility>

namespace {

#define let const auto
#define let_mut auto

#define RETRY_TIMES 10

#define CRC_START_MODBUS 0xFFFF
#define CRC_POLY_16 0xA001

bool crc_tab16_init = false;
uint16_t crc_tab16[256];
u_char read_cmd[8] = {0x01, 0x03, 0x00, 0x01, 0x00, 0x02, 0x95, 0xCB};
// u_char read_cmd[8] = {0x01, 0x03, 0x00, 0x01, 0x00, 0x02, 0x95, 0xCB};

void init_crc16_table() {

  uint16_t i;
  uint16_t j;
  uint16_t crc;
  uint16_t c;

  for (i = 0; i < 256; i++) {

    crc = 0;
    c = i;

    for (j = 0; j < 8; j++) {

      if (((crc ^ c) & 0x0001) > 0)
        crc = (crc >> 1) ^ CRC_POLY_16;
      else
        crc = crc >> 1;

      c = c >> 1;
    }

    crc_tab16[i] = crc;
  }

  crc_tab16_init = true;

} /* init_crc16_tab */

auto crc_modbus(const unsigned char *input_str, size_t num_bytes) -> uint16_t {

  uint16_t crc;
  const unsigned char *ptr;
  size_t a;

  if (!crc_tab16_init)
    init_crc16_table();

  crc = CRC_START_MODBUS;
  ptr = input_str;

  if (ptr != nullptr)
    for (a = 0; a < num_bytes; a++) {

      crc = (crc >> 8) ^
            crc_tab16[(crc ^ static_cast<uint16_t>(*ptr++)) & 0x00FF];
    }

  return crc;

} /* crc_modbus */

} // namespace
int main(void) {

  int fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_SYNC);

  struct termios options;

  if (tcgetattr(fd, &options) < 0) {
    std::cout << "Error from tcgetattr: " << strerror(errno) << std::endl;
    return -1;
  }

  options.c_cflag &= ~PARENB; // no parity
  options.c_cflag &= ~CSTOPB; // 1 stop bit
  options.c_cflag &= ~CSIZE;  // 8 data bits
  options.c_cflag |= CS8;
  options.c_cflag &= ~CRTSCTS; // no hardware flow control
  options.c_cflag |=
      CREAD | CLOCAL; // enable receiver, ignore modem control lines
  options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // raw input
  options.c_oflag &= ~OPOST;                          // raw output

  options.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
  options.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR |
                       ICRNL); // Disable any special handling of received bytes

  // set timeout to 0.1 seconds
  options.c_cc[VMIN] = 8;
  options.c_cc[VTIME] = 100;

  // set baud rate 9600, 8N1
  cfsetispeed(&options, B9600);

  if (tcsetattr(fd, TCSANOW, &options) < 0) {
    std::cout << "Error from tcsetattr: " << strerror(errno) << std::endl;
    return -1;
  }

  let read_once = [&fd]() -> std::optional<float> {
    let n_wr = write(fd, read_cmd, 8);
    if (n_wr < 0) {
      std::cout << "Error writing to serial port" << std::endl;
      return std::nullopt;
    }

    unsigned char response[1024];
    let n_rd = ::read(fd, response, 1024);
    if (n_rd < 8) {
      std::cout << "Error reading from serial port, get " << n_rd
                << " bytes, expected 8" << std::endl;
      return std::nullopt;
    }

    let crc = crc_modbus(response, n_rd - 2);
    let crc_lo = crc >> 8;
    let crc_hi = crc & 0xFF;
    if (crc_hi != response[n_rd - 2] || crc_lo != response[n_rd - 1]) {

      printf("CRC error expected %02X %02X but got %02X %02X\n", crc_hi, crc_lo,
             response[n_rd - 2], response[n_rd - 1]);

      return std::nullopt;
    }
    //  return
    let num_bytes = response[2];
    if (num_bytes != 4) {
      std::cout << "Unexpected number of bytes " << num_bytes << " != 4"
                << std::endl;

      return std::nullopt;
    }

    let int_part = response[3] << 8 | response[4];
    let dec_part = response[5] << 8 | response[6];

    return int_part + dec_part / 10000.0;
  };

  int i = 0;
  int success = 0;
  for (; i < RETRY_TIMES; i++) {
    if (let v = read_once(); v) {
      std::cout << "v = " << *v << std::endl;
      success++;
      if (success == 3)
        break;
    }
  }
  if (i == RETRY_TIMES) {
    std::cout << "Failed to read from serial port" << std::endl;
    return -1;
  }

  if (close(fd) < 0) {
    std::cout << "Error closing serial port" << std::endl;
    return -1;
  }

  return 0;
}