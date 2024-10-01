#include <stdio.h>
#include <chrono>
#include <thread>
#include <mutex>
#include <atomic>
#include <memory>

#include <CppLinuxSerial/SerialPort.hpp>
using namespace mn::CppLinuxSerial;

static constexpr uint8_t PMD_USB_VID = 0xEE;
static constexpr uint8_t PMD_USB_PID = 0x0A;
static constexpr double RX_TIMEOUT_MS = 2000;

// Overload to be able to print vectors to std::cout
template <typename T>
std::ostream& operator<<( std::ostream& ostrm, const std::vector<T>& vec ){
    if (vec.size() == 0) {
        return ostrm << "[]";
    }
    for( int j = 0, n = vec.size(); j < n; ++j ){
        ostrm << ",["[ !j ] << " " << vec[ j ];
    }
    return ostrm << " ]";
}


class PmdUsbDevice {
public:
  PmdUsbDevice(std::string port, size_t speed);
  ~PmdUsbDevice();

  void cont_read();
  void disable() {active_ = false;}
enum UART_CMD : uint8_t
{
    UART_CMD_WELCOME,
    UART_CMD_READ_ID,
    UART_CMD_READ_SENSORS,
    UART_CMD_READ_SENSOR_VALUES,
    UART_CMD_READ_CONFIG,
    UART_CMD_WRITE_CONFIG,
    UART_CMD_READ_ADC,
    UART_CMD_WRITE_CONFIG_CONT_TX,
    UART_CMD_WRITE_CONFIG_UART,
    UART_CMD_RESET = 0xF0,
    UART_CMD_BOOTLOADER = 0xF1,
    UART_CMD_NOP = 0xFF
};
  // UART interface commands
private:
  std::string port_;
  size_t speed_;
  std::atomic<bool> active_;

  SerialPort serial_port_;
  size_t id_;

  std::vector<uint8_t> rx_buffer_;
};
