#include "pmd-usb-device.h"


PmdUsbDevice::PmdUsbDevice(std::string port, size_t speed)
	: port_(port),
	  speed_(speed),
	  active_(true),
          serial_port_(port, BaudRate::B_115200, NumDataBits::EIGHT, Parity::NONE, NumStopBits::ONE) {

   serial_port_.SetTimeout(100); 

   std::cout << "Opening " << port_ << " at "<< speed << " baud, 8n1..." << std::flush;
   serial_port_.Open();
   std::cout << "OK." << std::endl;

   std::cout << "Sleeping to allow PMD to restart (happens after opening serial port)..." << std::flush;
   std::this_thread::sleep_for(std::chrono::seconds(3));
   std::cout << "OK." << std::endl;

   // Write command to read Device ID
   std::cout << "Writing CMD_READ_ID command..." << std::flush;
   std::vector<uint8_t> txDataBinary{UART_CMD::UART_CMD_READ_ID};
   serial_port_.WriteBinary(txDataBinary);
   std:: cout << "OK." << std::endl;
	
   std::cout << "Reading CMD_READ_ID response..." << std::flush;
   std::vector<uint8_t> rxDataBinary;
   auto t_start = std::chrono::high_resolution_clock::now();
   while(rxDataBinary.size() < 3) {
       serial_port_.ReadBinary(rxDataBinary);
       auto t_now = std::chrono::high_resolution_clock::now();
       double elapsed_time_ms = std::chrono::duration<double, std::milli>(t_now - t_start).count();
       if (elapsed_time_ms >= RX_TIMEOUT_MS) {
           std::cout << "ERROR: Did not receive enough binary data (" << rxDataBinary.size() << ") from PMD-USB." << std::endl;
           active_ = false;
       }
   }
   std:: cout << "OK." << std::endl;
   if (rxDataBinary.size() != 3 || rxDataBinary.at(0) != PMD_USB_VID || rxDataBinary.at(1) != PMD_USB_PID) {
     std::cout << "No PMD Device Found!" << std::endl;
     active_ = false;
   }

   if (active_ == true) {
     id_ =unsigned(rxDataBinary.at(2)); 
     std::cout << "PMD Device (#" << id_ << ") Found!" << std::endl;
   }
   rxDataBinary.clear();
}

PmdUsbDevice::~PmdUsbDevice() {
    if (active_ == false) {
       serial_port_.Close();
    }
    active_ = false;
}

void PmdUsbDevice::cont_read() {
    std::cout << "Writing CMD_WRITE_CONFIG_CONT_TX command..." << std::flush;
    // std::vector<uint8_t> cont_start_cmd{UART_CMD::UART_CMD_READ_ADC};
    // serialPort.WriteBinary(cont_start_cmd);
    // std::vector<uint8_t> cont_start_cmd{UART_CMD::UART_CMD_READ_SENSOR_VALUES};
    // serialPort.WriteBinary(cont_start_cmd);
    std::vector<uint8_t> cont_start_cmd{ UART_CMD::UART_CMD_WRITE_CONFIG_CONT_TX, 1, 0, 0xFF};
    serial_port_.WriteBinary(cont_start_cmd);
    std:: cout << "OK." << std::endl;
    while(active_) {
        auto t_start = std::chrono::high_resolution_clock::now();
        while(rx_buffer_.size() < 16) {
            serial_port_.ReadBinary(rx_buffer_);
            auto t_now = std::chrono::high_resolution_clock::now();
            double elapsed_time_ms = std::chrono::duration<double, std::milli>(t_now - t_start).count();
            if (elapsed_time_ms >= RX_TIMEOUT_MS) {
                std::cout << "ERROR: Did not receive enough binary data (" << rx_buffer_.size() << ") from PMD-USB." << std::endl;
                active_ = false;
            }
        }
        if (rx_buffer_.size() %  16 != 0) {
          std::cout << "Did not read enough data (" << rx_buffer_.size() << "/16)! Closing port..." << std::endl;
          active_ = false;
        } 
 	
        size_t num_sets = rx_buffer_.size() / 16;
        for (size_t i = 0; i < num_sets; i++){
            double pcie1_v = ((uint16_t)(rx_buffer_[1] << 8 | rx_buffer_[0]) >> 4) * 0.007568f;
            double pcie1_i = ((uint16_t)(rx_buffer_[3] << 8 | rx_buffer_[2]) >> 4) * 0.0488;
            double pcie1_p = pcie1_v * pcie1_i;
            double pcie2_v = ((uint16_t)(rx_buffer_[5] << 8 | rx_buffer_[4]) >> 4) * 0.007568f;
            double pcie2_i = ((uint16_t)(rx_buffer_[7] << 8 | rx_buffer_[6]) >> 4) * 0.0488;
            double pcie2_p = pcie2_v * pcie2_i;
            double eps1_v = ((uint16_t)(rx_buffer_[9] << 8 | rx_buffer_[8]) >> 4) * 0.007568f;
            double eps1_i = ((uint16_t)(rx_buffer_[11] << 8 | rx_buffer_[10]) >> 4) * 0.0488;
            double eps1_p = eps1_v * eps1_i;
            double eps2_v = ((uint16_t)(rx_buffer_[13] << 8 | rx_buffer_[12]) >> 4) * 0.007568f;
            double eps2_i = ((uint16_t)(rx_buffer_[15] << 8 | rx_buffer_[14]) >> 4) * 0.0488;
            double eps2_p = eps2_v * eps2_i;
            double gpu_power = pcie1_p + pcie2_p;
            double cpu_power = eps1_p + eps2_p;
            // double total_power = gpu_power + cpu_power;
            std::cout << "gpu power: " << gpu_power << ", cpu_power: " << cpu_power << std::endl;
	}
	rx_buffer_.clear();
    }
    std::cout << "Stopping CONT_TX command..." << std::flush;
    std::vector<uint8_t> cont_stop_cmd{UART_CMD::UART_CMD_WRITE_CONFIG_CONT_TX, 0, 0, 0};
    serial_port_.WriteBinary(cont_stop_cmd);
    std::cout << "OK." << std::endl;
}
