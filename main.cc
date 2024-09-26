#include "pmd-usb-device.h"

int main(int argc, char* argv[]) {

    if (argc != 2) {
      return 0; 
    }
    std::string input_prt = argv[1];
    PmdUsbDevice pmd(input_prt, 115200u);
    /*
    // Create serial port object and open serial port
    SerialPort serialPort(input_prt, BaudRate::B_115200, NumDataBits::EIGHT, Parity::NONE, NumStopBits::ONE);

    // Block for at most 100ms when receiving data
    // NOTE: I haven't had luck setting this to -1, Read() seems to never return
    // even though serial data has been sent to Linux
    serialPort.SetTimeout(100); 

    std::cout << "Opening " << input_prt << " at 115200 baud, 8n1..." << std::flush;
    serialPort.Open();
    std::cout << "OK." << std::endl;

    std::cout << "Sleeping to allow PMD to restart (happens after opening serial port)..." << std::flush;
    std::this_thread::sleep_for(std::chrono::seconds(3));
    std::cout << "OK." << std::endl;

    // Write command to read Device ID
    std::cout << "Writing CMD_READ_ID command..." << std::flush;
    std::vector<uint8_t> txDataBinary{ UART_CMD::UART_CMD_READ_ID};
    serialPort.WriteBinary(txDataBinary);
    std:: cout << "OK." << std::endl;
	
    std::cout << "Reading CMD_READ_ID response..." << std::flush;
    std::vector<uint8_t> rxDataBinary;
    auto t_start = std::chrono::high_resolution_clock::now();
    while(rxDataBinary.size() < 3) {
        serialPort.ReadBinary(rxDataBinary);
        auto t_now = std::chrono::high_resolution_clock::now();
        double elapsed_time_ms = std::chrono::duration<double, std::milli>(t_now - t_start).count();
        if (elapsed_time_ms >= RX_TIMEOUT_MS) {
            std::cout << "ERROR: Did not receive enough binary data (" << rxDataBinary.size() << ") from PMD-USB." << std::endl;
            return -1;
        }
    }
    std:: cout << "OK." << std::endl;
    if (rxDataBinary.size() != 3 || rxDataBinary.at(0) != PMD_USB_VID || rxDataBinary.at(1) != PMD_USB_PID) {
      std::cout << "No PMD Device Found!" << std::endl;
      serialPort.Close();
      return -1;
    } 
    std::cout << "PMD Device (#" << unsigned(rxDataBinary.at(2)) << ") Found!" << std::endl;
    rxDataBinary.clear();

    std::thread serial_reader_thread(update, serialPort);

    // Read Power Stats
    std::cout << "Writing CMD_READ_ADC command..." << std::flush;
    // std::vector<uint8_t> cont_start_cmd{UART_CMD::UART_CMD_READ_ADC};
    // serialPort.WriteBinary(cont_start_cmd);
    // std::vector<uint8_t> cont_start_cmd{UART_CMD::UART_CMD_READ_SENSOR_VALUES};
    // serialPort.WriteBinary(cont_start_cmd);
    std::vector<uint8_t> cont_start_cmd{UART_CMD::UART_CMD_WRITE_CONFIG_CONT_TX, 1, 0, 0xFF};
    serialPort.WriteBinary(cont_start_cmd);
    std:: cout << "OK." << std::endl;

    std::this_thread::sleep_for(std::chrono::seconds(5));
    std::cout << "Stopping CONT_TX command..." << std::flush;
    std::vector<uint8_t> cont_stop_cmd{UART_CMD::UART_CMD_WRITE_CONFIG_CONT_TX, 0, 0, 0};
    serialPort.WriteBinary(cont_stop_cmd);
    std:: cout << "OK." << std::endl;
    serial_reader_thread.join();
    // Close the serial port
    serialPort.Close();*/
    return 0;
}
