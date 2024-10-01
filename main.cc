#include "pmd-usb-device.h"

int main(int argc, char* argv[]) {

    if (argc != 2) {
      return 0; 
    }
    std::string input_prt = argv[1];
    std::unique_ptr<PmdUsbDevice> pmd =
           std::make_unique<PmdUsbDevice>(input_prt, 115200u);

    std::thread serial_reader_thread(&PmdUsbDevice::cont_read, pmd.get());
    std::this_thread::sleep_for(std::chrono::seconds(5));
    pmd.get()->disable();
    serial_reader_thread.join();
    return 0;
}
