#include <list_serial_devices.h>
#include <iostream>
#include <iomanip>

int main() {
	for (auto& [device, manufacturer, product, serialNumber, location, vid, pid] : list_serial_devices()) {
		std::cout << device << ": "
			<< (product.empty() ? "n/a" : product)
			<< " [" << std::hex << std::setfill('0')
			<< "VID:0x" << std::setw(4) << vid << " "
			<< "PID:0x" << std::setw(4) << pid << std::dec << "]\n";

		if (!manufacturer.empty())
			std::cout << "  Manufacturer: " << manufacturer << "\n";
		if (!serialNumber.empty())
			std::cout << "  Serial: " << serialNumber << "\n";
		if (!location.empty())
			std::cout << "  Location: " << location << "\n";
		std::cout << "\n";
	}

	return 0;
}