#include "list_serial_devices.h"

#include <vector>
#include <string>
#include <filesystem>
#include <fstream>
#include <set>
#include <glob.h>

// Read one line from a sysfs file
std::string read_line(const std::filesystem::path& path) {
	std::ifstream f(path);
	std::string line;
	if (f && std::getline(f, line)) {
		if (!line.empty() && line.back() == '\n')
			line.pop_back();
		return line;
	}
	return {};
}

// Expand a glob pattern (e.g. /dev/ttyUSB*)
void glob_devices(std::set<std::string>& devices, const std::string& pattern) {
	glob_t results{};
	if (glob(pattern.c_str(), 0, nullptr, &results) == 0) {
		for (size_t i = 0; i < results.gl_pathc; i++)
			devices.insert(results.gl_pathv[i]);
	}
	globfree(&results);
}


std::vector<SerialDeviceInfo> list_serial_devices() {
	std::set<std::string> devices;
	glob_devices(devices, "/dev/ttyS*");
	glob_devices(devices, "/dev/ttyUSB*");
	glob_devices(devices, "/dev/ttyACM*");
	glob_devices(devices, "/dev/ttyAMA*");
	glob_devices(devices, "/dev/rfcomm*");
	glob_devices(devices, "/dev/ttyAP*");
	glob_devices(devices, "/dev/ttyGS*");

	std::vector<SerialDeviceInfo> ports;

	for (const auto& dev : devices) {
		std::string name = std::filesystem::path(dev).filename();
		std::filesystem::path sys_tty = std::filesystem::path("/sys/class/tty") / name / "device";
		if (!std::filesystem::exists(sys_tty))
			continue;

		std::filesystem::path device_path = std::filesystem::canonical(sys_tty);
		std::filesystem::path subsystem_path;
		try {
			subsystem_path = std::filesystem::canonical(device_path / "subsystem");
		} catch (...) {
			continue;
		}

		std::string subsystem = subsystem_path.filename();

		SerialDeviceInfo info;
		info.device = dev;

		// Handle USB serial and CDC devices
		if (subsystem == "usb-serial" || subsystem == "usb") {
			std::filesystem::path usb_interface = (subsystem == "usb-serial")
				                                      ? device_path.parent_path()
				                                      : device_path;
			std::filesystem::path usb_device = usb_interface.parent_path();

			std::string idVendor = read_line(usb_device / "idVendor");
			std::string idProduct = read_line(usb_device / "idProduct");

			if (!idVendor.empty())
				info.vid = static_cast<uint16_t>(std::stoi(idVendor, nullptr, 16));
			if (!idProduct.empty())
				info.pid = static_cast<uint16_t>(std::stoi(idProduct, nullptr, 16));

			info.serialNumber = read_line(usb_device / "serial");
			info.manufacturer = read_line(usb_device / "manufacturer");
			info.product = read_line(usb_device / "product");
			info.location = usb_device.filename();
		}

		ports.push_back(info);
	}

	return ports;
}