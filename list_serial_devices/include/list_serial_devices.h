#pragma once
#include <string>
#include <cstdint>
#include <vector>

struct SerialDeviceInfo {
	std::string device;
	std::string manufacturer;
	std::string product;
	std::string serialNumber;
	std::string location;
	std::uint16_t vid = 0;
	std::uint16_t pid = 0;
};

std::vector<SerialDeviceInfo> list_serial_devices();