#include <devguid.h>
#include <devpkey.h>
#include <initguid.h>
#include <setupapi.h>
#include <windows.h>

#include <string>
#include <vector>

#include "list_serial_devices.h"

#pragma comment(lib, "setupapi.lib")

// Helper to get device registry property as string
std::string GetDeviceRegistryProperty(HDEVINFO deviceInfoSet, PSP_DEVINFO_DATA deviceInfoData, DWORD property) {
	DWORD dataType;
	DWORD bufferSize = 0;

	// Get required buffer size
	SetupDiGetDeviceRegistryPropertyA(deviceInfoSet, deviceInfoData, property, &dataType, nullptr, 0, &bufferSize);

	if (bufferSize == 0) return {};

	std::vector<char> buffer(bufferSize);
	if (SetupDiGetDeviceRegistryPropertyA(deviceInfoSet, deviceInfoData, property, &dataType, reinterpret_cast<PBYTE>(buffer.data()), bufferSize, nullptr)) {
		return std::string(buffer.data());
	}

	return {};
}

// Helper to get device property as string (newer API)
std::string GetDeviceProperty(HDEVINFO deviceInfoSet, PSP_DEVINFO_DATA deviceInfoData, const DEVPROPKEY* propertyKey) {
	DEVPROPTYPE propertyType;
	DWORD bufferSize = 0;

	// Get required buffer size
	SetupDiGetDevicePropertyW(deviceInfoSet, deviceInfoData, propertyKey, &propertyType, nullptr, 0, &bufferSize, 0);

	if (bufferSize == 0) return {};

	std::vector<BYTE> buffer(bufferSize);
	if (SetupDiGetDevicePropertyW(deviceInfoSet, deviceInfoData, propertyKey, &propertyType, buffer.data(), bufferSize, nullptr, 0)) {
		if (propertyType == DEVPROP_TYPE_STRING) {
			std::wstring wstr(reinterpret_cast<wchar_t*>(buffer.data()));
			int size = WideCharToMultiByte(CP_UTF8, 0, wstr.c_str(), -1, nullptr, 0, nullptr, nullptr);
			if (size > 0) {
				std::string result(size - 1, '\0');
				WideCharToMultiByte(CP_UTF8, 0, wstr.c_str(), -1, &result[0], size, nullptr, nullptr);
				return result;
			}
		}
	}

	return {};
}

// Extract VID/PID from hardware ID string (format: USB\VID_xxxx&PID_yyyy)
void ParseHardwareID(const std::string& hwid, uint16_t& vid, uint16_t& pid) {
	vid = 0;
	pid = 0;

	size_t vidPos = hwid.find("VID_");
	size_t pidPos = hwid.find("PID_");

	if (vidPos != std::string::npos && vidPos + 8 <= hwid.length()) {
		vid = static_cast<uint16_t>(std::stoi(hwid.substr(vidPos + 4, 4), nullptr, 16));
	}

	if (pidPos != std::string::npos && pidPos + 8 <= hwid.length()) {
		pid = static_cast<uint16_t>(std::stoi(hwid.substr(pidPos + 4, 4), nullptr, 16));
	}
}

// Get COM port name from device
std::string GetComPortName(HDEVINFO deviceInfoSet, PSP_DEVINFO_DATA deviceInfoData) {
	HKEY hKey = SetupDiOpenDevRegKey(deviceInfoSet, deviceInfoData, DICS_FLAG_GLOBAL, 0, DIREG_DEV, KEY_READ);
	if (hKey == INVALID_HANDLE_VALUE) return {};

	char portName[256] = {0};
	DWORD size = sizeof(portName);
	DWORD type = 0;

	if (RegQueryValueExA(hKey, "PortName", nullptr, &type, reinterpret_cast<LPBYTE>(portName), &size) == ERROR_SUCCESS) {
		RegCloseKey(hKey);
		return std::string(portName);
	}

	RegCloseKey(hKey);
	return {};
}

std::vector<SerialDeviceInfo> list_serial_devices() {
	std::vector<SerialDeviceInfo> ports;

	// Get all devices in the Ports class
	HDEVINFO deviceInfoSet = SetupDiGetClassDevsA(&GUID_DEVCLASS_PORTS, nullptr, nullptr, DIGCF_PRESENT);
	if (deviceInfoSet == INVALID_HANDLE_VALUE) return ports;

	SP_DEVINFO_DATA deviceInfoData;
	deviceInfoData.cbSize = sizeof(SP_DEVINFO_DATA);

	for (DWORD i = 0; SetupDiEnumDeviceInfo(deviceInfoSet, i, &deviceInfoData); i++) {
		SerialDeviceInfo info;

		// Get COM port name (e.g., COM3)
		std::string portName = GetComPortName(deviceInfoSet, &deviceInfoData);
		if (portName.empty()) continue;

		info.device = "\\\\.\\" + portName;

		// Get hardware ID to extract VID/PID
		std::string hardwareID = GetDeviceRegistryProperty(deviceInfoSet, &deviceInfoData, SPDRP_HARDWAREID);
		if (!hardwareID.empty()) {
			ParseHardwareID(hardwareID, info.vid, info.pid);
		}

		// Get manufacturer
		info.manufacturer = GetDeviceRegistryProperty(deviceInfoSet, &deviceInfoData, SPDRP_MFG);

		// Get friendly name as product description
		std::string friendlyName = GetDeviceRegistryProperty(deviceInfoSet, &deviceInfoData, SPDRP_FRIENDLYNAME);
		if (!friendlyName.empty()) {
			// Remove the (COMx) suffix if present
			size_t pos = friendlyName.find(" (COM");
			if (pos != std::string::npos) {
				info.product = friendlyName.substr(0, pos);
			} else {
				info.product = friendlyName;
			}
		}

		// Get location information
		std::string locationInfo = GetDeviceRegistryProperty(deviceInfoSet, &deviceInfoData, SPDRP_LOCATION_INFORMATION);
		if (!locationInfo.empty()) {
			info.location = locationInfo;
		}

		// Try to get serial number from device instance ID or parent device
		std::string instanceID = GetDeviceProperty(deviceInfoSet, &deviceInfoData, &DEVPKEY_Device_InstanceId);
		if (!instanceID.empty()) {
			// Serial number is often the last part of the instance ID after the last backslash
			size_t lastSlash = instanceID.find_last_of('\\');
			if (lastSlash != std::string::npos && lastSlash + 1 < instanceID.length()) {
				std::string serialCandidate = instanceID.substr(lastSlash + 1);
				// Check if it looks like a serial number (not just a generic identifier)
				if (serialCandidate.length() > 2 && serialCandidate.find('&') == std::string::npos) {
					info.serialNumber = serialCandidate;
				}
			}
		}

		ports.push_back(info);
	}

	SetupDiDestroyDeviceInfoList(deviceInfoSet);
	return ports;
}