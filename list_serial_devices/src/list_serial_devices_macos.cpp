#include "list_serial_devices.h"

#include <vector>
#include <string>

#include <CoreFoundation/CoreFoundation.h>
#include <IOKit/IOKitLib.h>
#include <IOKit/serial/IOSerialKeys.h>
#include <IOKit/usb/USB.h>
#include <IOKit/IOBSD.h>

std::string CFStringToStdString(CFStringRef const str) {
	if (!str) return "";
	char buffer[256];
	if (CFStringGetCString(str, buffer, sizeof(buffer), kCFStringEncodingUTF8))
		return buffer;
	return "";
}

uint32_t GetIntProperty(io_registry_entry_t const device, CFStringRef const key) {
	CFTypeRef const value = IORegistryEntryCreateCFProperty(device, key, kCFAllocatorDefault, 0);
	if (!value) return 0;
	uint32_t num = 0;
	if (CFGetTypeID(value) == CFNumberGetTypeID()) {
		CFNumberGetValue(static_cast<CFNumberRef>(value), kCFNumberSInt32Type, &num);
	}
	CFRelease(value);
	return num;
}

std::string GetStringProperty(io_registry_entry_t const device, CFStringRef const key) {
	CFTypeRef const value = IORegistryEntryCreateCFProperty(device, key, kCFAllocatorDefault, 0);
	std::string result;
	if (value) {
		if (CFGetTypeID(value) == CFStringGetTypeID()) {
			result = CFStringToStdString((CFStringRef)value);
		}
		CFRelease(value);
	}
	return result;
}

io_registry_entry_t GetParentDeviceByType(io_registry_entry_t device, std::string const& parentType) {
	io_registry_entry_t parent;
	while (true) {
		io_name_t className;
		IOObjectGetClass(device, className);
		if (parentType == className)
			return device;

		kern_return_t kr = IORegistryEntryGetParentEntry(device, kIOServicePlane, &parent);
		if (kr != KERN_SUCCESS)
			return IO_OBJECT_NULL;
		device = parent;
	}
}

std::string LocationToString(uint32_t locationID) {
	std::string result = std::to_string(locationID >> 24) + "-";
	while (locationID & 0xf00000) {
		if (result.back() != '-') result += '.';
		result += std::to_string((locationID >> 20) & 0xf);
		locationID <<= 4;
	}
	return result;
}


std::vector<SerialDeviceInfo> list_serial_devices() {
	std::vector<SerialDeviceInfo> ports;

	CFMutableDictionaryRef matchingDict = IOServiceMatching(kIOSerialBSDServiceValue);
	if (!matchingDict) return ports;
	CFDictionarySetValue(matchingDict, CFSTR(kIOSerialBSDTypeKey), CFSTR(kIOSerialBSDAllTypes));

	io_iterator_t iterator;
	if (IOServiceGetMatchingServices(kIOMainPortDefault, matchingDict, &iterator) != KERN_SUCCESS)
		return ports;

	io_object_t service;
	while ((service = IOIteratorNext(iterator))) {
		CFTypeRef const deviceCF = IORegistryEntryCreateCFProperty(service, CFSTR(kIOCalloutDeviceKey), kCFAllocatorDefault, 0);
		if (!deviceCF) {
			IOObjectRelease(service);
			continue;
		}

		std::string const devicePath = CFStringToStdString(static_cast<CFStringRef>(deviceCF));
		CFRelease(deviceCF);

		SerialDeviceInfo info;
		info.device = devicePath;

		// climb up to USB device
		io_registry_entry_t usbDevice = GetParentDeviceByType(service, "IOUSBHostDevice");
		if (usbDevice == IO_OBJECT_NULL)
			usbDevice = GetParentDeviceByType(service, "IOUSBDevice");

		if (usbDevice != IO_OBJECT_NULL) {
			info.vid = GetIntProperty(usbDevice, CFSTR("idVendor"));
			info.pid = GetIntProperty(usbDevice, CFSTR("idProduct"));
			info.serialNumber = GetStringProperty(usbDevice, CFSTR("USB Serial Number"));
			info.manufacturer = GetStringProperty(usbDevice, CFSTR("USB Vendor Name"));
			info.product = GetStringProperty(usbDevice, CFSTR("USB Product Name"));
			if (std::uint32_t const locationID = GetIntProperty(usbDevice, CFSTR("locationID")))
				info.location = LocationToString(locationID);
			IOObjectRelease(usbDevice);
		}

		ports.push_back(info);
		IOObjectRelease(service);
	}

	IOObjectRelease(iterator);
	return ports;
}