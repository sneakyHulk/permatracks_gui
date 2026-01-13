#include <array>
#include <iostream>

#include "SerialConnection.h"

struct test : SerialConnection {
	test() {
		auto open_result = open_serial_port("/dev/ttyUSB0");
		if (!open_result) {
			std::cerr << "open_serial_port failed: " << open_result.error().what() << '\n';
			return;
		}

		// -------- read_some(): span<const char> --------
		auto r1 = read_some();
		if (!r1) {
			std::cerr << "read_some() failed: " << r1.error().what() << '\n';
		} else {
			std::span<const char> data = *r1;
			std::cout << "read_some() returned " << data.size() << " bytes\n";
		}

		std::this_thread::sleep_for(std::chrono::milliseconds(100));

		// -------- read_some(span<uint8_t>) --------
		std::array<std::uint8_t, 256> buffer{};
		auto r2 = read_some(buffer);
		if (!r2) {
			std::cerr << "read_some(buffer) failed: " << r2.error().what() << '\n';
		} else {
			std::cout << "read_some(buffer) returned " << *r2 << " bytes\n";
		}

		close_serial_port();
	}
};

int main() {
	test test1;
	return 0;
}