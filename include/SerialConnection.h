#pragma once

#include <iostream>
#include <thread>
#include <expected>
#include <fcntl.h>
#include <unistd.h>
#include <sys/termios.h>

#include <cerrno>
#include <cstring>

#include "ring_buffer.h"

enum Baudrate: unsigned int {
	BAUD9600 = 9600,
	BAUD19200 = 19200,
	BAUD38400 = 38400,
	BAUD57600 = 57600,
	BAUD115200 = 115200,
	BAUD230400 = 230400,
	BAUD460800 = 460800,
	BAUD921600 = 921600,
};

struct SerialError {
	int code;
	std::string message;
};

class SerialConnection {
	Baudrate _baud = Baudrate::BAUD230400;
	int _serial_port = -1;
	bool _connected = false;

public:
	std::expected<void, SerialError> open_serial_port(std::string const& device) {
		_serial_port = open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
		if (_serial_port == -1) {
			return std::unexpected{SerialError{errno, std::strerror(errno)}};
		}

		struct termios tty{};
		if (tcgetattr(_serial_port, &tty) != 0) {
			close(_serial_port);
			return std::unexpected{SerialError{errno, std::strerror(errno)}};
		}

		// Configure serial port basics
		cfsetispeed(&tty, _baud);
		cfsetospeed(&tty, _baud);

		tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit chars
		tty.c_iflag &= ~IGNBRK; // disable break processing
		tty.c_lflag = 0; // no signaling chars, no echo
		tty.c_oflag = 0; // no remapping, no delays
		tty.c_cc[VMIN] = 0; // read doesn't block
		tty.c_cc[VTIME] = 0; // no timeout (1 = 0.1s (100ms); 1 decisecond = 100ms)

		tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl
		tty.c_cflag |= (CLOCAL | CREAD); // ignore modem controls, enable reading
		tty.c_cflag &= ~(PARENB | PARODD); // no parity
		tty.c_cflag &= ~CSTOPB; // one stop bit
		tty.c_cflag &= ~CRTSCTS; // no flow control

		if (tcsetattr(_serial_port, TCSANOW, &tty) != 0) {
			close(_serial_port);
			std::unexpected(SerialError{errno, std::strerror(errno)});
		}

		_connected = true;
		return {};
	}

	void close_serial_port() {
		_connected = false;
		close(_serial_port);
		_serial_port = -1;
	}

	std::expected<std::span<const char>, SerialError> read_some() const {
		static std::array<char, 128> tmp;
		if (ssize_t const bytes_transferred = read(_serial_port, tmp.data(), tmp.size()); bytes_transferred < 0) {
			if (errno == EAGAIN || errno == EWOULDBLOCK) {
				return std::span<const char>{tmp.begin(), static_cast<std::size_t>(bytes_transferred)};
			}
			return std::unexpected(SerialError{errno, std::strerror(errno)});
		} else {
			return std::span<const char>{tmp.begin(), static_cast<std::size_t>(bytes_transferred)};
		}
	}

	Baudrate& baud() { return _baud; }
	bool connected() const { return _connected; }

	SerialConnection() = default;

	//alive.test_and_set();
	//void disconnect() {
	//	alive.clear();
	//
	//	serial_thread.join();
	//}
	//	std::thread serial_thread;
	//std::atomic_flag alive;

	//ring_buffer<std::uint8_t, 8192> buffer;
	//std::span<char> linear_array() {
	//	static std::array<char, 8192> tmp;
	//
	//	for (std::size_t i = 0; i < buffer.size(); ++i) {
	//		tmp[i] = buffer[i];
	//	}
	//
	//	return {tmp.data(), buffer.size()};
	//}
};