#pragma once

#include <fcntl.h>
#include <sys/termios.h>
#include <unistd.h>

#include <boost/crc.hpp>
#include <cerrno>
#include <cstring>
#include <expected>
#include <iostream>
#include <memory>
#include <thread>

#include "ring_buffer.h"

// template <typename T>
// struct is_MESSAGE_TYPE : std::false_type {};
//
// template <typename T>
// struct id_of : std::integral_constant<char, 0> {};
//
// template <typename T>
// struct type_of {};
//
// template <typename T>
// struct message_size_of : std::integral_constant<std::size_t, 0> {};
//
// template <char id, typename crc_class, std::size_t message_size = std::numeric_limits<std::size_t>::max()>
// requires requires { crc_class::bit_count; } class MESSAGE_TYPE {};
//
// template <char id, typename crc_class, std::size_t message_size>
// struct is_MESSAGE_TYPE<MESSAGE_TYPE<id, crc_class, message_size>> : std::true_type {};
//
// template <char id, typename crc_class, std::size_t message_size>
// struct id_of<MESSAGE_TYPE<id, crc_class, message_size>> : std::integral_constant<char, id> {};
//
// template <char id, typename crc_class, std::size_t message_size>
// struct type_of<MESSAGE_TYPE<id, crc_class, message_size>> : std::type_identity<crc_class> {};
//
// template <char id, typename crc_class, std::size_t message_size>
// struct message_size_of<MESSAGE_TYPE<id, crc_class, message_size>> : std::integral_constant<std::size_t, message_size> {};
//
// template <typename... MESSAGE_TYPEs>
// requires(is_MESSAGE_TYPE<MESSAGE_TYPEs>::value && ...) class test {
//    public:
//	test() { (print<MESSAGE_TYPEs>(), ...); }
//
//	template <typename m>
//	void print() {
//		std::cout << "Printing type: " << message_size_of<m>::value << '\n';
//	}
// };
//
// inline test<MESSAGE_TYPE<'T', boost::crc_optimal<8, 0x07, 0x00, 0x00, false, false>, 19>, MESSAGE_TYPE<'M', boost::crc_16_type>> teste;

enum Baudrate : unsigned int {
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

		tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;  // 8-bit chars
		tty.c_iflag &= ~IGNBRK;                      // disable break processing
		tty.c_lflag = 0;                             // no signaling chars, no echo
		tty.c_oflag = 0;                             // no remapping, no delays
		tty.c_cc[VMIN] = 0;                          // read doesn't block
		tty.c_cc[VTIME] = 1;                         // no timeout (1 = 0.1s (100ms); 1 decisecond = 100ms)

		tty.c_iflag &= ~(IXON | IXOFF | IXANY);  // shut off xon/xoff ctrl
		tty.c_cflag |= (CLOCAL | CREAD);         // ignore modem controls, enable reading
		tty.c_cflag &= ~(PARENB | PARODD);       // no parity
		tty.c_cflag &= ~CSTOPB;                  // one stop bit
		tty.c_cflag &= ~CRTSCTS;                 // no flow control

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

	[[nodiscard]] std::expected<std::span<const char>, SerialError> read_some() const {
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

	[[nodiscard]] std::expected<std::size_t, SerialError> read_some(std::span<std::uint8_t> buffer) const {
		if (ssize_t const bytes_transferred = read(_serial_port, buffer.data(), buffer.size()); bytes_transferred < 0) {
			if (errno == EAGAIN || errno == EWOULDBLOCK) {
				return 0;
			}
			return std::unexpected(SerialError{errno, std::strerror(errno)});
		} else {
			std::cout << static_cast<std::size_t>(bytes_transferred) << std::endl;
			return static_cast<std::size_t>(bytes_transferred);
		}
	}

	// template <typename... MESSAGE_TYPEs>
	// requires(is_MESSAGE_TYPE<MESSAGE_TYPEs>::value && ...) [[nodiscard]] std::expected<std::span<const char>, SerialError> read_message() {
	//	static ring_buffer<std::uint8_t, 8192> buffer;
	//	constexpr std::size_t min_message_size = std::min({message_size_of<MESSAGE_TYPEs>::value...});
	//
	//	while (true) {
	//		do {
	//			auto t = buffer.linear_sub_array();
	//			if (ssize_t const bytes_transferred = read(_serial_port, t.data(), t.size()); bytes_transferred < 0) {
	//				if (errno != EAGAIN && errno != EWOULDBLOCK) {
	//					return std::unexpected(SerialError{errno, std::strerror(errno)});
	//				}
	//
	//				buffer.rotate(static_cast<std::size_t>(bytes_transferred));
	//			}
	//		} while (buffer.size() < min_message_size);
	//
	//		auto decode = []<typename T>(auto i) -> std::expected<void, > {
	//			if constexpr (message_size_of<T>::value < std::numeric_limits<std::size_t>::max()) {
	//				if (buffer[i] == id_of<T>::value) {
	//					if (buffer.size() < i + message_size_of<T>::value || buffer[i + message_size_of<T>::value - 1] != id_of<T>::value) return {};
	//				}
	//			} else {
	//			}
	//		};
	//
	//		for (auto i = 0; i < buffer.size(); ++i) {
	//			if (auto e = (decode.template operator()<MESSAGE_TYPEs>(i) && ...)) {
	//			} else {
	//				return e.value();
	//			}
	//		}
	//	}
	//}

	Baudrate& baud() { return _baud; }
	[[nodiscard]] bool connected() const { return _connected; }

	SerialConnection() = default;
};