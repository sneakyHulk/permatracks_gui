#pragma once

// #include <common_output.h>

// #define SERIALCONNECTION_USE_BOOST

#ifdef SERIALCONNECTION_USE_BOOST
#include <boost/asio.hpp>
#endif

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <deque>
#include <expected>
#include <iostream>
#include <optional>
#include <span>
#include <thread>

#include "ERROR.h"

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

struct MessagePart {
	std::uint64_t timestamp;
	std::span<const char> data;
};

class SerialConnection {
	enum class ConnectionState {
		NONE,
		CONNECTED,
		// READING,
	};
	std::atomic<ConnectionState> state = ConnectionState::NONE;

	std::thread connection_thread;

#ifdef SERIALCONNECTION_USE_BOOST
	boost::asio::io_context _context;
	boost::asio::serial_port _serial_port;
#else
	int _serial_port = -1;
#endif

   protected:
	Baudrate baud = Baudrate::BAUD230400;

   public:
#ifdef SERIALCONNECTION_USE_BOOST
	explicit SerialConnection() : _serial_port(_context) { std::cout << "Connection()" << std::endl; }
#else
	explicit SerialConnection() { std::cout << "Connection()" << std::endl; }
#endif

	std::expected<void, ERR> open_serial_port(std::string const& device) {
#ifdef SERIALCONNECTION_USE_BOOST
		_serial_port.open(device);
		_serial_port.set_option(boost::asio::serial_port_base::baud_rate(230400));
#else
		_serial_port = open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
		if (_serial_port == -1) {
			std::cout << std::strerror(errno) << " (" << errno << ")" << std::endl;
			return std::unexpected{ERR{errno, std::strerror(errno)}};
		}

		struct termios tty{};
		if (tcgetattr(_serial_port, &tty) != 0) {
			close(_serial_port);
			std::cout << std::strerror(errno) << " (" << errno << ")" << std::endl;
			return std::unexpected{ERR{errno, std::strerror(errno)}};
		}

		// Configure serial port basics
		cfsetispeed(&tty, baud);
		cfsetospeed(&tty, baud);

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
			std::cout << std::strerror(errno) << " (" << errno << ")" << std::endl;
			std::unexpected(ERR{errno, std::strerror(errno)});
		}
#endif

		state.store(ConnectionState::CONNECTED);
		return {};
	}

	void close_serial_port() {
		if (auto expected = ConnectionState::CONNECTED; state.compare_exchange_strong(expected, ConnectionState::NONE)) {
#ifdef SERIALCONNECTION_USE_BOOST
			_serial_port.close();
#else
			close(_serial_port);
			_serial_port = -1;
#endif
		} else {
			std::cout << "state is " << (expected == ConnectionState::CONNECTED ? "CONNECTED" : expected == ConnectionState::NONE ? "NONE" : "ERROR") << std::endl;
		}
	}

	std::expected<void, ERR> write_all(std::span<std::uint8_t const> const buffer) {
#ifdef SERIALCONNECTION_USE_BOOST
		boost::system::error_code ec;
		std::size_t n = boost::asio::write(_serial_port, boost::asio::buffer(buffer.data(), buffer.size()), ec);

		if (ec) {
			return std::unexpected(ERR{ec.value(), ec.message()});
		}
#else
		struct pollfd pfd{.fd = _serial_port, .events = POLLOUT, .revents = 0};

		if (poll(&pfd, 1, 100) < 0) {  // up to 100ms timeout
			return std::unexpected(ERR{errno, std::strerror(errno)});
		}

		std::size_t total_written = 0;
		while (total_written < buffer.size()) {
			ssize_t const bytes_written = write(_serial_port, buffer.data() + total_written, buffer.size() - total_written);

			if (bytes_written < 0) {
				if (errno == EAGAIN || errno == EWOULDBLOCK) {
					if (poll(&pfd, 1, 100) < 0) {  // up to 100ms timeout
						return std::unexpected(ERR{errno, std::strerror(errno)});
					}

					continue;
				}
				return std::unexpected(ERR{errno, std::strerror(errno)});
			}

			total_written += static_cast<std::size_t>(bytes_written);
		}
#endif

		return {};
	}

	std::expected<MessagePart, ERR> read_some() {
		static std::array<char, 512> tmp;

		if (state.load() == ConnectionState::CONNECTED) {
#ifdef SERIALCONNECTION_USE_BOOST
			boost::system::error_code ec;
			std::size_t const bytes_transferred = _serial_port.read_some(boost::asio::buffer(tmp.data(), tmp.size()), ec);
			std::uint64_t const timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

			if (ec) {
				return std::unexpected(ERR{ec.value(), ec.message()});
			}

			return MessagePart{timestamp, std::span<const char>{tmp.data(), bytes_transferred}};
#else
			ssize_t const bytes_transferred = read(_serial_port, tmp.data(), tmp.size());
			std::uint64_t const timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

			if (bytes_transferred < 0) {
				if (errno == EAGAIN || errno == EWOULDBLOCK) {
					return MessagePart{timestamp, std::span<const char>{tmp.data(), 0}};
				}

				return std::unexpected(ERR{errno, std::strerror(errno)});
			}

			return MessagePart{timestamp, std::span<const char>{tmp.data(), static_cast<std::size_t>(bytes_transferred)}};
#endif
		}

		return std::unexpected(ERR{0, "Not connected or a read operation is already in progress."});
	}

   protected:
	~SerialConnection() { close_serial_port(); }

	bool connected() const { return state.load() != ConnectionState::NONE; }
};