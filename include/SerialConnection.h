#pragma once

#include <common_output.h>

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

class SerialConnection {
	enum class ConnectionState {
		NONE,
		CONNECTED,
		READING,
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

	std::deque<std::uint8_t> data;
	std::optional<ERR> error = std::nullopt;

	std::uint64_t t_latest_message = 0;

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
			common::println_error_loc(std::strerror(errno), " (", errno, ")");
			return std::unexpected{ERR{errno, std::strerror(errno)}};
		}

		struct termios tty{};
		if (tcgetattr(_serial_port, &tty) != 0) {
			close(_serial_port);
			common::println_error_loc(std::strerror(errno), " (", errno, ")");
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
			common::println_error_loc(std::strerror(errno), " (", errno, ")");
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
			common::println_error_loc("state is ", expected == ConnectionState::CONNECTED ? "CONNECTED" : expected == ConnectionState::NONE ? "NONE" : expected == ConnectionState::READING ? "READING" : "ERROR");
		}
	}

	void start_reading() {
		if (auto expected = ConnectionState::CONNECTED; state.compare_exchange_strong(expected, ConnectionState::READING)) {
			connection_thread = std::thread([this]() {
				std::cout << "connected" << std::endl;
				std::array<std::uint8_t, 512> data;
				while (state.load() == ConnectionState::READING) {
#ifdef SERIALCONNECTION_USE_BOOST
					boost::system::error_code ec;
					if (std::size_t const bytes_transferred = _serial_port.read_some(boost::asio::buffer(data), ec); ec) {
						error.emplace(ERR{ec.value()});
						common::println_error_loc(ec.what(), " (", ec.value(), ")");
						break;
					} else if (bytes_transferred > 0) {
						t_latest_message = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
						parse(std::span{data.data(), bytes_transferred});
					}
#else
					if (ssize_t const bytes_transferred = read(_serial_port, data.data(), data.size()); bytes_transferred < 0) {
						if (errno != EAGAIN && errno != EWOULDBLOCK) {
							error.emplace(ERR{errno, std::strerror(errno)});
							common::println_error_loc(std::strerror(errno), " (", errno, ")");
							break;
						}
					} else if (bytes_transferred > 0) {
						t_latest_message = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
						parse(std::span{data.data(), static_cast<std::size_t>(bytes_transferred)});
					}
#endif
				}
				std::cout << "disconnected!" << std::endl;
			});
		} else {
			common::println_error_loc("state is ", expected == ConnectionState::CONNECTED ? "CONNECTED" : expected == ConnectionState::NONE ? "NONE" : expected == ConnectionState::READING ? "READING" : "ERROR");
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

	void stop_reading() {
		if (auto expected = ConnectionState::READING; state.compare_exchange_strong(expected, ConnectionState::CONNECTED)) {
			// if (connection_thread.joinable()) {
			connection_thread.join();
			//}
		} else {
			common::println_error_loc("state is ", expected == ConnectionState::CONNECTED ? "CONNECTED" : expected == ConnectionState::NONE ? "NONE" : expected == ConnectionState::READING ? "READING" : "ERROR");
		}
	}

	std::expected<std::span<const char>, ERR> read_some() const {
		if (state.load() == ConnectionState::CONNECTED) {
			static std::array<char, 128> tmp;
			if (ssize_t const bytes_transferred = read(_serial_port, tmp.data(), tmp.size()); bytes_transferred < 0) {
				if (errno == EAGAIN || errno == EWOULDBLOCK) {
					return std::span<const char>{tmp.begin(), static_cast<std::size_t>(bytes_transferred)};
				}
				return std::unexpected(ERR{errno, std::strerror(errno)});
			} else {
				return std::span<const char>{tmp.begin(), static_cast<std::size_t>(bytes_transferred)};
			}
		}

		return std::unexpected(ERR{0, "Not connected or a read operation is already in progress."});
	}

   protected:
	~SerialConnection() {
		stop_reading();
		close_serial_port();
	}

	bool connected() const { return state.load(std::memory_order_acquire) != ConnectionState::NONE; }


   public:
	virtual void parse(std::span<std::uint8_t>&& data) = 0;
};