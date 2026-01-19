#include <tbb/concurrent_queue.h>

//#define USE_BOOST

#ifdef USE_BOOST
#include <boost/asio.hpp>
#endif

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <deque>
#include <expected>
#include <forward_list>
#include <iostream>
#include <list>
#include <memory>
#include <ranges>
#include <thread>

#include "Array.h"
#include "ERROR.h"
#include "MagneticFluxDensityData.h"
#include "Message.h"
#include "common.h"
#include "common_output.h"

class Connection {
#ifdef USE_BOOST
	boost::asio::io_context _context;
	boost::asio::serial_port _serial_port;
#else
	int _serial_port = -1;
#endif

	enum class ConnectionState {
		NONE,
		CONNECTED,
		READING,
	};
	std::atomic<ConnectionState> state = ConnectionState::NONE;

	// std::size_t current_buffer_index = 0;

	std::thread connection_thread;

	std::list<Message<Array<MagneticFluxDensityData, 25>>> mag_data;

	std::optional<ERR> error = std::nullopt;

   public:
#ifdef USE_BOOST
	explicit Connection() : _serial_port(_context) { std::cout << "Connection()" << std::endl; }
#else
	explicit Connection() { std::cout << "Connection()" << std::endl; }
#endif

	std::expected<void, ERR> open_serial_port(std::string const& device) {
#ifdef USE_BOOST
		_serial_port.open(device);
		_serial_port.set_option(boost::asio::serial_port_base::baud_rate(230400));
#else
		_serial_port = open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
		if (_serial_port == -1) {
			return std::unexpected{ERR{errno, std::strerror(errno)}};
		}

		struct termios tty{};
		if (tcgetattr(_serial_port, &tty) != 0) {
			close(_serial_port);
			return std::unexpected{ERR{errno, std::strerror(errno)}};
		}

		// Configure serial port basics
		cfsetispeed(&tty, 230400);
		cfsetospeed(&tty, 230400);

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
			std::unexpected(ERR{errno, std::strerror(errno)});
		}
#endif

		state.store(ConnectionState::CONNECTED);
		return {};
	}

	void close_serial_port() {
		if (auto expected = ConnectionState::CONNECTED; state.compare_exchange_strong(expected, ConnectionState::NONE)) {
#ifdef USE_BOOST
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
				std::array<std::uint8_t, 256> buffer;
				while (state.load() == ConnectionState::READING) {
#ifdef USE_BOOST
					boost::system::error_code ec;
					if (size_t const bytes_transferred = _serial_port.read_some(boost::asio::buffer(buffer), ec); ec) {
						error.emplace(ERR{ec.value()});
						std::cout << "ERRORRRRRRRR: " << ec << std::endl;
						break;
					} else if (bytes_transferred > 0) {
						std::cout << "Read " << bytes_transferred << " bytes: ";
						for (auto const e : buffer | std::ranges::views::take(bytes_transferred)) {
							std::cout << static_cast<char>(e);
						}
						std::cout << std::endl;
					}
#else
					if (ssize_t const bytes_transferred = read(_serial_port, buffer.data(), buffer.size()); bytes_transferred < 0) {
						if (errno != EAGAIN && errno != EWOULDBLOCK) {
							error.emplace(ERR{errno, std::strerror(errno)});
							std::cout << "ERRORRRRRRRR: " << errno << std::endl;
							break;
						}
					} else if (bytes_transferred > 0) {
						std::cout << "Read " << static_cast<std::size_t>(bytes_transferred) << " bytes: ";
						for (auto const e : buffer | std::ranges::views::take(bytes_transferred)) {
							std::cout << static_cast<char>(e);
						}
						std::cout << std::endl;
					}
#endif
				}
				std::cout << "disconnected!" << std::endl;
			});
		}
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

	~Connection() {
		stop_reading();
		close_serial_port();
	}
};

int main() {
	Connection conn;

	conn.open_serial_port("/dev/cu.usbserial-0001");

	conn.start_reading();

	std::this_thread::sleep_for(std::chrono::milliseconds(10000));

	std::cout << "test" << std::endl;
}

// struct test {
//	static int i;
//	int ii = ++i;
//	int iii;
//	explicit test(int const iii) : iii(iii) { std::cout << "test() " << ii << std::endl; }
//	~test() { std::cout << "~test() " << ii << std::endl; }
// };
//
// int test::i = 0;
//
// int main() {
//	std::list<std::shared_ptr<test>> tests;
//
//	std::thread thread1([&tests]() {
//		while (true) {
//			auto test1 = std::make_shared<test>(0);
//			tests.push_front(test1);
//		}
//	});
//
//	std::thread thread2([&tests]() {
//		while (true) {
//			auto t = tests.begin();
//			for (auto i = 0; i < 6; ++i) {
//				if (t != tests.end()) {
//					std::cout << t->get()->ii << std::endl;
//					++t;
//				} else {
//					break;
//				}
//			}
//
//			tests.erase(t, tests.end());
//		}
//	});
//
//	std::this_thread::sleep_for(std::chrono::milliseconds(100000));
//
//	return 0;
// }
