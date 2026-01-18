#include <tbb/concurrent_queue.h>

#include <boost/asio.hpp>
#include <deque>
#include <expected>
#include <iostream>

#include "Array.h"
#include "ERROR.h"
#include "MagneticFluxDensityData.h"
#include "Message.h"

class Connection {
	int _serial_port = -1;
	std::atomic_bool _connected = false;

	std::array<std::array<std::uint8_t, 1024>, 10> buffers;
	std::size_t current_buffer_index = 0;

	std::thread connection_thread;

	std::deque<std::expected<Message<Array<MagneticFluxDensityData, 25>>, ERR>> mag_data;

   public:
	explicit Connection() { std::cout << "Connection()" << std::endl; }

	std::expected<void, ERR> open_serial_port(std::string const& device) {
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

		_connected = true;
		return {};
	}

	void close_serial_port() {
		if (bool expected = true; _connected.compare_exchange_strong(expected, false)) {
			close(_serial_port);
			_serial_port = -1;
		}
	}

	void connect() {
		if (auto expected = false; _connected.compare_exchange_strong(expected, true)) {
			connection_thread = std::thread([this]() {
				while (_connected) {
					boost::system::error_code ec;
					std::size_t const bytes = 0;//_serial_port.read_some(boost::asio::buffer(buffers[current_buffer_index]), ec);

					if (ec) {
						std::cout << "Read error: " << ec.message() << std::endl;
						break;
					}

					std::cout << "Read " << bytes << " bytes: ";
					std::cout.write(reinterpret_cast<char*>(buffers[current_buffer_index].data()), bytes);
					std::cout << std::endl;

					current_buffer_index = (current_buffer_index + 1) % buffers.size();
				}
			});
		}
	}

	void disconnect() {
		_connected.store(false);
		if (connection_thread.joinable()) {
			connection_thread.join();
		}
	}

	~Connection() { disconnect(); }
};

int main() {
	std::cout << sizeof(test) << std::endl;

	std::exit(0);

	Connection conn;

	conn.connect();

	std::this_thread::sleep_for(std::chrono::milliseconds(1000));

	std::cout << "test" << std::endl;
}