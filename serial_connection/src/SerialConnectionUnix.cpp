#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>

#include "SerialConnection.h"

SerialConnection::SerialConnection() { std::cout << "SerialConnection()" << std::endl; }
std::expected<void, ERR> SerialConnection::open_serial_port(std::string const& device) {
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
		std::unexpected(ERR{errno, std::strerror(errno)});
	}

	_connected = true;
	return {};
}
void SerialConnection::close_serial_port() {
	if (bool expected = true; _connected.compare_exchange_strong(expected, false)) {
		close(_serial_port);
		_serial_port = -1;
	}
}
std::expected<std::span<const char>, ERR> SerialConnection::read_some() const {
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
std::expected<std::size_t, ERR> SerialConnection::read_some(std::span<std::uint8_t> buffer) const {
	if (ssize_t const bytes_transferred = read(_serial_port, buffer.data(), buffer.size()); bytes_transferred < 0) {
		if (errno == EAGAIN || errno == EWOULDBLOCK) {
			return 0;
		}
		return std::unexpected(ERR{errno, std::strerror(errno)});
	} else {
		return static_cast<std::size_t>(bytes_transferred);
	}
}