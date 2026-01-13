#include "SerialConnectionBoost.h"

using namespace std::chrono_literals;

SerialConnectionBoost::SerialConnectionBoost() : _serial_port(io) {}
std::expected<void, ERR> SerialConnectionBoost::open_serial_port(std::string const& device) {
	boost::system::error_code ec;

	_serial_port.open(device, ec);
	if (ec) {
		return std::unexpected(ERR{ec.value(), ec.message()});
	}

	_connected.store(true);
	return {};
}

void SerialConnectionBoost::close_serial_port() {
	if (bool expected = true; _connected.compare_exchange_strong(expected, false)) {
		boost::system::error_code ec;
		_serial_port.cancel(ec);
		_serial_port.close(ec);
	}
}
std::expected<std::span<const char>, ERR> SerialConnectionBoost::read_some() {
	static std::array<char, 128> tmp;

	boost::system::error_code ec;
	std::size_t const n = _serial_port.read_some(boost::asio::buffer(tmp.data(), tmp.size()), ec);

	if (ec == boost::asio::error::would_block || ec == boost::asio::error::try_again) {
		return std::span<const char>{tmp.data(), 0};
	}

	if (ec) {
		return std::unexpected(ERR{ec.value(), ec.message()});
	}

	return std::span<const char>{tmp.data(), n};
}

std::expected<std::size_t, ERR> SerialConnectionBoost::read_some(std::span<std::uint8_t> buffer) {
	boost::system::error_code ec;
	std::size_t n = _serial_port.read_some(boost::asio::buffer(buffer.data(), buffer.size()), ec);

	if (ec == boost::asio::error::would_block || ec == boost::asio::error::try_again) {
		return 0;
	}

	if (ec) {
		return std::unexpected(ERR{ec.value(), ec.message()});
	}

	return n;
}

std::expected<void, ERR> SerialConnectionBoost::write_all(std::span<std::uint8_t const> const buffer) {
	boost::system::error_code ec;

	boost::asio::write(_serial_port, boost::asio::buffer(buffer.data(), buffer.size()), ec);

	if (ec) {
		return std::unexpected(ERR{ec.value(), ec.message()});
	}

	return {};
}

Baudrate& SerialConnectionBoost::baud() { return _baud; }
[[nodiscard]] bool SerialConnectionBoost::connected() const { return _connected.load(); }