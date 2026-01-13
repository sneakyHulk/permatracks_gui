#pragma once

#include <boost/asio.hpp>
#include <cstring>
#include <expected>
#include <iostream>
#include <span>
#include <thread>

#include "ERROR.h"

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

class SerialConnectionBoost {
   protected:
	Baudrate _baud = Baudrate::BAUD230400;

	boost::asio::io_context io;
	boost::asio::serial_port _serial_port;
	std::thread _io_thread;

	std::atomic_bool _connected = false;

	SerialConnectionBoost();

   public:
	std::expected<void, ERR> open_serial_port(std::string const& device);

	void close_serial_port();
	[[nodiscard]] std::expected<std::span<const char>, ERR> read_some();
	[[nodiscard]] std::expected<std::size_t, ERR> read_some(std::span<std::uint8_t> buffer);
	std::expected<void, ERR> write_all(std::span<std::uint8_t const> buffer);

	Baudrate& baud();
	[[nodiscard]] bool connected() const;
};