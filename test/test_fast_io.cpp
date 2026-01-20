#include <ATen/core/interned_strings.h>
#include <tbb/concurrent_queue.h>

#include <boost/asio/buffer.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/asio/serial_port_base.hpp>
#include <boost/crc.hpp>
#include <boost/system/detail/error_code.hpp>

#include "MagneticFluxDensityDataRawLIS3MDL.h"
#include "MagneticFluxDensityDataRawMMC5983MA.h"

// #define USE_BOOST

#ifdef USE_BOOST
#include <boost/asio.hpp>
#endif

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
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

class SerialConnection {
	enum class ConnectionState {
		NONE,
		CONNECTED,
		READING,
	};
	std::atomic<ConnectionState> state = ConnectionState::NONE;

	std::thread connection_thread;

#ifdef USE_BOOST
	boost::asio::io_context _context;
	boost::asio::serial_port _serial_port;
#else
	int _serial_port = -1;
#endif

   protected:
	std::deque<std::uint8_t> data;
	std::optional<ERR> error = std::nullopt;

   public:
#ifdef USE_BOOST
	explicit SerialConnection() : _serial_port(_context) { std::cout << "Connection()" << std::endl; }
#else
	explicit SerialConnection() { std::cout << "Connection()" << std::endl; }
#endif

	std::expected<void, ERR> open_serial_port(std::string const& device) {
#ifdef USE_BOOST
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
			common::println_error_loc(std::strerror(errno), " (", errno, ")");
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
				std::array<std::uint8_t, 512> data;
				while (state.load() == ConnectionState::READING) {
#ifdef USE_BOOST
					boost::system::error_code ec;
					if (std::size_t const bytes_transferred = _serial_port.read_some(boost::asio::buffer(data), ec); ec) {
						error.emplace(ERR{ec.value()});
						common::println_error_loc(ec.what(), " (", ec.value(), ")");
						break;
					} else if (bytes_transferred > 0) {
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

	void stop_reading() {
		if (auto expected = ConnectionState::READING; state.compare_exchange_strong(expected, ConnectionState::CONNECTED)) {
			// if (connection_thread.joinable()) {
			connection_thread.join();
			//}
		} else {
			common::println_error_loc("state is ", expected == ConnectionState::CONNECTED ? "CONNECTED" : expected == ConnectionState::NONE ? "NONE" : expected == ConnectionState::READING ? "READING" : "ERROR");
		}
	}

	virtual ~SerialConnection() {
		stop_reading();
		close_serial_port();
	}

	virtual void parse(std::span<std::uint8_t>&& data) = 0;
};

// Primary template: defaults to false
template <typename T>
struct is_SENSOR_TYPE : std::false_type {};

template <typename T>
struct start_index_of : std::integral_constant<std::size_t, 0> {};

template <typename T>
struct n_sensors_of : std::integral_constant<std::size_t, 0> {};

template <typename T>
struct type_of {};

template <typename MagDataType, std::size_t start_index, std::size_t n_sensors>
requires requires { MagDataType::bytes; } class SENSOR_TYPE {};

// Specialization for SENSOR_TYPE<...>
template <typename MagDataType, std::size_t start_index, std::size_t n_sensors>
struct is_SENSOR_TYPE<SENSOR_TYPE<MagDataType, start_index, n_sensors>> : std::true_type {};

template <typename MagDataType, std::size_t start_index, std::size_t n_sensors>
struct start_index_of<SENSOR_TYPE<MagDataType, start_index, n_sensors>> : std::integral_constant<std::size_t, start_index> {};

template <typename MagDataType, std::size_t start_index, std::size_t n_sensors>
struct n_sensors_of<SENSOR_TYPE<MagDataType, start_index, n_sensors>> : std::integral_constant<std::size_t, n_sensors> {};

template <typename MagDataType, std::size_t start_index, std::size_t n_sensors>
struct type_of<SENSOR_TYPE<MagDataType, start_index, n_sensors>> : std::type_identity<MagDataType> {};

template <typename... SENSOR_TYPEs>
requires(is_SENSOR_TYPE<SENSOR_TYPEs>::value && ...) class MiMedMagnetometerArraySerialConnectionBinary : virtual public SerialConnection {
   protected:
	static constexpr std::size_t total_size = (0 + ... + n_sensors_of<SENSOR_TYPEs>::value);
	static constexpr std::size_t magnetic_flux_density_message_size = 1 + ((4 + n_sensors_of<SENSOR_TYPEs>::value * sizeof(typename type_of<SENSOR_TYPEs>::type)) + ...) + sizeof(std::uint64_t) + 2 + 1;
	static constexpr std::size_t timestamp_message_size = 1 + 8 + 8 + 1 + 1;
	static constexpr std::size_t min_info_message_size = 1 + 0 + 1 + 1 + 1;
	static constexpr std::size_t max_info_message_size = 1 + 255 + 1 + 1 + 1;
	std::deque<std::uint8_t> buffer;

	std::size_t index_magnetic_flux_density_message = 0;
	std::size_t index_timestamp_message = 0;
	std::size_t index_info_message = 0;

	std::list<std::shared_ptr<Message<Array<MagneticFluxDensityData, total_size>>>> magnetic_flux_density_messages;

	std::uint64_t total_bytes_received = 0;
	std::uint64_t total_message_bytes = 0;
	std::uint64_t total_message_bytes2 = 0;
	std::chrono::time_point<std::chrono::system_clock> last_message;

	void parse_magnetic_flux_density_messages() {
		for (; index_magnetic_flux_density_message + magnetic_flux_density_message_size <= buffer.size(); ++index_magnetic_flux_density_message) {
			if (buffer[index_magnetic_flux_density_message] == 'M' && buffer[index_magnetic_flux_density_message + magnetic_flux_density_message_size - 1] == 'M') {
				boost::crc_16_type crc;
				for (auto j = 1; j <= magnetic_flux_density_message_size - 4; ++j) {
					crc.process_byte(buffer[index_magnetic_flux_density_message + j]);
				}

				if (std::uint8_t crc0 = crc.checksum() & 0xFF, crc1 = (crc.checksum() >> 8) & 0xFF;
				    crc0 == buffer[index_magnetic_flux_density_message + magnetic_flux_density_message_size - 3] && crc1 == buffer[index_magnetic_flux_density_message + magnetic_flux_density_message_size - 2]) {
					Message<Array<MagneticFluxDensityData, total_size>> out;
					out.src = "array";

					auto fill = [&]<typename T>() {
						auto const scale = std::bit_cast<std::uint32_t>(
						    std::array{buffer[++index_magnetic_flux_density_message], buffer[++index_magnetic_flux_density_message], buffer[++index_magnetic_flux_density_message], buffer[++index_magnetic_flux_density_message]});

						for (auto& e : out | std::ranges::views::drop(start_index_of<T>::value) | std::ranges::views::take(n_sensors_of<T>::value)) {
							typename type_of<T>::type mag_data;
							for (auto k = 0; k < sizeof(typename type_of<T>::type); ++k) {
								mag_data.bytes[k] = buffer[++index_magnetic_flux_density_message];
							}

							e.x = static_cast<double>(mag_data.x) / static_cast<double>(scale);
							e.y = static_cast<double>(mag_data.y) / static_cast<double>(scale);
							e.z = static_cast<double>(mag_data.z) / static_cast<double>(scale);
						}
					};

					(fill.template operator()<SENSOR_TYPEs>(), ...);

					out.timestamp = std::bit_cast<std::uint64_t>(
					    std::array{buffer[++index_magnetic_flux_density_message], buffer[++index_magnetic_flux_density_message], buffer[++index_magnetic_flux_density_message], buffer[++index_magnetic_flux_density_message],
					        buffer[++index_magnetic_flux_density_message], buffer[++index_magnetic_flux_density_message], buffer[++index_magnetic_flux_density_message], buffer[++index_magnetic_flux_density_message]});

					index_magnetic_flux_density_message += 3;
					magnetic_flux_density_messages.push_front(std::make_shared<Message<Array<MagneticFluxDensityData, total_size>>>(out));

					total_message_bytes2 += magnetic_flux_density_message_size;
					std::cout << static_cast<double>(total_message_bytes2) / static_cast<double>(total_bytes_received) << std::endl;
					auto const now = std::chrono::system_clock::now();
					std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(now - last_message) << std::endl << std::endl;
					last_message = now;
				}
			}
		}
	}

	void parse_info_messages() {
		int i = index_info_message + min_info_message_size - 1;
		for (; i < buffer.size(); ++i) {
			if (int const frame_end = i; buffer[frame_end] == 'I') {
				if (int const length = 1 + buffer[frame_end - 1] + 1 + 1 + 1, frame_start = i + 1 - length; frame_start >= 0 && buffer[frame_start] == 'I') {
					boost::crc_optimal<8, 0x07, 0x00, 0x00, false, false> crc;
					for (auto j = 1; j <= length - 4; ++j) {
						crc.process_byte(buffer[frame_start + j]);
					}

					if (std::uint8_t const crc0 = crc.checksum() & 0xFF; crc0 == buffer[frame_end - 2]) {
						std::string info_message(buffer.begin() + frame_start + 1, buffer.begin() + frame_end - 2);

						std::cout << i << " " << static_cast<char>(buffer[0]) << " " << static_cast<char>(buffer[1]) << info_message << std::endl;
						index_info_message = i + 1;
						i += min_info_message_size;

						total_message_bytes += length;
						std::cout << static_cast<double>(total_message_bytes) / static_cast<double>(total_bytes_received) << std::endl;
						auto const now = std::chrono::system_clock::now();
						std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(now - last_message) << std::endl << std::endl;
						last_message = now;
					}
				}
			}
		}

		index_info_message = std::max(index_info_message, i - 1 - max_info_message_size + min_info_message_size);
	}

	void parse(std::span<std::uint8_t>&& data) override {
		buffer.append_range(data);
		total_bytes_received += data.size();

		parse_magnetic_flux_density_messages();
		parse_info_messages();

		auto const remove = std::min(index_magnetic_flux_density_message, index_info_message);

		buffer.erase(buffer.begin(), buffer.begin() + remove);

		index_info_message -= remove;
		index_magnetic_flux_density_message -= remove;
	}
};

int main() {
	MiMedMagnetometerArraySerialConnectionBinary<SENSOR_TYPE<MagneticFluxDensityDataRawMMC5983MA, 0, 25>, SENSOR_TYPE<MagneticFluxDensityDataRawLIS3MDL, 25, 16>> conn;

	conn.open_serial_port("/dev/ttyUSB0");

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
