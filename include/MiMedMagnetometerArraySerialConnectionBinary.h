#pragma once

#include <common_literals.h>
#include <ring_buffer.h>
#include <tbb/concurrent_queue.h>

#include <array>
#include <boost/crc.hpp>
#include <chrono>
#include <cstdint>
#include <deque>
#include <ranges>
#include <thread>

#include "Array.h"
#include "ERROR.h"
#include "MagneticFluxDensityData.h"
#include "Message.h"
#include "SerialConnection.h"

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
requires(is_SENSOR_TYPE<SENSOR_TYPEs>::value && ...) class MiMedMagnetometerArraySerialConnectionBinary : virtual protected SerialConnection {
   protected:
	static constexpr std::size_t total_size = (0 + ... + n_sensors_of<SENSOR_TYPEs>::value);
	static constexpr std::size_t magnetic_flux_density_message_size = 1 + ((4 + n_sensors_of<SENSOR_TYPEs>::value * sizeof(typename type_of<SENSOR_TYPEs>::type)) + ...) + sizeof(std::uint64_t) + 2 + 1;

	static constexpr std::size_t min_timestamp_message_size = 1 + 8 + 8 + 1 + 1;
	static constexpr std::size_t max_timestamp_message_size = 1 + 8 + 8 + 1 + 1;

	static constexpr std::size_t n_messages = 2;

   public:
	typedef std::expected<Message<Array<MagneticFluxDensityData, total_size>>, ERR> Output;
	typedef std::expected<Message<Array<MagneticFluxDensityData, total_size>>, ERR> MagOutput;
	static constexpr std::size_t OutputSize = total_size;

	std::uint64_t total_bytes_received = 0;
	std::uint64_t total_message_bytes = 0;
	std::chrono::time_point<std::chrono::system_clock> last_message;

	std::expected<Message<Array<MagneticFluxDensityData, total_size>>, ERR> push(std::function<bool()> const& running = []() { return true; }) {
		static std::deque<std::uint8_t> buffer;

		Message<Array<MagneticFluxDensityData, total_size>> out;
		out.src = "array";

		while (running()) {
			do {
				std::array<std::uint8_t, 256> message;
				if (auto const bytes_transferred = SerialConnection::read_some(message); bytes_transferred.has_value()) {
					total_bytes_received += bytes_transferred.value();
					buffer.insert(buffer.end(), message.begin(), message.begin() + bytes_transferred.value());
					break;
				} else {
					return std::unexpected(bytes_transferred.error());
				}
			} while (true);

			for (int i = 0; i < buffer.size(); ++i) {
				auto query_size = 0;
				auto query_letter = 0;

				query_letter += buffer[i] == 'T' ? 1_u8 : 0_u8;
				query_size += i + min_timestamp_message_size <= buffer.size() ? 1_u8 : 0_u8;

				query_letter += buffer[i] == 'M' ? 2_u8 : 0_u8;
				query_size += i + magnetic_flux_density_message_size <= buffer.size() ? 2_u8 : 0_u8;

				if (!query_size) {
					break;
				}

				if (!query_letter) {
					buffer.pop_front();
					--i;

					continue;
				}

				if (auto const check = query_letter & query_size) {
					if (check & 1) {
					} else if (check & 2) {
						boost::crc_16_type crc;
						for (auto j = 1; j <= magnetic_flux_density_message_size - 4; ++j) {
							crc.process_byte(buffer[i + j]);
						}
						if (std::uint8_t crc0 = crc.checksum() & 0xFF, crc1 = (crc.checksum() >> 8) & 0xFF; crc0 == buffer[i + magnetic_flux_density_message_size - 3] && crc1 == buffer[i + magnetic_flux_density_message_size - 2]) {
							auto j = i;
							auto fill = [&]<typename T>() {
								auto const scale = std::bit_cast<std::uint32_t>(std::array{buffer[++j], buffer[++j], buffer[++j], buffer[++j]});

								for (auto& e : out | std::ranges::views::drop(start_index_of<T>::value) | std::ranges::views::take(n_sensors_of<T>::value)) {
									typename type_of<T>::type mag_data;
									for (auto k = 0; k < sizeof(typename type_of<T>::type); ++k) {
										mag_data.bytes[k] = buffer[++j];
									}

									e.x = static_cast<double>(mag_data.x) / static_cast<double>(scale);
									e.y = static_cast<double>(mag_data.y) / static_cast<double>(scale);
									e.z = static_cast<double>(mag_data.z) / static_cast<double>(scale);
								}
							};

							(fill.template operator()<SENSOR_TYPEs>(), ...);

							out.timestamp = std::bit_cast<std::uint64_t>(std::array{buffer[++j], buffer[++j], buffer[++j], buffer[++j], buffer[++j], buffer[++j], buffer[++j], buffer[++j]});

							// buffer.pop(i + magnetic_flux_density_message_size);
							buffer.erase(buffer.begin(), buffer.begin() + i + magnetic_flux_density_message_size);
							total_message_bytes += magnetic_flux_density_message_size;
							std::cout << static_cast<double>(total_message_bytes) / static_cast<double>(total_bytes_received) << std::endl;
							auto const now = std::chrono::system_clock::now();
							std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(now - last_message) << std::endl << std::endl;
							last_message = now;
							return out;
						}
					}

					buffer.pop_front();
					--i;

					continue;
				}
			}
		}

		return {};
	}
};