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
	static constexpr std::size_t timestamp_message_size = 1 + 8 + 8 + 1 + 1;
	static constexpr std::size_t min_info_message_size = 1 + 0 + 1 + 1 + 1;
	static constexpr std::size_t max_info_message_size = 1 + 255 + 1 + 1 + 1;

   public:
	typedef std::expected<Message<Array<MagneticFluxDensityData, total_size>>, ERR> Output;
	typedef std::expected<Message<Array<MagneticFluxDensityData, total_size>>, ERR> MagOutput;
	static constexpr std::size_t OutputSize = total_size;

	std::uint64_t total_bytes_received = 0;
	std::uint64_t total_message_bytes = 0;
	std::chrono::time_point<std::chrono::system_clock> last_message;

	std::expected<Message<Array<MagneticFluxDensityData, total_size>>, ERR> push(std::function<bool()> const& running = []() { return true; }) {
		static std::deque<std::uint8_t> buffer1;
		static std::deque<std::uint8_t> buffer2;
		static std::deque<std::uint8_t> buffer3;

		Message<Array<MagneticFluxDensityData, total_size>> out;
		out.src = "array";

		while (running()) {
			std::array<std::uint8_t, 256> tmp;
			std::uint64_t t1;
			if (auto const bytes_transferred = SerialConnection::read_some(tmp); bytes_transferred.has_value()) {
				t1 = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

				total_bytes_received += bytes_transferred.value();
				buffer1.insert(buffer1.end(), tmp.begin(), tmp.begin() + bytes_transferred.value());
				buffer2.insert(buffer2.end(), tmp.begin(), tmp.begin() + bytes_transferred.value());
				buffer3.insert(buffer3.end(), tmp.begin(), tmp.begin() + bytes_transferred.value());
			} else {
				return std::unexpected(bytes_transferred.error());
			}

			for (int i = buffer1.size() - 1; i >= timestamp_message_size - 1; --i) {  // parsing only latest timestamp message
				if (int const frame_start = i + 1 - timestamp_message_size, frame_end = i; buffer1[frame_end] == 'T' && buffer1[frame_start] == 'T') {
					boost::crc_optimal<8, 0x07, 0x00, 0x00, false, false> crc;
					for (auto j = 1; j <= timestamp_message_size - 3; ++j) {
						crc.process_byte(buffer1[frame_start + j]);
					}

					if (std::uint8_t const crc0 = crc.checksum() & 0xFF; crc0 == buffer1[frame_end - 1]) {
						//std::cout << "Attempting time synchronization..." << std::endl;
						std::uint64_t const t2 = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

						std::array<std::uint8_t, timestamp_message_size> message;

						message[0] = 'T';
						std::memcpy(message.data() + 1, &t1, sizeof(t1));
						std::memcpy(message.data() + 1 + sizeof(t1), &t2, sizeof(t2));
						crc.process_block(message.data() + 1, message.data() + 1 + sizeof(t1) + sizeof(t2));
						message[timestamp_message_size - 2] = crc.checksum();
						message[timestamp_message_size - 1] = 'T';

						if (auto ret = write_all(message); !ret.has_value()) {
							std::cout << "Attempt failed." << std::endl;
						}

						break;  // latest timestamp parsed -> no need to parse another one
					}
				}
			}
			if (buffer1.size() > timestamp_message_size) buffer1.erase(buffer1.begin(), buffer1.end() - timestamp_message_size);

			for (int i = 0; i + magnetic_flux_density_message_size <= buffer2.size(); ++i) {
				if (buffer2[i] == 'M' && buffer2[i + magnetic_flux_density_message_size - 1] == 'M') {
					boost::crc_16_type crc;
					for (auto j = 1; j <= magnetic_flux_density_message_size - 4; ++j) {
						crc.process_byte(buffer2[i + j]);
					}

					if (std::uint8_t crc0 = crc.checksum() & 0xFF, crc1 = (crc.checksum() >> 8) & 0xFF; crc0 == buffer2[i + magnetic_flux_density_message_size - 3] && crc1 == buffer2[i + magnetic_flux_density_message_size - 2]) {
						auto j = i;
						auto fill = [&]<typename T>() {
							auto const scale = std::bit_cast<std::uint32_t>(std::array{buffer2[++j], buffer2[++j], buffer2[++j], buffer2[++j]});

							for (auto& e : out | std::ranges::views::drop(start_index_of<T>::value) | std::ranges::views::take(n_sensors_of<T>::value)) {
								typename type_of<T>::type mag_data;
								for (auto k = 0; k < sizeof(typename type_of<T>::type); ++k) {
									mag_data.bytes[k] = buffer2[++j];
								}

								e.x = static_cast<double>(mag_data.x) / static_cast<double>(scale);
								e.y = static_cast<double>(mag_data.y) / static_cast<double>(scale);
								e.z = static_cast<double>(mag_data.z) / static_cast<double>(scale);
							}
						};

						(fill.template operator()<SENSOR_TYPEs>(), ...);

						out.timestamp = std::bit_cast<std::uint64_t>(std::array{buffer2[++j], buffer2[++j], buffer2[++j], buffer2[++j], buffer2[++j], buffer2[++j], buffer2[++j], buffer2[++j]});

						buffer2.erase(buffer2.begin(), buffer2.begin() + i + magnetic_flux_density_message_size);

						// info
						total_message_bytes += magnetic_flux_density_message_size;
						std::cout << static_cast<double>(total_message_bytes) / static_cast<double>(total_bytes_received) << std::endl;
						auto const now = std::chrono::system_clock::now();
						std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(now - last_message) << std::endl << std::endl;
						last_message = now;

						return out;
					}
				}
			}
			if (buffer2.size() > magnetic_flux_density_message_size) buffer2.erase(buffer2.begin(), buffer2.end() - magnetic_flux_density_message_size);

			for (int i = min_info_message_size; i < buffer3.size(); ++i) {
				if (int const frame_end = i; buffer3[frame_end] == 'I') {
					if (int const length = 1 + buffer3[frame_end - 1] + 1 + 1 + 1, frame_start = i + 1 - length; frame_start >= 0 && buffer3[frame_start] == 'I') {
						boost::crc_optimal<8, 0x07, 0x00, 0x00, false, false> crc;
						for (auto j = 1; j <= length - 4; ++j) {
							crc.process_byte(buffer3[frame_start + j]);
						}

						if (std::uint8_t const crc0 = crc.checksum() & 0xFF; crc0 == buffer3[frame_end - 2]) {
							std::string info_message(buffer3.begin() + frame_start + 1, buffer3.begin() + frame_end - 2);

							std::cout << info_message << std::endl;

							buffer3.erase(buffer3.begin(), buffer3.begin() + frame_end);
							i = min_info_message_size - 1;
						}
					}
				}
			}
			if (buffer3.size() > max_info_message_size) buffer3.erase(buffer3.begin(), buffer3.end() - max_info_message_size);
		}

		return {};
	}
};