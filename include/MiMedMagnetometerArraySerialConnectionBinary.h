#pragma once

#include <boost/crc.hpp>
#include <iostream>
#include <list>
#include <memory>
#include <ranges>

#include "Array.h"
#include "ERROR.h"
#include "MagneticFluxDensityData.h"
#include "Message.h"
#include "SerialConnection.h"
#include "common_output.h"

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
	static constexpr std::size_t total_mag_sensors = (0 + ... + n_sensors_of<SENSOR_TYPEs>::value);
	static constexpr int magnetic_flux_density_message_size = 1 + ((4 + n_sensors_of<SENSOR_TYPEs>::value * sizeof(typename type_of<SENSOR_TYPEs>::type)) + ...) + sizeof(std::uint64_t) + 2 + 1;
	static constexpr int timestamp_message_size = 1 + 8 + 8 + 1 + 1;
	static constexpr int min_info_message_size = 1 + 0 + 1 + 1 + 1;
	static constexpr int max_info_message_size = 1 + 255 + 1 + 1 + 1;
	std::deque<std::uint8_t> buffer;

	std::size_t index_magnetic_flux_density_message = 0;
	std::size_t index_timestamp_message = 0;
	std::size_t index_info_message = 0;

	std::list<std::shared_ptr<Message<Array<MagneticFluxDensityData, total_mag_sensors>>>> magnetic_flux_density_messages;

#ifdef DEBUG
	std::uint64_t total_bytes_received = 0;
	std::uint64_t total_message_bytes_timestamp_message = 0;
	std::uint64_t total_message_bytes_magnetic_flux_density_message = 0;
	std::uint64_t total_message_bytes_info_message = 0;
	std::chrono::time_point<std::chrono::system_clock> last_message;
#endif

	void parse_latest_timestamp_message() {
		for (int i = buffer.size() - 1; i >= index_timestamp_message + timestamp_message_size - 1; --i) {
			if (int const frame_start = i + 1 - timestamp_message_size, frame_end = i; buffer[frame_end] == 'T' && buffer[frame_start] == 'T') {
				boost::crc_optimal<8, 0x07, 0x00, 0x00, false, false> crc;
				for (auto j = 1; j <= timestamp_message_size - 3; ++j) {
					crc.process_byte(buffer[frame_start + j]);
				}

				if (std::uint8_t const crc0 = crc.checksum() & 0xFF; crc0 == buffer[frame_end - 1]) {
					std::cout << "Attempting time synchronization..." << std::endl;
					std::uint64_t const t2 = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

					std::array<std::uint8_t, timestamp_message_size> message;

					message[0] = 'T';
					std::memcpy(message.data() + 1, &t_latest_message, sizeof(t_latest_message));
					std::memcpy(message.data() + 1 + sizeof(t_latest_message), &t2, sizeof(t2));
					crc.process_block(message.data() + 1, message.data() + 1 + sizeof(t_latest_message) + sizeof(t2));
					message[timestamp_message_size - 2] = crc.checksum();
					message[timestamp_message_size - 1] = 'T';

					if (auto const ret = write_all(message); !ret.has_value()) {
						std::cout << "Attempt failed." << std::endl;
					}

#ifdef DEBUG
					total_message_bytes_timestamp_message += timestamp_message_size;
					std::cout << static_cast<double>(total_message_bytes_timestamp_message) / static_cast<double>(total_bytes_received) << std::endl;
					auto const now = std::chrono::system_clock::now();
					std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(now - last_message) << std::endl << std::endl;
					last_message = now;
#endif

					break;  // latest timestamp parsed -> no need to parse another one
				}
			}
		}

		index_timestamp_message = buffer.size() - std::min(buffer.size(), static_cast<std::size_t>(timestamp_message_size - 1));
	}

	void parse_magnetic_flux_density_messages() {
		for (; index_magnetic_flux_density_message + magnetic_flux_density_message_size <= buffer.size(); ++index_magnetic_flux_density_message) {
			if (buffer[index_magnetic_flux_density_message] == 'M' && buffer[index_magnetic_flux_density_message + magnetic_flux_density_message_size - 1] == 'M') {
				boost::crc_16_type crc;
				for (auto j = 1; j <= magnetic_flux_density_message_size - 4; ++j) {
					crc.process_byte(buffer[index_magnetic_flux_density_message + j]);
				}

				if (std::uint8_t crc0 = crc.checksum() & 0xFF, crc1 = (crc.checksum() >> 8) & 0xFF;
				    crc0 == buffer[index_magnetic_flux_density_message + magnetic_flux_density_message_size - 3] && crc1 == buffer[index_magnetic_flux_density_message + magnetic_flux_density_message_size - 2]) {
					Message<Array<MagneticFluxDensityData, total_mag_sensors>> out;
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
					magnetic_flux_density_messages.push_front(std::make_shared<Message<Array<MagneticFluxDensityData, total_mag_sensors>>>(out));

#ifdef DEBUG
					total_message_bytes_magnetic_flux_density_message += magnetic_flux_density_message_size;
					std::cout << static_cast<double>(total_message_bytes_magnetic_flux_density_message) / static_cast<double>(total_bytes_received) << std::endl;
					auto const now = std::chrono::system_clock::now();
					std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(now - last_message) << std::endl << std::endl;
					last_message = now;
#endif
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

						std::cout << info_message << std::endl;
						index_info_message = i + 1;
						i += min_info_message_size;
#ifdef DEBUG
						total_message_bytes_info_message += length;
						std::cout << static_cast<double>(total_message_bytes_info_message) / static_cast<double>(total_bytes_received) << std::endl;
						auto const now = std::chrono::system_clock::now();
						std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(now - last_message) << std::endl << std::endl;
						last_message = now;
#endif
					}
				}
			}
		}
		if (int const tmp = i - 1 - max_info_message_size + min_info_message_size; tmp > 0) index_info_message = std::max(index_info_message, static_cast<std::size_t>(tmp));
	}

	void parse(std::span<std::uint8_t>&& data) override {
#ifdef DEBUG
		total_bytes_received += data.size();
#endif

		buffer.append_range(data);

		parse_latest_timestamp_message();
		parse_magnetic_flux_density_messages();
		parse_info_messages();

		auto const remove = std::min({index_timestamp_message, index_magnetic_flux_density_message, index_info_message});

		buffer.erase(buffer.begin(), buffer.begin() + remove);

		index_timestamp_message -= remove;
		index_magnetic_flux_density_message -= remove;
		index_info_message -= remove;
	}

   public:
	~MiMedMagnetometerArraySerialConnectionBinary() { std::cout << "~MiMedMagnetometerArraySerialConnectionBinary()" << std::endl; }
};