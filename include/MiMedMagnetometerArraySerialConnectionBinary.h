#pragma once

#include <ring_buffer.h>

#include <array>
#include <boost/crc.hpp>
#include <chrono>
#include <cstdint>
#include <ranges>

#include "Array.h"
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
requires(is_SENSOR_TYPE<SENSOR_TYPEs>::value && ...) class MiMedMagnetometerArraySerialConnectionBinary {
   protected:
	static constexpr std::size_t total_size = (0 + ... + n_sensors_of<SENSOR_TYPEs>::value);
	static constexpr std::size_t magnetic_flux_density_message_size = 1 + ((4 + n_sensors_of<SENSOR_TYPEs>::value * sizeof(typename type_of<SENSOR_TYPEs>::type)) + ...) + sizeof(std::uint64_t) + 2 + 1;
	static constexpr std::size_t max_message_size = magnetic_flux_density_message_size;
	static constexpr std::size_t min_message_size = magnetic_flux_density_message_size;

	SerialConnection& connection;

   public:
	explicit MiMedMagnetometerArraySerialConnectionBinary(SerialConnection& connection) : connection(connection) {}
	std::expected<Message<Array<MagneticFluxDensityData, total_size>>, SerialError> push() {
		static ring_buffer<std::uint8_t, 2 * max_message_size +1> buffer;

		Message<Array<MagneticFluxDensityData, total_size>> out;
		out.src = "array";

		while (true) {
			do {
				std::span<std::uint8_t> const t = buffer.linear_sub_array();
				if (auto const bytes_transferred = connection.read_some(t); bytes_transferred.has_value()) {
					buffer.rotate(bytes_transferred.value());
				} else {
					return std::unexpected(bytes_transferred.error());
				}
			} while (buffer.size() < min_message_size);

			for (auto i = 0; i < buffer.size(); ++i) {
				if (buffer[i] == 'M') {
					if (buffer.size() < i + magnetic_flux_density_message_size || buffer[i + magnetic_flux_density_message_size - 1] != 'M') continue;

					boost::crc_16_type crc;
					for (auto j = 1; j <= magnetic_flux_density_message_size - 4; ++j) {
						crc.process_byte(buffer[i + j]);
					}

					if (std::uint8_t crc0 = crc.checksum() & 0xFF, crc1 = (crc.checksum() >> 8) & 0xFF; crc0 == buffer[i + magnetic_flux_density_message_size - 3] && crc1 == buffer[i + magnetic_flux_density_message_size - 2]) {
						auto j = i;
						auto fill = [&]<typename T>() {
							auto scale = std::bit_cast<std::uint32_t>(std::array{buffer[++j], buffer[++j], buffer[++j], buffer[++j]});

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

						buffer.pop(i + magnetic_flux_density_message_size);
						return out;
					}
				}
			}
		}
	}
};