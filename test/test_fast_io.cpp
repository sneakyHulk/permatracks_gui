#include <MagneticFluxDensityData.h>
#include <MagneticFluxDensityDataRawLIS3MDL.h>
#include <MagneticFluxDensityDataRawMMC5983MA.h>
#include <MiMedMagnetometerArraySerialConnectionBinary.h>
#include <SerialConnection.h>

#include <expected>

#include "MagneticFluxDensityDataRawAK09940A.h"

class DataPrinterV1 : public virtual SerialConnection, public MiMedMagnetometerArraySerialConnectionBinary<SENSOR_TYPE<MagneticFluxDensityDataRawLIS3MDL, 25, 16>, SENSOR_TYPE<MagneticFluxDensityDataRawMMC5983MA, 0, 25>> {
   private:
	void handle_parse_result(Message<Array<MagneticFluxDensityData, total_mag_sensors>>& magnetic_flux_density_message) override { std::cout << magnetic_flux_density_message << std::endl; }
};

class DataPrinterV2 : public virtual SerialConnection, public MiMedMagnetometerArraySerialConnectionBinary<SENSOR_TYPE<MagneticFluxDensityDataRawAK09940A, 0, 111>> {
   private:
	void handle_parse_result(Message<Array<MagneticFluxDensityData, total_mag_sensors>>& magnetic_flux_density_message) override { std::cout << magnetic_flux_density_message << std::endl; }
};

void setup() {}

int main() {
	DataPrinterV2 test;

	// if (auto const res = test.open_serial_port("/dev/cu.usbserial-0001"); !res.has_value()) return 1;
	if (auto const res = test.open_serial_port("/dev/cu.usbmodem3958386634341"); !res.has_value()) return 1;

	while (true) {
		if (auto serial_data = test.read_some(); serial_data.has_value()) {
			test.parse(serial_data.value());
		} else {
			test.close_serial_port();
			break;
		}
	}
}