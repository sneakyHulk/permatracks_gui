#pragma once

#include <common_output.h>

#include <ranges>

#include "Array.h"
#include "Averager.h"
#include "MagneticFluxDensityData.h"
#include "Message.h"

template <std::size_t N>
class MagnetometerArrayMagneticFluxDensityDataAverager : public Averager<Message<Array<MagneticFluxDensityData, N> > > {
   public:
	MagnetometerArrayMagneticFluxDensityDataAverager() : Averager<Message<Array<MagneticFluxDensityData, N> > >(Message<Array<MagneticFluxDensityData, N> >{}) {}

   private:
	[[nodiscard]] Message<Array<MagneticFluxDensityData, N> > compute_add(Message<Array<MagneticFluxDensityData, N> > const &sum, Message<Array<MagneticFluxDensityData, N> > const &data) const override {
		Message<Array<MagneticFluxDensityData, N> > out;

		for (auto const &[out, sum, data] : std::ranges::views::zip(out, sum, data)) {
			out.x = sum.x + data.x;
			out.y = sum.y + data.y;
			out.z = sum.z + data.z;
		}

		out.timestamp = data.timestamp;
		out.src = data.src;

		return out;
	}

	[[nodiscard]] Message<Array<MagneticFluxDensityData, N> > compute_average(Message<Array<MagneticFluxDensityData, N> > const &sum, std::size_t n) const override {
		Message<Array<MagneticFluxDensityData, N> > out = sum;

		for (auto const &[out, sum] : std::ranges::views::zip(out, sum)) {
			out.x = sum.x / static_cast<double>(n);
			out.y = sum.y / static_cast<double>(n);
			out.z = sum.z / static_cast<double>(n);
		}

		common::println_loc("Avereage consists of ", n, " samples.");

		out.src = sum.src + "_mean";

		return out;
	}
};