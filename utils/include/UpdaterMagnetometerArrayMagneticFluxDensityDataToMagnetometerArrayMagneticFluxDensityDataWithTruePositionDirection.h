#pragma once

#include <Processor.h>
#include <common_output.h>

#include <numbers>
#include <semaphore>

#include "Array.h"
#include "Direction.h"
#include "DirectionVector.h"
#include "MagneticFluxDensityData.h"
#include "Message.h"
#include "Pack.h"
#include "Position.h"

template <std::size_t N>
class UpdaterMagnetometerArrayMagneticFluxDensityDataToMagnetometerArrayMagneticFluxDensityDataWithTruePositionDirection
    : public Processor<Message<Array<MagneticFluxDensityData, N>>, Pack<Message<Array<MagneticFluxDensityData, N>>, Position, Direction>> {
	Position position;
	Direction direction;
	std::binary_semaphore data_semaphore{1};

   public:
	UpdaterMagnetometerArrayMagneticFluxDensityDataToMagnetometerArrayMagneticFluxDensityDataWithTruePositionDirection(Position &&position, Direction &&direction)
	    : position(std::forward<decltype(position)>(position)), direction(std::forward<decltype(direction)>(direction)) {}

	void update(Position &&position_, Direction &&direction_) {
		data_semaphore.acquire();

		position = position_;
		direction = direction_;

		data_semaphore.release();
	}

   private:
	Pack<Message<Array<MagneticFluxDensityData, N>>, Position, Direction> process(Message<Array<MagneticFluxDensityData, N>> const &data) override {
		Pack<Message<Array<MagneticFluxDensityData, N>>, Position, Direction> out;

		static auto assign = []<typename Type>(Type *const out, Type const *const data) { *out = *data; };

		assign.template operator()<Message<Array<MagneticFluxDensityData, N>>>(&out, &data);

		data_semaphore.acquire();

		assign.template operator()<Position>(&out, &position);
		assign.template operator()<Direction>(&out, &direction);

		data_semaphore.release();

		return out;
	}
};

template <std::size_t N, std::size_t M = 1>
class UpdaterMagnetometerArrayMagneticFluxDensityDataToMagnetometerArrayMagneticFluxDensityDataWithTruePositionDirectionVector
    : public Processor<Message<Array<MagneticFluxDensityData, N>>, Pack<Message<Array<MagneticFluxDensityData, N>>, Array<Pack<Position, DirectionVector>, M>>> {
	std::array<Pack<Position, DirectionVector>, M> position_direction;
	std::binary_semaphore data_semaphore{1};

   public:
	UpdaterMagnetometerArrayMagneticFluxDensityDataToMagnetometerArrayMagneticFluxDensityDataWithTruePositionDirectionVector(std::array<Pack<Position, DirectionVector>, M> &&position_direction)
	    : position_direction(std::forward<decltype(position_direction)>(position_direction)) {}

	void update(std::array<Pack<Position, DirectionVector>, M> &&position_direction_) {
		data_semaphore.acquire();

		position_direction = position_direction_;

		data_semaphore.release();
	}

   private:
	Pack<Message<Array<MagneticFluxDensityData, N>>, Array<Pack<Position, DirectionVector>, M>> process(Message<Array<MagneticFluxDensityData, N>> const &data) override {
		Pack<Message<Array<MagneticFluxDensityData, N>>, Array<Pack<Position, DirectionVector>, M>> out;

		static auto assign = []<typename Type>(Type *const out, Type const *const data) { *out = *data; };

		assign.template operator()<Message<Array<MagneticFluxDensityData, N>>>(&out, &data);

		data_semaphore.acquire();

		assign.template operator()<std::array<Pack<Position, DirectionVector>, M>>(&out, &position_direction);

		data_semaphore.release();

		return out;
	}
};