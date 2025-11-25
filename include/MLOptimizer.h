#pragma once
#include <MagnetSelection.h>
#include <torch/script.h>

#include <filesystem>

#include "Array.h"
#include "DirectionVector.h"
#include "MagnetSelection.h"
#include "MagneticFluxDensityData.h"
#include "Message.h"
#include "Pack.h"
#include "Position.h"

template <std::size_t N>
class MLOptimizer : virtual protected MagnetSelection {
	torch::jit::script::Module model;

   public:
	MLOptimizer() = default;

	void set_model(std::filesystem::path&& model_path) { model = torch::jit::load(model_path.c_str(), torch::kMPS); }

	Message<Pack<Position, DirectionVector>> process(Message<Array<MagneticFluxDensityData, N>> const& data) {
		std::array<float, N * 3> tensor_data;
		for (auto i = 0; auto const& [x, y, z] : data) {
			tensor_data[i++] = x * 1e6;
			tensor_data[i++] = y * 1e6;
			tensor_data[i++] = z * 1e6;
		}

		torch::Tensor input = torch::from_blob(tensor_data.data(), {1, N * 3}, torch::TensorOptions().dtype(torch::kFloat32)).to(torch::kMPS);

		auto const output = model.forward({input});
		if (output.isTuple()) {
			auto const output_tuple = output.toTuple();

			auto const position = output_tuple->elements().at(0).toTensor();
			auto const direction = output_tuple->elements().at(1).toTensor();

			Message<Pack<Position, DirectionVector>> out{};
			out.timestamp = data.timestamp;
			out.src = data.src;
			out.x = position[0][0].template item<float>() / 1e3;
			out.y = position[0][1].template item<float>() / 1e3;
			out.z = position[0][2].template item<float>() / 1e3;

			out.mx = direction[0][0].template item<float>();
			out.my = direction[0][1].template item<float>();
			out.mz = direction[0][2].template item<float>();

			return out;
		}
		if (output.isTensor()) {
			auto const& position = output.toTensor();

			Message<Pack<Position, DirectionVector>> out{};
			out.timestamp = data.timestamp;
			out.src = data.src;
			out.x = position[0][0].template item<float>() / 1e3;
			out.y = position[0][1].template item<float>() / 1e3;
			out.z = position[0][2].template item<float>() / 1e3;

			return out;
		}

		return {};
	}
};