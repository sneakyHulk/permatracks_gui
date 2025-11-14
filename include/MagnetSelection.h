#pragma once
#include <iostream>
#include <vector>

#include "Magnet.h"

class MagnetSelection {
   protected:
	std::vector<Magnet> _magnets;
	// std::vector<MagnetConstraints> constraints;

   public:
	MagnetSelection() { std::cout << "MagnetSelection()" << std::endl; }
	~MagnetSelection() = default;

	void add_magnet(Magnet const& magnet) { _magnets.push_back(magnet); }

	bool magnets_selected() const { return !_magnets.empty(); }
};