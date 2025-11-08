#pragma once

#include <cstdint>
#include <ostream>

template <typename Type>
struct Message : public Type {
	std::uint64_t timestamp;
	std::string src;

	friend std::ostream& operator<<(std::ostream& os, Message<Type> const& msg) {
		os << msg.timestamp << "," << static_cast<Type const&>(msg);

		return os;
	}
};

// template <typename Type>
// std::ostream& operator<<(std::ostream& os, Message<Type> const& msg);