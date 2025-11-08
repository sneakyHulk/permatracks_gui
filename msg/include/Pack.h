#pragma once

#include <ostream>
#include <tuple>

template <typename Type, typename... Types>
struct Pack : public Type, public Types... {};

template <typename... Types>
std::ostream& operator<<(std::ostream& os, Pack<Types...> const& msg) {

	char space[]{0, 0};
	(... << (os << space << *static_cast<Types const* const>(&msg), *space = ','));
	return os;
}