#pragma once

#include <ostream>
#include <tuple>

template <typename Type, typename... Types>
struct Box : public std::tuple<Type, Types...> {};

template <typename... Types>
std::ostream& operator<<(std::ostream& os, Box<Types...> const& msg) {
	static auto apply = []<std::size_t... I>(std::ostream& os, Box<Types...> const& msg, std::index_sequence<I...>) { ((os << std::get<I>(msg) << '\n'), ...); };

	apply(os, msg, std::index_sequence_for<Types...>{});
	return os;
}