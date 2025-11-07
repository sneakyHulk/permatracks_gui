#pragma once
#include <array>
#include <cassert>
#include <span>
#include <utility>

template <typename T, std::size_t N>
class ring_buffer : protected std::array<T, N> {
   public:
	template <std::size_t max_sub_array_size = N>
	constexpr std::span<T> linear_sub_array() requires(max_sub_array_size <= N) {
		return {std::array<T, N>::begin() + index, std::array<T, N>::begin() + std::min(index + max_sub_array_size, N)};
	}
	constexpr std::span<T> linear_sub_array() {
		auto* base_ptr = static_cast<std::array<T, N>*>(this);
		return { base_ptr->begin() + index, base_ptr->end() };
		//return {std::array<T, N>::begin() + index, std::array<T, N>::end()};
	}
	constexpr void rotate(std::size_t const n) {
		index = (index + n) % N;
		if (n > pops)
			pops = 0;
		else
			pops -= n;
	}
	constexpr void push_back(T const value) {
		std::array<T, N>::operator[](index) = value;
		index = (index + 1) % N;
		if (pops) --pops;
	}

	[[nodiscard]] constexpr std::size_t size() const { return N - pops; }

	constexpr ring_buffer() : std::array<T, N>() {}
	explicit constexpr ring_buffer(T&& value)
	    : std::array<T, N>([]<std::size_t... I>(T&& value, std::index_sequence<I...>) { return std::array{(static_cast<void>(I), value)...}; }(std::forward<decltype(value)>(value), std::make_index_sequence<N>{})) {}

	constexpr T const& back() const { return operator[](size() - 1); }
	constexpr T& back() { return operator[](size() - 1); }

	constexpr T const& front() const { return operator[](0); }
	constexpr T& front() { return operator[](0); }

	constexpr T const& operator[](std::size_t const n) const {
		assert(n < size());  // -1 will produce std::size_t max value
		return std::array<T, N>::operator[]((index + pops + n) % N);
	}
	constexpr T& operator[](std::size_t const n) {
		assert(n < size());  // -1 will produce std::size_t max value
		return std::array<T, N>::operator[]((index + pops + n) % N);
	}

	constexpr void pop() {
		++pops;
		assert(pops <= N);
	}
	constexpr void pop(std::size_t const n) {
		pops += n;
		assert(pops <= N);
	}

   private:
	std::size_t index = 0;
	std::size_t pops = N;
};