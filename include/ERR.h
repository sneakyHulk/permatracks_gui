#pragma once
#include <string>

struct ERR {
	std::string message;

	ERR(int const code, std::string const& message) : message(message + " (" + std::to_string(code) + ")") {}

	explicit ERR(int const code) : message(std::to_string(code)) {}

	explicit ERR(std::string const& message) : message(message) {}

	~ERR() = default;

	char const* what() const noexcept { return message.c_str(); }
};