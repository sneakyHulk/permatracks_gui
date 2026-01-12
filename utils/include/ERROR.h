#pragma once
#include <string>

struct ERR {
	std::string message;

	ERR(int code, std::string const& message);
	explicit ERR(int code);
	explicit ERR(std::string  message);

	~ERR() = default;

	char const* what() const noexcept;
};