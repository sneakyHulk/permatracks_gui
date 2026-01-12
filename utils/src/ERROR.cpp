#include "ERROR.h"

#include <utility>
ERR::ERR(int const code, std::string const& message) : message(message + " (" + std::to_string(code) + ")") {}
ERR::ERR(int const code) : message(std::to_string(code)) {}
ERR::ERR(std::string message) : message(std::move(message)) {}
char const* ERR::what() const noexcept { return message.c_str(); }