#include <windows.h>
#include "SerialConnection.h"

static ERR win_error() {
	DWORD err = GetLastError();
	char buf[256];
	FormatMessageA(FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS, nullptr, err, MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), buf, sizeof(buf), nullptr);
	return ERR{static_cast<int>(err), buf};
}

SerialConnection::SerialConnection() { std::cout << "SerialConnection()" << std::endl; }
std::expected<void, ERR> SerialConnection::open_serial_port(std::string const& device) {
	// Open the serial port
	_serial_port = CreateFileA(device.c_str(), GENERIC_READ | GENERIC_WRITE,
	    0,  // No sharing
	    nullptr, OPEN_EXISTING,
	    0,  // Non-overlapped I/O
	    nullptr);

	if (_serial_port == INVALID_HANDLE_VALUE) {
		return std::unexpected(win_error());
	}

	// Configure device buffer sizes
	if (!SetupComm(_serial_port, 4096, 4096)) {
		CloseHandle(_serial_port);
		_serial_port = INVALID_HANDLE_VALUE;
		return std::unexpected(win_error());
	}

	// Configure timeouts for non-blocking read behavior
	COMMTIMEOUTS timeouts = {};
	timeouts.ReadIntervalTimeout = MAXDWORD;
	timeouts.ReadTotalTimeoutMultiplier = 0;
	timeouts.ReadTotalTimeoutConstant = 0;
	timeouts.WriteTotalTimeoutMultiplier = 0;
	timeouts.WriteTotalTimeoutConstant = 0;

	if (!SetCommTimeouts(_serial_port, &timeouts)) {
		CloseHandle(_serial_port);
		_serial_port = INVALID_HANDLE_VALUE;
		return std::unexpected(win_error());
	}

	// Configure serial port parameters
	DCB dcb = {};
	dcb.DCBlength = sizeof(DCB);

	if (!GetCommState(_serial_port, &dcb)) {
		CloseHandle(_serial_port);
		_serial_port = INVALID_HANDLE_VALUE;
		return std::unexpected(win_error());
	}

	dcb.BaudRate = _baud;
	dcb.ByteSize = 8;
	dcb.StopBits = ONESTOPBIT;
	dcb.Parity = NOPARITY;
	dcb.fBinary = TRUE;
	dcb.fParity = FALSE;
	dcb.fOutxCtsFlow = FALSE;
	dcb.fOutxDsrFlow = FALSE;
	dcb.fDtrControl = DTR_CONTROL_DISABLE;
	dcb.fDsrSensitivity = FALSE;
	dcb.fTXContinueOnXoff = FALSE;
	dcb.fOutX = FALSE;
	dcb.fInX = FALSE;
	dcb.fErrorChar = FALSE;
	dcb.fNull = FALSE;
	dcb.fRtsControl = RTS_CONTROL_DISABLE;
	dcb.fAbortOnError = FALSE;

	if (!SetCommState(_serial_port, &dcb)) {
		CloseHandle(_serial_port);
		_serial_port = INVALID_HANDLE_VALUE;
		return std::unexpected(win_error());
	}

	// Purge any existing data
	PurgeComm(_serial_port, PURGE_RXCLEAR | PURGE_TXCLEAR | PURGE_RXABORT | PURGE_TXABORT);

	_connected = true;
	return {};
}

void SerialConnection::close_serial_port() {
	if (bool expected = true; _connected.compare_exchange_strong(expected, false)) {
		if (_serial_port != INVALID_HANDLE_VALUE) {
			CloseHandle(_serial_port);
			_serial_port = INVALID_HANDLE_VALUE;
		}
	}
}

[[nodiscard]] std::expected<std::span<const char>, ERR> SerialConnection::read_some() const {
	static std::array<char, 128> tmp;
	DWORD bytes_read = 0;

	if (!ReadFile(_serial_port, tmp.data(), static_cast<DWORD>(tmp.size()), &bytes_read, nullptr)) {
		return std::unexpected(win_error());
	}

	return std::span<const char>{tmp.begin(), static_cast<std::size_t>(bytes_read)};
}

[[nodiscard]] std::expected<std::size_t, ERR> SerialConnection::read_some(std::span<std::uint8_t> buffer) const {
	DWORD bytes_read = 0;

	if (!ReadFile(_serial_port, buffer.data(), static_cast<DWORD>(buffer.size()), &bytes_read, nullptr)) {
		return std::unexpected(win_error());
	}

	return static_cast<std::size_t>(bytes_read);
}

Baudrate& SerialConnection::baud() { return _baud; }
[[nodiscard]] bool SerialConnection::connected() const { return _connected.load(); }