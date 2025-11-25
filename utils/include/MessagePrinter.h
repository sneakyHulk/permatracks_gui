#pragma once
#include <iostream>

#include "Message.h"
#include "RunnerSynchronous.h"

template <typename T>
class MessagePrinter : public RunnerSynchronous<Message<T>> {
   public:
	MessagePrinter() = default;
	void run(Message<T> const& data) override { std::cout << data << std::endl; }
};