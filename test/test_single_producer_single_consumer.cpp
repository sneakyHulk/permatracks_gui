
#include <atomic>
#include <chrono>
#include <iostream>
#include <memory>
#include <thread>

using namespace std::chrono_literals;
std::atomic_bool running = true;
struct Node {
	std::chrono::time_point<std::chrono::steady_clock> t;
	std::shared_ptr<Node> next;
	explicit Node(std::shared_ptr<Node> const& next) : t(std::chrono::steady_clock::now()), next(next) {}
};

std::shared_ptr<Node> head = nullptr;
void producer() {
	while (running) {
		std::atomic_store_explicit(&head, std::make_shared<Node>(head), std::memory_order_release);

		// When not restricting this will result in SIGSEGV, because of stack overflow because of the recursive ~Node()
		std::this_thread::sleep_for(std::chrono::nanoseconds(1));
	}
}

void consumer() {
	while (running) {
		auto current_head = std::atomic_load_explicit(&head, std::memory_order_acquire);
		auto current_time = std::chrono::steady_clock::now();

		for (; current_head; current_head = current_head->next) {
			if (current_head->t + 25ms > current_time) {
				std::cout << current_head->t.time_since_epoch().count() << ", " << std::flush;
			} else {
				std::cout << std::endl;
				current_head->next = nullptr;
				break;
			}
		}
	}
}

int main() {
	std::thread producer_thread(producer);
	std::thread consumer_thread(consumer);

	std::this_thread::sleep_for(std::chrono::seconds(5));

	std::cout << "---- STOP ----" << std::endl;
	running = false;

	consumer_thread.join();
	producer_thread.join();
}