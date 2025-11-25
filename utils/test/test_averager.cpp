#include <chrono>

#include <Pusher.h>
#include <RunnerSynchronous.h>
#include "Averager.h"

using namespace std::chrono_literals;

class Push1 : public Pusher<int> {
	int i = 0;

   public:
	Push1() { std::cout << "Push1()" << std::endl; }
	int push() final {
		return ++i;
	}
	int push_once() override {
		return i;
	}
};

class Run1 : public RunnerSynchronous<int> {
   public:
	Run1() { std::cout << "Run1()" << std::endl; }

	void run_once(int const& data) override { std::cout << " run_once(" << data << ")" << std::endl; }
	void run(int const& data) override { std::cout << " run(" << data << ")" << std::endl; }
};

class Averager1 : public Averager<int> {
   public:
	Averager1() : Averager(0) {}

   private:
	[[nodiscard]] int compute_add(int const& sum, int const& data) const override { return sum + data; }
	[[nodiscard]] int compute_average(int const& sum, std::size_t n) const override { return sum / n; }
};

int main() {
	Push1 push1;
	Averager1 averager1;
	Run1 run1;

	push1.synchronously_connect(averager1);
	averager1.synchronously_connect(run1);

	auto t1 = push1();
	auto t2 = averager1();

	std::this_thread::sleep_for(100ms);

	averager1.average();

	std::this_thread::sleep_for(100ms);

	averager1.average();
}