#pragma once

#include <atomic>
#include <semaphore>

#include "Pusher.h"
#include "RunnerSynchronous.h"

template <typename T>
class Averager : public Pusher<T>, public RunnerSynchronous<T> {
	T sum;
	T start;
	std::size_t n = 0;
	std::binary_semaphore do_average_semaphore{0};
	std::binary_semaphore data_semaphore{1};

   public:
	Averager(T&& start) : sum(std::forward<decltype(start)>(start)), start(std::forward<decltype(start)>(start)) {}

	void average() { do_average_semaphore.release(); }

   protected:
	virtual T compute_average(T const& sum, std::size_t n) const = 0;
	virtual T compute_add(T const& sum, T const& data) const = 0;

   private:
	T push() {
		if (!do_average_semaphore.try_acquire_for(std::chrono::milliseconds(1000))) {
			throw std::runtime_error("Averager: No more data available");
		}

		data_semaphore.acquire();

		T ret = compute_average(sum, n);
		sum = start;
		n = 0;

		data_semaphore.release();

		return ret;
	}

	void run(T const& data) {
		data_semaphore.acquire();
		sum = compute_add(sum, data);
		++n;
		data_semaphore.release();
	}
};