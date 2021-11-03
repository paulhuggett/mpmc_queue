#include <array>
#include <atomic>
#include <cassert>
#include <chrono>
#include <iostream>
#include <mutex>
#include <thread>
#include <vector>

#include "mpmc_queue.hpp"

class foo {
public:
    explicit constexpr foo(int a) noexcept : a_{a} {
        assert (a >= 0);
    }
    enum endjob { };
    explicit constexpr foo(endjob) noexcept : a_{end_} {}
    foo(foo const & other) noexcept = default;
    foo(foo && other) noexcept = default;

    ~foo () noexcept = default;

    foo & operator= (foo const & ) noexcept = default;
    foo & operator= (foo && ) noexcept = default;

    constexpr bool end_job() const noexcept { return a_ == end_; }
    constexpr int value() const noexcept { return a_; }

private:
    static constexpr int end_ = -1;
    int a_ = 0;
};

using namespace std::chrono_literals;

namespace {

void producer (std::vector<foo> & jobs, rigtorp::mpmc_queue<foo*> & q) {
    for (auto & j: jobs) {
        q.push(&j);
    }
}

void consumer (rigtorp::mpmc_queue<foo*> & q, unsigned th) {
    for (;;) {
        auto * const a = q.pop ();
        if (a->end_job ()) {
            break;
        }
        std::printf("%d\t%d\n", th, a->value());
    }
}

} // end anonymous namespace

int main () {
    std::mutex io_mut;
    std::vector<foo> jobs;
    constexpr auto num_jobs = 10000;
    unsigned int const num_workers = std::max (std::thread::hardware_concurrency(), 2U) - 1U;
    jobs.reserve (num_jobs + num_workers);

    for (auto ctr = 0; ctr < num_jobs; ++ctr) {
        jobs.emplace_back (ctr);
    }
    for (auto ctr = 0U; ctr < num_workers; ++ctr) {
        jobs.emplace_back (foo::endjob{});
    }

    rigtorp::mpmc_queue<foo*> q{1024};

    // Create the worker threads.
    std::vector<std::thread> workers;
    workers.reserve(num_workers);
    for (auto th = 0U; th < num_workers; ++th) {
        workers.emplace_back (consumer, std::ref(q), th);
    }

    std::thread p{producer, std::ref(jobs), std::ref(q)};
    p.join();
    for (auto & w: workers) {
        w.join ();
    }
}

