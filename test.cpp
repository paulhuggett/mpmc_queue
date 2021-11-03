/*
Copyright (c) 2018 Erik Rigtorp <erik@rigtorp.se>
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
 */

#undef NDEBUG

#include <cassert>
#include <chrono>
#include <iostream>
#include <set>
#include <thread>
#include <vector>

#include "mpmc_queue.hpp"

// TestType tracks correct usage of constructors and destructors
struct TestType {
  static std::set<const TestType *> constructed;

  TestType() noexcept {
    assert(constructed.count(this) == 0);
    constructed.insert(this);
  }

  TestType(const TestType &other) noexcept {
    assert(constructed.count(this) == 0);
    assert(constructed.count(&other) == 1);
    constructed.insert(this);
  }

  TestType(TestType &&other) noexcept {
    assert(constructed.count(this) == 0);
    assert(constructed.count(&other) == 1);
    constructed.insert(this);
  }

  TestType &operator=(const TestType &other) noexcept {
    assert(constructed.count(this) == 1);
    assert(constructed.count(&other) == 1);
    return *this;
  }

  TestType &operator=(TestType &&other) noexcept {
    assert(constructed.count(this) == 1);
    assert(constructed.count(&other) == 1);
    return *this;
  }

  ~TestType() noexcept {
    assert(constructed.count(this) == 1);
    constructed.erase(this);
  }

  // To verify that alignment and padding calculations are handled correctly
  char data[129];
};

std::set<const TestType *> TestType::constructed;

int main() {
    using namespace rigtorp;

    {
        mpmc_queue<TestType> q(11);
        for (int i = 0; i < 10; i++) {
            q.emplace();
        }
        assert(TestType::constructed.size() == 10U);
        
        {
            auto const t1 = q.pop();
            assert(TestType::constructed.size() == 10U);
        }
        {
            auto const t2 = q.pop();
            q.emplace();
            assert(TestType::constructed.size() == 10U);
        }
    }
    assert(TestType::constructed.size() == 0U);

    {
        mpmc_queue<int> q{1};
        assert(q.try_push(1) == true);
        assert(q.try_push(2) == false);
        std::optional<int> t1 = q.try_pop();
        assert(t1 && *t1 == 1);
        std::optional<int> t2 = q.try_pop();
        assert(!t2);
    }

    // Copyable only type
    {
        struct copyable_only {
            copyable_only() noexcept = default;
            copyable_only(copyable_only const &) noexcept = default;
            copyable_only(copyable_only &&) = delete;
            
            copyable_only &operator=(copyable_only const &) noexcept { return *this; }
            copyable_only &operator=(copyable_only &&) noexcept = delete;
        };
        mpmc_queue<copyable_only> q{16U};
        // lvalue
        copyable_only v;
        q.emplace(v);
        q.try_emplace(v);
        q.push(v);
        q.try_push(v);
        // xvalue
        q.push(copyable_only{});
        q.try_push(copyable_only{});
    }
    
    // Movable only type
    {
        using uptr = std::unique_ptr<int>;
        mpmc_queue<uptr> q{16U};
        // lvalue
        // auto v = uptr(new int(1));
        // q.emplace(v);
        // q.try_emplace(v);
        // q.push(v);
        // q.try_push(v);
        // xvalue
        q.emplace(uptr{new int{1}});
        q.try_emplace(uptr{new int{1}});
        q.push(uptr{new int{1}});
        q.try_push(uptr{new int{1}});
    }
    
    {
        bool throws = false;
        try {
            mpmc_queue<int> q{0U};
        } catch (std::exception const &) {
            throws = true;
        }
        assert(throws);
    }
    
    // Fuzz test
    {
        constexpr auto num_ops = 1000ULL;
        constexpr auto num_threads = 10U;
        mpmc_queue<unsigned long long> q{num_threads};
        std::atomic<bool> flag = false;
        std::vector<std::thread> threads;
        std::atomic<unsigned long long> sum = 0ULL;
        for (auto i = 0U; i < num_threads; ++i) {
            threads.emplace_back([&q, &flag] (unsigned const tctr) {
                while (!flag) {
                }
                for (unsigned long long j = tctr; j < num_ops; j += num_threads) {
                    q.push(j);
                }
            }, i);
        }
    
        for (auto i = 0U; i < num_threads; ++i) {
            threads.emplace_back([&q, &flag, &sum] (unsigned const tctr) {
                while (!flag) {
                }
                auto thread_sum = 0ULL;
                for (auto j = tctr; j < num_ops; j += num_threads) {
                    thread_sum += q.pop();
                }
                sum += thread_sum;
            }, i);
        }
        flag = true;
        for (auto &thread : threads) {
            thread.join();
        }
        assert(sum == num_ops * (num_ops - 1) / 2);
    }
    std::cout << "Done\n";
}
