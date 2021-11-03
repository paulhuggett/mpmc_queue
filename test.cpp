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

#include <memory>
#include <unordered_set>
#include <thread>
#include <vector>

#include <gmock/gmock.h>

#include "mpmc_queue.hpp"

namespace {

    class MPMCQueueXtorCountFixture : public ::testing::Test {
    protected:
        struct counter {
            explicit counter (MPMCQueueXtorCountFixture * const host) noexcept
                    : host_{host} {
                try {
                    EXPECT_EQ (host_->constructed.count (this), 0U);
                    host_->constructed.insert (this);
                } catch (...) {
                    host->exception = true;
                }
            }

            counter (counter const & other) noexcept
                    : host_{other.host_} {
                try {
                    EXPECT_EQ (host_->constructed.count (this), 0U);
                    EXPECT_EQ (host_->constructed.count (&other), 1U);
                    host_->constructed.insert (this);
                } catch (...) {
                    host_->exception = true;
                }
            }

            counter (counter && other) noexcept
                    : host_{other.host_} {
                try {
                    EXPECT_EQ (host_->constructed.count (this), 0U);
                    EXPECT_EQ (host_->constructed.count (&other), 1U);
                    host_->constructed.insert (this);
                } catch (...) {
                    host_->exception = true;
                }
            }

            counter & operator= (counter const & other) noexcept {
                if (&other != this) {
                    host_ = other.host_;
                    EXPECT_EQ (host_->constructed.count (this), 1U);
                    EXPECT_EQ (host_->constructed.count (&other), 1U);
                }
                return *this;
            }

            counter & operator= (counter && other) noexcept {
                if (&other != this) {
                    host_ = other.host_;
                    EXPECT_EQ (host_->constructed.count (this), 1U);
                    EXPECT_EQ (host_->constructed.count (&other), 1U);
                }
                return *this;
            }

            ~counter () noexcept {
                try {
                    EXPECT_EQ (host_->constructed.count (this), 1U);
                    host_->constructed.erase (this);
                } catch (...) {
                    host_->exception = true;
                }
            }

            MPMCQueueXtorCountFixture * host_;
            // To verify that alignment and padding calculations are handled correctly
            char data[129];
        };

        std::unordered_set<counter const *> constructed;
        bool exception = false;
    };

} // end anonymous namespace


using namespace rigtorp;

TEST_F (MPMCQueueXtorCountFixture, ObjectCounts) {
    {
        constexpr auto entries = 10U;

        mpmc_queue<counter> q{11};
        for (auto ctr = 0U; ctr < entries; ctr++) {
            q.emplace (this);
        }
        EXPECT_EQ (constructed.size (), entries);

        {
            auto const t1 = q.pop ();
            EXPECT_EQ (constructed.size (), entries);
        }
        {
            auto const t2 = q.pop ();
            q.emplace (this);
            EXPECT_EQ (constructed.size (), entries);
        }
    }
    EXPECT_EQ (constructed.size (), 0U);
    ASSERT_FALSE (exception);
}

TEST (MPMCQueue, TryPushTryPop) {
    mpmc_queue<int> q{1};
    EXPECT_TRUE (q.try_push (1));
    EXPECT_FALSE (q.try_push (2));
    std::optional<int> t1 = q.try_pop ();
    ASSERT_TRUE (t1);
    EXPECT_EQ (*t1, 1);
    std::optional<int> t2 = q.try_pop ();
    ASSERT_FALSE (t2);
}

TEST (MPMCQueue, CopyableOnlyType) {
    struct copyable_only {
        copyable_only () noexcept = default;
        copyable_only (copyable_only const &) noexcept = default;
        copyable_only (copyable_only &&) = delete;

        copyable_only & operator= (copyable_only const &) noexcept { return *this; }
        copyable_only & operator= (copyable_only &&) noexcept = delete;
    };

    mpmc_queue<copyable_only> q{16U};
    // lvalue
    copyable_only v;
    q.emplace (v);
    q.try_emplace (v);
    q.push (v);
    q.try_push (v);
    // xvalue
    q.push (copyable_only{});
    q.try_push (copyable_only{});
}

TEST (MPMCQueue, MovableOnlyType) {
    mpmc_queue<std::unique_ptr<int>> q{16U};
    // lvalue
    // auto v = uptr(new int(1));
    // q.emplace(v);
    // q.try_emplace(v);
    // q.push(v);
    // q.try_push(v);
    // xvalue
    q.emplace (std::make_unique<int> (1));
    q.try_emplace (std::make_unique<int> (1));
    q.push (std::make_unique<int> (1));
    q.try_push (std::make_unique<int> (1));
}

TEST (MPMCQueue, ThrowsOnBadCapacity) {
    EXPECT_THROW (mpmc_queue<int> q{0U}, std::invalid_argument);
}

// Work around a bug in cl.exe which does not implicitly capture constexpr values.
#ifdef _MSC_VER
#    define CAPTURE_BUG(...) , __VA_ARGS__
#else
#    define CAPTURE_BUG(...)
#endif

TEST (MPMCQueue, Fuzz) {
    constexpr auto num_ops = 1000ULL;
    constexpr auto num_threads = 10U;
    mpmc_queue<unsigned long long> q{num_threads};
    std::atomic<bool> flag = false;
    std::vector<std::thread> threads;
    std::atomic<unsigned long long> sum = 0ULL;
    for (auto ctr = 0U; ctr < num_threads; ++ctr) {
        threads.emplace_back (
            [&q, &flag CAPTURE_BUG (num_ops, num_threads)] (unsigned const tctr) {
                while (!flag) {
                }
                for (unsigned long long j = tctr; j < num_ops; j += num_threads) {
                    q.push (j);
                }
            },
            ctr);
    }

    for (auto ctr = 0U; ctr < num_threads; ++ctr) {
        threads.emplace_back (
            [&q, &flag, &sum CAPTURE_BUG (num_ops, num_threads)] (unsigned const tctr) {
                while (!flag) {
                }
                auto thread_sum = 0ULL;
                for (auto j = tctr; j < num_ops; j += num_threads) {
                    thread_sum += q.pop ();
                }
                sum += thread_sum;
            },
            ctr);
    }
    flag = true;
    for (auto & thread : threads) {
        thread.join ();
    }
    ASSERT_EQ (sum, num_ops * (num_ops - 1) / 2);
}

#undef CAPTURE_BUG
