/*
Copyright (c) 2020 Erik Rigtorp <erik@rigtorp.se>

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
#ifndef MPMC_QUEUE_HPP
#define MPMC_QUEUE_HPP

#include <atomic>
#include <cassert>
#include <cstddef> // offsetof
#include <limits>
#include <memory>
#include <new> // std::hardware_destructive_interference_size
#include <optional>
#include <stdexcept>

#ifndef __cpp_aligned_new
#    ifdef _WIN32
#        include <malloc.h> // _aligned_malloc
#    else
#        include <stdlib.h> // posix_memalign
#    endif
#endif

namespace rigtorp {
    namespace mpmc {

#ifdef __cpp_lib_hardware_interference_size
        static constexpr std::size_t hardware_interference_size =
            std::hardware_destructive_interference_size;
#else
        static constexpr std::size_t hardware_interference_size = 64;
#endif // __cpp_lib_hardware_interference_size

        //*       _ _                  _        _ _              _            *
        //*  __ _| (_)__ _ _ _  ___ __| |  __ _| | |___  __ __ _| |_ ___ _ _  *
        //* / _` | | / _` | ' \/ -_) _` | / _` | | / _ \/ _/ _` |  _/ _ \ '_| *
        //* \__,_|_|_\__, |_||_\___\__,_| \__,_|_|_\___/\__\__,_|\__\___/_|   *
        //*          |___/                                                    *
#if defined(__cpp_aligned_new)
        template <typename T>
        using aligned_allocator = std::allocator<T>;
#else
        template <typename T>
        struct aligned_allocator {
            using value_type = T;
            T * allocate (std::size_t n);
            void deallocate (T * p, std::size_t);
        };

#ifdef _WIN32
        // allocate
        // ~~~~~~~~
        template <typename T>
        T * aligned_allocator<T>::allocate (std::size_t const n) {
            if (n > std::numeric_limits<std::size_t>::max () / sizeof (T)) {
                throw std::bad_array_new_length ();
            }
            auto * p = static_cast<T *> (::_aligned_malloc (sizeof (T) * n, alignof (T)));
            if (p == nullptr) {
                throw std::bad_alloc ();
            }
            return p;
        }

        // deallocate
        // ~~~~~~~~~~
        template <typename T>
        inline void aligned_allocator<T>::deallocate (T * const p, std::size_t) {
            ::_aligned_free (p);
        }
#else
        // allocate
        // ~~~~~~~~
        template <typename T>
        T * aligned_allocator<T>::allocate (std::size_t const n) {
            if (n > std::numeric_limits<std::size_t>::max () / sizeof (T)) {
                throw std::bad_array_new_length ();
            }
            T * p = nullptr;
            if (::posix_memalign (reinterpret_cast<void **> (&p), alignof (T), sizeof (T) * n) != 0) {
                throw std::bad_alloc ();
            }
            return p;
        }

        // deallocate
        // ~~~~~~~~~~
        template <typename T>
        inline void aligned_allocator<T>::deallocate (T * const p, std::size_t) {
            std::free (p);
        }
#endif // !_WIN32

#endif //__cpp_aligned_new

        //*     _     _    *
        //*  __| |___| |_  *
        //* (_-< / _ \  _| *
        //* /__/_\___/\__| *
        //*                *
        template <typename T>
        struct slot {
            constexpr slot () noexcept = default;
            ~slot () noexcept {
                if (turn & 1U) {
                    destroy ();
                }
            }

            template <typename... Args>
            void construct (Args &&... args) noexcept {
                static_assert (std::is_nothrow_constructible_v<T, Args &&...>, "T must be nothrow constructible with Args&&...");
                new (&storage) T (std::forward<Args> (args)...);
            }

            void destroy () noexcept {
                static_assert (std::is_nothrow_destructible_v<T>, "T must be nothrow destructible");
                reinterpret_cast<T *> (&storage)->~T ();
            }

            T && move () noexcept { return reinterpret_cast<T &&> (storage); }

            // Align to avoid false sharing between adjacent slots
            alignas (hardware_interference_size) std::atomic<std::size_t> turn = {0};
            typename std::aligned_storage_t<sizeof (T), alignof (T)> storage;
        };

        //*                          *
        //*  __ _ _  _ ___ _  _ ___  *
        //* / _` | || / -_) || / -_) *
        //* \__, |\_,_\___|\_,_\___| *
        //*    |_|                   *
        template <typename T, typename Allocator = aligned_allocator<slot<T>>>
        class queue {
            static_assert (std::is_nothrow_copy_assignable_v<T> || std::is_nothrow_move_assignable_v<T>, "T must be nothrow copy or move assignable");
            static_assert (std::is_nothrow_destructible_v<T>, "T must be nothrow destructible");

        public:
            explicit queue (std::size_t capacity, Allocator const & allocator = Allocator ());
            queue (queue const &) = delete;
            queue (queue && ) noexcept = delete;

            ~queue () noexcept = default;

            queue & operator= (queue const &) = delete;
            queue & operator= (queue &&) noexcept = delete;

            /// Enqueue an item using inplace construction. Blocks if the queue is full.
            template <typename... Args>
            void emplace (Args &&... args) noexcept;

            /// Try to enqueue an item using inplace construction.
            /// \returns Returns true on success and false if the queue is full.
            template <typename... Args>
            bool try_emplace (Args &&... args) noexcept;

            /// Enqueue an item using copy construction. Blocks if the queue is full.
            void push (T const & v) noexcept {
                static_assert (std::is_nothrow_copy_constructible_v<T>, "T must be nothrow copy constructible");
                emplace (v);
            }

            /// Enqueue an item using move construction. Participates in overload resolution only
            /// if type P is nothrow constructible. Blocks if the queue is full.
            template <typename P, typename = typename std::enable_if_t<std::is_nothrow_constructible_v<T, P &&>>>
            void push (P && v) noexcept {
                emplace (std::forward<P> (v));
            }

            /// Try to enqueue an item using copy construction.
            /// \returns True on success and false if the queue is full.
            bool try_push (T const & v) noexcept {
                static_assert (std::is_nothrow_copy_constructible_v<T>, "T must be nothrow copy constructible");
                return try_emplace (v);
            }

            /// Try to enqueue an item using move construction. Participates in overload resolution only if type P is nothrow constructible.
            /// \returns true on success, false if the queue is full.
            template <typename P, typename = typename std::enable_if_t<std::is_nothrow_constructible_v<T, P &&>>>
            bool try_push (P && v) noexcept {
                return try_emplace (std::forward<P> (v));
            }

            /// Dequeue an item by copying or moving the item into v. Blocks if the queue is empty.
            T pop () noexcept;
            std::optional<T> try_pop () noexcept;

        private:
            constexpr std::size_t idx (std::size_t i) const noexcept { return i % capacity_; }
            constexpr std::size_t turn (std::size_t i) const noexcept { return i / capacity_; }

            std::size_t const capacity_;
            std::vector<slot<T>, Allocator> slots_;

            // Align to avoid false sharing between head_ and tail_
            alignas (hardware_interference_size) std::atomic<std::size_t> head_ = {0};
            alignas (hardware_interference_size) std::atomic<std::size_t> tail_ = {0};
        };

        // (ctor)
        // ~~~~~~
        template <typename T, typename Allocator>
        queue<T, Allocator>::queue (std::size_t const capacity, Allocator const & allocator)
                : capacity_{capacity}
                , slots_{capacity + 1U, allocator} // One extra to prevent false sharing on the last
        {
            if (capacity_ < 1U) {
                throw std::invalid_argument ("capacity < 1");
            }
            static_assert (
                alignof (slot<T>) == hardware_interference_size,
                "Slot must be aligned to cache line boundary to prevent false sharing");
            static_assert (sizeof (slot<T>) % hardware_interference_size == 0,
                           "Slot size must be a multiple of cache line size to prevent false "
                           "sharing between adjacent slots");
            static_assert (sizeof (queue) % hardware_interference_size == 0,
                           "Queue size must be a multiple of cache line size to prevent false "
                           "sharing between adjacent queues");
            static_assert (offsetof (queue, tail_) - offsetof (queue, head_) ==
                               static_cast<std::ptrdiff_t> (hardware_interference_size),
                           "head and tail must be a cache line apart to prevent false sharing");
        }

        // emplace
        // ~~~~~~~
        template <typename T, typename Allocator>
        template <typename... Args>
        void queue<T, Allocator>::emplace (Args &&... args) noexcept {
            static_assert (std::is_nothrow_constructible_v<T, Args &&...>,
                           "T must be nothrow constructible with Args&&...");
            auto const head = head_.fetch_add (1);
            auto & slot = slots_[idx (head)];
            while (turn (head) * 2 != slot.turn.load (std::memory_order_acquire)) {
            }
            slot.construct (std::forward<Args> (args)...);
            slot.turn.store (turn (head) * 2 + 1, std::memory_order_release);
        }

        // try_emplace
        // ~~~~~~~~~~~
        template <typename T, typename Allocator>
        template <typename... Args>
        bool queue<T, Allocator>::try_emplace (Args &&... args) noexcept {
            static_assert (std::is_nothrow_constructible_v<T, Args &&...>,
                           "T must be nothrow constructible with Args&&...");
            auto head = head_.load (std::memory_order_acquire);
            for (;;) {
                auto & slot = slots_[idx (head)];
                if (turn (head) * 2 == slot.turn.load (std::memory_order_acquire)) {
                    if (head_.compare_exchange_strong (head, head + 1)) {
                        slot.construct (std::forward<Args> (args)...);
                        slot.turn.store (turn (head) * 2 + 1, std::memory_order_release);
                        return true;
                    }
                } else {
                    auto const prev_head = head;
                    head = head_.load (std::memory_order_acquire);
                    if (head == prev_head) {
                        return false;
                    }
                }
            }
        }

        // pop
        // ~~~
        template <typename T, typename Allocator>
        T queue<T, Allocator>::pop () noexcept {
            auto const tail = tail_.fetch_add (1);
            auto & slot = slots_[idx (tail)];
            while (turn (tail) * 2 + 1 != slot.turn.load (std::memory_order_acquire)) {
            }
            T const result = slot.move ();
            slot.destroy ();
            slot.turn.store (turn (tail) * 2 + 2, std::memory_order_release);
            return result;
        }

        // try_pop
        // ~~~~~~~
        template <typename T, typename Allocator>
        std::optional<T> queue<T, Allocator>::try_pop () noexcept {
            auto tail = tail_.load (std::memory_order_acquire);
            for (;;) {
                auto & slot = slots_[idx (tail)];
                if (turn (tail) * 2 + 1 == slot.turn.load (std::memory_order_acquire)) {
                    if (tail_.compare_exchange_strong (tail, tail + 1)) {
                        T const result = slot.move ();
                        slot.destroy ();
                        slot.turn.store (turn (tail) * 2 + 2, std::memory_order_release);
                        return result;
                    }
                } else {
                    auto const prev_tail = tail;
                    tail = tail_.load (std::memory_order_acquire);
                    if (tail == prev_tail) {
                        return {std::nullopt};
                    }
                }
            }
        }

    } // end namespace mpmc

    template <typename T, typename Allocator = mpmc::aligned_allocator<mpmc::slot<T>>>
    using mpmc_queue = mpmc::queue<T, Allocator>;

} // end namespace rigtorp

#endif // MPMC_QUEUE_HPP
