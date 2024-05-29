/*
BSD 3-Clause License

Copyright (c) 2024, Bibrak Qamar

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef LCO_FUTURE_HPP
#define LCO_FUTURE_HPP

#include "Function.hpp"

enum class lcoFutureState : u_int32_t
{
    empty = 0, // Initial state.
    pending,   // When it is waiting on a continuation to return and fullfil it.
    fulfilled, // When it is set with a valid value.
    invalid,   // Placeholder for something unsual.
    lcoFutureState_count
};

template<typename T>
class LCO_Future
{
  public:
    // The local value.
    T local_val{};

    // The actions pending on this future. Right now just hardcoding it to be 5 later may be
    // made flexible.
    inline static constexpr u_int32_t queue_max_size = 30;
    // Action queue[queue_max_size];
    // CCAFunctionEvent: the continuation, ActionArgumentType: whatever that is needed to resume the
    // continuation.
    Closure queue[queue_max_size];
    u_int32_t queue_size{}; // Current size of the queue.
    u_int32_t queue_head{}; // Points to the front of the queue (dequeue point).
    u_int32_t queue_tail{}; // Points to the next position to enqueue.

    [[nodiscard]] auto enqueue(Closure continuation_closure) -> bool
    {

        if (this->queue_size == LCO_Future::queue_max_size) {
            return false;
        }

        this->queue[this->queue_tail] = continuation_closure;
        this->queue_tail = (this->queue_tail + 1) % queue_max_size;
        ++this->queue_size;

        return true;
    }

    auto dequeue() -> std::optional<Closure>
    {
        if (this->queue_size == 0) {
            return std::nullopt; // Queue is empty
        }

        Closure continuation_closure = this->queue[this->queue_head];
        this->queue_head = (this->queue_head + 1) % queue_max_size;
        --this->queue_size;

        return continuation_closure;
    }

    // State of the future lco.
    lcoFutureState state;

    // Sets to `true` when all dependencies for this LCO have been satisfied.
    // TODO: Not sure where and how to use it... for now...
    // bool is_ready{};

    inline auto is_empty() -> bool { return (this->state == lcoFutureState::empty); }
    inline auto is_fulfilled() -> bool { return (this->state == lcoFutureState::fulfilled); }
    inline auto is_pending() -> bool { return (this->state == lcoFutureState::pending); }

    void reset() { assert(false && "reset() is not implemented yet!"); }
    void set_state(lcoFutureState state_in) { this->state = state_in; }

    auto get() -> T
    {
        assert(this->state == lcoFutureState::fulfilled);
        return this->local_val;
    }

    // Overloading the + operator for LCO_Future<T> + T
    LCO_Future<T> operator+(const T& other) const = delete;

    // Overload the += operator for LCO_Future<T> += T
    LCO_Future<T>& operator+=(const T& other) = delete;

    // Assignment, set.
    LCO_Future<T>& operator=(const T& other)
    {
        assert(this->is_empty() or this->is_pending());

        this->local_val = other;
        this->state = lcoFutureState::fulfilled;

        return *this;
    }

    LCO_Future() { this->state = lcoFutureState::empty; }
};

#endif // LCO_FUTURE_HPP
