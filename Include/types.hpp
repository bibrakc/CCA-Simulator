/*
BSD 3-Clause License

Copyright (c) 2023, Bibrak Qamar

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

#ifndef TYPES_HPP
#define TYPES_HPP

#include <cassert>
#include <ostream>
#include <queue>

// How many channels in a single direction. How many parallel send/recv occur in a single cycle per
// channel.
// TODO: Not using this right now. Currently, there will be only 1 send/recv per channel.
constexpr u_int32_t lane_width = 1;

// Size of the recv buffer at each channel.
constexpr u_int32_t buffer_size = RECVBUFFSIZE;

using Coordinates = std::pair<u_int32_t, u_int32_t>;

// Overload printing for Coordinates
auto
operator<<(std::ostream& os, const Coordinates& coord) -> std::ostream&;

// Fixed size queue to be used for storing Operons. The fixed queue size is the bandwidth at that
// HtreeNode
template<typename T>
class FixedSizeQueue
{
  private:
    std::queue<T> underlying_queue;
    u_int32_t size_max;
    u_int32_t buffer_space_for_priority;

  public:
    // Default constructor
    FixedSizeQueue() // Default constructor initialize it to 0.
        : size_max(0)
        , buffer_space_for_priority(0)
    {
    }

    FixedSizeQueue(u_int32_t size_max_in)
        : size_max(size_max_in)
        , buffer_space_for_priority(0)
    {
    }

    FixedSizeQueue(u_int32_t size_max_in, u_int32_t buffer_space_for_priority_in)
        : size_max(size_max_in + buffer_space_for_priority_in)
        , buffer_space_for_priority(buffer_space_for_priority_in)
    {
    }

    [[nodiscard]] auto push(const T& value) -> bool
    {
        // Not able to enqueue. Return false
        if (underlying_queue.size() >= this->size_max - this->buffer_space_for_priority) {
            return false;
        }
        this->underlying_queue.push(value);
        return true;
    }

    [[nodiscard]] auto push(const T& value, bool priority) -> bool
    {
        if (priority) {
            if (underlying_queue.size() <= this->size_max) {
                this->underlying_queue.push(value);
                return true;
            }
        }
        return this->push(value);
    }

    // Get from front FIFO
    [[nodiscard]] auto front() const -> T { return underlying_queue.front(); }

    // Pop/Dequeue
    void pop() { underlying_queue.pop(); }

    // Return the current size of the queue
    [[nodiscard]] auto size() const -> u_int32_t { return underlying_queue.size(); }

    // Return the max size of the queue
    [[nodiscard]] auto queue_size_max() const -> u_int32_t { return this->size_max; }

    // Return the whether it is empty
    [[nodiscard]] auto empty() const -> bool { return this->underlying_queue.empty(); }

    // Return whether there is a slot in the queue
    [[nodiscard]] auto has_room() const -> bool
    {
        return (this->size() < this->size_max - this->buffer_space_for_priority);
    }

    // Return whether there is a slot in the queue with priority
    [[nodiscard]] auto has_room(bool priority) const -> bool
    {
        if (priority) {
            return (this->size() != this->size_max);
        } else {
            return this->has_room();
        }
    }

    // Experimental: For prioritizing the action, diffuse, and task queues
    // Return whether there is a slot in the queue.
    // Check if the queue is `percent`% full.
    [[nodiscard]] auto is_percent_full(double percent) const -> bool
    {
        assert(percent > 0.0 && percent < 100.0 && "Percentage must be between 0 and 100");

        double threshold = size_max / (100.0 / percent);
        return (this->underlying_queue.size() > static_cast<u_int32_t>(threshold));
    }
};
#include <iostream>
class MaxCounter
{
  private:
    u_int32_t counter{ 0 };
    u_int32_t max_counter{ 0 };
    u_int32_t total_counter{ 0 };

  public:
    MaxCounter() {}
    // u_int32_t temp_cc_id{ 0 };

    void increment()
    {
        this->counter++;
        this->total_counter++;
        if (this->counter > this->max_counter) {
            this->max_counter = counter;
        }
    }

    void reset() { this->counter = 0; }
    void decrement()
    {
        assert(this->counter > 0);
        this->counter = this->counter - 1;
    }
    [[nodiscard]] auto get_count() const -> u_int32_t { return this->counter; }
    [[nodiscard]] auto get_max_count() const -> u_int32_t { return this->max_counter; }
    [[nodiscard]] auto get_total_count() const -> u_int32_t { return this->total_counter; }
};

#endif // TYPES_HPP
