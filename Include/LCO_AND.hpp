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

#ifndef LCO_AND_HPP
#define LCO_AND_HPP

template<typename T>
class LCO_AND
{
  public:
    // The local value.
    T local_val{};

    // The total number of gates.
    u_int32_t N{};
    // How many gates have been set.
    u_int32_t count{};

    // Sets to `true` when all dependencies for this LCO have been satisfied.
    // TODO: Not sure where and how to use it... for now...
    bool is_ready{};

    bool increment()
    {
        this->count++;
        // if (this->count == 1){//this->N) {
        if (this->count == this->N) {
            this->is_ready = true;
        }
        return this->is_ready;
    }

    void reset()
    {
        this->local_val = 0;
        this->count = 0;
        this->is_ready = false;
    }

    // Overloading the + operator for LCO_AND<T> + T
    LCO_AND<T> operator+(const T& other) const
    {
        // Create a copy of the current object
        LCO_AND<T> result(*this);
        // +=
        result.local_val += other;
        return result;
    }

    // Overload the += operator for LCO_AND<T> += T
    LCO_AND<T>& operator+=(const T& other)
    {
        this->local_val += other;
        return *this;
    }
};

#endif // LCO_AND_HPP
