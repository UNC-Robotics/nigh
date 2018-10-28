// Software License Agreement (BSD-3-Clause)
//
// Copyright 2018 The University of North Carolina at Chapel Hill
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
// INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
// STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
// OF THE POSSIBILITY OF SUCH DAMAGE.

//! @author Jeff Ichnowski

#pragma once
#ifndef NIGH_METRIC_SCALED_HPP
#define NIGH_METRIC_SCALED_HPP

#include "metric.hpp"
#include <ratio>

namespace unc::robotics::nigh::metric {

    template <typename M, typename W>
    struct Scaled {
        static_assert(std::is_floating_point_v<W>, "weight type must be a floating point value, or std::ratio");
        using Metric = M;
        using Weight = W;
    };

    template <typename M, std::intmax_t num, std::intmax_t den>
    struct Scaled<M, std::ratio<num, den>> : private M {
        using Metric = M;
        using Weight = std::ratio<num, den>;

        static_assert(Weight::num > 0, "ratio must be positive");

        constexpr const Metric& metric() const { return *this; }

        template <typename S>
        constexpr S weight() const {
            return static_cast<S>(Weight::num) / static_cast<S>(Weight::den);
        }
    };

    template <typename M, typename W>
    struct is_metric<Scaled<M, W>> : is_metric<M> {};
}

#include "../impl/kdtree_batch/nearest_traversals.hpp"
#include "../impl/kdtree_batch/regions.hpp"
#include "../impl/kdtree_batch/traversals.hpp"

#endif
