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
#ifndef NIGH_METRIC_IMPL_SO2_HPP
#define NIGH_METRIC_IMPL_SO2_HPP

#include <type_traits>
#include <cmath>
#include <Eigen/Dense>
#include "constants.hpp"

namespace unc::robotics::nigh::impl::so2 {
    template <typename S>
    std::enable_if_t<std::is_floating_point_v<S>, S>
    angularDistance(S a, S b) {
        S d = std::fmod(std::abs(a - b), impl::PI<S> * 2);
        return std::min(d, impl::PI<S>*2 - d);
    }

    template <int p, typename A, typename B>
    std::common_type_t<typename A::Scalar, typename B::Scalar>
    angularDistance(const Eigen::ArrayBase<A>& a, const Eigen::ArrayBase<B>& b) {
        using Scalar = std::common_type_t<typename A::Scalar, typename B::Scalar>;
        Eigen::Array<Scalar, A::RowsAtCompileTime, A::ColsAtCompileTime> r = (a - b).cwiseAbs();
        r -= (r / (2*impl::PI<Scalar>)).floor() * (2*impl::PI<Scalar>);
        r = (r < -impl::PI<Scalar>).select(r + 2*impl::PI<Scalar>, r);
        r = (r >  impl::PI<Scalar>).select(r - 2*impl::PI<Scalar>, r);
        return r.matrix().template lpNorm<p>();
    }

    template <int p, typename A, typename B>
    auto angularDistance(const Eigen::MatrixBase<A>& a, const Eigen::MatrixBase<B>& b) {
        return angularDistance<p>(a.array(), b.array());
    }

        // Returns the argument bound to the range -PI..PI
    template <typename Scalar>
    std::enable_if_t<std::is_floating_point_v<Scalar>, Scalar>
    bound(Scalar a) {
        if ((a = std::fmod(a, PI<Scalar>*2)) <= -PI<Scalar>)
            return a + PI<Scalar>*2;
        return (a > PI<Scalar>) ? a - PI<Scalar>*2 : a;
    }

    // computes the counter-clockwise distance from a to b
    template <typename Scalar>
    Scalar ccwDist(Scalar a, Scalar b) {
        Scalar d = std::fmod(b - a, PI<Scalar>*2);
        return d < 0 ? d + PI<Scalar>*2 : d;
    }

    // This method is exactly the same ad ccwDist except that two
    // overlapping values are considered 2*PI instead of 0.
    template <typename Scalar>
    Scalar ccwRange(Scalar a, Scalar b) {
        Scalar d = std::fmod(b - a, PI<Scalar>*2);
        return d <= 0 ? d + PI<Scalar>*2 : d;
    }

    template <typename Scalar>
    Scalar antipode(Scalar a) {
        return a <= 0 ? (a + PI<Scalar>) : (a - PI<Scalar>);
    }
}

#endif
