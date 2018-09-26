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
#ifndef NIGH_IMPL_KDTREE_BATCH_TRAVERSAL_SO2_HPP
#define NIGH_IMPL_KDTREE_BATCH_TRAVERSAL_SO2_HPP

#include "../constants.hpp"
#include "../so2.hpp"
#include "traversal.hpp"
#include "types.hpp"
#include "lp_branch.hpp"
#include <cmath>

namespace unc::robotics::nigh::impl::kdtree_batch {

    // Finds a split on a set of SO(2) values normalized to the range
    // [-pi,pi].  The split divides the values into two evenly sized
    // (_N/2) sets, and maximizes the distance of the bounds to the
    // split.
    template <typename Iter>
    auto so2split(Iter first, Iter last) {
        using Scalar = typename std::iterator_traits<Iter>::value_type;
        std::size_t n = std::distance(first, last);
        assert(n > 1); // can only split more than 1 element

        // The loop finds the `i` at which half elements are to the
        // left of the line bisecting line it define.  To find it we
        // test for `j = i + N/2`, that the ccw distance to `[j] < pi`
        // and `[j+1] > pi`.  The "best" split is the one that
        // maximizes the distance between the points closest to the
        // split, thus `[i]`, `[i+1]`, `[j]`, and `[j+1]`.

        // distances: pi-D(i,j), D(i,j+1)-pi, D(i,i+1)
        //
        //        i
        //        |@@@
        //        |@@@@
        //   -----X@@@@
        //   ####/|\@@@
        //    ##/ | \@
        //     j    j+1

        //          /
        // j+1 ----X
        //     ###/|\     |
        //     ##/ |@\    |
        //     #/  |@@\   |
        //     i       j

        //        j+1 j
        //      ###| /@
        //     ####|/@@@
        //     ####X@@@@
        //     ###/ \@@@
        //      #/   \@
        //      i

        std::sort(first, last);
        Scalar dBest = -1, split = 0;
        Iter i1 = first;
        Iter j1 = first + n/2;
        do {
            Iter i0 = i1;
            Iter j0 = j1;
            if (++i1 == last) i1 = first;
            if (++j1 == last) j1 = first;
            Scalar d0 = PI<Scalar> - so2::ccwDist(*i0, *j0); // vals[i], vals[j%n]);
            Scalar d1 = PI<Scalar> - so2::ccwDist(*j1, *i0); // vals[(j+1)%n], vals[i]);

            if (d0 >= 0 && d1 >= 0) {
                Scalar di = so2::ccwDist(*i0, *i1); // vals[i], vals[(i+1)%n]);
                // Scalar split = vals[i] + std::min(di, d1) * 0.5;

                Scalar range = 2*PI<Scalar> - (d0+d1);

                if (range < PI<Scalar>) {
                    // The range of values is less than half a circle.
                    // split halfway between i0 and i1
                    if (range > dBest) {
                        dBest = range;
                        split = *i0 + di * 0.5;
                    }
                } else {
                    // The range of values is more than half a circle
                    // split considering both sides of split plane.
                    // we cannot split halfway between i0 and i1 since
                    // it could result in moving j1 to the other side
                    // of the split.
                    //
                    // An easy split to do would be i0 + min(di,d1)/2,
                    // since that would be halfway from the split to
                    // the next bound.  This is also guaranteed to be
                    // in the bounds.
                    //
                    // A possibly better split is to try to maximize
                    // the sum of square distances from the split.
                    //
                    // define x as the split offset from i0.  The
                    // distances from the split plane are thus:
                    //
                    //   i0 to split = x
                    //   split to i1 = di - x
                    //   j0 to split = d0 + x
                    //   split to j1 = d1 - x
                    //
                    // summing the square of the above quantities,
                    // then differentiating and solving for 0, we get:
                    //
                    // x = (di + d1 - d0) / 4;

                    // Compute the range as the sum of distances from
                    // the split.  We add PI so that we prefer
                    // splitting these axes over axes that are already
                    // split.
                    Scalar dSum = PI<Scalar> + (di+d0+d1)/2; // std::min({di, d0, d1});
                    if (dSum > dBest) {
                        dBest = dSum;
                        split = *i0 + std::min(di, d1) * Scalar(0.5);
                    }
                }
            }
        } while (i1 != first);

        //if (sBest > PI<Scalar>) sBest -= 2*PI<Scalar>;
        split = so2::bound(split);

        assert(0 <= dBest && dBest <= 2*PI<Scalar>);
        assert(-PI<Scalar> <= split && split <= PI<Scalar>);

        return std::make_pair(dBest, split);
    }

    template <typename Container>
    auto so2split(Container& container) {
        return so2split(container.begin(), container.end());
    }

    template <typename Tree, typename Key, int p, typename Get>
    class Traversal<Tree, Key, metric::SO2<p>, Get> {
    protected:
        using Metric = metric::SO2<p>;
        using Space = metric::Space<Key, Metric>;
        using Distance = typename Space::Distance;
        using Leaf = leaf_t<Tree>;
        using Node = node_t<Tree>;
        using Concurrency = concurrency_t<Tree>;
        using NodePointer = node_pointer_t<Tree>;

        static constexpr std::size_t batchSize = kBatchSize<Tree>;

        using SO2Branch = kdtree_batch::LPBranch<
            value_t<Tree>,
            space_t<Tree>,
            Concurrency,
            kBatchSize<Tree>,
            p>;

    public:
        Traversal(const Space&) {
        }

        void grow(const Space&, Region<Key, Metric, Concurrency>& region, const Key& key) {
            // region.grow(q);
        }

        Distance selectAxis(Tree& tree, const Space& space, const Leaf* leaf, const Key& key, unsigned *axis) const {
            Distance maxDist = -1;
            std::size_t dim = space.dimensions();
            std::array<Distance, batchSize> Q;
            *axis = 0;
            for (std::size_t a=0 ; a<dim ; ++a) {
                for (std::size_t j=0 ; j<batchSize ; ++j)
                    Q[j] = so2::bound(Space::coeff(Get::part(tree.getKey(leaf->elements()[j])), a));

                [[maybe_unused]] auto [distToSplit, split] = so2split(Q);
                (void)split; // GCC seems to ignore [[maybe_unused]]

                // distToSplit will be in the range 0..2pi and is the
                // sum of the bounds distances to the split.  We wish
                // to find the split that minimizes this.
                if (distToSplit > maxDist) {
                    maxDist = distToSplit;
                    *axis = a;
                }
            }
            assert(maxDist != -1);
            return maxDist;
        }

        template <typename Traversal>
        Node *split(Tree& tree, const Space& space, Leaf *leaf, const Key& key, unsigned axis, Traversal& traversal) {
            using T = value_t<Tree>;
            std::array<Distance, batchSize> Q;
            for (std::size_t j=0 ; j<batchSize ; ++j)
                Q[j] = so2::bound(Space::coeff(Get::part(tree.getKey(leaf->elements()[j])), axis));

            // TODO: so2split works for all arrangements.  But we
            // can use an nth_element (e.g., faster) if we are in
            // a bounded subregion.  (Do same with following.)
            [[maybe_unused]] auto [distToSplit, split] = so2split(Q);
            (void)distToSplit; // GCC seems to ignore [[maybe_unused]]

            std::size_t dst0 = 0;
            std::size_t dst1 = batchSize;
            T *elements[batchSize];
            for (std::size_t i=0 ; i<batchSize ; ++i) {
                T* t = leaf->elements() + i;
                if (so2::ccwDist(split, Space::coeff(Get::part(tree.getKey(*t)), axis)) > PI<Distance>) {
                    elements[--dst1] = t;
                } else {
                    elements[dst0++] = t;
                }
            }
            assert(dst0 == dst1);
            assert(dst0 == batchSize/2);

            Leaf *c0 = tree.template allocWithSpace<Leaf>(traversal, tree.keyFn(), elements, elements + batchSize/2);
            Leaf *c1 = tree.template allocWithSpace<Leaf>(traversal, tree.keyFn(), elements + batchSize/2, elements + batchSize);

            return tree.template alloc<SO2Branch>(leaf, axis, split, c0, c1);
        }

        NodePointer& follow(const Space& space, Node *node, unsigned axis, const Key& key) {
            SO2Branch *branch = static_cast<SO2Branch*>(node);
            // TODO: signed dist?
            Distance dist = so2::ccwDist(branch->split(), Space::coeff(key, axis));
            return branch->child(dist > PI<Distance>);
        }

        template <typename Visitor>
        void visit(Visitor& visitor, const Space&, const Node *node, unsigned) {
            const SO2Branch *branch = static_cast<const SO2Branch*>(node);
            visitor(branch->child(0));
            visitor(branch->child(1));
        }


        template <typename Clear>
        void clear(Clear& visitor, const Space&, Node *node, unsigned) {
            SO2Branch *branch = static_cast<SO2Branch*>(node);
            visitor(branch->child(0));
            visitor(branch->child(1));
            visitor.dealloc(branch);
        }
    };
}

#endif
