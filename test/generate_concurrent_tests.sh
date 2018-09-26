#!/bin/bash

count=0

for space in l2_3 ; do
    for scalar in float double long_double ; do
        scalar_type=$scalar
        if [[ $scalar == long_double ]] ; then
            scalar_type="long double"
        fi

        case $space in
            l2_*)
                dim=${space#*_}
                metric_space="L2Space<${scalar_type}, ${dim}>"
                space_include="lp_space"
                ;;
        esac

        for strategy in batch_8 linear gnat ; do
            N=10000
            strategy_include=$strategy;
            case $strategy in
                batch_8)
                    strategy_type="KDTreeBatch<8>"
                    strategy_include="kdtree_batch"
                    ;;
                linear)
                    strategy_type=Linear
                    N=1000
                    ;;
                gnat)
                    strategy_type="GNAT<>"
                    ;;
            esac
            name="${space}_${scalar}_${strategy}"
            file="generated_${name}_ctest.cpp"
            cat > "$file" <<EOF
// This test case was automatically generated
#include "concurrent_test_template.hpp"
#include <nigh/${strategy_include}.hpp>
#include <nigh/metric/${space_include}.hpp>

TEST($name) {
    using namespace unc::robotics::nigh;
    using Strategy = ${strategy_type};
    using Space = metric::${metric_space};
    nigh_test::runConcurrentTest<Strategy>(Space{});
}
EOF
            count=$((count + 1))
        done
    done
done

echo "Test count: $count"

