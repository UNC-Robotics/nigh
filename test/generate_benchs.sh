#!/bin/bash

declare -A CONCURRENCY_TYPES=(
    [rw]="Concurrent"
    [ro]="ConcurrentRead"
    [nt]="NoThreadSafety"
)


scalar="double"
concurrency="rw"
for space in l2_3 so2_5 so3 se3 ; do
    case $space in
        l2*)
            metric="L2"
            dim=${space#*_}
            state="Eigen::Matrix<Scalar, $dim, 1>"
            state_name="eigenmatrix"
            ;;
        so2*)
            metric="SO2<1>"
            dim=${space#*_}
            state="Eigen::Matrix<Scalar, $dim, 1>"
            state_name="eigenmatrix"
            ;;
        so3)
            metric="SO3"
            state="Eigen::Quaternion<Scalar>"
            state_name="eigenquaternion"
            ;;
        se3)
            metric="Cartesian<SO3, L2>"
            state="std::tuple<Eigen::Quaternion<Scalar>, Eigen::Matrix<Scalar, 3, 1>>"
            state_name="tuple"
            ;;
    esac

    for strat in batch_8 linear gnat ; do
        case $strat in
            batch_8) strategy="KDTreeBatch<8>" ;;
            linear) strategy="Linear" ;;
            gnat) strategy="GNAT<>" ;;
        esac
        file="generated_${space}_${scalar}_${state_name}_${strat}_bench.cpp"
        echo "$file"
        cat >"$file" <<EOF
#include "bench_template.hpp"

int main(int argc, char *argv[]) {
    using namespace unc::robotics::nigh;
    using namespace unc::robotics::nigh::metric;
    using namespace nigh_test;
    using Scalar = $scalar;
    using State = $state;
    using Metric = $metric;
    using Concurrency = ${CONCURRENCY_TYPES[$concurrency]};
    using Strategy = $strategy;

    metric::Space<State, Metric> space;
    runBench<State>(argc, argv, space, nigh_test::Identity{}, Concurrency{}, Strategy{});
}

EOF
    done
done
