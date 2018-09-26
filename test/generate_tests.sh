#!/bin/bash

declare -A METRIC_TYPES=(
    [l1]="LP<1>"
    [l2]="LP<2>"
    [linf]="LInf"
    [so3]="SO3"
    [so2]="SO2<1>"
    [se3]="Cartesian<SO3, LP<2>>"
    [se3r]="Cartesian<LP<2>, SO3>"
)
declare -A VALUE_TYPES=(
    [node]="Node<State>"
    [state]="State"
    [nodeptr]="Node<State>*"
    [stateptr]="State*"
)
declare -A MEMBER_TYPES=(
    [node]="NodeKey"
    [state]="Identity"
    [nodeptr]="NodePtrKey"
    [stateptr]="Deref"
)
declare -A CONCURRENCY_TYPES=(
    [rw]="Concurrent"
    [ro]="ConcurrentRead"
    [nt]="NoThreadSafety"
)
declare -A STRATEGY_TYPES=(
    [linear]=Linear
    [batch_8]="KDTreeBatch<8>"
    [batch_16]="KDTreeBatch<16>"
    [gnat]="GNAT<>"
)
declare -A SCALAR_TYPES=(
    [float]=float
    [double]=double
    [long_double]="long double"
)
count=0
# TODO: l2_1 (scalar) so2_1 (scalar) so2_3 (joint chain) se3 se3x2 se3wt52
for space in l1_2 l2_3 linf_7 so2_1 so2_7 so3 se3 se3r; do
    declare -a includes
    case $space in
        l*)
            metric=${space%_*}
            dim=${space#*_}
            includes=("sampler_lp")
            ;;
        so2*)
            metric=${space%_*}
            dim=${space#*_}
            includes=("sampler_so2")
            ;;
        so3)
            metric=$space
            dim=4
            includes=("sampler_so3")
            ;;
        se3*)
            metric=$space
            dim=-1
            includes=("sampler_lp" "sampler_so3" "sampler_cartesian")
            ;;
    esac

    if [[ $space == l2_3 ]] ; then
        state_list="eigen_vector eigen_array std_vector std_array"
        value_list="node state nodeptr stateptr"
    elif [[ $space = so2_1 ]] ; then
        # TODO: more...
        state_list="scalar"
        value_list="state"
    elif [[ $metric = so3 ]] ; then
        state_list="eigen_quaternion eigen_vector"
        value_list="state"
    elif [[ $metric = se3 ]] ; then
        state_list="tuple pair se3"
        value_list="state"
    elif [[ $metric = se3r ]] ; then
        state_list="tuple"
        value_list="state"
    else
        state_list="eigen_vector"
        value_list="state"
    fi

    for state in $state_list ; do
        space_args=""
        for scalar in float double long_double ; do
            STYPE="${SCALAR_TYPES[$scalar]}"
            case $state in
                eigen_quaternion) store="Eigen::Quaternion<$STYPE>" ;;
                eigen_vector)     store="Eigen::Matrix<$STYPE, $dim, 1>" ;;
                eigen_array)      store="Eigen::Array<$STYPE, $dim, 1>" ;;
                std_vector)
                    store="std::vector<$STYPE>"
                    space_args="($dim)"
                    ;;
                std_array)        store="std::array<$STYPE, $dim>" ;;
                scalar)           store="$STYPE" ;;
                se3)              store="nigh_test::SE3State<$STYPE>" ;;
                tuple|pair)
                    case $metric in
                        se3)  store="std::$state<Eigen::Quaternion<$STYPE>, Eigen::Matrix<$STYPE, 3, 1>>" ;;
                        se3r) store="std::$state<Eigen::Matrix<$STYPE, 3, 1>, Eigen::Quaternion<$STYPE>>" ;;
                    esac
                    ;;
            esac
                    
            for value in $value_list ; do
                for strategy in batch_8 linear gnat ; do
                    if [[ $space = l2_3 && $state = eigen_vector && $value = state ]] ; then
                        concurrency_list="rw ro nt"
                    else
                        concurrency_list="rw"
                    fi

                    case $strategy in
                        linear)
                            strategy_include="linear.hpp"
                            ;;
                        batch_*)
                            strategy_include="kdtree_batch.hpp"
                            ;;
                        gnat)
                            strategy_include="gnat.hpp"
                            ;;
                    esac

                    if [[ $dim -gt 6 || $strategy == linear ]] ; then
                        N=1000
                    else
                        N=5000
                    fi
                    
                    for concurrency in $concurrency_list ; do
                        name="${space}_${state}_${scalar}_${value}_${concurrency}_${strategy}"
                        file="generated_${name}_test.cpp"
                        cat > "$file" <<EOF
// This test case was automatically generated
// generated_${name}_test.cpp
$(printf "#include \"%s.hpp\"\n" "${includes[@]}")
#include <nigh/${strategy_include}>
#include "test_template.hpp"

TEST($name) {
    using namespace unc::robotics::nigh;
    using namespace unc::robotics::nigh::metric;
    using namespace nigh_test;  
    using Metric = ${METRIC_TYPES[$metric]};
    using State = $store;
    using Space = metric::Space<State, Metric>;
    using Concurrency = ${CONCURRENCY_TYPES[$concurrency]};
    using Strategy = ${STRATEGY_TYPES[$strategy]};
    Space space${space_args};
#ifdef BUILD_TYPE_VALGRIND
    static constexpr std::size_t N = ${N}/10;
    static constexpr std::size_t K = 1;
#else
    static constexpr std::size_t N = ${N};
    static constexpr std::size_t K = 50;
#endif
    runTest<${VALUE_TYPES[$value]}, Space, ${MEMBER_TYPES[$value]}, Concurrency, Strategy>(space, N, K);
}
EOF
                        count=$((count + 1))
                    done
                done
            done
        done
    done
done

# for metric in so3 ; do
#     for state in eigen_quaternion eigen_vector ; do
#         for scalar in float double long_double ; do
#             case $state in
#                 eigen_quaternion)
#                     store="Eigen::Quaternion<${SCALAR_TYPES[$scalar]}>";
#                     ;;
#                 eigen_vector)
#                     store="Eigen::Matrix<${SCALAR_TYPES[$scalar]}, 4>";
#                     ;;
#             esac
#             for value in state ; do
#                 for strategy in batch_8 linear ; do
#                     for concurrency in rw ; do
#                         name="${metric}_${state}_${scalar}_${value}_${concurrency}_${strategy}"
#                         echo "TEST($name) {"
#                         echo "    using namespace unc::robotics::nigh;"
#                         echo "    using Metric = ${METRIC_TYPES[$metric]};"
#                         echo "    using State = $store;"
#                         echo "    using Concurrency = ${CONCURRENCY_TYPES[$concurrency]};"
#                         echo "    using Strategy = ${STRATEGY_TYPES[$strategy]};"
#                         echo "    runTest<${VALUE_TYPES[$value]}, Metric, ${MEMBER_TYPES[$value]}, Concurrency, Strategy>();"
#                         echo "}"
#                         count=$((count + 1))
#                     done
#                 done
#             done
#         done
#     done
# done

echo "Test count: $count"

# for space in so3 se3 se3_scaled ; do
#     for store in eigen_quaternion eigen_vector eigen_array std_vector std_array ; do

#     done
# done
