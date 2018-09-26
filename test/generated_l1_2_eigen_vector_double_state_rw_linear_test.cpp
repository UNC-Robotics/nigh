// This test case was automatically generated
// generated_l1_2_eigen_vector_double_state_rw_linear_test.cpp
#include "sampler_lp.hpp"
#include <nigh/linear.hpp>
#include "test_template.hpp"

TEST(l1_2_eigen_vector_double_state_rw_linear) {
    using namespace unc::robotics::nigh;
    using namespace unc::robotics::nigh::metric;
    using namespace nigh_test;  
    using Metric = LP<1>;
    using State = Eigen::Matrix<double, 2, 1>;
    using Space = metric::Space<State, Metric>;
    using Concurrency = Concurrent;
    using Strategy = Linear;
    Space space;
#ifdef BUILD_TYPE_VALGRIND
    static constexpr std::size_t N = 1000/10;
    static constexpr std::size_t K = 1;
#else
    static constexpr std::size_t N = 1000;
    static constexpr std::size_t K = 50;
#endif
    runTest<State, Space, Identity, Concurrency, Strategy>(space, N, K);
}
