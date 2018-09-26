// This test case was automatically generated
// generated_linf_7_eigen_vector_double_state_rw_gnat_test.cpp
#include "sampler_lp.hpp"
#include <nigh/gnat.hpp>
#include "test_template.hpp"

TEST(linf_7_eigen_vector_double_state_rw_gnat) {
    using namespace unc::robotics::nigh;
    using namespace unc::robotics::nigh::metric;
    using namespace nigh_test;  
    using Metric = LInf;
    using State = Eigen::Matrix<double, 7, 1>;
    using Space = metric::Space<State, Metric>;
    using Concurrency = Concurrent;
    using Strategy = GNAT<>;
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
