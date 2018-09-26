// This test case was automatically generated
// generated_so2_7_eigen_vector_float_state_rw_gnat_test.cpp
#include "sampler_so2.hpp"
#include <nigh/gnat.hpp>
#include "test_template.hpp"

TEST(so2_7_eigen_vector_float_state_rw_gnat) {
    using namespace unc::robotics::nigh;
    using namespace unc::robotics::nigh::metric;
    using namespace nigh_test;  
    using Metric = SO2<1>;
    using State = Eigen::Matrix<float, 7, 1>;
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
