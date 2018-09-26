// This test case was automatically generated
// generated_se3r_tuple_double_state_rw_gnat_test.cpp
#include "sampler_lp.hpp"
#include "sampler_so3.hpp"
#include "sampler_cartesian.hpp"
#include <nigh/gnat.hpp>
#include "test_template.hpp"

TEST(se3r_tuple_double_state_rw_gnat) {
    using namespace unc::robotics::nigh;
    using namespace unc::robotics::nigh::metric;
    using namespace nigh_test;  
    using Metric = Cartesian<LP<2>, SO3>;
    using State = std::tuple<Eigen::Matrix<double, 3, 1>, Eigen::Quaternion<double>>;
    using Space = metric::Space<State, Metric>;
    using Concurrency = Concurrent;
    using Strategy = GNAT<>;
    Space space;
#ifdef BUILD_TYPE_VALGRIND
    static constexpr std::size_t N = 5000/10;
    static constexpr std::size_t K = 1;
#else
    static constexpr std::size_t N = 5000;
    static constexpr std::size_t K = 50;
#endif
    runTest<State, Space, Identity, Concurrency, Strategy>(space, N, K);
}
