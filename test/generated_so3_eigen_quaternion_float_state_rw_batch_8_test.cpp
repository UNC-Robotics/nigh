// This test case was automatically generated
// generated_so3_eigen_quaternion_float_state_rw_batch_8_test.cpp
#include "sampler_so3.hpp"
#include <nigh/kdtree_batch.hpp>
#include "test_template.hpp"

TEST(so3_eigen_quaternion_float_state_rw_batch_8) {
    using namespace unc::robotics::nigh;
    using namespace unc::robotics::nigh::metric;
    using namespace nigh_test;  
    using Metric = SO3;
    using State = Eigen::Quaternion<float>;
    using Space = metric::Space<State, Metric>;
    using Concurrency = Concurrent;
    using Strategy = KDTreeBatch<8>;
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
