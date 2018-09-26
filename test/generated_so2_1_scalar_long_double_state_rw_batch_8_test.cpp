// This test case was automatically generated
// generated_so2_1_scalar_long_double_state_rw_batch_8_test.cpp
#include "sampler_so2.hpp"
#include <nigh/kdtree_batch.hpp>
#include "test_template.hpp"

TEST(so2_1_scalar_long_double_state_rw_batch_8) {
    using namespace unc::robotics::nigh;
    using namespace unc::robotics::nigh::metric;
    using namespace nigh_test;  
    using Metric = SO2<1>;
    using State = long double;
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
