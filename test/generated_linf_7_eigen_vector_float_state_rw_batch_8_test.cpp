// This test case was automatically generated
// generated_linf_7_eigen_vector_float_state_rw_batch_8_test.cpp
#include "sampler_lp.hpp"
#include <nigh/kdtree_batch.hpp>
#include "test_template.hpp"

TEST(linf_7_eigen_vector_float_state_rw_batch_8) {
    using namespace unc::robotics::nigh;
    using namespace unc::robotics::nigh::metric;
    using namespace nigh_test;  
    using Metric = LInf;
    using State = Eigen::Matrix<float, 7, 1>;
    using Space = metric::Space<State, Metric>;
    using Concurrency = Concurrent;
    using Strategy = KDTreeBatch<8>;
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
