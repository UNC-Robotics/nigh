// This test case was automatically generated
// generated_l2_3_eigen_vector_double_state_nt_batch_8_test.cpp
#include "sampler_lp.hpp"
#include <nigh/kdtree_batch.hpp>
#include "test_template.hpp"

TEST(l2_3_eigen_vector_double_state_nt_batch_8) {
    using namespace unc::robotics::nigh;
    using namespace unc::robotics::nigh::metric;
    using namespace nigh_test;  
    using Metric = LP<2>;
    using State = Eigen::Matrix<double, 3, 1>;
    using Space = metric::Space<State, Metric>;
    using Concurrency = NoThreadSafety;
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
