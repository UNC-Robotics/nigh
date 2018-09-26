// This test case was automatically generated
// generated_l2_3_std_vector_float_nodeptr_rw_batch_8_test.cpp
#include "sampler_lp.hpp"
#include <nigh/kdtree_batch.hpp>
#include "test_template.hpp"

TEST(l2_3_std_vector_float_nodeptr_rw_batch_8) {
    using namespace unc::robotics::nigh;
    using namespace unc::robotics::nigh::metric;
    using namespace nigh_test;  
    using Metric = LP<2>;
    using State = std::vector<float>;
    using Space = metric::Space<State, Metric>;
    using Concurrency = Concurrent;
    using Strategy = KDTreeBatch<8>;
    Space space(3);
#ifdef BUILD_TYPE_VALGRIND
    static constexpr std::size_t N = 5000/10;
    static constexpr std::size_t K = 1;
#else
    static constexpr std::size_t N = 5000;
    static constexpr std::size_t K = 50;
#endif
    runTest<Node<State>*, Space, NodePtrKey, Concurrency, Strategy>(space, N, K);
}
