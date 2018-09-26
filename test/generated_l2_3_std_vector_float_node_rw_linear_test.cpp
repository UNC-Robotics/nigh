// This test case was automatically generated
// generated_l2_3_std_vector_float_node_rw_linear_test.cpp
#include "sampler_lp.hpp"
#include <nigh/linear.hpp>
#include "test_template.hpp"

TEST(l2_3_std_vector_float_node_rw_linear) {
    using namespace unc::robotics::nigh;
    using namespace unc::robotics::nigh::metric;
    using namespace nigh_test;  
    using Metric = LP<2>;
    using State = std::vector<float>;
    using Space = metric::Space<State, Metric>;
    using Concurrency = Concurrent;
    using Strategy = Linear;
    Space space(3);
#ifdef BUILD_TYPE_VALGRIND
    static constexpr std::size_t N = 1000/10;
    static constexpr std::size_t K = 1;
#else
    static constexpr std::size_t N = 1000;
    static constexpr std::size_t K = 50;
#endif
    runTest<Node<State>, Space, NodeKey, Concurrency, Strategy>(space, N, K);
}
