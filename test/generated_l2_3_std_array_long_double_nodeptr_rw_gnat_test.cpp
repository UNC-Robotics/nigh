// This test case was automatically generated
// generated_l2_3_std_array_long_double_nodeptr_rw_gnat_test.cpp
#include "sampler_lp.hpp"
#include <nigh/gnat.hpp>
#include "test_template.hpp"

TEST(l2_3_std_array_long_double_nodeptr_rw_gnat) {
    using namespace unc::robotics::nigh;
    using namespace unc::robotics::nigh::metric;
    using namespace nigh_test;  
    using Metric = LP<2>;
    using State = std::array<long double, 3>;
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
    runTest<Node<State>*, Space, NodePtrKey, Concurrency, Strategy>(space, N, K);
}
