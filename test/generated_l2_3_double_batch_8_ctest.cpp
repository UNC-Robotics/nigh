// This test case was automatically generated
#include "concurrent_test_template.hpp"
#include <nigh/kdtree_batch.hpp>
#include <nigh/lp_space.hpp>

TEST(l2_3_double_batch_8) {
    using namespace unc::robotics::nigh;
    using Strategy = KDTreeBatch<8>;
    using Space = metric::L2Space<double, 3>;
    nigh_test::runConcurrentTest<Strategy>(Space{});
}
