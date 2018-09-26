// This test case was automatically generated
#include "concurrent_test_template.hpp"
#include <nigh/linear.hpp>
#include <nigh/lp_space.hpp>

TEST(l2_3_long_double_linear) {
    using namespace unc::robotics::nigh;
    using Strategy = Linear;
    using Space = metric::L2Space<long double, 3>;
    nigh_test::runConcurrentTest<Strategy>(Space{});
}
