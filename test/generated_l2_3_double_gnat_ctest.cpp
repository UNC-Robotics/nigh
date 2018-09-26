// This test case was automatically generated
#include "concurrent_test_template.hpp"
#include <nigh/gnat.hpp>
#include <nigh/lp_space.hpp>

TEST(l2_3_double_gnat) {
    using namespace unc::robotics::nigh;
    using Strategy = GNAT<>;
    using Space = metric::L2Space<double, 3>;
    nigh_test::runConcurrentTest<Strategy>(Space{});
}
