#include "bench_template.hpp"

int main(int argc, char *argv[]) {
    using namespace unc::robotics::nigh;
    using namespace unc::robotics::nigh::metric;
    using namespace nigh_test;
    using Scalar = double;
    using State = Eigen::Quaternion<Scalar>;
    using Metric = SO3;
    using Concurrency = Concurrent;
    using Strategy = GNAT<>;

    metric::Space<State, Metric> space;
    runBench<State>(argc, argv, space, nigh_test::Identity{}, Concurrency{}, Strategy{});
}

