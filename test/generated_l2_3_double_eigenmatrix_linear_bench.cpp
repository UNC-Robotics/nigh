#include "bench_template.hpp"

int main(int argc, char *argv[]) {
    using namespace unc::robotics::nigh;
    using namespace unc::robotics::nigh::metric;
    using namespace nigh_test;
    using Scalar = double;
    using State = Eigen::Matrix<Scalar, 3, 1>;
    using Metric = L2;
    using Concurrency = Concurrent;
    using Strategy = Linear;

    metric::Space<State, Metric> space;
    runBench<State>(argc, argv, space, nigh_test::Identity{}, Concurrency{}, Strategy{});
}

