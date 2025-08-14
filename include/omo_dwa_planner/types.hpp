// include/omo_dwa_planner/types.hpp
#pragma once
#include <vector>
#include <cstddef>

namespace omo_dwa {

struct Pose2D { double x{0.0}, y{0.0}, yaw{0.0}; };
struct VelPair { double v{0.0}, w{0.0}; };

using Trajectory = std::vector<Pose2D>;
using TrajSet    = std::vector<Trajectory>;

} // namespace omo_dwa