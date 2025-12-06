#pragma once
#include "environment.hpp"
#include "vec3.hpp"
#include <cmath>
#include <vector>

struct LidarPoint {
  float x, y, z;
};

class Lidar {
public:
  int num_rays = 360;
  float max_range = 10.0f;
  std::vector<LidarPoint> last_scan;

  void scan(Vec3 position, Environment &env) {
    last_scan.clear();

    // 2D Lidar scan in XY plane for now (simplest for avoiding ground clutter
    // in demo) Can easily expand to 3D spherical scan
    for (int i = 0; i < num_rays; ++i) {
      float angle = (float)i / num_rays * 2.0f * M_PI;
      float dx = std::cos(angle);
      float dy = std::sin(angle);
      Vec3 dir(dx, dy, 0);

      float dist = env.intersect_ray(position, dir, max_range);

      if (dist > 0) {
        Vec3 hit_point = position + dir * dist;
        last_scan.push_back({hit_point.x, hit_point.y, hit_point.z});
      }
    }
  }
};
