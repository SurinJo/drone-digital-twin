#pragma once
#include "vec3.hpp"
#include <vector>

struct Obstacle {
  Vec3 position;
  float radius;
};

class Environment {
public:
  std::vector<Obstacle> obstacles;

  void add_obstacle(Vec3 pos, float r) { obstacles.push_back({pos, r}); }

  // Returns distance to intersection, or -1 if no intersection
  float intersect_ray(Vec3 origin, Vec3 direction, float max_range) {
    float min_dist = max_range + 1.0f;
    bool hit = false;

    // Naive O(N) check - good enough for demo
    for (const auto &obs : obstacles) {
      Vec3 oc = origin - obs.position;
      float a = direction.dot(direction);
      float b = 2.0f * oc.dot(direction);
      float c = oc.dot(oc) - obs.radius * obs.radius;
      float discriminant = b * b - 4 * a * c;

      if (discriminant > 0) {
        float sqrt_disc = std::sqrt(discriminant);
        float t1 = (-b - sqrt_disc) / (2 * a);
        float t2 = (-b + sqrt_disc) / (2 * a);

        float t = -1.0f;
        if (t1 > 0 && t2 > 0)
          t = std::min(t1, t2);
        else if (t1 > 0)
          t = t1;
        else if (t2 > 0)
          t = t2;

        if (t > 0 && t < min_dist) {
          min_dist = t;
          hit = true;
        }
      }
    }

    if (hit && min_dist <= max_range) {
      return min_dist;
    }
    return -1.0f;
  }
};
