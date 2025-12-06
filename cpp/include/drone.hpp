#pragma once
#include "vec3.hpp"
#include <vector>

class Drone {
public:
  Vec3 position;
  Vec3 velocity;
  Vec3 acceleration;

  // Physical properties
  float mass = 1.0f;
  float drag_coefficient = 0.1f;
  float max_thrust = 20.0f;

  // Control Input (Target Velocity for simplicity in this demo, or Force)
  // Let's use Force control for more "physics" feel
  Vec3 input_forces;

  Drone(Vec3 start_pos = Vec3(0, 0, 0)) : position(start_pos) {}

  void apply_force(Vec3 force) { input_forces += force; }

  void update(float dt) {
    // Gravity
    Vec3 gravity(0, 0, -9.81f * mass);

    // Drag (Air resistance) - simplified linear drag
    Vec3 drag = velocity * -drag_coefficient;

    // Total force
    Vec3 total_force = gravity + drag + input_forces;

    // Newton's Second Law: F = ma -> a = F/m
    acceleration = total_force / mass;

    // Euler Integration (Semi-implicit)
    velocity += acceleration * dt;
    position += velocity * dt;

    // Floor collision (Simple constraint)
    if (position.z < 0) {
      position.z = 0;
      velocity.z = 0;
      if (acceleration.z < 0)
        acceleration.z = 0;
    }

    // Reset input forces for next frame
    input_forces = Vec3(0, 0, 0);
  }
};
