#include <chrono>
#include <iostream>
#include <nlohmann/json.hpp>
#include <thread>
#include <zmq.hpp>

#include "drone.hpp"
#include "environment.hpp"
#include "sensor.hpp"
#include "vec3.hpp"

using json = nlohmann::json;

int main() {
  // 1. Setup ZeroMQ
  zmq::context_t ctx;
  zmq::socket_t publisher(ctx, zmq::socket_type::pub);
  publisher.bind("tcp://*:5555");

  zmq::socket_t subscriber(ctx, zmq::socket_type::sub);
  subscriber.bind("tcp://*:5556"); // Control input
  subscriber.set(zmq::sockopt::subscribe, "");

  // Non-blocking receive for controls
  // In newer cppzmq, we can use flags or poll items. Let's use simple
  // non-blocking recv.
  int timeout = 0;
  subscriber.set(zmq::sockopt::rcvtimeo, timeout);

  // 2. Setup Simulation World
  Drone drone(Vec3(0, 0, 0));
  Environment env;
  Lidar lidar;

  // Add some obstacles (Spheres)
  env.add_obstacle(Vec3(5, 5, 0), 2.0f);
  env.add_obstacle(Vec3(10, 0, 0), 3.0f);
  env.add_obstacle(Vec3(5, -5, 0), 2.0f);
  env.add_obstacle(Vec3(15, 5, 0), 1.5f);

  std::cout << "Simulation Started..." << std::endl;

  // 3. Main Loop (60Hz)
  const int target_fps = 60;
  const std::chrono::milliseconds frame_duration(1000 / target_fps);
  float dt = 1.0f / target_fps;
  int tick = 0;

  while (true) {
    auto start_time = std::chrono::steady_clock::now();

    // --- Input Handling (Non-blocking) ---
    zmq::message_t msg;
    if (subscriber.recv(msg, zmq::recv_flags::dontwait)) {
      try {
        auto j = json::parse(msg.to_string());
        float fx = j.value("fx", 0.0f);
        float fy = j.value("fy", 0.0f);
        float fz = j.value("fz", 0.0f);
        drone.apply_force(Vec3(fx, fy, fz));
      } catch (...) {
        // Ignore malformed JSON
      }
    }

    // --- Physics Update ---
    drone.update(dt);

    // --- Sensor Update ---
    lidar.scan(drone.position, env);

    // --- Serialization & Publish ---
    json j_out;
    j_out["tick"] = tick;
    j_out["time"] = tick * dt;

    // Drone State
    j_out["drone"]["pos"] = {drone.position.x, drone.position.y,
                             drone.position.z};
    j_out["drone"]["vel"] = {drone.velocity.x, drone.velocity.y,
                             drone.velocity.z};

    // Lidar Data
    std::vector<std::vector<float>> points;
    for (const auto &p : lidar.last_scan) {
      points.push_back({p.x, p.y, p.z});
    }
    j_out["lidar"] = points;

    // Obstacles (Static, but sending for visualization simplicity initially)
    // Optimization: Don't send this every frame in real app, but fine for demo
    std::vector<json> obs_list;
    for (const auto &o : env.obstacles) {
      obs_list.push_back({{"pos", {o.position.x, o.position.y, o.position.z}},
                          {"radius", o.radius}});
    }
    j_out["obstacles"] = obs_list;

    std::string json_str = j_out.dump();
    zmq::message_t pub_msg(json_str.size());
    memcpy(pub_msg.data(), json_str.c_str(), json_str.size());
    publisher.send(pub_msg, zmq::send_flags::none);

    // --- FPS Control ---
    auto end_time = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        end_time - start_time);
    if (elapsed < frame_duration) {
      std::this_thread::sleep_for(frame_duration - elapsed);
    }
    tick++;
  }

  return 0;
}
