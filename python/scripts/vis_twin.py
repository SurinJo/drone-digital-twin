import zmq
import rerun as rr
import numpy as np
import json
import time

def main():
    # 1. Setup Rerun
    rr.init("Drone Digital Twin", spawn=True)
    
    # 2. Setup ZeroMQ
    context = zmq.Context()
    
    # Subscriber for State (from C++)
    subscriber = context.socket(zmq.SUB)
    subscriber.connect("tcp://localhost:5555")
    subscriber.setsockopt_string(zmq.SUBSCRIBE, "")

    # Publisher for Control (to C++)
    publisher = context.socket(zmq.PUB)
    publisher.connect("tcp://localhost:5556")

    print("Python Digital Twin Started...")

    while True:
        try:
            # Non-blocking receive
            try:
                msg = subscriber.recv(flags=zmq.NOBLOCK)
                data = json.loads(msg)
                
                # --- Visualization ---
                # 1. Drone Position
                pos = data["drone"]["pos"]
                rr.log("world/drone", rr.Points3D([pos], radii=0.3, colors=[255, 0, 0]))
                
                # 2. Lidar
                lidar_points = data.get("lidar", [])
                if lidar_points:
                    rr.log("world/lidar", rr.Points3D(lidar_points, radii=0.05, colors=[0, 255, 0]))
                
                # 3. Obstacles
                # Log once or check if they changed (Simplicity: Log every time for now)
                obstacles = data.get("obstacles", [])
                for i, obs in enumerate(obstacles):
                    rr.log(f"world/obstacles/{i}", rr.Points3D([obs["pos"]], radii=obs["radius"], colors=[200, 200, 200]))

                # --- Simple Avoidance Logic (The "Brain") ---
                # Logic: If any lidar point is within 2m in front (x+ direction), apply side force
                drone_pos = np.array(pos)
                min_dist = 999.0
                obs_dir = None
                
                for p in lidar_points:
                    p_vec = np.array(p)
                    dist = np.linalg.norm(p_vec - drone_pos)
                    if dist < min_dist:
                        min_dist = dist
                        obs_dir = p_vec - drone_pos
                
                fx, fy, fz = 0, 0, 0
                
                # Simple P-controller to keep height at 2m
                target_z = 2.0
                fz = (target_z - pos[2]) * 5.0
                
                # Forward motion
                fx = 2.0 # Constant forward force
                
                if min_dist < 4.0 and obs_dir is not None:
                     # Obstacle avoidance: push away from obstacle
                     # Normalize obs_dir
                     obs_dir = obs_dir / np.linalg.norm(obs_dir)
                     
                     # If obstacle is generated from forward-ish direction
                     if obs_dir[0] > 0.5: # Mostly in front
                         fy = 10.0 # Dodge Y
                         fx = 0.0 # Slow down

                # Send Command
                cmd = {"fx": fx, "fy": fy, "fz": fz}
                publisher.send_string(json.dumps(cmd))
                
            except zmq.Again:
                pass # No message yet

        except Exception as e:
            print(f"Error: {e}")
            time.sleep(0.1)

if __name__ == "__main__":
    main()
