import zmq
import rerun as rr
import numpy as np
import json
import time
import sys
import select
import termios
import tty
import atexit

# --- Non-blocking Keyboard Input Class ---
class KBHit:
    def __init__(self):
        self.fd = sys.stdin.fileno()
        self.new_term = termios.tcgetattr(self.fd)
        self.old_term = termios.tcgetattr(self.fd)
        self.new_term[3] = (self.new_term[3] & ~termios.ICANON & ~termios.ECHO)
        termios.tcsetattr(self.fd, termios.TCSAFLUSH, self.new_term)
        atexit.register(self.set_normal_term)

    def set_normal_term(self):
        termios.tcsetattr(self.fd, termios.TCSAFLUSH, self.old_term)

    def getch(self):
        return sys.stdin.read(1)

    def kbhit(self):
        dr, dw, de = select.select([sys.stdin], [], [], 0)
        return dr != []

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
    print("=========================================")
    print("Controls:")
    print("  [M] Toggle Auto/Manual Mode")
    print("  [W/S] Forward/Backward Force")
    print("  [A/D] Left/Right Force")
    print("  [Space] Up Force (Lift)")
    print("  [Shift] Down Force")
    print("  [Q] Quit")
    print("=========================================")

    kb = KBHit()
    manual_mode = False
    
    while True:
        try:
            # --- Input Handling ---
            key = None
            if kb.kbhit():
                key = kb.getch()
                if key == 'q':
                    break
                elif key == 'm':
                    manual_mode = not manual_mode
                    state_str = "MANUAL" if manual_mode else "AUTO"
                    print(f"Switched to {state_str} MODE")

            # --- State Receive ---
            try:
                msg = subscriber.recv(flags=zmq.NOBLOCK)
                data = json.loads(msg)
                
                # Visualization
                pos = data["drone"]["pos"]
                rr.log("world/drone", rr.Points3D([pos], radii=0.3, colors=[255, 0, 0]))
                
                lidar_points = data.get("lidar", [])
                if lidar_points:
                    rr.log("world/lidar", rr.Points3D(lidar_points, radii=0.05, colors=[0, 255, 0]))
                
                obstacles = data.get("obstacles", [])
                for i, obs in enumerate(obstacles):
                    rr.log(f"world/obstacles/{i}", rr.Points3D([obs["pos"]], radii=obs["radius"], colors=[200, 200, 200]))

                # --- Control Logic ---
                fx, fy, fz = 0, 0, 0
                
                if manual_mode:
                    # Manual Control
                    # Reading current key state is tricky in terminal without event loop
                    # We modify this to be "pulse" based or state based if we had a key-down listener.
                    # With getch(), we only get key press events.
                    # Simplification: Apply force momentarily when key is detected in loop (might need continuous press)
                    # BETTER APPROACH: Since getch consumes buffer, we can't 'hold' keys easily in this simple loop
                    # without repeat rate.
                    # Let's trust the OS key repeat for continuous movement.
                    
                    if key == 'w': fx = 20.0
                    if key == 's': fx = -20.0
                    if key == 'a': fy = 20.0
                    if key == 'd': fy = -20.0
                    if key == ' ': fz = 20.0 # Space
                    
                    # Gravity compensation (always apply some lift unless down is pressed?)
                    # Let's make it raw physics control.
                    # "Hover" assistance:
                    if fz == 0: fz = 9.81 * 1.0 # 1.0 is mass, roughly cancel gravity
                    if key == 'Z': fz = 0 # Shift (ish) -> Drop
                    
                else:
                    # Auto (Avoidance) Logic
                    drone_pos = np.array(pos)
                    min_dist = 999.0
                    obs_dir = None
                    
                    for p in lidar_points:
                        p_vec = np.array(p)
                        dist = np.linalg.norm(p_vec - drone_pos)
                        if dist < min_dist:
                            min_dist = dist
                            obs_dir = p_vec - drone_pos
                    
                    target_z = 2.0
                    fz = (target_z - pos[2]) * 5.0
                    fx = 2.0 
                    
                    if min_dist < 4.0 and obs_dir is not None:
                         obs_dir = obs_dir / np.linalg.norm(obs_dir)
                         if obs_dir[0] > 0.5:
                             fy = 10.0
                             fx = 0.0

                # Send Command
                cmd = {"fx": fx, "fy": fy, "fz": fz}
                publisher.send_string(json.dumps(cmd))
                
            except zmq.Again:
                pass 

        except Exception as e:
            print(f"Error: {e}")
            time.sleep(0.1)

    print("Exiting...")
    kb.set_normal_term()

if __name__ == "__main__":
    main()
