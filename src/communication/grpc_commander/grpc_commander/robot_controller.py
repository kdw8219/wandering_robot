import json
import queue
import threading
from typing import Optional

from geometry_msgs.msg import Twist

class RobotController():
    def __init__(self, queue: queue.Queue, cmd_vel_pub):
        self.queue = queue
        self.cmd_vel_pub = cmd_vel_pub
        self.linear_speed = 0.2
        self.angular_speed = 0.5
        self.worker = threading.Thread(target = self.workingThread)
        self.stop_event = threading.Event()
    
    def __del__(self):
        self.stop_event.set()
        self.worker.join()
        
    def run(self):
        self.worker.start()
        
    def workingThread(self):
        print("Robot Controller thread start!")
        while not self.stop_event.is_set():
            try:
                item = self.queue.get(timeout = 0.1) #block 아님?

                if isinstance(item, bytes):
                    item = item.decode("utf-8")
                if isinstance(item, str):
                    try:
                        item = json.loads(item)
                    except json.JSONDecodeError:
                        print(f"Invalid queue item (not JSON): {item}")
                        self.queue.task_done()
                        continue

                command = item.get("command") if isinstance(item, dict) else None
                print(f"get Item!{command}")

                # Accept direct /cmd_vel-style JSON.
                if isinstance(item, dict) and "linear" in item and "angular" in item:
                    self.publish_twist(item)
                    self.queue.task_done()
                    continue

                if command == "MOVE":
                    move = item.get("move", {}) if isinstance(item, dict) else {}
                    direction = str(move.get("direction", "")).lower()
                    speed = move.get("speed", None)
                    self.handle_move(direction, speed)
                elif command in ("STOP", "EMERGENCY_STOP"):
                    self.publish_twist({"linear": {"x": 0.0, "y": 0.0, "z": 0.0},
                                        "angular": {"x": 0.0, "y": 0.0, "z": 0.0}})
                elif command == "SET_SPEED":
                    set_speed = item.get("set_speed", {}) if isinstance(item, dict) else {}
                    speed = set_speed.get("speed", None)
                    if isinstance(speed, (int, float)):
                        self.linear_speed = float(speed)
                else:
                    if command:
                        print(f"Unsupported command: {command}")
                
                self.queue.task_done()
            except queue.Empty:
                pass

    def handle_move(self, direction: str, speed: Optional[float]):
        lin = float(speed) if isinstance(speed, (int, float)) else self.linear_speed
        ang = float(speed) if isinstance(speed, (int, float)) else self.angular_speed

        twist = {
            "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": 0.0},
        }

        if direction in ("forward", "front", "fwd"):
            twist["linear"]["x"] = lin
        elif direction in ("backward", "back", "bwd"):
            twist["linear"]["x"] = -lin
        elif direction in ("left", "turn_left", "rotate_left"):
            twist["angular"]["z"] = ang
        elif direction in ("right", "turn_right", "rotate_right"):
            twist["angular"]["z"] = -ang
        elif direction in ("stop", ""):
            pass
        else:
            print(f"Unknown MOVE direction: {direction}")
            return

        self.publish_twist(twist)

    def publish_twist(self, data: dict):
        if self.cmd_vel_pub is None:
            print("cmd_vel publisher is not set.")
            return

        msg = Twist()
        linear = data.get("linear", {})
        angular = data.get("angular", {})
        msg.linear.x = float(linear.get("x", 0.0))
        msg.linear.y = float(linear.get("y", 0.0))
        msg.linear.z = float(linear.get("z", 0.0))
        msg.angular.x = float(angular.get("x", 0.0))
        msg.angular.y = float(angular.get("y", 0.0))
        msg.angular.z = float(angular.get("z", 0.0))
        self.cmd_vel_pub.publish(msg)
