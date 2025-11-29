import rclpy
from rclpy.node import Node
import asyncio
import threading
from std_msgs.msg import String
import json
import random
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from concurrent.futures import ThreadPoolExecutor

class dummy_status_node(Node):
    
    def __init__(self):
        super().__init__('dummy_status_node')
        
        self.status = ['IDLE', 'MOVING', 'WAITING', 'CHARGING']
        self.current_status = self.status[0]
        self.battery_decreasing = [0.2, 0.01, 0.1]
        self.battery_increasing = [1, 2, 3]
        self.error = "None"
        self.loop = asyncio.new_event_loop()
        
        self.loop_thread = threading.Thread(target=self.start_async, daemon=True)
        self.loop_thread.start()
        
        self.status_pub = self.create_publisher(String, '/robot/status', qos_profile_sensor_data)
        self.pos_pub = self.create_publisher(String, '/robot/pos', qos_profile_sensor_data)
        
        self.battery = 100.0
        
        self.create_timer(3.0, self.run_timer)
        self.executor = ThreadPoolExecutor(max_workers=4)
        
        # subscribe gazebo odometry
        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos_profile=qos_profile_sensor_data
        )
        
        self.robot_state = {
            "x": 0,
            "y": 0,
            "z": 0,
            "orientation": {
                "x": 0, "y": 0, "z": 0, "w": 0
            },
            "linear_speed": 0,
            "angular_speed": 0,
        }
        
        return
    
    def run_timer(self):
        asyncio.run_coroutine_threadsafe(self.pub_dummy(), self.loop)
        return
    
    async def pub_dummy(self):
        
        self.battery -= self.battery_decreasing[int(random.random()) % 3]
        
        if self.current_status == self.status[3] and self.battery < 90.0:
            self.battery += self.battery_increasing[int(random.random()) % 3]
            
        elif self.current_status == self.status[3] and self.battery >=90.0:
            self.current_status = self.status[0]
            
        elif self.battery < 20.0:
            self.current_status = self.status[3]
            
        else:
            pick = (int(random.random()) % 2) + 1
            print(pick)
            self.current_status = self.status[pick] # 1 or 2
            
            
        if (int(random.random()) % 100) == 99:
            self.error = "Error"
        
        data = {
            "battery": round(self.battery, 4),
            "status": self.current_status,
            "error": self.error
        }
        
        msg = String()
        msg.data = json.dumps(data)
        
        await self.status_pub.publish(msg)
        
        self.get_logger().info(f"Published: {msg.data}")
        
        return
    
    def process_odom(self, msg):
        
        # position
        self.robot_state["x"] = msg.pose.pose.position.x
        self.robot_state["y"] = msg.pose.pose.position.y
        self.robot_state["z"] = msg.pose.pose.position.z
        
        # orientation
        self.robot_state["orientation"] = {
            "x": msg.pose.pose.orientation.x,
            "y": msg.pose.pose.orientation.y,
            "z": msg.pose.pose.orientation.z,
            "w": msg.pose.pose.orientation.w,
        }

        # speed
        self.robot_state["linear_speed"] = msg.twist.twist.linear.x
        self.robot_state["angular_speed"] = msg.twist.twist.angular.z

        #get_logger().info(f"Robot pose: {self.robot_state}")
        
        pos = String()
        pos.data = json.dumps(self.robot_state)
        
        self.pos_pub.publish(pos)
    
    def odom_callback(self, msg: Odometry):
        self.executor.submit(self, msg)
        
    
    def start_async(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()
        return
    
    async def async_shutdown(self):
        if self.client:
            await self.client.close()
            self.get_logger().info("HTTP Session Closed.")
        self.loop.stop()

    def destroy_node(self):
        future = asyncio.run_coroutine_threadsafe(self.async_shutdown(), self.loop)
        future.result()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    dummy_stat = dummy_status_node()
    
    rclpy.spin(dummy_stat)
    
    dummy_stat.destroy_node()
    rclpy.shutdown()
    
    return

if __name__ == '___main___':
    main()

