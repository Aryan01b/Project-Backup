import rclpy
from rclpy.node import Node
import rclpy.service 
from geometry_msgs.msg import Twist, Pose
from turtlesim.msg import Pose
from turtlesim.srv import Kill, Spawn
import random
import math

class TurtlebotController(Node):
    def __init__(self):
        super().__init__('one')
        
        # Spawn Turtle at random position
        self.turtle_kill = self.create_client(Kill, '/kill')
        self.turtle_spawn = self.create_client(Spawn, '/spawn')
        
        while not self.turtle_kill.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting for /kill service...')
        while not self.turtle_spawn.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting for /spawn service...')
            
        # 2D Goal Movement
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.subscriber_ = self.create_subscription(Pose, 'turtle1/pose', self.odom_callback, 10)
        
        self.goal = Pose()
        self.goal.x = 1.0
        self.goal.y = 1.0
    
        # (0, 1, 2)
        # (kp, ki, kd)
        self.linear_gain = [2, 0, 0]
        # self.angular_gain = [10, 0.05, 0.001]
        self.angular_gain = [10, 0, 0]
        
        # error_sum, prev_error
        self.linear_error = [0, 0]
        self.angular_error = [0, 0]
        
        
        
    def odom_callback(self, current_pose):
        error_x = self.goal.x - current_pose.x
        error_y = self.goal.y - current_pose.y
        
        lin_error = math.sqrt(error_x**2 + error_y**2)
        ang_error = math.atan2(error_y, error_x) - current_pose.theta
        
        # PID linear
        self.linear_error[0] += lin_error
        lin_error_diff = lin_error - self.linear_error[1]
        self.linear_error[1] = lin_error
        # PID angular
        self.angular_error[0] += ang_error
        ang_error_diff = ang_error - self.angular_error[1]
        self.angular_error[1] = ang_error
        
        lin_vel = self.linear_gain[0] * lin_error + self.linear_gain[1] * self.linear_error[0] + self.linear_gain[2] * lin_error_diff
        ang_vel = self.angular_gain[0] * ang_error + self.angular_gain[1] * self.angular_error[0] + self.angular_gain[2] * ang_error_diff
        
        # Publish Velocity Commands
        cmd_vel = Twist()
        cmd_vel.linear.x = lin_vel
        cmd_vel.angular.z = ang_vel
        self.publisher_.publish(cmd_vel)
        
        if lin_error <= 0.5:
            self.publisher_.publish(Twist())

    def kill_turtle(self):
        req = Kill.Request()
        req.name = 'turtle1'
        future = self.turtle_kill.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def spawn_turtle(self):
        req = Spawn.Request()
        req.name = 'turtle1'
        req.x = random.uniform(2.0, 11.0)
        req.y = random.uniform(2.0, 11.0)
        req.theta = random.uniform(0, 2*math.pi)
        future = self.turtle_spawn.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()        
         
def main(args=None):
    rclpy.init(args=args)
    node = TurtlebotController()
    
    node.kill_turtle()
    node.spawn_turtle()
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__=='__main__':
    main()