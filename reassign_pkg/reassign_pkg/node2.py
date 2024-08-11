import rclpy
from rclpy.node import Node
import rclpy.service 
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Kill, Spawn
from turtlesim.action import RotateAbsolute
from rclpy.callback_groups import ReentrantCallbackGroup
import random
import math

class TurtlebotController(Node):
    def __init__(self):
        super().__init__('two_action')
        
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
        self.action_client_ = ActionClient(self, RotateAbsolute, 'turtle1/rotate_absolute', callback_group=ReentrantCallbackGroup())
        self.timer = self.create_timer(0.1, self.grid)
        
        self.goal = Pose()
        self.goal.x = 1.0
        self.goal.y = 1.0
        
        self.current = Pose()
        self.is_rotating = False
    
        #Grid Parameters
        self.count = 0
        self.temp = 1.0
        self.a = 1.0
        self.b = 9.0
        self.theta_list = (0.0, math.pi/2 , math.pi, math.pi/2)
    
        # (kp, ki, kd)
        self.linear_gain = [2, 0, 0]
        # self.angular_gain = [10, 0.05, 0.001]
        self.angular_gain = [10, 0, 0]
        
        # error_sum, prev_error
        self.linear_error = [0, 0]
        self.angular_error = [0, 0]
        
    def grid_points(self):
        if self.count%2 == 0:
            self.goal.x = self.b
            self.goal.y = self.temp
            self.a, self.b = self.b, self.a
            self.temp += 2
        else:
            self.goal.x = self.a
            self.goal.y = self.temp
        # self.goal.theta = self.theta_list[(self.count % 4)-1]
        self.count += 1
        self.get_logger().info(f"({self.goal.x},{self.goal.y}, {self.goal.theta})")
        
    def send_goal(self):
        if not self.action_client_.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server not available after waiting')
            self.is_rotating = False
            return
        goal_msg = RotateAbsolute.Goal()
        # goal_msg.theta = self.goal.theta
        goal_msg.theta = self.theta_list[(self.count % 4)]
        
        self.get_logger().info(f'Sending goal to rotate to {self.theta_list[(self.count % 4)-1]} radians...')
        
        # Send the goal
        self.action_client_.send_goal_async(
            goal_msg, 
            feedback_callback=self.feedback_callback
        ).add_done_callback(self.goal_response_callback)
    
    def feedback_callback(self, feedback):
        self.get_logger().info(f'Received feedback: {feedback.feedback}')
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            self.is_rotating = False
            return
        
        self.get_logger().info('Goal accepted')
        goal_handle.get_result_async().add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result}')
        self.is_rotating = False
        # rclpy.shutdown()
        
    def odom_callback(self, current_pose):
        self.current = current_pose
        
    def grid(self):
        # Check if the turtle is currently rotating
        if self.is_rotating:
            self.get_logger().info('Waiting for rotation to complete...')
            return
        error_x = self.goal.x - self.current.x
        error_y = self.goal.y - self.current.y
        
        lin_error = math.sqrt(error_x**2 + error_y**2)
        ang_error = math.atan2(error_y, error_x) - self.current.theta
        
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
        
        if lin_error <= 0.1:
            self.publisher_.publish(Twist())
            self.is_rotating = True
            self.send_goal()
            self.grid_points()
            

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