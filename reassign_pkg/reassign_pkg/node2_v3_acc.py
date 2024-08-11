import rclpy
from rclpy.node import Node
import rclpy.service 
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
from time import time

class TurtlebotController(Node):
    # def __init__(self, accel_list):
    def __init__(self):
        super().__init__('two_acc')
        
             
        # 2D Goal Movement
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.subscriber_ = self.create_subscription(Pose, 'turtle1/pose', self.odom_callback, 10)
        self.timer = self.create_timer(0.1, self.grid)
        
        self.goal = Pose()
        self.goal.x = 1.0
        self.goal.y = 1.0
        
        # self.max_lin = accel_list[0]
        # self.max_ang = accel_list[1]
        
        self.cmd_vel_prev = Twist()
        
        self.current = Pose()
        self.is_rotating = False

        #Grid Parameters
        self.count = 0
        self.a = 1.0
        self.b = 10.0
        self.temp = self.a
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
            self.temp += 1
        else:
            self.goal.x = self.a
            self.goal.y = self.temp
        self.goal.theta = self.theta_list[(self.count % 4)]
        self.count += 1
        self.get_logger().info(f"({self.goal.x},{self.goal.y}, {self.goal.theta})")
        
        
    def odom_callback(self, current_pose):
        self.current = current_pose
        
    def acc_dec_profile(self, v_f, v_i):
        max_a_lin = 1
        max_a_ang = 120

        if (v_f.linear.x - v_i.linear.x)>0:
            # Accelerate
            # max_a_lin = self.max_lin
            # max_a_ang = self.max_ang
            max_a_lin = abs(max_a_lin)
            max_a_ang = abs(max_a_ang)
        else:
            # Decelerate
            # max_a_lin = -self.max_lin
            # max_a_ang = -self.max_ang
            max_a_lin = -abs(max_a_lin)
            max_a_ang = -abs(max_a_ang)
            
        dv = (v_f.linear.x - v_i.linear.x)  
        
        if abs(dv) > abs(max_a_lin):
            dv = max_a_lin
            
        d_omega = (v_f.angular.z - v_i.angular.z)  
        if abs(d_omega) > abs(max_a_ang):
            d_omega = max_a_ang
            
        v_i.linear.x += dv
        v_i.angular.z += d_omega
        
        self.publisher_.publish(v_i)            
        return v_i
        
    def grid(self):
        start = time()
        cmd_vel = Twist()
        # Check if the turtle is currently rotating
        if self.is_rotating:
            if abs(self.goal.theta - self.current.theta) >0.05:
                cmd_vel.angular.z = (self.goal.theta - self.current.theta)
                self.publisher_.publish(cmd_vel)
            else:
                self.is_rotating = False
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
        cmd_vel.linear.x = lin_vel
        cmd_vel.angular.z = ang_vel
        self.cmd_vel_prev = self.acc_dec_profile(cmd_vel, self.cmd_vel_prev)
        # self.publisher_.publish(cmd_vel)
        
        if lin_error <= 0.1:
            self.publisher_.publish(Twist())
            self.is_rotating = True
            self.grid_points()
            end = time()
            self.get_logger().info(f'Time to reach : {end-start}')
            

         
def main(args=None):
    rclpy.init(args=args)
    
    # acceleration_vals = list(map(float, input("Enter space separated Max Linear and Max Angular acceleration:").split(' ')))
    # node = TurtlebotController(acceleration_vals)
    
    node = TurtlebotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__=='__main__':
    main()