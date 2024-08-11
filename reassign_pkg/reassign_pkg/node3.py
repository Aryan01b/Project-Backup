import rclpy
from rclpy.node import Node
import rclpy.service 
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import random

class TurtlebotController(Node):
    def __init__(self, radius, speed):
        super().__init__('three')
        
            
        # 2D Goal Movement
        self.vel_pub_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.real_pose_pub_ = self.create_publisher(Pose, '/rt_real_pose', 10)
        self.noisy_pose_pub_ = self.create_publisher(Pose, '/rt_noisy_pose', 10)
        
        self.turtle_pose_sub_ = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)

        self.move_timer = self.create_timer(0.1, self.move_callback)
        self.pose_pub_timer = self.create_timer(5.0, self.pose_send_callback)
        
        self.current = Pose()
        self.cmd_vel_prev = Twist()
        self.radius = radius
        self.linear_speed = speed
        self.radius_step = 0.5
        
        #Gaussian Noise Parameter
        self.noise_std_dev = 3
        
        
    def pose_callback(self, current_pose):
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
        
        self.vel_pub_.publish(v_i)            
        return v_i
    
    # def move_callback(self):
    #     cmd_vel = Twist()
        
    #     radius_step = 0.1
    #     while radius_step <= self.radius:
    #         angular_speed = self.linear_speed/radius_step
    #         cmd_vel.linear.x = self.linear_speed
    #         cmd_vel.angular.z = angular_speed
    #         self.cmd_vel_prev = self.acc_dec_profile(cmd_vel, self.cmd_vel_prev)
    #         radius_step += 0.1
    def move_callback(self):
        cmd_vel = Twist()
        
        if self.radius_step <= self.radius:
            angular_speed = 10.0/self.radius_step
            cmd_vel.linear.x = 10.0
            cmd_vel.angular.z = angular_speed
            self.vel_pub_.publish(cmd_vel)
            # self.cmd_vel_prev = self.acc_dec_profile(cmd_vel, self.cmd_vel_prev)
            self.radius_step += 0.2
            return
        
        cmd_vel.linear.x = self.linear_speed
        cmd_vel.angular.z = self.linear_speed/self.radius
        self.vel_pub_.publish(cmd_vel)
        # self.cmd_vel_prev = self.acc_dec_profile(cmd_vel, self.cmd_vel_prev)
        
    def pose_send_callback(self):
        real_pose = Pose()
        noisy_pose = Pose()
        
        real_pose = self.current
        
        noisy_pose.x = self.current.x + random.gauss(0, self.noise_std_dev)
        noisy_pose.y = self.current.y + random.gauss(0, self.noise_std_dev)
        noisy_pose.theta = self.current.theta + random.gauss(0, self.noise_std_dev)
        noisy_pose.linear_velocity = self.current.linear_velocity + random.gauss(0, self.noise_std_dev)
        noisy_pose.angular_velocity = self.current.angular_velocity + random.gauss(0, self.noise_std_dev)
        
        self.noisy_pose_pub_.publish(noisy_pose)
        self.real_pose_pub_.publish(real_pose)
         
def main(args=None):
    rclpy.init(args=args)
    radius = float(input("Enter the Radius: "))
    speed = float(input("Enter the Linear speed: "))
    
    node = TurtlebotController(radius, speed)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__=='__main__':
    main()