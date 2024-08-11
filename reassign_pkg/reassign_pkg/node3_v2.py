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
    
    def move_callback(self):
        cmd_vel = Twist()
        cmd_vel.linear.x = self.linear_speed
        cmd_vel.angular.z = self.linear_speed/self.radius
        self.vel_pub_.publish(cmd_vel)
        
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