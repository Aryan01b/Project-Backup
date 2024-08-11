import rclpy
from rclpy.node import Node
import rclpy.service 
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import random

class TurtlebotController(Node):
    def __init__(self, radius, speed):
        super().__init__('RT_rotate')
 
        # 2D Goal Movement
        self.vel_pub_ = self.create_publisher(Twist, 'RT/cmd_vel', 10)
        self.real_pose_pub_ = self.create_publisher(Pose, '/rt_real_pose', 10)
        self.noisy_pose_pub_ = self.create_publisher(Pose, '/rt_noisy_pose', 10)
        
        self.turtle_pose_sub_ = self.create_subscription(Pose, '/RT/pose', self.pose_callback, 10)
        
        self.status_sub_ = self.create_subscription(Bool, '/caught', self.status_callback, 10)
        
        self.move_timer = self.create_timer(0.1, self.move_callback)
        self.pose_pub_timer = self.create_timer(5.0, self.pose_send_callback)
        
        self.current = Pose()
        
        self.status = Bool()
        self.status.data = False
        
        self.radius = radius
        self.linear_speed = speed
        
        #Gaussian Noise Parameter
        self.noise_std_dev = 3
        
    def status_callback(self, msg):
        self.status = msg
    
    def pose_callback(self, current_pose):
        self.current = current_pose
        
    def pose_send_callback(self):
        real_pose = Pose()
        noisy_pose = Pose()
        
        real_pose = self.current
        
        noisy_pose.x = self.current.x + random.gauss(0, self.noise_std_dev)
        noisy_pose.y = self.current.y + random.gauss(0, self.noise_std_dev)
        noisy_pose.theta = self.current.theta + random.gauss(0, self.noise_std_dev)
        noisy_pose.linear_velocity = self.current.linear_velocity + random.gauss(0, self.noise_std_dev)
        noisy_pose.angular_velocity = self.current.angular_velocity + random.gauss(0, self.noise_std_dev)
        
        self.real_pose_pub_.publish(real_pose)
        self.noisy_pose_pub_.publish(noisy_pose)
        
        self.get_logger().info(f"Real Pose(x,y): ({real_pose.x}, {real_pose.y})")
        
    def move_callback(self):
        if self.status.data:
            self.vel_pub_.publish(Twist())
            rclpy.shutdown()
            return
                        
        cmd_vel = Twist()
        cmd_vel.linear.x = self.linear_speed
        cmd_vel.angular.z = self.linear_speed/self.radius
        self.vel_pub_.publish(cmd_vel)
         
def main(args=None):
    rclpy.init(args=args)
    
    # radius and Linear Velocity
    node = TurtlebotController(5.0, 10.0)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__=='__main__':
    main()