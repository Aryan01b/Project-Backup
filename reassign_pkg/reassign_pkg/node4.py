import rclpy
from rclpy.node import Node
import rclpy.parameter
import rclpy.service 
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class TurtlebotController(Node):
    def __init__(self):
        super().__init__('pt_move') 
        
        self.declare_parameter('caught', False)

        # 2D Goal Movement        
        self.pt_pub_ = self.create_publisher(Twist, '/PT/cmd_vel', 10)
        self.real_pose_sub_ = self.create_subscription(Pose, '/rt_real_pose', self.real_pose_callback, 10)
        self.pt_pose_sub_ = self.create_subscription(Pose, '/PT/Pose', self.PT_pose_callback, 10)
        
        self.move_timer = self.create_timer(0.1, self.move_callback)

        # RT Pose
        self.rt_pose = Pose()
        self.rt_pose.x = 1.0
        self.rt_pose.y = 1.0
        # PT Pose
        self.pt_pose = Pose()
        
        self.cmd_vel_prev = Twist()
        # (kp, ki, kd)
        self.linear_gain = [2, 0, 0]
        # self.angular_gain = [10, 0.05, 0.001]
        self.angular_gain = [10, 0, 0]
        
        # error_sum, prev_error
        self.linear_error = [0, 0]
        self.angular_error = [0, 0]
    
    def real_pose_callback(self, msg):
        # self.rt_pose = msg
        pass
        
    def PT_pose_callback(self, msg):
        self.pt_pose = msg
        
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
        
        self.pt_pub_.publish(v_i)            
        return v_i
    
    def move_callback(self):
        cmd_vel = Twist()
        error_x = self.rt_pose.x - self.pt_pose.x
        error_y = self.rt_pose.y - self.pt_pose.y
        
        lin_error = math.sqrt(error_x**2 + error_y**2)
        ang_error = math.atan2(error_y, error_x) - self.pt_pose.theta
        
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
        # self.cmd_vel_prev = self.acc_dec_profile(cmd_vel, self.cmd_vel_prev)
        self.pt_pub_.publish(cmd_vel)
        
        if lin_error < 0.5:
            self.pt_pub_.publish(Twist())
            self.set_parameters([rclpy.parameter.Parameter('caught', rclpy.Parameter.Type.BOOL, True)])
         
def main(args=None):
    rclpy.init(args=args)
    
    node = TurtlebotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__=='__main__':
    main()