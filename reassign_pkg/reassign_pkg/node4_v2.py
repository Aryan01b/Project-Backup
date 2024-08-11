import rclpy
from rclpy.node import Node
import rclpy.parameter
import rclpy.service 
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_msgs.msg import Bool
from sympy import symbols, solve
import math
from time import time

class TurtlebotController(Node):
    def __init__(self):
        super().__init__('chase_fast')
             
        # 2D Goal Movement
        self.status_pub_ = self.create_publisher(Bool, '/caught', 10)
        
        self.publisher_ = self.create_publisher(Twist, 'PT/cmd_vel', 10)
        self.subscriber_ = self.create_subscription(Pose, 'PT/pose', self.odom_callback, 10)
        
        self.real_pose_sub_ = self.create_subscription(Pose, '/rt_real_pose', self.real_pose_callback, 10)
        
        self.move_timer = self.create_timer(0.1, self.move_callback)
        
        # RT Pose
        self.rt_pose = Pose()
        # PT Pose
        self.pt_pose = Pose()
        self.goal = Pose()
        
        self.points = []
        self.should_move = False

        self.cmd_vel_prev = Twist()
    
        # (kp, ki, kd)
        self.linear_gain = [2, 0, 0]
        # self.angular_gain = [10, 0.05, 0.001]
        self.angular_gain = [10, 0, 0]
        
        # error_sum, prev_error
        self.linear_error = [0, 0]
        self.angular_error = [0, 0]
        
    def real_pose_callback(self, msg):
        if len(self.points)<3:
            self.points.append([msg.x, msg.y])
            return
        self.should_move = True
        self.goal = self.predict_pose(self.points)
        self.rt_pose = msg
        self.check_dist()
            
        
    def odom_callback(self, current_pose):
        self.pt_pose = current_pose
        
    def acc_dec_profile(self, v_f, v_i):
        max_a_lin = 10
        max_a_ang = 110

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
        
    def check_dist(self):
        error_x = self.rt_pose.x - self.pt_pose.x
        error_y = self.rt_pose.y - self.pt_pose.y
        
        dist= math.sqrt(error_x**2 + error_y**2)
        if dist<= 0.5:
            status = Bool()
            status.data = True
            self.status_pub_.publish(status)
            self.get_logger().warn("PT caught RT")
            rclpy.shutdown()
        
    def move_callback(self):
        if self.should_move:
            cmd_vel = Twist()
                
            goal = self.goal
            
            error_x = goal.x - self.pt_pose.x
            error_y = goal.y - self.pt_pose.y
            
            lin_error = math.sqrt(error_x**2 + error_y**2)
            ang_error = math.atan2(error_y, error_x) - self.pt_pose.theta
            
            if lin_error <= 0.1:
                self.publisher_.publish(Twist())
                self.get_logger().fatal(f'Reached Goal : [{goal.x}, {goal.y}]')
                return
            
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
            
    def can_reach(self, x, y, max_vel, time_available):
        # Time = Distance/Speed
        error_x = x - self.pt_pose.x
        error_y = y - self.pt_pose.y
        dist= math.sqrt(error_x**2 + error_y**2)
        
        if dist/max_vel < time_available:
            return True
        
        return False
         
    def predict_pose(self, pt_list):
        # Derive an Equation of circle from 3 points
        x, y, g, f, c = symbols('x y g f c')
        F_x = x**2 + y**2 + 2*g*x + 2*f*y + c
        
        eqns = [F_x.subs({x: pt_list[i][0], y: pt_list[i][1]}) for i in range(3)]
        sol = solve(eqns, (g, f, c))
        x_c = -sol[g]
        y_c = -sol[f]
        radius = (x_c**2 + y_c**2 - sol[c])**(1/2)
        print(f"(xc, yc, radius):{x_c},{y_c},{radius}")
        
        theta_1 = math.atan2(pt_list[1][0] - x_c, pt_list[1][1] - y_c)
        theta_2 = math.atan2(pt_list[2][0] - x_c, pt_list[2][1] - y_c)
        dtheta = theta_2-theta_1
        
        temp = Pose()
        second_pt = 2
        for i in range(10):
            theta_t = theta_2 + (second_pt+i)*dtheta
            x_t = x_c + radius * math.sin(theta_t)
            y_t = y_c + radius * math.cos(theta_t)  
            if self.can_reach(x_t, y_t, 10, (second_pt+i)*5):
                temp.x = float(x_t)
                temp.y = float(y_t)
                return temp
        
        
         
def main(args=None):
    rclpy.init(args=args)
   
    node = TurtlebotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__=='__main__':
    main()
