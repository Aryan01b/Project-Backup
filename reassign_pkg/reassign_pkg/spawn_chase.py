import rclpy
from rclpy.node import Node
import rclpy.service
from turtlesim.srv import Kill, Spawn
from std_srvs.srv import Empty
import random, math, time

class TurtleSpawn(Node):
    def __init__(self):
        super().__init__('chase_spawn')
        
        self.turtle_kill = self.create_client(Kill, '/kill')
        self.turtle_spawn = self.create_client(Spawn, '/spawn')
        self.turtlesim_clear_ = self.create_client(Empty, '/clear')
        
        while not self.turtlesim_clear_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting for /clear service...')
        while not self.turtle_kill.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting for /kill service...')
        while not self.turtle_spawn.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting for /spawn service...')
            
        self.clear_turtle()
            
    def clear_turtle(self):
        req = Empty.Request()
        future = self.turtlesim_clear_.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()
    
    def kill_turtle(self, name):
        req = Kill.Request()
        req.name = name
        future = self.turtle_kill.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def spawn_turtle(self, name):
        req = Spawn.Request()
        req.name = name
        if req.name == 'RT':
            req.x = 5.4
            req.y = 0.7
            req.theta = 0.0
            self.flag = True
        else:    
            req.x = random.uniform(1.0, 11.0)
            req.y = random.uniform(1.0, 11.0)
            req.theta = random.uniform(0, 2*math.pi)
        future = self.turtle_spawn.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

def main(args=None):
    
    rclpy.init(args=args)
    node = TurtleSpawn()

    try:
        node.kill_turtle('turtle1')   
        node.kill_turtle('RT')     
        node.kill_turtle('PT')     
    except:
        print('Doesn\'t Exist')
    finally:
        print("Killed All existing")
    node.spawn_turtle('RT')
    time.sleep(10)
    node.spawn_turtle('PT')

    rclpy.shutdown()

if __name__ == '__main__':
    main()
