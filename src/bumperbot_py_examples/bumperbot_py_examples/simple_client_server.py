import rclpy
from rclpy.node import Node
from bumperbot_msgs.srv import AddTwoInts




class SimpleClientServerNode(Node):
    def __init__(self, a, b):
        super().__init__('simple_client_server_node')
        
        self.client_ = self.create_client(AddTwoInts, 'add_two_ints')
        
        while not self.client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
            
        self.request_ = AddTwoInts.Request()
        self.request_.a = a
        self.request_.b = b
        
        self.future_ = self.client_.call_async(self.request_)
        self.future_.add_done_callback(self.callback)
        
    def callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Result of add_two_ints: {response.sum}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
        rclpy.shutdown()
        
def main(args=None):
    rclpy.init(args=args)
    
    a = 19
    b = 6
    
    simple_client_server_node = SimpleClientServerNode(a, b)
    
    rclpy.spin(simple_client_server_node)
    
    simple_client_server_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
            
        
        
        