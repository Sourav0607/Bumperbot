import rclpy
from rclpy.node import Node
from bumperbot_msgs.srv import AddTwoInts




class SimpleServiceServerNode(Node):
    def __init__(self):
        super().__init__('simple_service_server')
        
        self.service_ =  self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.service_callback
        )
        
        self.get_logger().info("Simple Service Server Node has been started.")
        
    def service_callback(self, request, response):
        self.get_logger().info(f"Received request: a={request.a}, b={request.b}. Sending response: sum={response.sum}")
        response.sum = request.a + request.b
        self.get_logger().info(f"Response sent: sum={response.sum}")
        return response
    
def main():
    rclpy.init()
    node = SimpleServiceServerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()