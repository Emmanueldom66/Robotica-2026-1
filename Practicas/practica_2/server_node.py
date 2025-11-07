import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from example_interfaces.srv import AddTwoInts

class Server(Node):
  def __init__(self):
    super().__init__('server_node')
    server_name = "/mult_two_ints"
    self.srv = self.create_service(AddTwoInts, server_name, self.listener_callback)
    self.get_logger().info('Servicio disponible en {}'.format(server_name))

  def listener_callback(self, request:AddTwoInts.Request, response:AddTwoInts.Response):
    response.sum = request.a * request.b
    self.get_logger().info("Recibido: a={}, b={}, respuesta={}".format(request.a, request.b, response.sum))
    return response

# def main(args=None):
#   rclpy.init(args=args)
#   node = Server()
#   rclpy.spin(node)
#   node.destroy_node()
#   rclpy.shutdown()

# if __name__ == '__main__':
#   main()