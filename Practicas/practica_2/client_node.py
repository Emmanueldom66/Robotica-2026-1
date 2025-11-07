import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts
from rclpy.task import Future

class Client(Node):
  def __init__(self):
    # Constructor de inicio
    super().__init__('client_node')
    service_name = "/mult_two_ints"
    self.a = int(input("Ingrese el primer entero (a): "))
    self.b = int(input("Ingrese el segundo entero (b): "))

    self.client = self.create_client(AddTwoInts, service_name)
  
    while not self.client.wait_for_service(timeout_sec=1.0):
      self.get_logger().info('Esperando al servicio {}...'.format(service_name))
    
    self.timer = self.create_timer(2.0, self.timer_callback)

  def timer_callback(self):
    # Función de timer para enviar solicitudes periódicamente
    self.get_logger().info("Enviando solicitud...")
    request = AddTwoInts.Request()
    request.a = self.a
    request.b = self.b
    future = self.client.call_async(request)
    future.add_done_callback(self.callback_result)

  def callback_result(self, future:Future):
    try:
      response = future.result()
      response:AddTwoInts.Response
      self.get_logger().info("Resultado recibido: {} * {} = {}".format(self.a, self.b, response.sum))
    except Exception as e:
      self.get_logger().error(f"Error al llamar al servicio: {e}")

# def main(args=None):
#   rclpy.init(args=args)
#   node = Client()
#   rclpy.spin(node)
#   node.destroy_node()
#   rclpy.shutdown()

# if __name__ == '__main__':
#   main()
