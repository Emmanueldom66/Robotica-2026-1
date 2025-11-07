import rclpy
from server_node import Server
from client_node import Client
import threading
from rclpy.executors import MultiThreadedExecutor

def initialize_ROS2():
    if not rclpy.ok():
        rclpy.init(args=None)

def instantiate_executor():
    executor = MultiThreadedExecutor()
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    # instantiate server and client nodes
    server_node = Server()
    client_node = Client()
    # add nodes to executor
    executor.add_node(server_node)
    executor.add_node(client_node)
    try:
        # keep main thread alive to let executor run
        while rclpy.ok():
            pass
    except KeyboardInterrupt:
        pass
    finally:
      # destroy nodes and shutdown ROS2 when done
      executor.remove_node(server_node)
      executor.remove_node(client_node)
      server_node.destroy_node()
      client_node.destroy_node()
      if rclpy.ok():
          rclpy.shutdown()

def main():
    initialize_ROS2()
    instantiate_executor()

if __name__ == '__main__':
    main()