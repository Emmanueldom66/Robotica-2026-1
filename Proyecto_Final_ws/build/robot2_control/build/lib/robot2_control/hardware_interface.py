#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class Robot2HardwareInterface(Node):
  def __init__(self):
    super().__init__("robot2_hardware_interface")
    
    # SUSCRIPTORES
    # Recibir información de posiciones deseadas desde controller_manager
    self.joint_hardware_objectives_subscriber = self.create_subscription(
      JointState, "/robot2/joint_hardware_objectives", 
      self.hardware_obj_callback, 10
    )
    
    # PUBLICADORES
    # Retroalimentación de las posiciones actuales del robot
    self.joint_states_publisher = self.create_publisher(
      JointState, "/robot2/joint_states", 10
    )
    
    # Mensaje que contiene la posición actual del robot
    self.current_joint_state = JointState()
    self.current_joint_state.header.stamp = self.get_clock().now().to_msg()
    self.current_joint_state.name = ["turret_joint_r2", "shoulder_joint_r2", "arm_joint_r2"]
    self.current_joint_state.position = [0.1, 0.1, 0.1]
    
    # Timer para publicar el estado actual periódicamente (10 Hz)
    self.create_timer(0.1, self.joint_states_timer_callback)
    
    self.get_logger().info("Robot 2 - Hardware Interface inicializada")

  def hardware_obj_callback(self, msg: JointState):
    # En esta parte se mandaría a través de alguna comunicación 
    # la información al hardware
    # Aquí estamos asignando los comandos como si fueran la posición real del robot
    
    self.current_joint_state = msg

  def joint_states_timer_callback(self):
    msg = JointState()
    # En esta parte recibiría la información del estado del robot
    # ---
    msg.header.stamp = self.get_clock().now().to_msg()
    msg.name = ["turret_joint_r2", "shoulder_joint_r2", "arm_joint_r2"]
    msg.position = [0.1, 0.1, 0.1]
    
    # Actualizar marca de tiempo
    self.current_joint_state.header.stamp = self.get_clock().now().to_msg()
    
    # Publicar estado actual
    self.joint_states_publisher.publish(self.current_joint_state)

def main(args=None):
  try:
    rclpy.init(args=args)
    node = Robot2HardwareInterface()
    rclpy.spin(node)
  except KeyboardInterrupt as e:
    print("Robot 2 - Hardware Interface stopped")
  finally: 
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
  main()