#!/usr/bin/env python3

import sys
import termios
import tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CustomKeyboardTeleop(Node):
    def __init__(self):
        super().__init__('custom_keyboard_teleop')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.get_logger().info(
            "\n=========================================="
            "\n Teleop Personalizado do Tigo iniciado!"
            "\n Controles:"
            "\n   w : Frente (+1.0 m/s)"
            "\n   s : Trás (-1.0 m/s)"
            "\n   a : Esquerda (+1.0 m/s)"
            "\n   d : Direita (-1.0 m/s)"
            "\n   q : Girar Anti-horário (+1.0 rad/s)"
            "\n   e : Girar Horário (-1.0 rad/s)"
            "\n   Espaço / k : Parada de emergência (0.0)"
            "\n   Ctrl+C : Sair"
            "\n=========================================="
        )

    def get_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

    def run(self):
        twist = Twist()
        try:
            while rclpy.ok():
                key = self.get_key().lower()
                
                # Zera os comandos a cada leitura
                twist.linear.x = 0.0
                twist.linear.y = 0.0
                twist.angular.z = 0.0

                if key == 'w':
                    twist.linear.x = 1.0
                elif key == 's':
                    twist.linear.x = -1.0
                elif key == 'a':
                    twist.linear.y = 1.0
                elif key == 'd':
                    twist.linear.y = -1.0
                elif key == 'q':
                    twist.angular.z = 1.0
                elif key == 'e':
                    twist.angular.z = -1.0
                elif key in [' ', 'k']:
                    pass  # Mantém tudo zerado
                elif ord(key) == 3:  # Captura Ctrl+C
                    break
                else:
                    continue

                self.publisher_.publish(twist)
                self.get_logger().info(
                    f"Comando enviado -> Vx: {twist.linear.x:.1f} | Vy: {twist.linear.y:.1f} | Wz: {twist.angular.z:.1f}"
                )

        except Exception as e:
            self.get_logger().error(f"Erro na leitura do teclado: {e}")
        finally:
            # Envia parada final antes de encerrar
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.angular.z = 0.0
            self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = CustomKeyboardTeleop()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
