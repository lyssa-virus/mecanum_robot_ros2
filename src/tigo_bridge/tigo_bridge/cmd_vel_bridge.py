import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time

class CmdVelSerialBridge(Node):
    def __init__(self):
        super().__init__('cmd_vel_serial_bridge')
        
        # 1. Configuração da porta serial do Arduino (idêntica ao código de referência)
        try:
            self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
            time.sleep(2)  # Aguarda o Arduino reiniciar após a conexão
            self.get_logger().info("Ponte Serial CMD_VEL conectada em /dev/ttyUSB0 com sucesso!")
        except Exception as e:
            self.get_logger().error(f"Falha ao abrir a porta serial: {e}")
            raise e

        # 2. Cria a inscrição para ouvir os comandos de velocidade do Nav2 ou teleop
        self.cmd_sub = self.create_subscription(
            Twist, 
            '/cmd_vel', 
            self.cmd_vel_callback, 
            10
        )

    def cmd_vel_callback(self, msg):
        # Extrai as velocidades do tópico e aplica limites de segurança (-1.0 a 1.0)
        vx = max(-1.0, min(1.0, msg.linear.x))
        vy = max(-1.0, min(1.0, msg.linear.y))
        wz = max(-1.0, min(1.0, msg.angular.z))

        # Formata a string no padrão "V,vx,vy,wz\n" esperado pelo código C++ do Arduino
        comando = f'V,{vx:.3f},{vy:.3f},{wz:.3f}\n'
        
        # Envia a string formatada em bytes para o Arduino
        try:
            self.ser.write(comando.encode())
            # self.get_logger().debug(f"Enviado: {comando.strip()}") # Descomente para depurar
        except Exception as e:
            self.get_logger().error(f"Erro ao enviar dados pela serial: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelSerialBridge()
    
    try:
        # Mantém o nó rodando e escutando o tópico continuamente
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Encerramento solicitado pelo usuário (Ctrl+C).")
    finally:
        # Fechamento seguro da porta serial
        if hasattr(node, 'ser') and node.ser.is_open:
            node.ser.close()
            node.get_logger().info("Porta serial fechada com segurança.")
        
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
