#!/usr/bin/env python3

import math
import threading
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

from tf2_ros import TransformBroadcaster

import serial


class MecanumSerialBridge(Node):

    def __init__(self):

        super().__init__('mecanum_serial_bridge')

        # =====================================================
        # PARÂMETROS
        # =====================================================

        self.declare_parameter('port', '/dev/arduino')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('send_rate', 20.0)
        self.declare_parameter('cmd_timeout', 0.5)

        # Enquanto não usamos EKF:
        # publish_tf = True
        #
        # Quando o EKF assumir odom -> base_link:
        # publish_tf = False
        self.declare_parameter('publish_tf', True)

        self.port = self.get_parameter('port').value
        self.baud = self.get_parameter('baud').value
        self.send_rate = self.get_parameter('send_rate').value
        self.cmd_timeout = self.get_parameter('cmd_timeout').value
        self.publish_tf = self.get_parameter('publish_tf').value

        # =====================================================
        # GEOMETRIA DO ROBÔ
        # =====================================================

        self.wheel_radius = 0.041

        self.lx = 0.082
        self.ly = 0.106

        self.k = self.lx + self.ly

        # =====================================================
        # ENCODERS
        # =====================================================

        self.ticks_per_rev = 1319.1

        self.meters_per_tick = (
            2.0 * math.pi * self.wheel_radius
        ) / self.ticks_per_rev

        # =====================================================
        # ESTADO DA ODOMETRIA
        # =====================================================

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.last_fl = None
        self.last_fr = None
        self.last_rl = None
        self.last_rr = None

        self.last_encoder_time = None

        # =====================================================
        # COMANDO DE VELOCIDADE
        # =====================================================

        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0

        self.last_cmd_time = time.monotonic()

        # =====================================================
        # SERIAL
        # =====================================================

        self.serial = serial.Serial(
            self.port,
            self.baud,
            timeout=0.1
        )

        time.sleep(2.0)

        self.serial.reset_input_buffer()
        self.serial.reset_output_buffer()

        self.get_logger().info(
            f'Serial aberta em {self.port} @ {self.baud}'
        )

        self.get_logger().info(
            f'Ticks/volta: {self.ticks_per_rev:.2f}'
        )

        self.get_logger().info(
            f'Metros/tick: {self.meters_per_tick:.9f}'
        )

        self.get_logger().info(
            f'Lx={self.lx:.3f} m | '
            f'Ly={self.ly:.3f} m | '
            f'Lx+Ly={self.k:.3f} m'
        )

        self.get_logger().info(
            f'Publicar TF odom -> base_link: {self.publish_tf}'
        )

        # =====================================================
        # ROS
        # =====================================================

        self.cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.odom_pub = self.create_publisher(
            Odometry,
            '/wheel_odom',
            20
        )

        # Criamos o broadcaster somente se esta bridge
        # for responsável pela TF.
        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)
        else:
            self.tf_broadcaster = None

        # =====================================================
        # TIMER DE ENVIO
        # =====================================================

        period = 1.0 / self.send_rate

        self.send_timer = self.create_timer(
            period,
            self.send_command
        )

        # =====================================================
        # THREAD SERIAL
        # =====================================================

        self.running = True

        self.serial_thread = threading.Thread(
            target=self.serial_reader,
            daemon=True
        )

        self.serial_thread.start()

        self.get_logger().info(
            'Mecanum Serial Bridge iniciado'
        )

    # =========================================================
    # CMD_VEL
    # =========================================================

    def cmd_vel_callback(self, msg):

        self.vx = max(
            -1.0,
            min(1.0, msg.linear.x)
        )

        self.vy = max(
            -1.0,
            min(1.0, msg.linear.y)
        )

        self.wz = max(
            -1.0,
            min(1.0, msg.angular.z)
        )

        self.last_cmd_time = time.monotonic()

    # =========================================================
    # ENVIO PARA ARDUINO
    # =========================================================

    def send_command(self):

        if (
            time.monotonic() - self.last_cmd_time
            > self.cmd_timeout
        ):

            vx = 0.0
            vy = 0.0
            wz = 0.0

        else:

            vx = self.vx
            vy = self.vy
            wz = self.wz

        comando = (
            f'V,{vx:.3f},{vy:.3f},{wz:.3f}\n'
        )

        try:

            self.serial.write(
                comando.encode()
            )

        except Exception as e:

            self.get_logger().error(
                f'Erro escrevendo na serial: {e}'
            )

    # =========================================================
    # LEITURA SERIAL
    # =========================================================

    def serial_reader(self):

        while self.running:

            try:

                linha = self.serial.readline()

                if not linha:
                    continue

                linha = linha.decode(
                    errors='replace'
                ).strip()

                if linha.startswith('E,'):

                    self.process_encoder_line(
                        linha
                    )

            except Exception as e:

                if self.running:

                    self.get_logger().error(
                        f'Erro lendo serial: {e}'
                    )

    # =========================================================
    # PROCESSAMENTO DOS ENCODERS
    # =========================================================

    def process_encoder_line(self, linha):

        partes = linha.split(',')

        if len(partes) != 5:
            return

        try:

            fl = int(partes[1])
            fr = int(partes[2])
            rl = int(partes[3])
            rr = int(partes[4])

        except ValueError:

            return

        agora = time.monotonic()

        # -----------------------------------------------------
        # PRIMEIRA LEITURA
        # -----------------------------------------------------

        if self.last_fl is None:

            self.last_fl = fl
            self.last_fr = fr
            self.last_rl = rl
            self.last_rr = rr

            self.last_encoder_time = agora

            return

        # -----------------------------------------------------
        # DELTA DOS ENCODERS
        # -----------------------------------------------------

        delta_fl = fl - self.last_fl
        delta_fr = fr - self.last_fr
        delta_rl = rl - self.last_rl
        delta_rr = rr - self.last_rr

        self.last_fl = fl
        self.last_fr = fr
        self.last_rl = rl
        self.last_rr = rr

        dt = agora - self.last_encoder_time

        self.last_encoder_time = agora

        if dt <= 0.0:
            return

        # -----------------------------------------------------
        # TICKS -> METROS
        # -----------------------------------------------------

        ds_fl = (
            delta_fl *
            self.meters_per_tick
        )

        ds_fr = (
            delta_fr *
            self.meters_per_tick
        )

        ds_rl = (
            delta_rl *
            self.meters_per_tick
        )

        ds_rr = (
            delta_rr *
            self.meters_per_tick
        )

        # -----------------------------------------------------
        # CINEMÁTICA MECANUM
        # -----------------------------------------------------

        dx_body = (
            ds_fl +
            ds_fr +
            ds_rl +
            ds_rr
        ) / 4.0

        dy_body = (
            -ds_fl +
            ds_fr +
            ds_rl -
            ds_rr
        ) / 4.0

        dtheta = (
            -ds_fl +
            ds_fr -
            ds_rl +
            ds_rr
        ) / (
            4.0 * self.k
        )

        # -----------------------------------------------------
        # INTEGRAÇÃO NO REFERENCIAL ODOM
        # -----------------------------------------------------

        theta_mid = (
            self.theta +
            dtheta / 2.0
        )

        dx_world = (
            math.cos(theta_mid) * dx_body
            -
            math.sin(theta_mid) * dy_body
        )

        dy_world = (
            math.sin(theta_mid) * dx_body
            +
            math.cos(theta_mid) * dy_body
        )

        self.x += dx_world
        self.y += dy_world
        self.theta += dtheta

        self.theta = math.atan2(
            math.sin(self.theta),
            math.cos(self.theta)
        )

        # -----------------------------------------------------
        # VELOCIDADES
        # -----------------------------------------------------

        vx = dx_body / dt
        vy = dy_body / dt
        wz = dtheta / dt

        self.publish_odometry(
            vx,
            vy,
            wz
        )

    # =========================================================
    # PUBLICAÇÃO DA ODOMETRIA
    # =========================================================

    def publish_odometry(
        self,
        vx,
        vy,
        wz
    ):

        now = self.get_clock().now()

        stamp = now.to_msg()

        qz = math.sin(
            self.theta / 2.0
        )

        qw = math.cos(
            self.theta / 2.0
        )

        msg = Odometry()

        msg.header.stamp = stamp
        msg.header.frame_id = 'odom'

        msg.child_frame_id = 'base_link'

        # -----------------------------------------------------
        # POSE
        # -----------------------------------------------------

        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0

        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        # -----------------------------------------------------
        # COVARIÂNCIA DA POSE
        #
        # Ordem:
        # x, y, z, roll, pitch, yaw
        #
        # Valores iniciais conservadores.
        # y recebe incerteza maior por causa do
        # escorregamento lateral do mecanum.
        # -----------------------------------------------------

        msg.pose.covariance = [
            0.02, 0.0,  0.0,    0.0,    0.0,  0.0,
            0.0,  0.05, 0.0,    0.0,    0.0,  0.0,
            0.0,  0.0,  9999.0, 0.0,    0.0,  0.0,
            0.0,  0.0,  0.0,    9999.0, 0.0,  0.0,
            0.0,  0.0,  0.0,    0.0,    9999.0, 0.0,
            0.0,  0.0,  0.0,    0.0,    0.0,  0.04
        ]

        # -----------------------------------------------------
        # TWIST
        # -----------------------------------------------------

        msg.twist.twist.linear.x = vx
        msg.twist.twist.linear.y = vy
        msg.twist.twist.linear.z = 0.0

        msg.twist.twist.angular.x = 0.0
        msg.twist.twist.angular.y = 0.0
        msg.twist.twist.angular.z = wz

        # -----------------------------------------------------
        # COVARIÂNCIA DAS VELOCIDADES
        #
        # Ordem:
        # vx, vy, vz, vroll, vpitch, vyaw
        # -----------------------------------------------------

        msg.twist.covariance = [
            0.03, 0.0,  0.0,    0.0,    0.0,  0.0,
            0.0,  0.08, 0.0,    0.0,    0.0,  0.0,
            0.0,  0.0,  9999.0, 0.0,    0.0,  0.0,
            0.0,  0.0,  0.0,    9999.0, 0.0,  0.0,
            0.0,  0.0,  0.0,    0.0,    9999.0, 0.0,
            0.0,  0.0,  0.0,    0.0,    0.0,  0.06
        ]

        self.odom_pub.publish(
            msg
        )

        # -----------------------------------------------------
        # TF ODOM -> BASE_LINK
        #
        # Só é publicada pela bridge quando publish_tf=True.
        #
        # Quando o robot_localization assumir esta TF,
        # iniciaremos a bridge com publish_tf:=false.
        # -----------------------------------------------------

        if self.publish_tf:

            tf = TransformStamped()

            tf.header.stamp = stamp
            tf.header.frame_id = 'odom'

            tf.child_frame_id = 'base_link'

            tf.transform.translation.x = self.x
            tf.transform.translation.y = self.y
            tf.transform.translation.z = 0.0

            tf.transform.rotation.x = 0.0
            tf.transform.rotation.y = 0.0
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw

            self.tf_broadcaster.sendTransform(
                tf
            )

    # =========================================================
    # ENCERRAMENTO
    # =========================================================

    def destroy_node(self):

        self.running = False

        try:

            self.serial.write(
                b'S\n'
            )

        except Exception:

            pass

        try:

            self.serial.close()

        except Exception:

            pass

        super().destroy_node()


def main(args=None):

    rclpy.init(
        args=args
    )

    node = MecanumSerialBridge()

    try:

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':

    main()
