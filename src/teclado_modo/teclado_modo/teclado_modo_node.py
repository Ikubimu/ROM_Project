import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String

import sys
import termios
import tty
import select


class TecladoModoNode(Node):
    def __init__(self):
        super().__init__('teclado_modo_node')

        # Publishers numéricos
        self.modo_publisher = self.create_publisher(Int32, 'mode_listener', 10)
        self.cmd_publisher = self.create_publisher(Int32, 'movements_listener', 10)

        # Publishers de texto descriptivo
        self.modo_text_publisher = self.create_publisher(String, 'mode_text', 10)
        self.cmd_text_publisher = self.create_publisher(String, 'movement_text', 10)

        self.timer = self.create_timer(0.1, self.leer_tecla)

        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        # Estado actual del modo (0 = Manual por defecto, o puedes poner None)
        self.modo_actual = 0

        self.get_logger().info(
            '\nNodo de teclado iniciado\n\n'
            'Presiona\n 1 (Modo Manual),\n 2 (Modo Dummy),\n 3 (Modo Inteligente)\n para cambiar el modo\n\n'
            '           w (avanzar)\na (izquierda)  s (parar)   d (derecha)\npara controlar el robot (solo en modo Manual).\n'
        )

    def leer_tecla(self):
        if self.tecla_disponible():
            ch = sys.stdin.read(1)

            # Cambiar modo
            if ch in ['1', '2', '3']:
                modo_dict = {'1': 'Manual', '2': 'Dummy', '3': 'Inteligente'}
                self.modo_actual = int(ch) - 1

                int_msg = Int32()
                str_msg = String()

                int_msg.data = self.modo_actual
                str_msg.data = f"Modo cambiado a: {modo_dict[ch]}"

                self.modo_publisher.publish(int_msg)
                self.modo_text_publisher.publish(str_msg)

                self.get_logger().info(str_msg.data)

            # Enviar comando solo si está en modo manual
            elif ch in ['a', 's', 'd', 'w']:
                if self.modo_actual != 0:
                    self.get_logger().warn("Movimiento ignorado: No estás en modo Manual.")
                    return

                movimiento_dict = {'a': 'Girar Izquierda', 's': 'Parar', 'd': 'Girar Derecha', 'w': 'Avanzar'}
                comando = {'a': 3, 's': 0, 'd': 2, 'w': 1}[ch]

                int_msg = Int32()
                str_msg = String()

                int_msg.data = comando
                str_msg.data = f"Movimiento: {movimiento_dict[ch]}"

                self.cmd_publisher.publish(int_msg)
                self.cmd_text_publisher.publish(str_msg)

                self.get_logger().info(str_msg.data)

    def tecla_disponible(self):
        return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

    def destroy_node(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TecladoModoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
