import rclpy
from rclpy.node import Node


class Rotaciones(Node):

    def __init__(self):
        super().__init__('rotaciones')

        self.get_logger().info('Nodo de rotaciones iniciado')


def main(args=None):
    rclpy.init(args=args)

    node = Rotaciones()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()