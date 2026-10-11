import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Point, PoseStamped, Vector3Stamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker


class Rotaciones(Node):
    """Dibuja en RViz2 lo que el display Pose no muestra solo:
    la flecha de velocidad y el rastro del recorrido.

    Por defecto trabaja con el camino del filtro propio (1.1):
      recibe   /imu_pose  (PoseStamped)     posicion y orientacion
               /imu_vel   (Vector3Stamped)  velocidad, en los mismos ejes
      publica  /imu_vel_marker  (Marker)    flecha de velocidad
               /imu_path        (Path)      rastro del recorrido

    Para el camino del DMP (1.2) se lanza una segunda copia con otros
    nombres de topicos y otro color (ver comparar.launch.py).
    """

    def __init__(self):
        super().__init__('rotaciones')

        # Topicos de entrada y de salida.
        self.declare_parameter('topico_pose', '/imu_pose')
        self.declare_parameter('topico_vel', '/imu_vel')
        self.declare_parameter('topico_flecha', '/imu_vel_marker')
        self.declare_parameter('topico_rastro', '/imu_path')
        # Color de la flecha: [rojo, verde, azul], de 0.0 a 1.0.
        self.declare_parameter('color_flecha', [1.0, 0.5, 0.1])

        # Metros de flecha por cada m/s de velocidad.
        self.declare_parameter('escala_flecha', 1.0)
        # Por debajo de esta velocidad (m/s) no se dibuja la flecha.
        self.declare_parameter('vel_minima', 0.01)
        # Distancia minima (m) entre dos puntos del rastro.
        self.declare_parameter('paso_rastro', 0.005)
        # Cantidad maxima de puntos del rastro.
        self.declare_parameter('largo_rastro', 2000)

        topico_pose = self.get_parameter('topico_pose').value
        topico_vel = self.get_parameter('topico_vel').value
        topico_flecha = self.get_parameter('topico_flecha').value
        topico_rastro = self.get_parameter('topico_rastro').value
        self.color = list(self.get_parameter('color_flecha').value)

        self.escala = self.get_parameter('escala_flecha').value
        self.vel_minima = self.get_parameter('vel_minima').value
        self.paso_rastro = self.get_parameter('paso_rastro').value
        self.largo_rastro = self.get_parameter('largo_rastro').value

        self.pose_actual = None
        self.rastro = Path()
        self.rastro_nuevo = False

        # El ESP32 publica en best effort: hay que suscribirse igual.
        self.create_subscription(
            PoseStamped, topico_pose,
            self.al_recibir_pose, qos_profile_sensor_data)

        self.create_subscription(
            Vector3Stamped, topico_vel,
            self.al_recibir_vel, qos_profile_sensor_data)

        self.pub_rastro = self.create_publisher(Path, topico_rastro, 10)
        self.pub_flecha = self.create_publisher(Marker, topico_flecha, 10)

        # El rastro se publica 10 veces por segundo, no en cada mensaje.
        self.create_timer(0.1, self.publicar_rastro)

        self.get_logger().info(
            f'Nodo iniciado: {topico_pose} + {topico_vel} -> '
            f'{topico_flecha}, {topico_rastro}')

    def al_recibir_pose(self, msg):
        self.pose_actual = msg

        if not self.rastro.poses:
            self.rastro.header.frame_id = msg.header.frame_id
        else:
            ultimo = self.rastro.poses[-1].pose.position
            p = msg.pose.position

            # Si casi no se movio, no se agrega otro punto.
            if math.dist((p.x, p.y, p.z),
                         (ultimo.x, ultimo.y, ultimo.z)) < self.paso_rastro:
                return

        self.rastro.poses.append(msg)

        if len(self.rastro.poses) > self.largo_rastro:
            self.rastro.poses.pop(0)

        self.rastro_nuevo = True

    def publicar_rastro(self):
        if not self.rastro_nuevo:
            return

        self.rastro.header.stamp = self.get_clock().now().to_msg()
        self.pub_rastro.publish(self.rastro)
        self.rastro_nuevo = False

    def al_recibir_vel(self, msg):
        # Sin pose todavia no se sabe de donde sale la flecha.
        if self.pose_actual is None:
            return

        v = msg.vector
        rapidez = math.sqrt(v.x ** 2 + v.y ** 2 + v.z ** 2)

        marcador = Marker()
        marcador.header.frame_id = self.pose_actual.header.frame_id
        # stamp 0: RViz usa la ultima transformacion disponible.
        marcador.header.stamp = rclpy.time.Time().to_msg()
        marcador.ns = 'velocidad'
        marcador.id = 0
        marcador.type = Marker.ARROW

        # Casi quieto: se borra la flecha.
        if rapidez < self.vel_minima:
            marcador.action = Marker.DELETE
            self.pub_flecha.publish(marcador)
            return

        p = self.pose_actual.pose.position

        marcador.action = Marker.ADD
        marcador.points = [
            Point(x=p.x, y=p.y, z=p.z),
            Point(x=p.x + self.escala * v.x,
                  y=p.y + self.escala * v.y,
                  z=p.z + self.escala * v.z),
        ]

        # Con dos puntos: x = grosor del palo, y = grosor de la punta,
        # z = largo de la punta.
        marcador.scale.x = 0.02
        marcador.scale.y = 0.04
        marcador.scale.z = 0.06

        marcador.color.r = float(self.color[0])
        marcador.color.g = float(self.color[1])
        marcador.color.b = float(self.color[2])
        marcador.color.a = 1.0

        self.pub_flecha.publish(marcador)


def main(args=None):
    rclpy.init(args=args)

    node = Rotaciones()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()