"""Mide el error del filtro propio (1.1) y del DMP (1.2) en una prueba.

Escucha las dos poses que publica el ESP32:
    /imu_pose      + /imu_vel        (filtro propio, 1.1)
    /imu_pose_dmp  + /imu_vel_dmp    (DMP, 1.2)

Como funciona: espera `inicio_s` segundos (para que sueltes el sensor),
mide durante `duracion_s` segundos, imprime una tabla, y guarda un CSV
(y un grafico .png si hay matplotlib) en la carpeta actual.

Las posiciones se miden RELATIVAS al instante en que empieza la medicion.

Pruebas (parametro `prueba`):
  reposo    sensor quieto en la mesa. Lo ideal es 0 en todo.
            Mide la deriva de la posicion, la velocidad que aparece sola
            y cuanto se corre la orientacion.
  traslado  ida y vuelta de `real_m` metros sobre una regla.
            Mide cuanto se aleja (deberia ser real_m) y cuanto le falta
            para volver al punto de partida (deberia ser 0).
  giro      giro de `real_grados` grados contra una escuadra.
            Mide el angulo girado (deberia ser real_grados).

Ejemplos:
  python3 medir_error.py --ros-args -p prueba:=reposo -p duracion_s:=30.0
  python3 medir_error.py --ros-args -p prueba:=traslado -p real_m:=0.5 -p duracion_s:=10.0
  python3 medir_error.py --ros-args -p prueba:=giro -p real_grados:=90.0 -p duracion_s:=8.0
"""
import csv
import math
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PoseStamped, Vector3Stamped

METODOS = ('propio', 'dmp')
TOPICOS = {
    'propio': ('/imu_pose', '/imu_vel'),
    'dmp': ('/imu_pose_dmp', '/imu_vel_dmp'),
}


# ----------------------------------------------------------------------
# Matematica de orientaciones (cuaterniones w, x, y, z)
# ----------------------------------------------------------------------

def q_normalizar(q):
    n = math.sqrt(sum(c * c for c in q))
    if n < 1e-9:
        return (1.0, 0.0, 0.0, 0.0)
    return tuple(c / n for c in q)


def q_conjugado(q):
    return (q[0], -q[1], -q[2], -q[3])


def q_mult(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def angulo_entre(q1, q2):
    """Angulo total (grados) de la rotacion que lleva q1 a q2."""
    rel = q_mult(q_conjugado(q1), q2)
    w = min(1.0, abs(rel[0]))
    return math.degrees(2.0 * math.acos(w))


def vertical_en_cuerpo(q):
    """Donde esta 'arriba' (eje Z del mundo) visto desde el sensor.
    Solo depende de la inclinacion: no le afecta el yaw."""
    w, x, y, z = q
    return (
        2.0 * (x * z - w * y),
        2.0 * (w * x + y * z),
        w * w - x * x - y * y + z * z,
    )


def inclinacion_entre(q1, q2):
    """Cuanto cambio la inclinacion (roll/pitch) entre dos orientaciones,
    en grados. Ignora el yaw."""
    a = vertical_en_cuerpo(q1)
    b = vertical_en_cuerpo(q2)
    d = max(-1.0, min(1.0, sum(i * j for i, j in zip(a, b))))
    return math.degrees(math.acos(d))


def norma(v):
    return math.sqrt(sum(c * c for c in v))


# ----------------------------------------------------------------------
# Calculo de metricas a partir de las muestras (sin ROS: se puede probar)
# ----------------------------------------------------------------------

def calcular_metricas(muestras, prueba, real_m=None, real_grados=None):
    """muestras: lista de dicts con
         t, q_<m>, p_<m> (relativa), v_<m>   para m en METODOS.
       Devuelve {metodo: {metrica: valor}} y metricas entre metodos."""
    res = {}

    for m in METODOS:
        q0 = muestras[0]['q_' + m]
        qf = muestras[-1]['q_' + m]
        pf = muestras[-1]['p_' + m]
        vf = muestras[-1]['v_' + m]

        r = {
            'pos_final_m': norma(pf),
            'pos_max_m': max(norma(s['p_' + m]) for s in muestras),
            'vel_final_mps': norma(vf),
            'vel_max_mps': max(norma(s['v_' + m]) for s in muestras),
            'giro_total_deg': angulo_entre(q0, qf),
            'inclinacion_deg': inclinacion_entre(q0, qf),
        }

        if prueba == 'traslado' and real_m:
            r['error_alcance_m'] = r['pos_max_m'] - real_m
            r['error_alcance_pct'] = 100.0 * (r['pos_max_m'] - real_m) / real_m

        if prueba == 'giro' and real_grados:
            r['error_giro_deg'] = r['giro_total_deg'] - real_grados

        res[m] = r

    # Diferencia de inclinacion entre un metodo y el otro, en cada instante.
    dif = [
        inclinacion_entre(s['q_propio'], s['q_dmp']) for s in muestras
    ]
    # Ojo: dos orientaciones con distinto yaw tienen distinta "inclinacion
    # relativa" solo si la vertical difiere; como se compara la vertical
    # vista desde el sensor, el yaw no influye.
    entre = {
        'dif_inclinacion_media_deg': sum(dif) / len(dif),
        'dif_inclinacion_max_deg': max(dif),
        'dif_inclinacion_final_deg': dif[-1],
        'dif_posicion_final_m': norma([
            a - b for a, b in zip(muestras[-1]['p_propio'],
                                  muestras[-1]['p_dmp'])
        ]),
    }

    return res, entre


def formatear_tabla(res, entre, prueba, real_m, real_grados):
    filas = [
        ('Posicion final (deriva)  [m]', 'pos_final_m', '%.3f'),
        ('Alejamiento maximo       [m]', 'pos_max_m', '%.3f'),
        ('Velocidad final        [m/s]', 'vel_final_mps', '%.3f'),
        ('Velocidad maxima       [m/s]', 'vel_max_mps', '%.3f'),
        ('Giro total             [deg]', 'giro_total_deg', '%.2f'),
        ('Cambio de inclinacion  [deg]', 'inclinacion_deg', '%.2f'),
    ]

    if prueba == 'traslado' and real_m:
        filas.append(('Error de alcance        [m]', 'error_alcance_m', '%+.3f'))
        filas.append(('Error de alcance        [%]', 'error_alcance_pct', '%+.1f'))

    if prueba == 'giro' and real_grados:
        filas.append(('Error de giro          [deg]', 'error_giro_deg', '%+.2f'))

    out = []
    out.append('=' * 62)
    out.append(' Prueba: %s' % prueba)
    out.append('=' * 62)
    out.append('%-32s %12s %12s' % ('', '1.1 propio', '1.2 DMP'))
    out.append('-' * 62)

    for nombre, clave, fmt in filas:
        out.append('%-32s %12s %12s' % (
            nombre, fmt % res['propio'][clave], fmt % res['dmp'][clave]))

    out.append('-' * 62)
    out.append('Entre los dos metodos:')
    out.append('  diferencia de inclinacion media : %.2f deg'
               % entre['dif_inclinacion_media_deg'])
    out.append('  diferencia de inclinacion maxima: %.2f deg'
               % entre['dif_inclinacion_max_deg'])
    out.append('  diferencia de posicion final    : %.3f m'
               % entre['dif_posicion_final_m'])
    out.append('=' * 62)
    return '\n'.join(out)


# ----------------------------------------------------------------------
# Nodo de ROS 2
# ----------------------------------------------------------------------

class MedirError(Node):

    def __init__(self):
        super().__init__('medir_error')

        self.declare_parameter('prueba', 'reposo')
        self.declare_parameter('inicio_s', 5.0)
        self.declare_parameter('duracion_s', 30.0)
        self.declare_parameter('real_m', 0.5)
        self.declare_parameter('real_grados', 90.0)
        self.declare_parameter('carpeta', '.')

        self.prueba = self.get_parameter('prueba').value
        self.inicio_s = float(self.get_parameter('inicio_s').value)
        self.duracion_s = float(self.get_parameter('duracion_s').value)
        self.real_m = float(self.get_parameter('real_m').value)
        self.real_grados = float(self.get_parameter('real_grados').value)
        self.carpeta = self.get_parameter('carpeta').value

        if self.prueba not in ('reposo', 'traslado', 'giro'):
            raise ValueError('prueba debe ser reposo, traslado o giro')

        self.q = {m: None for m in METODOS}
        self.p = {m: None for m in METODOS}
        self.v = {m: None for m in METODOS}
        self.origen = None
        self.muestras = []
        self.terminado = False
        self.t_creado = time.monotonic()
        self.t_datos = None
        self._avisado = -1

        for m in METODOS:
            t_pose, t_vel = TOPICOS[m]
            self.create_subscription(
                PoseStamped, t_pose,
                lambda msg, m=m: self.al_recibir_pose(m, msg),
                qos_profile_sensor_data)
            self.create_subscription(
                Vector3Stamped, t_vel,
                lambda msg, m=m: self.al_recibir_vel(m, msg),
                qos_profile_sensor_data)

        self.create_timer(0.05, self.tomar_muestra)   # 20 Hz

        self.get_logger().info(
            'Prueba "%s": empieza en %.0f s y mide %.0f s. Esperando datos...'
            % (self.prueba, self.inicio_s, self.duracion_s))

    def al_recibir_pose(self, m, msg):
        o = msg.pose.orientation
        self.q[m] = q_normalizar((o.w, o.x, o.y, o.z))
        p = msg.pose.position
        self.p[m] = (p.x, p.y, p.z)

    def al_recibir_vel(self, m, msg):
        v = msg.vector
        self.v[m] = (v.x, v.y, v.z)

    def hay_datos(self):
        return all(
            self.q[m] is not None and self.v[m] is not None
            for m in METODOS)

    def tomar_muestra(self):
        if self.terminado:
            return

        if not self.hay_datos():
            ahora = int(time.monotonic() - self.t_creado)
            if ahora % 5 == 0 and ahora != self._avisado:
                self._avisado = ahora
                faltan = [
                    TOPICOS[m][0] for m in METODOS
                    if self.q[m] is None or self.v[m] is None]
                self.get_logger().warn(
                    'Todavia no llegan datos de: %s' % ', '.join(faltan))
            return

        if self.t_datos is None:
            self.t_datos = time.monotonic()

        t = time.monotonic() - self.t_datos

        # Cuenta regresiva para soltar el sensor.
        if t < self.inicio_s:
            restante = int(self.inicio_s - t) + 1
            if restante != self._avisado:
                self._avisado = restante
                self.get_logger().info('Empieza en %d...' % restante)
            return

        if self.origen is None:
            self.origen = {m: self.p[m] for m in METODOS}
            self.t_inicio = t
            self.get_logger().info('MIDIENDO. Hace el movimiento ahora.')

        t_med = t - self.t_inicio

        muestra = {'t': t_med}
        for m in METODOS:
            muestra['q_' + m] = self.q[m]
            muestra['p_' + m] = tuple(
                a - b for a, b in zip(self.p[m], self.origen[m]))
            muestra['v_' + m] = self.v[m]
        self.muestras.append(muestra)

        if t_med >= self.duracion_s:
            self.finalizar()

    def finalizar(self):
        self.terminado = True

        res, entre = calcular_metricas(
            self.muestras, self.prueba, self.real_m, self.real_grados)

        print()
        print(formatear_tabla(
            res, entre, self.prueba, self.real_m, self.real_grados))
        print()

        marca = datetime.now().strftime('%Y%m%d_%H%M%S')
        base = '%s/error_%s_%s' % (self.carpeta, self.prueba, marca)

        self.guardar_csv(base + '.csv')
        self.guardar_grafico(base + '.png')

        self.get_logger().info('Listo. Podes cerrar con Ctrl+C.')

    def guardar_csv(self, ruta):
        with open(ruta, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t_s'] + [
                '%s_%s' % (campo, m)
                for m in METODOS
                for campo in ('px', 'py', 'pz', 'vx', 'vy', 'vz',
                              'qw', 'qx', 'qy', 'qz')
            ] + ['dif_inclinacion_deg'])

            for s in self.muestras:
                fila = ['%.3f' % s['t']]
                for m in METODOS:
                    fila += ['%.5f' % c for c in s['p_' + m]]
                    fila += ['%.5f' % c for c in s['v_' + m]]
                    fila += ['%.6f' % c for c in s['q_' + m]]
                fila.append('%.3f' % inclinacion_entre(
                    s['q_propio'], s['q_dmp']))
                w.writerow(fila)

        self.get_logger().info('CSV guardado: %s' % ruta)

    def guardar_grafico(self, ruta):
        try:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt
        except ImportError:
            self.get_logger().info(
                'Sin matplotlib: no se genera el grafico (el CSV si).')
            return

        t = [s['t'] for s in self.muestras]
        fig, ax = plt.subplots(3, 1, figsize=(9, 9), sharex=True)

        for m, etiqueta in (('propio', '1.1 propio'), ('dmp', '1.2 DMP')):
            ax[0].plot(t, [norma(s['p_' + m]) for s in self.muestras],
                       label=etiqueta)
            ax[1].plot(t, [norma(s['v_' + m]) for s in self.muestras],
                       label=etiqueta)

        ax[2].plot(t, [inclinacion_entre(s['q_propio'], s['q_dmp'])
                       for s in self.muestras], color='gray')

        ax[0].set_ylabel('|posicion| [m]')
        ax[1].set_ylabel('|velocidad| [m/s]')
        ax[2].set_ylabel('dif. inclinacion\n entre metodos [deg]')
        ax[2].set_xlabel('tiempo [s]')
        ax[0].legend()
        ax[0].set_title('Prueba: %s' % self.prueba)

        for a in ax:
            a.grid(True, alpha=0.3)

        fig.tight_layout()
        fig.savefig(ruta, dpi=120)
        plt.close(fig)

        self.get_logger().info('Grafico guardado: %s' % ruta)


def main(args=None):
    rclpy.init(args=args)
    node = MedirError()

    try:
        while rclpy.ok() and not node.terminado:
            rclpy.spin_once(node, timeout_sec=0.1)

        # Dejar un instante para que se vea el resultado.
        time.sleep(0.2)
    except KeyboardInterrupt:
        pass

    node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()