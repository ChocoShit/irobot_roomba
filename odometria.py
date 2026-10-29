"""Teleoperación del Roomba 650 con odometría y recorrido dibujado en vivo.

Guárdalo en la misma carpeta que main.py y ejecútalo con:  py odometria.py
Requiere matplotlib:  pip install matplotlib

Controles (con la ventana del gráfico seleccionada):
  flecha arriba / abajo     avanzar / retroceder
  flecha izquierda / derecha girar a la izquierda / a la derecha
  espacio o x               parar
  r                         reiniciar la posición a (0, 0, 0)
  i                         imprimir los acumulados crudos de distancia y ángulo
  q                         salir

Ruta con los controles de la ventana:
  F100,R90,F150             avanza 100 cm, gira 90° a la derecha, avanza 150 cm
  F = frente, B = atrás, L = giro izquierda, R = giro derecha
  El botón Volver invierte los tramos completados en orden inverso.
"""

import argparse
import math
import re

import serial

from main import Roomba  # reutiliza tu clase: conexión, drive_direct, sensores de seguridad

DISTANCIA_RUEDAS_MM = 235.0  # "b" de las diapositivas; ajústala si el giro dibujado sale de más o de menos
ANGULO_EN_GRADOS = True      # El paquete OI 20 del Roomba 600 devuelve el ángulo en grados.
ESCALA_GIRO = 1.0            # factor de calibración del giro (real / medido)
VEL_RECTO = 150              # mm/s
VEL_GIRO = 100               # mm/s por rueda
PERIODO_MS = 100             # cada cuánto se leen los sensores de odometría
REVERSE_ROUTE = {"F": "B", "B": "F", "L": "R", "R": "L"}


def parse_route(value: str) -> list[tuple[str, float]]:
    """Parsea, por ejemplo, F100,R90,F150 (distancias en cm, giros en grados)."""
    steps = []
    for chunk in value.split(","):
        match = re.fullmatch(r"\s*([FBLR])\s*(\d+(?:\.\d+)?)\s*", chunk, re.IGNORECASE)
        if not match:
            raise ValueError(f"Tramo no válido: {chunk!r}. Usa F100, B20, L90 o R90, separados por comas.")
        command = match.group(1).upper()
        amount = float(match.group(2))
        if amount <= 0:
            raise ValueError("Las distancias y los ángulos deben ser mayores que cero.")
        if command in "LR" and amount > 360:
            raise ValueError("El giro debe ser de 360 grados o menos.")
        if command in "FB" and amount > 500:
            raise ValueError("Cada tramo recto está limitado a 500 cm.")
        steps.append((command, amount))
    if not steps:
        raise ValueError("Escribe al menos un tramo de ruta.")
    return steps


def read_odometry(robot: Roomba) -> tuple[int, int] | None:
    """Lee Distance (paquete 19) y Angle (paquete 20) con una sola consulta (comando 149).

    Ambos valores son enteros con signo de 16 bits y se ponen a cero en cada lectura.
    """
    ser = robot.serial
    ser.reset_input_buffer()
    ser.write(bytes([149, 2, 19, 20]))
    ser.flush()
    data = ser.read(4)
    if len(data) != 4:
        return None
    distancia = int.from_bytes(data[0:2], byteorder="big", signed=True)
    angulo = int.from_bytes(data[2:4], byteorder="big", signed=True)
    return distancia, angulo


class Odometria:
    """Integra los incrementos (Distance, Angle) en una posición (x, y, theta)."""

    def __init__(self) -> None:
        self.reiniciar()

    def reiniciar(self) -> None:
        self.x = 0.0       # mm
        self.y = 0.0       # mm
        self.theta = 0.0   # rad, positivo hacia la izquierda (antihorario)
        self.camino = [(0.0, 0.0)]
        self.total_distancia = 0
        self.total_angulo = 0

    def actualizar(self, distancia: int, angulo: int) -> None:
        self.total_distancia += distancia
        self.total_angulo += angulo

        ds = float(distancia)
        if ANGULO_EN_GRADOS:
            dth = math.radians(angulo)
        else:
            dth = 2.0 * angulo / DISTANCIA_RUEDAS_MM  # fórmula de las diapositivas
        dth *= ESCALA_GIRO

        # Movimiento en el sistema del robot (arco de circunferencia de radio r).
        if abs(dth) < 1e-6:
            dx, dy = ds, 0.0  # casi recto: evita dividir por ~0
        else:
            r = ds / dth
            dx = r * math.sin(dth)
            dy = r * 2.0 * math.sin(dth / 2.0) ** 2  # equivale a r·(1 - cos dth), más estable

        # Paso al sistema global con la orientación anterior.
        c, s = math.cos(self.theta), math.sin(self.theta)
        self.x += dx * c - dy * s
        self.y += dx * s + dy * c
        self.theta += dth
        self.camino.append((self.x, self.y))


class Teleoperacion:
    ORDENES = {
        "up": (VEL_RECTO, VEL_RECTO),      # (rueda derecha, rueda izquierda)
        "down": (-VEL_RECTO, -VEL_RECTO),
        "left": (VEL_GIRO, -VEL_GIRO),
        "right": (-VEL_GIRO, VEL_GIRO),
    }

    def __init__(self, robot: Roomba) -> None:
        import matplotlib.pyplot as plt

        # Quita los atajos por defecto de matplotlib (guardar, zoom, retroceder...) para que
        # no choquen con nuestras teclas.
        for clave in list(plt.rcParams):
            if clave.startswith("keymap."):
                plt.rcParams[clave] = []

        self.plt = plt
        self.robot = robot
        self.odo = Odometria()
        self.orden: str | None = None
        self.fallos = 0
        self.route_active = False
        self.route_returning = False
        self.route_queue: list[tuple[str, float]] = []
        self.route_current: tuple[str, float] | None = None
        self.route_progress = 0.0
        self.route_done: list[tuple[str, float]] = []
        self.route_text = None

        self.fig, self.ax = plt.subplots(figsize=(9, 7))
        self.fig.subplots_adjust(bottom=0.24)
        self.ax.set_aspect("equal")
        self.ax.set_xlabel("x (m)")
        self.ax.set_ylabel("y (m)")
        self.ax.set_title("Odometría y rutas del Roomba 650")
        self.ax.grid(True)
        (self.linea,) = self.ax.plot([0.0], [0.0], "-", color="tab:blue")
        (self.punto,) = self.ax.plot([0.0], [0.0], "o", color="tab:red")
        (self.rumbo,) = self.ax.plot([], [], "-", color="tab:red")

        from matplotlib.widgets import Button, TextBox

        route_ax = self.fig.add_axes([0.14, 0.095, 0.40, 0.055])
        self.route_text = TextBox(route_ax, "Ruta ", initial="F100,R90,F150")
        run_ax = self.fig.add_axes([0.57, 0.095, 0.16, 0.055])
        return_ax = self.fig.add_axes([0.75, 0.095, 0.15, 0.055])
        stop_ax = self.fig.add_axes([0.92, 0.095, 0.07, 0.055])
        self.run_button = Button(run_ax, "Ejecutar")
        self.return_button = Button(return_ax, "Volver")
        self.stop_button = Button(stop_ax, "Parar")
        self.run_button.on_clicked(self.ejecutar_ruta)
        self.return_button.on_clicked(self.volver_ruta)
        self.stop_button.on_clicked(lambda _event: self.cancelar_ruta("Parada solicitada."))
        self.estado = self.fig.text(0.14, 0.035, "F/B: cm; L/R: grados. Flechas: teleoperación. Espacio: parada.")

        self.fig.canvas.mpl_connect("key_press_event", self.tecla)
        self.timer = self.fig.canvas.new_timer(interval=PERIODO_MS)
        self.timer.add_callback(self.tick)

    # --- control -----------------------------------------------------------------
    def tecla(self, evento) -> None:
        k = evento.key
        if k in self.ORDENES:
            if self.route_active:
                return  # evita mezclar teleoperación manual con la ruta automática
            if self.route_done:
                self.route_done = []  # un movimiento manual invalida el punto de retorno guardado
            if k == self.orden:
                return  # tecla mantenida: esa orden ya está en marcha
            if self.orden is None:  # arrancando desde parado: comprobar seguridad
                bloqueos = self.robot.check_safety_sensors()
                if bloqueos is None:
                    print("No muevo: sin respuesta del Roomba. Revisa baudios y cableado.")
                    return
                if bloqueos:
                    print("No muevo; Safe mode detectaría:", ", ".join(bloqueos))
                    return
            self.robot.drive_direct(*self.ORDENES[k])
            self.orden = k
        elif k in (" ", "x"):
            self.cancelar_ruta("Parada de emergencia solicitada.")
        elif k == "r":
            self.odo.reiniciar()
            self.dibujar()
        elif k == "i":
            print(f"Acumulado crudo: Distance = {self.odo.total_distancia} mm, "
                  f"Angle = {self.odo.total_angulo}")
        elif k == "q":
            self.cancelar_ruta("Cerrando mapa.")
            self.plt.close(self.fig)

    def parar(self) -> None:
        self.robot.stop()
        self.orden = None

    def ejecutar_ruta(self, _event=None) -> None:
        if self.route_active or self.orden is not None:
            self.estado.set_text("Para el movimiento actual antes de iniciar una ruta.")
            self.fig.canvas.draw_idle()
            return
        try:
            self.route_queue = parse_route(self.route_text.text)
        except ValueError as error:
            self.estado.set_text(str(error))
            self.fig.canvas.draw_idle()
            return
        blocked = self.robot.check_safety_sensors()
        if blocked is None or blocked:
            reason = "sin respuesta de sensores" if blocked is None else ", ".join(blocked)
            self.estado.set_text(f"Ruta no iniciada: {reason}.")
            self.fig.canvas.draw_idle()
            return
        self.route_done = []
        self.route_returning = False
        self.route_active = True
        self._siguiente_tramo()

    def volver_ruta(self, _event=None) -> None:
        if self.route_active or self.orden is not None:
            self.estado.set_text("Para el movimiento actual antes de regresar.")
            self.fig.canvas.draw_idle()
            return
        if not self.route_done:
            self.estado.set_text("No hay una ruta completada para regresar.")
            self.fig.canvas.draw_idle()
            return
        blocked = self.robot.check_safety_sensors()
        if blocked is None or blocked:
            reason = "sin respuesta de sensores" if blocked is None else ", ".join(blocked)
            self.estado.set_text(f"Regreso no iniciado: {reason}.")
            self.fig.canvas.draw_idle()
            return
        self.route_queue = [(REVERSE_ROUTE[cmd], amount)
                            for cmd, amount in reversed(self.route_done)]
        self.route_returning = True
        self.route_active = True
        self._siguiente_tramo()

    def _siguiente_tramo(self) -> None:
        if not self.route_queue:
            self.robot.stop()
            self.route_active = False
            self.route_current = None
            if self.route_returning:
                self.estado.set_text("Regreso terminado (posición estimada por odometría).")
                self.route_done = []
            else:
                self.estado.set_text("Ruta terminada. Pulsa «Volver» para invertir el recorrido.")
            self.fig.canvas.draw_idle()
            return

        blocked = self.robot.check_safety_sensors()
        if blocked is None or blocked:
            reason = "sin respuesta de sensores" if blocked is None else ", ".join(blocked)
            self.cancelar_ruta(f"Ruta detenida por seguridad: {reason}.")
            return

        self.route_current = self.route_queue.pop(0)
        command, amount = self.route_current
        self.route_progress = 0.0
        wheels = {
            "F": (VEL_RECTO, VEL_RECTO),
            "B": (-VEL_RECTO, -VEL_RECTO),
            "L": (VEL_GIRO, -VEL_GIRO),
            "R": (-VEL_GIRO, VEL_GIRO),
        }[command]
        self.robot.drive_direct(*wheels)
        unit = "cm" if command in "FB" else "°"
        action = {"F": "avanzando", "B": "retrocediendo", "L": "girando izquierda", "R": "girando derecha"}[command]
        self.estado.set_text(f"Ruta: {action} {amount:g} {unit}…")
        self.fig.canvas.draw_idle()

    def cancelar_ruta(self, message: str) -> None:
        if self.route_active and not self.route_returning and self.route_current:
            command, _amount = self.route_current
            done = self.route_progress / 10.0 if command in "FB" else self.route_progress
            if done > (0.5 if command in "FB" else 1.0):
                self.route_done.append((command, done))
        self.route_queue = []
        self.route_current = None
        self.route_active = False
        self.parar()
        self.estado.set_text(message)
        self.fig.canvas.draw_idle()

    def _actualizar_ruta(self, distancia: int, angulo: int) -> None:
        if not self.route_active or self.route_current is None:
            return
        command, amount = self.route_current
        if command == "F":
            self.route_progress += distancia
            target = amount * 10.0
        elif command == "B":
            self.route_progress -= distancia
            target = amount * 10.0
        elif command == "L":
            self.route_progress += angulo
            target = amount
        else:  # R: los giros horarios se reportan como ángulos negativos.
            self.route_progress -= angulo
            target = amount

        unit = "cm" if command in "FB" else "°"
        current = self.route_progress / 10.0 if command in "FB" else self.route_progress
        action = {"F": "avanzando", "B": "retrocediendo", "L": "girando izquierda", "R": "girando derecha"}[command]
        self.estado.set_text(f"Ruta: {action} {max(0.0, current):.1f}/{amount:g} {unit}")
        if self.route_progress >= target:
            self.robot.stop()
            if not self.route_returning:
                actual = self.route_progress / 10.0 if command in "FB" else self.route_progress
                self.route_done.append((command, actual))
            self.route_current = None
            self._siguiente_tramo()
        self.fig.canvas.draw_idle()

    # --- odometría y dibujo --------------------------------------------------------
    def tick(self) -> None:
        lectura = read_odometry(self.robot)
        if lectura is None:
            self.fallos += 1
            if self.fallos == 5:
                print("Sin respuesta del Roomba: parando.")
                if self.route_active:
                    self.cancelar_ruta("Ruta detenida: se perdió la comunicación con el Roomba.")
                else:
                    self.parar()
            return
        self.fallos = 0
        if lectura != (0, 0):
            self.odo.actualizar(*lectura)
            self.dibujar()
        if self.route_active:
            bump = self.robot.read_packet(7)
            if bump is None:
                self.cancelar_ruta("Ruta detenida: no responden los sensores de golpe.")
                return
            if bump[0] & 0b00000011:
                self.cancelar_ruta("Ruta detenida por contacto con un obstáculo.")
                return
            mode = self.robot.read_packet(35)
            if mode is None or mode[0] != 2:
                self.cancelar_ruta("Ruta detenida: Safe mode dejó de estar activo.")
                return
            self._actualizar_ruta(*lectura)

    def dibujar(self) -> None:
        xs = [p[0] / 1000.0 for p in self.odo.camino]  # mm -> m
        ys = [p[1] / 1000.0 for p in self.odo.camino]
        self.linea.set_data(xs, ys)
        self.punto.set_data([xs[-1]], [ys[-1]])
        largo = 0.15  # m, línea corta que indica hacia dónde mira el robot
        self.rumbo.set_data([xs[-1], xs[-1] + largo * math.cos(self.odo.theta)],
                            [ys[-1], ys[-1] + largo * math.sin(self.odo.theta)])
        cx, cy = (min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2
        medio = max(max(xs) - min(xs), max(ys) - min(ys), 1.0) / 2 * 1.2
        self.ax.set_xlim(cx - medio, cx + medio)
        self.ax.set_ylim(cy - medio, cy + medio)
        self.fig.canvas.draw_idle()

    def ejecutar(self) -> None:
        self.dibujar()
        self.timer.start()
        self.plt.show()


def main() -> None:
    parser = argparse.ArgumentParser(description="Teleoperación y odometría de un Roomba 650")
    parser.add_argument("--port", default="COM8")
    parser.add_argument("--baud", type=int, default=115200)
    args = parser.parse_args()

    try:
        robot = Roomba(args.port, args.baud)
    except serial.SerialException as error:
        print(f"No pude abrir {args.port}: {error}")
        print("Cierra main.py, otro programa o el monitor serie de VS Code que pueda estar usando ese puerto.")
        print("Comprueba en el Administrador de dispositivos que el adaptador siga asignado a COM8.")
        return
    try:
        if robot.bump_sensors() is None:
            print("COM abierto, pero el Roomba no responde. Revisa que esté encendido y los baudios.")
            return
        read_odometry(robot)  # descarta lo acumulado antes de empezar
        print("Usa las flechas en la ventana del gráfico; espacio para parar; q para salir.")
        Teleoperacion(robot).ejecutar()
    finally:
        robot.close()  # envía parada y cierra el puerto


if __name__ == "__main__":
    main()
