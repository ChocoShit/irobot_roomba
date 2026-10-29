"""Control sencillo para Roomba 650 por su interfaz Open Interface."""

import argparse
import time

import serial


def word(value: int) -> bytes:
    """Codifica una palabra OI: valores con signo o el marcador especial 0x8000."""
    if value == 0x8000:  # Radio especial: conducir en línea recta.
        return b"\xff\xff"
    return int(value).to_bytes(2, byteorder="big", signed=True)


class Roomba:
    def __init__(self, port: str, baud: int) -> None:
        self.serial = serial.Serial(
            port=port,
            baudrate=baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1,
            write_timeout=2,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False,
        )
        self.last_failed_packet: int | None = None
        time.sleep(0.2)
        self.serial.write(bytes([128, 131]))  # Start, después Safe
        self.serial.flush()
        time.sleep(0.2)

    def drive(self, velocity: int, radius: int) -> None:
        # OI Drive (137): velocidad y radio como enteros con signo de 16 bits.
        self.serial.write(bytes([137]) + word(velocity) + word(radius))
        self.serial.flush()

    def drive_direct(self, right_velocity: int, left_velocity: int) -> None:
        # OI Drive Direct (145): controla cada rueda por separado.
        self.serial.write(bytes([145]) + word(right_velocity) + word(left_velocity))
        self.serial.flush()

    def stop(self) -> None:
        self.drive_direct(0, 0)

    def read_packet(self, packet_id: int, size: int = 1) -> bytes | None:
        self.serial.reset_input_buffer()
        self.serial.write(bytes([142, packet_id]))
        self.serial.flush()
        data = self.serial.read(size)
        if len(data) != size:
            self.last_failed_packet = packet_id
            return None
        self.last_failed_packet = None
        return data

    def bump_sensors(self) -> int | None:
        data = self.read_packet(7)
        return data[0] if data else None

    def check_safety_sensors(self) -> list[str] | None:
        # Safe mode rechaza movimiento si hay rueda levantada o detecta un desnivel.
        # Reintentar Safe: una parada por protección devuelve el OI a Passive (modo 1).
        self.serial.write(bytes([131]))
        self.serial.flush()
        time.sleep(0.05)
        bumps_data = self.read_packet(7)
        if bumps_data is None:
            return None
        bumps = bumps_data[0]
        blocked = []
        if bumps & 0b00000100:
            blocked.append("rueda derecha levantada")
        if bumps & 0b00001000:
            blocked.append("rueda izquierda levantada")
        if bumps & 0b00010000:
            blocked.append("rueda delantera levantada")
        cliff_names = [(9, "desnivel izquierdo"), (10, "desnivel frontal izquierdo"),
                       (11, "desnivel frontal derecho"), (12, "desnivel derecho")]
        for packet_id, name in cliff_names:
            state_data = self.read_packet(packet_id)
            if state_data is None:
                return None
            if state_data[0]:
                blocked.append(name)
        charger = self.read_packet(34)
        mode = self.read_packet(35)
        if charger is None or mode is None:
            return None
        if charger[0] != 0:
            blocked.append("cargador/base conectado")
        if mode[0] != 2:
            blocked.append(f"OI no está en modo seguro (modo {mode[0]})")
        return blocked

    def close(self) -> None:
        try:
            if self.serial.is_open:
                try:
                    self.stop()
                    self.serial.write(bytes([128]))  # Dejar la interfaz en modo pasivo.
                    self.serial.flush()
                except serial.SerialException:
                    pass
        finally:
            self.serial.close()


def move_for(robot: Roomba, right_velocity: int, left_velocity: int, duration: float) -> None:
    try:
        started = time.monotonic()
        robot.drive_direct(right_velocity, left_velocity)
        requested_right = robot.read_packet(41, 2)
        requested_left = robot.read_packet(42, 2)
        if requested_right is None or requested_left is None:
            print("No pude leer las velocidades solicitadas a las ruedas.")
        else:
            right = int.from_bytes(requested_right, byteorder="big", signed=True)
            left = int.from_bytes(requested_left, byteorder="big", signed=True)
            print(f"Velocidad solicitada al OI (derecha, izquierda): {right}, {left} mm/s")
        time.sleep(max(0.0, duration - (time.monotonic() - started)))
    finally:
        robot.stop()


def main() -> None:
    parser = argparse.ArgumentParser(description="Control por teclado de un Roomba 650")
    parser.add_argument("--port", default="COM8", help="Puerto serie (por defecto COM8)")
    parser.add_argument("--baud", type=int, default=115200, help="Baudios (por defecto 115200)")
    args = parser.parse_args()

    print(f"Conectando a {args.port} a {args.baud} baudios…")
    print("Enciende el Roomba con CLEAN y colócalo en una zona despejada.")
    robot = Roomba(args.port, args.baud)
    response = robot.bump_sensors()
    if response is None:
        print("COM abierto, pero no hay respuesta del Roomba. No se habilitarán movimientos.")
        print("Comprueba que esté encendido, fuera de la base y que el cable sea compatible con OI.")
        print("Prueba también: py main.py --baud 19200")
    else:
        print(f"Roomba responde por OI (paquete 7 = {response:#04x}).")
    print("Velocidad limitada a 200 mm/s; cada movimiento dura como máximo 10 s.")
    print("Comandos: w adelante, s atrás, a giro izquierda, d giro derecha, x parar, b sensores, q salir")

    try:
        while True:
            command = input("> ").strip().lower()
            if command == "q":
                break
            if command == "x":
                robot.stop()
                print("Parado.")
            elif command == "b":
                blocked = robot.check_safety_sensors()
                if blocked is None:
                    print(f"Sin respuesta al paquete {robot.last_failed_packet}: revisa baud, alimentación y cableado.")
                elif blocked:
                    print("Safe mode bloqueará movimiento:", ", ".join(blocked))
                else:
                    print("Sensores de rueda y desnivel: despejados.")
            elif command in ("w", "s", "a", "d"):
                try:
                    duration = min(10.0, max(0.1, float(input("Duración en segundos (0.1–10): "))))
                except ValueError:
                    print("Duración no válida.")
                    continue
                blocked = robot.check_safety_sensors()
                if blocked is None:
                    print(f"No muevo: falló la lectura del paquete {robot.last_failed_packet}. Prueba --baud 19200 y revisa el cableado.")
                    continue
                if blocked:
                    print("No muevo; Safe mode detectaría:", ", ".join(blocked))
                    print("Retíralo de la base, ponlo en suelo plano y revisa ruedas/sensores.")
                    continue
                robot.serial.write(bytes([131]))  # Reafirmar Safe tras una parada por sensor.
                robot.serial.flush()
                wheels = {
                    "w": (150, 150),
                    "s": (-150, -150),
                    "a": (100, -100),
                    "d": (-100, 100),
                }[command]
                move_for(robot, wheels[0], wheels[1], duration)
                print("Orden terminada; parada enviada. El programa no puede confirmar visualmente el movimiento.")
            else:
                print("Usa w, s, a, d, x, b o q.")
    except (KeyboardInterrupt, EOFError):
        print("\nInterrumpido; enviando parada…")
    finally:
        robot.close()
        print("Conexión cerrada.")


if __name__ == "__main__":
    main()
