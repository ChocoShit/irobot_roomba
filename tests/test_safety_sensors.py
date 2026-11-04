import unittest
from unittest.mock import Mock, patch

from main import Roomba, move_for
from odometria import Teleoperacion


class FakeSerial:
    def __init__(self, response: bytes) -> None:
        self.response = response
        self.writes: list[bytes] = []
        self.read_sizes: list[int] = []

    def reset_input_buffer(self) -> None:
        pass

    def write(self, data: bytes) -> None:
        self.writes.append(data)

    def flush(self) -> None:
        pass

    def read(self, size: int) -> bytes:
        self.read_sizes.append(size)
        return self.response[:size]


class SafetySensorTests(unittest.TestCase):
    def make_robot(self, response: bytes) -> Roomba:
        robot = Roomba.__new__(Roomba)
        robot.serial = FakeSerial(response)
        robot.last_failed_packet = None
        return robot

    def test_reads_all_safety_sensors_with_one_query(self) -> None:
        robot = self.make_robot(bytes([0, 0, 0, 0, 0, 0, 2]))

        self.assertEqual(robot.read_safety_sensors(), [])
        self.assertEqual(robot.serial.writes, [bytes([149, 7, 7, 9, 10, 11, 12, 34, 35])])
        self.assertEqual(robot.serial.read_sizes, [7])
        self.assertIsNone(robot.last_failed_packet)

    def test_reports_bumpers_and_safety_conditions(self) -> None:
        robot = self.make_robot(bytes([3, 0, 1, 0, 0, 0, 2]))

        self.assertEqual(
            robot.read_safety_sensors(),
            ["parachoques derecho", "parachoques izquierdo", "desnivel frontal izquierdo"],
        )

    def test_reports_missing_sensor_response(self) -> None:
        robot = self.make_robot(b"")

        self.assertIsNone(robot.read_safety_sensors())
        self.assertEqual(robot.last_failed_packet, 149)

    def test_terminal_motion_stops_on_bumper_contact(self) -> None:
        robot = Mock()
        robot.read_safety_sensors.return_value = ["parachoques izquierdo"]
        with patch("main.time.monotonic", return_value=0):
            move_for(robot, 150, 150, 1)

        robot.drive_direct.assert_called_once_with(150, 150)
        robot.stop.assert_called_once()

    def test_gui_manual_motion_stops_on_bumper_contact(self) -> None:
        teleoperation = Teleoperacion.__new__(Teleoperacion)
        teleoperation.robot = Mock()
        teleoperation.robot.read_safety_sensors.return_value = ["parachoques derecho"]
        teleoperation.route_active = False
        teleoperation.orden = "up"
        teleoperation.sensor_estado = Mock()
        teleoperation.fallos = 0
        teleoperation.cancelar_ruta = Mock()
        teleoperation.fig = Mock()

        teleoperation.tick()

        teleoperation.cancelar_ruta.assert_called_once_with(
            "Movimiento detenido por contacto con un obstáculo.",
        )


if __name__ == "__main__":
    unittest.main()
