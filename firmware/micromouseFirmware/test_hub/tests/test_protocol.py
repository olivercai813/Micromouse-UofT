import unittest

from micromouse_hub.protocol import MAX_PWM, Telemetry, clamp_pwm, motor_command, parse_line


class ProtocolTests(unittest.TestCase):
    def test_pwm_is_clamped(self):
        self.assertEqual(clamp_pwm(100), MAX_PWM)
        self.assertEqual(clamp_pwm(-100), -MAX_PWM)
        self.assertEqual(motor_command(90, -90, 12_000), "M10 710 -710 10000")

    def test_negative_pwm_is_preserved_for_reverse(self):
        self.assertEqual(motor_command(-50.1, -35.5, 1000), "M10 -501 -355 1000")

    def test_pwm_is_rounded_to_one_decimal_place(self):
        self.assertEqual(clamp_pwm(12.345), 12.3)
        self.assertEqual(motor_command(12.34, -12.36, 500), "M10 123 -124 500")

    def test_parse_telemetry(self):
        packet = "T,1000,501,-355,360,-180,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,255"
        result = parse_line(packet)
        self.assertIsInstance(result, Telemetry)
        assert isinstance(result, Telemetry)
        self.assertEqual(result.left_rotations, 1.0)
        self.assertEqual(result.right_rotations, -0.5)
        self.assertEqual(result.left_pwm, 50.1)
        self.assertEqual(result.right_pwm, -35.5)
        self.assertEqual(result.flags, 255)

    def test_parse_response(self):
        self.assertEqual(parse_line("ACK HELLO"), ("ACK", "HELLO"))

    def test_reject_malformed_packet(self):
        with self.assertRaises(ValueError):
            parse_line("T,1,2")


if __name__ == "__main__":
    unittest.main()
