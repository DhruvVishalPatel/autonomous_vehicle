"""4WD skid-steer motors on an L298N, using gpiozero's Robot (proven on this robot).

    m = Motors(cfg)
    m.drive(left, right)   # each -1.0 (full reverse) .. 1.0 (full forward)
    m.stop()
    m.close()
"""
from gpiozero import OutputDevice, Robot


class Motors:
    def __init__(self, cfg):
        c = cfg["motors"]
        self.ena = OutputDevice(c["ena"], initial_value=True)
        self.enb = OutputDevice(c["enb"], initial_value=True)
        self.robot = Robot(left=(c["left_forward"], c["left_backward"]),
                           right=(c["right_forward"], c["right_backward"]))
        self.inv_l = -1.0 if c.get("invert_left") else 1.0
        self.inv_r = -1.0 if c.get("invert_right") else 1.0
        self.max = float(c.get("max_speed", 1.0))

    def _clip(self, x):
        return max(-self.max, min(self.max, float(x)))

    def drive(self, left, right):
        self.robot.value = (self._clip(left) * self.inv_l, self._clip(right) * self.inv_r)

    def stop(self):
        self.robot.stop()

    def close(self):
        self.stop()
        self.robot.close()
        self.ena.close()
        self.enb.close()
