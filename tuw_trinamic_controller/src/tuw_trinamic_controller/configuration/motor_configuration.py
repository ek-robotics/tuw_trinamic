class MotorConfiguration:
    def __init__(self):
        self.pole_pairs = None
        self.max_torque = None
        self.hall_configuration = MotorHallConfiguration()
        self.motion_configuration = MotorMotionConfiguration()
        self.pid_configuration = MotorPIDConfiguration()


class MotorHallConfiguration:
    def __init(self):
        self.hall_invert = None


class MotorMotionConfiguration:
    def __init(self):
        self.max_velocity = None
        self.acceleration = None
        self.ramp_enable = None
        self.target_reached_velocity = None
        self.target_reached_distance = None
        self.motor_halted_velocity = None


class MotorPIDConfiguration:
    def __init(self):
        self.torque_p = None
        self.torque_i = None
        self.velocity_p = None
        self.velocity_i = None
        self.position_parameter = None
