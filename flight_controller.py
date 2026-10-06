from enum import Enum
class Motor(Enum):
    FL = "FL",
    FR = "FR",
    BL = "BL", #Backleft
    BR = "BR"

class State():
    def __init__(
            self,
            time_stamp: float,
            position: list = [0, 0, 0],
            velocity: list = [0, 0, 0],
            rotation: list = [0, 0, 0],
            angular_velocity: list = [0, 0, 0],
            
            ):
        self.position = position
        self.velocity = velocity
        self.rotation = rotation
        self.angular_velocity = angular_velocity
        self.time_stamp = time_stamp
    def to_string(self):
        return (
            f"position=({self.position[0]:+7.3f}, {self.position[1]:+7.3f}, {self.position[2]:+7.3f}) m | " +
            f"velocity=({self.velocity[0]:+7.3f}, {self.velocity[1]:+7.3f}, {self.velocity[2]:+7.3f}) m/s | " +
            f"rotation=({self.rotation[0]:+7.3f}, {self.rotation[1]:+7.3f}, {self.rotation[2]:+7.3f}) rad | " +
            f"angular_velocity=({self.angular_velocity[0]:+7.3f}, {self.angular_velocity[1]:+7.3f}, {self.angular_velocity[2]:+7.3f}) rad/s"
        )

class Action():
    def __init__(
            self,
            time_stamp: float,
            motor_thrusts: dict = {Motor.FL: 0, Motor.FR: 0, Motor.BL: 0, Motor.BR: 0},
            ):
        self.motor_thrusts = motor_thrusts #Newtons
        self.time_stamp = time_stamp
        for thrust in self.motor_thrusts.values():
            if thrust < 0:
                raise ValueError("Motor thrust cannot be negative")
    def to_string(self):
        return (
            f"motor_thrusts=({self.motor_thrusts[Motor.FL]:+7.3f}, {self.motor_thrusts[Motor.FR]:+7.3f}, " +
            f"{self.motor_thrusts[Motor.BL]:+7.3f}, {self.motor_thrusts[Motor.BR]:+7.3f})"
        )
    def clamp(value: float, minimum: float, maximum: float) -> float:
        return max(minimum, min(maximum, value))

#Generic Flight Controller Template, assuming using pyserial or something similiar
class FlightController():
    CONTROL_HZ = 100.0
    ASSUMED_MAX_THRUST = 4.0 #Newtons
    def __init__(self) -> None:
        pass
    def send(self,action : Action) -> None:
        action = FlightController.normalize_action(action)
        pass
    def read(self) -> State:
        pass
    def open(self) -> None:
        pass
    def close(self) -> None:
        pass
    @staticmethod
    def normalize_action(action: Action) -> Action:
        normalized_motor_thrusts = {
            motor: min(max(thrust / FlightController.ASSUMED_MAX_THRUST, 0.0), 1.0)
            for motor, thrust in action.motor_thrusts.items()
        }
        return Action(action.time_stamp,motor_thrusts=normalized_motor_thrusts)