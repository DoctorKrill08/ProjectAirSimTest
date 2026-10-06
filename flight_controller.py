from enum import Enum

import numpy as np
import numpy as np

def quaternion_to_rotation_matrix(w, x, y, z):
    norm = np.sqrt(w*w + x*x + y*y + z*z)

    w /= norm
    x /= norm
    y /= norm
    z /= norm

    return np.array([
        [
            1 - 2*(y*y + z*z),
            2*(x*y - w*z),
            2*(x*z + w*y)
        ],
        [
            2*(x*y + w*z),
            1 - 2*(x*x + z*z),
            2*(y*z - w*x)
        ],
        [
            2*(x*z - w*y),
            2*(y*z + w*x),
            1 - 2*(x*x + y*y)
        ]
    ])

class Motor(Enum):
    FL = "FL",
    FR = "FR",
    BL = "BL", #Backleft
    BR = "BR"

class State():
    def __init__(
            self,
            position: np.ndarray = np.zeros(3,dtype=float),
            velocity: np.ndarray = np.zeros(3,dtype=float),
            rotation: np.ndarray = np.zeros((3,3),dtype=float),
            angular_velocity: np.ndarray = np.zeros(3,dtype=float),
            ):
        self.position = position
        self.velocity = velocity
        self.rotation = rotation
        self.angular_velocity = angular_velocity
    def to_string(self):
        return (
            f"position=({self.position[0]:+7.3f}, {self.position[1]:+7.3f}, {self.position[2]:+7.3f}) m | " +
            f"velocity=({self.velocity[0]:+7.3f}, {self.velocity[1]:+7.3f}, {self.velocity[2]:+7.3f}) m/s | " +
            f"rotation=([\n" +
            f"  {self.rotation[0,0]:+7.3f}, {self.rotation[0,1]:+7.3f}, {self.rotation[0,2]:+7.3f}\n" +
            f"  {self.rotation[1,0]:+7.3f}, {self.rotation[1,1]:+7.3f}, {self.rotation[1,2]:+7.3f}\n" +
            f"  {self.rotation[2,0]:+7.3f}, {self.rotation[2,1]:+7.3f}, {self.rotation[2,2]:+7.3f}\n" +
            f"]) rad | " +
            f"angular_velocity=({self.angular_velocity[0]:+7.3f}, {self.angular_velocity[1]:+7.3f}, {self.angular_velocity[2]:+7.3f}) rad/s"
        )

class Action():
    def __init__(
            self,
            motor_thrusts: dict = {Motor.FL: 0, Motor.FR: 0, Motor.BL: 0, Motor.BR: 0},
            ):
        self.motor_thrusts = motor_thrusts #Newtons
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
    def read(self) -> tuple[State, float]:
        #Returns the current state and the timestamp.
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
        return Action(motor_thrusts=normalized_motor_thrusts)
