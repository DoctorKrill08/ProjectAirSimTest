from enum import Enum

import numpy as np
from Subsystems.action import Action
from Subsystems.state import State
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


#Generic Flight Controller Template, assuming using pyserial or something similiar
class FlightController():
    CONTROL_HZ = 100.0
    def __init__(self) -> None:
        pass
    def send(self,action : Action) -> None:
        action.normalize()
        pass
    def read(self) -> tuple[State, float]:
        #Returns the current state and the timestamp.
        pass
    def open(self) -> None:
        pass
    def step(self,action: Action | None = None,
            dt: np.double | None = None,) -> tuple[State, np.double]:
        #Steps the flight controller and returns the current state and timestamp.
        pass
    def close(self) -> None:
        pass
