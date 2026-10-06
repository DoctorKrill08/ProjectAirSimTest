from enum import Enum
from flight_controller import State, Action, Motor, FlightController
class Drone():
    CONTROL_HZ = 100.0
    def __init__(self, target : State = None, flight_controller: FlightController = None):
        self.state = State(0)
        self.action_history = []
        self.state_history = []
        self.target = target
        self.flight_controller = flight_controller
        if self.flight_controller is None:
            raise ValueError("A flight controller must be provided.")
    def to_string(self):
        return (
            f"state=({self.state.to_string()}) | " +
            f"target=({self.target.to_string() if self.target else 'None'})"
        )
