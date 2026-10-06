from time import time

from flight_controller import Action, FlightController, State
from database import Database


class Drone():
    CONTROL_HZ = 100.0
    def __init__(self, target : State | None = None, flight_controller: FlightController | None = None):
        self.state = State(0)
        self.target = target
        self.flight_controller = flight_controller
        self.database = Database()
        if self.flight_controller is None:
            raise ValueError("A flight controller must be provided.")
    def to_string(self):
        return (
            f"state=({self.state.to_string()}) | " +
            f"target=({self.target.to_string() if self.target else 'None'})"
        )
    
    def update(self, action : Action):
        self.state,time_stamp = self.flight_controller.read()
        self.flight_controller.send(action)
        self.database.update_history(self.state, action, time_stamp, True)