from time import time
import numpy as np
import asyncio
from Subsystems import action
from Subsystems.flight_controller import FlightController, State
from Subsystems.action import Action, ActionSequence
from Subsystems.database import Database


class Drone():
    def __init__(self, target : State | None = None, flight_controller: FlightController | None = None, action_sequence : ActionSequence | None = None):
        self.state = State(0)
        self.target = target
        self.flight_controller = flight_controller
        self.database = Database()
        self.time_stamp = 0
        self.action_sequence = action_sequence
        if self.flight_controller is None:
            raise ValueError("A flight controller must be provided.")
    def to_string(self):
        return (
            f"state=({self.state.to_string()}) | " +
            f"target=({self.target.to_string() if self.target else 'None'})"
        )
    
    async def update(self, action : Action | None = None, display: bool = True) -> bool:
        await asyncio.sleep(FlightController.CONTROL_PERIOD)
        if action is None and self.action_sequence is not None:
            action = self.action_sequence.get_action(self.time_stamp)
        if (action is None):
            return False
        self.state,self.time_stamp = self.flight_controller.step(action=action)
        self.database.update_history(self.state, action, self.time_stamp, display = display)
        return True