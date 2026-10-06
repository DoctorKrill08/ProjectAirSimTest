from flight_controller import Action, State
import numpy as np

class Database:
    HISTORY_TIME: float = 0.5  # seconds
    SAVE_FREQUENCY: int = 20  # Hz
    STEPS: int = int(HISTORY_TIME * SAVE_FREQUENCY)

    def __init__(self) -> None:
        self.step = 0
        self.action_history : list[Action] = []
        self.state_history : list[State] = []
        self.time_history : list[np.double] = []

    def update_history(self, state: State, action: Action, time_stamp: np.double,display : bool = False):
        #Make sure enough time has passed to update the state history
        if (self.step > 0 and self.time_history[self.step - 1] is not None and time_stamp - self.time_history[self.step - 1] < self.HISTORY_TIME):
            return
        self.step = min(self.step, self.STEPS - 1)
        self.step = max(self.step, 0)

        self.state_history.append(state)
        self.action_history.append(action)
        self.time_history.append(time_stamp)
        
        if (self.step < self.STEPS - 1):
            self.step += 1
        else:
            self.state_history.pop(0)
            self.action_history.pop(0)
            self.time_history.pop(0)
        if display:
            print(self.to_string())
    def to_string(self):
        if (self.step == 0):
            return "No history available"
        return (
            f"Step: {self.step}\n" +
            f"Action History: {self.action_history[self.step - 1].to_string()}\n" +
            f"State History: {self.state_history[self.step - 1].to_string()}\n" +
            f"Time History: {self.time_history[self.step - 1]}"
        )