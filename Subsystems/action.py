from enum import Enum

class Motor(Enum):
    FL = "FL",
    FR = "FR",
    BL = "BL", #Backleft
    BR = "BR"

class Action():
    ASSUMED_MAX_THRUST : float = 4.0  # Newtons
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
    def normalize(self) ->None:
        for motor in self.motor_thrusts:
            self.motor_thrusts[motor] = min(max(self.motor_thrusts[motor] / Action.ASSUMED_MAX_THRUST, 0.0), 1.0)

class ActionSequence():
    TIME_STAMP_FREQUENCY : int = 20  # Hz
    def __init__(self, frequency: float | None = None) -> None:
        self.actions: list[Action] = []
        self.time_stamps: list[float] = []
        self.frequency = frequency if frequency is not None else ActionSequence.TIME_STAMP_FREQUENCY

    def append(self, action: Action, time_stamp: float) -> None:
        self.actions.append(action)
        self.time_stamps.append(time_stamp)

    def to_string(self) -> str:
        return "\n".join(
            f"{time_stamp:+7.3f}: {action.to_string()}"
            for action, time_stamp in zip(self.actions, self.time_stamps)
        )
    def get_action(self, time_stamp: float) -> Action | None:
        if not self.time_stamps:
            return None
        start_time = self.time_stamps[0] if self.time_stamps else 0
        length : int = len(self.time_stamps)
        index : int = int((time_stamp - start_time) * self.frequency)

        if index <= length - 1:
            return self.actions[index]
        return None
    def to_string(self) -> str:
        return "\n".join(
            f"{time_stamp:+7.3f}: {action.to_string()}"
            for action, time_stamp in zip(self.actions, self.time_stamps)
        )

    @staticmethod
    def generate_random_sequence(time : float, frequency: float | None = None, start_time : float = 0) -> "ActionSequence":
        import random
        sequence = ActionSequence(frequency=frequency)
        steps = int(time * sequence.frequency)
        for i in range(steps):
            action = Action(
                motor_thrusts={
                    Motor.FL: random.uniform(0, Action.ASSUMED_MAX_THRUST),
                    Motor.FR: random.uniform(0, Action.ASSUMED_MAX_THRUST),
                    Motor.BL: random.uniform(0, Action.ASSUMED_MAX_THRUST),
                    Motor.BR: random.uniform(0, Action.ASSUMED_MAX_THRUST),
                }
            )
            time_stamp = start_time + (i / sequence.frequency)
            sequence.append(action, time_stamp)
        return sequence
    @staticmethod
    def fly_up(time : float, frequency: float | None = None, start_time : float = 0) -> "ActionSequence":
        sequence = ActionSequence(frequency=frequency)
        steps = int(time * sequence.frequency)
        for i in range(steps):
            action = Action(
                motor_thrusts={
                    Motor.FL: Action.ASSUMED_MAX_THRUST,
                    Motor.FR: Action.ASSUMED_MAX_THRUST,
                    Motor.BL: Action.ASSUMED_MAX_THRUST,
                    Motor.BR: Action.ASSUMED_MAX_THRUST,
                }
            )
            time_stamp = start_time + (i / sequence.frequency)
            sequence.append(action, time_stamp)
        return sequence