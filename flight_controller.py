from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
from typing import Any, Dict


@dataclass(frozen=True)
class BodyRateCommand:
    """
    Desired low-level command sent to the flight controller.

    Rates are body-frame angular rates in rad/s:
        roll_rate  = p
        pitch_rate = q
        yaw_rate   = r

    collective_thrust is currently treated as a normalized [0, 1] collective
    motor command. Later, you can redefine this interface to use Newtons or
    another convention without changing the simulator.
    """

    roll_rate: float
    pitch_rate: float
    yaw_rate: float
    collective_thrust: float


class PID:
    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        integral_limit: float = 0.25,
        output_limit: float = 0.20,
    ) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.integral_limit = abs(integral_limit)
        self.output_limit = abs(output_limit)

        self.integral = 0.0
        self.previous_error = 0.0
        self.initialized = False

    def reset(self) -> None:
        self.integral = 0.0
        self.previous_error = 0.0
        self.initialized = False

    def update(self, error: float, dt: float) -> float:
        if dt <= 0.0:
            return 0.0

        self.integral += error * dt
        self.integral = _clamp(
            self.integral,
            -self.integral_limit,
            self.integral_limit,
        )

        derivative = 0.0
        if self.initialized:
            derivative = (error - self.previous_error) / dt
        else:
            self.initialized = True

        self.previous_error = error

        output = (
            self.kp * error
            + self.ki * self.integral
            + self.kd * derivative
        )

        return _clamp(
            output,
            -self.output_limit,
            self.output_limit,
        )


class MockFlightController:
    """
    Minimal mock flight controller.

    Responsibilities:
      1. Produce a state estimate.
         For now this is simply a copy of the simulator's ground-truth state.

      2. Track commanded body rates using three PID loops.

      3. Mix collective thrust + roll/pitch/yaw corrections into four
         normalized motor commands.

    The class has no Project AirSim imports. That is intentional: the flight
    controller should not care whether it is connected to Project AirSim,
    another simulator, or eventually real hardware.
    """

    def __init__(
        self,
        roll_gains: tuple[float, float, float] = (0.08, 0.00, 0.002),
        pitch_gains: tuple[float, float, float] = (0.08, 0.00, 0.002),
        yaw_gains: tuple[float, float, float] = (0.05, 0.00, 0.001),
    ) -> None:
        self.roll_pid = PID(*roll_gains)
        self.pitch_pid = PID(*pitch_gains)
        self.yaw_pid = PID(*yaw_gains)

        self.state_estimate: Dict[str, Any] | None = None

    def reset(self) -> None:
        self.roll_pid.reset()
        self.pitch_pid.reset()
        self.yaw_pid.reset()
        self.state_estimate = None

    def estimate_state(self, actual_state: Dict[str, Any]) -> Dict[str, Any]:
        """
        Mock state estimator.

        For now:
            estimated state == actual simulator ground-truth state

        Later this is where you can replace ground truth with an EKF or another
        estimator using IMU/GPS/barometer/etc.
        """
        self.state_estimate = deepcopy(actual_state)
        return self.state_estimate

    def update(
        self,
        command: BodyRateCommand,
        actual_state: Dict[str, Any],
        dt: float,
    ) -> tuple[Dict[str, float], Dict[str, Any]]:
        """
        Run one flight-controller iteration.

        Returns:
            motor_outputs:
                Generic normalized motor commands with keys:
                front_left, front_right, rear_left, rear_right

            state_estimate:
                The controller's current state estimate.
        """

        state = self.estimate_state(actual_state)

        body_rates = state["body_rates"]
        p = float(body_rates["x"])
        q = float(body_rates["y"])
        r = float(body_rates["z"])

        roll_correction = self.roll_pid.update(
            command.roll_rate - p,
            dt,
        )
        pitch_correction = self.pitch_pid.update(
            command.pitch_rate - q,
            dt,
        )
        yaw_correction = self.yaw_pid.update(
            command.yaw_rate - r,
            dt,
        )

        collective = _clamp(command.collective_thrust, 0.0, 1.0)

        # Quad-X mixer for the Project AirSim example geometry:
        #
        #               FRONT (+X)
        #
        #          FL                 FR
        #
        #
        #          RL                 RR
        #
        # Left motors have negative body Y.
        # Front motors have positive body X.
        #
        # Positive roll (+X):  increase left, decrease right
        # Positive pitch (+Y): increase front, decrease rear
        #
        # For yaw, the stock Project AirSim example uses:
        #   FL, RR = clockwise
        #   FR, RL = counter-clockwise
        #
        # If your custom vehicle's positive yaw response is reversed, invert
        # the four yaw signs below.
        front_left = (
            collective
            + roll_correction
            + pitch_correction
            - yaw_correction
        )
        front_right = (
            collective
            - roll_correction
            + pitch_correction
            + yaw_correction
        )
        rear_left = (
            collective
            + roll_correction
            - pitch_correction
            + yaw_correction
        )
        rear_right = (
            collective
            - roll_correction
            - pitch_correction
            - yaw_correction
        )

        motor_outputs = {
            "front_left": _clamp(front_left, 0.0, 1.0),
            "front_right": _clamp(front_right, 0.0, 1.0),
            "rear_left": _clamp(rear_left, 0.0, 1.0),
            "rear_right": _clamp(rear_right, 0.0, 1.0),
        }

        return motor_outputs, state


def _clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))
