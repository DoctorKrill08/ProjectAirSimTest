from __future__ import annotations

from pathlib import Path
from typing import Any, Callable, Dict

from projectairsim import Drone, ProjectAirSimClient, World

import time


class ProjectAirSimSimulation:

    MOTOR_TO_ACTUATOR = {
        "front_left": "Prop_FL_actuator",
        "front_right": "Prop_FR_actuator",
        "rear_left": "Prop_RL_actuator",
        "rear_right": "Prop_RR_actuator",
    }

    def __init__(
        self,
        scene_config: str = "scene_basic_drone.jsonc",
        drone_name: str = "Drone1",
        control_hz: float = 100.0,
        display_hz: float = 20.0,
        sim_config_path: str | Path | None = None,
    ) -> None:
        if control_hz <= 0:
            raise ValueError("control_hz must be greater than zero.")

        self.scene_config = scene_config
        self.drone_name = drone_name
        self.control_hz = float(control_hz)
        self.dt = 1.0 / self.control_hz

        self.elapsed = 0
        self.start_time = time.perf_counter()

        self.display_hz = float(display_hz)
        self.display_period = (
            1.0 / self.display_hz
            if self.display_hz > 0
            else float("inf")
        )

        if sim_config_path is None:
            simulation_folder = Path(__file__).resolve().parent
            sim_config_path = simulation_folder / "sim_config"

        self.sim_config_path = Path(sim_config_path)

        self.client: ProjectAirSimClient | None = None
        self.world: World | None = None
        self.drone: Drone | None = None

        self._display_accumulator = 0.0
        self._started = False

    def start(self) -> None:
        """
        Connect to Project AirSim, load the world, initialize the drone, and
        pause the steppable simulation clock.

        For training, pausing the world and advancing it explicitly gives a
        deterministic control -> physics -> observation loop.
        """
        if self._started:
            return

        self.client = ProjectAirSimClient()
        self.client.connect()

        self.start_time = time.perf_counter()
        self.elapsed = 0

        self.world = World(
            self.client,
            self.scene_config,
            delay_after_load_sec=0,
            sim_config_path=str(self.sim_config_path),
        )

        self.drone = Drone(
            self.client,
            self.world,
            self.drone_name,
        )

        # scene_basic_drone.jsonc uses a steppable clock. Pausing here lets
        # step() advance simulation time explicitly.
        self.world.pause()

        self._started = True

    def get_ground_truth_state(self) -> Dict[str, Any]:
        """
        Return Project AirSim ground truth in a simulator-independent shape.

        Project AirSim documents ground-truth kinematics as:
            pose.position
            pose.orientation
            twist.linear
            twist.angular
            accels.linear
            accels.angular

        This starter treats twist.angular x/y/z as p/q/r feedback. If you later
        switch to a model/configuration whose angular twist is expressed in a
        different frame, perform the frame conversion here and keep the flight
        controller unchanged.
        """
        self._require_started()

        assert self.drone is not None

        kinematics = self.drone.get_ground_truth_kinematics()

        pose = kinematics["pose"]
        twist = kinematics["twist"]
        accels = kinematics["accels"]

        position = pose["position"]
        orientation = pose["orientation"]

        state = {
            "time_ns": int(kinematics.get("time_stamp", 0)),
            "position": {
                "x": float(position["x"]),
                "y": float(position["y"]),
                "z": float(position["z"]),
            },
            "orientation": {
                "w": float(orientation["w"]),
                "x": float(orientation["x"]),
                "y": float(orientation["y"]),
                "z": float(orientation["z"]),
            },
            "linear_velocity": {
                "x": float(twist["linear"]["x"]),
                "y": float(twist["linear"]["y"]),
                "z": float(twist["linear"]["z"]),
            },
            "body_rates": {
                "x": float(twist["angular"]["x"]),
                "y": float(twist["angular"]["y"]),
                "z": float(twist["angular"]["z"]),
            },
            "linear_acceleration": {
                "x": float(accels["linear"]["x"]),
                "y": float(accels["linear"]["y"]),
                "z": float(accels["linear"]["z"]),
            },
            "angular_acceleration": {
                "x": float(accels["angular"]["x"]),
                "y": float(accels["angular"]["y"]),
                "z": float(accels["angular"]["z"]),
            },
        }

        return state

    def apply_motor_outputs(self, motor_outputs: Dict[str, float]) -> None:
        """
        Send generic [0, 1] motor commands to Project AirSim actuators.
        """
        self._require_started()

        assert self.drone is not None

        missing = set(self.MOTOR_TO_ACTUATOR) - set(motor_outputs)
        if missing:
            raise KeyError(
                f"Missing motor outputs: {sorted(missing)}"
            )

        control_signals = {
            actuator_name: _clamp(
                float(motor_outputs[motor_name]),
                0.0,
                1.0,
            )
            for motor_name, actuator_name in self.MOTOR_TO_ACTUATOR.items()
        }

        self.drone.set_control_signals(control_signals)

    def step(
        self,
        motor_outputs: Dict[str, float],
        dt: float | None = None,
        display: bool = True,
    ) -> Dict[str, Any]:
        """
        Apply one action, advance simulation time, and return the new state.

        This is the key interface you will eventually call once per MPPI action.
        """
        self._require_started()

        if dt is None:
            dt = self.dt

        if dt <= 0:
            raise ValueError("dt must be greater than zero.")

        assert self.world is not None

        self.apply_motor_outputs(motor_outputs)

        delta_time_ns = max(1, int(dt * 1_000_000_000))
        self.world.continue_for_sim_time(
            delta_time_ns,
            wait_until_complete=True,
        )

        state = self.get_ground_truth_state()
        self.elapsed = self.start_time - time.perf_counter()

        if display:
            self._display_accumulator += dt
            if self._display_accumulator >= self.display_period:
                self.print_state(state)
                self._display_accumulator = 0.0

        return state

    def takeoff(
        self,
        motor_supplier: Callable[
            [Dict[str, Any], float],
            Dict[str, float],
        ],
        target_height_m: float = 1.0,
        max_duration_s: float = 3.0,
    ) -> Dict[str, Any]:

        self._require_started()

        if target_height_m <= 0:
            raise ValueError("target_height_m must be greater than zero.")

        state = self.get_ground_truth_state()
        starting_z = state["position"]["z"]
        target_z = starting_z - target_height_m

        elapsed = 0.0

        print(
            f"Takeoff: z={starting_z:.3f} m -> "
            f"target z={target_z:.3f} m"
        )

        while (
            state["position"]["z"] > target_z
            and elapsed < max_duration_s
        ):
            motor_outputs = motor_supplier(state, self.dt)
            state = self.step(
                motor_outputs,
                dt=self.dt,
                display=True,
            )
            elapsed += self.dt

        if state["position"]["z"] > target_z:
            print(
                "WARNING: Takeoff target was not reached before the "
                f"{max_duration_s:.1f} s timeout."
            )
        else:
            print(
                f"Takeoff complete at z={state['position']['z']:.3f} m."
            )

        return state
    @staticmethod
    def state_to_string(state: Dict[str, Any]) -> None:
        position = state["position"]
        rates = state["body_rates"]

        return(
            "STATE | " +
            f"pos NED=({position['x']:+7.3f}, " +
            f"{position['y']:+7.3f}, " +
            f"{position['z']:+7.3f}) m | " +
            f"rates=({rates['x']:+6.3f}, " +
            f"{rates['y']:+6.3f}, " +
            f"{rates['z']:+6.3f}) rad/s"
        )

    def print_state(self,state: Dict[str, Any]) -> None:
        print(ProjectAirSimSimulation.state_to_string(state) + f" |elapsed: {self.elapsed}")

    def close(self):
        if not self._started:
            return

        try:
            if self.drone is not None:
                try:
                    self.drone.set_control_signals(
                        {
                            actuator: 0.0
                            for actuator in self.MOTOR_TO_ACTUATOR.values()
                        }
                    )
                except Exception:
                    # Vehicle may not support manual control, or the
                    # connection may already be partially shut down.
                    pass

        finally:
            if self.client is not None:
                self.client.disconnect()

            self._started = False

    def _require_started(self) -> None:
        if not self._started:
            raise RuntimeError(
                "Simulation has not been started. Call start() first."
            )


def _clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))
