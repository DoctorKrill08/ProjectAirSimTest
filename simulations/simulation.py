from __future__ import annotations
from flight_controller import FlightController, Motor, State, Action

from pathlib import Path
from typing import Any, Callable, Dict, override

from projectairsim import Drone, ProjectAirSimClient, World

import time

class ProjectAirSimSimulation(FlightController):

    MOTOR_TO_ACTUATOR = {
        Motor.FL: "Prop_FL_actuator",
        Motor.FR: "Prop_FR_actuator",
        Motor.BL: "Prop_RL_actuator",
        Motor.BR: "Prop_RR_actuator",
    }
    SCENE = "scene_basic_drone.jsonc"
    DEFAULT_DRONE_NAME = "Drone1"
    DISPLAY_HZ = 20.0
    SIMULATION_FOLDER = Path(__file__).resolve().parent
    SIM_CONFIG_PATH = SIMULATION_FOLDER / "sim_config"

    def __init__(
        self,
    ) -> None:
        self.client: ProjectAirSimClient | None = None
        self.world: World | None = None
        self.drone: Drone | None = None

        self._started = False
    @override
    def start(self) -> None:
        if self._started:
            return

        self.client = ProjectAirSimClient()
        self.client.connect()

        self.start_time = time.perf_counter()
        self.elapsed = 0

        self.world = World(
            self.client,
            self.SCENE,
            delay_after_load_sec=0,
            sim_config_path=str(self.SIM_CONFIG_PATH),
        )

        self.drone = Drone(
            self.client,
            self.world,
            self.DEFAULT_DRONE_NAME,
        )

        #Pausing lets step() advance simulation time explicitly.
        self.world.pause()

        self._started = True

    @override
    def read(self) -> State:
        self._require_started()

        assert self.drone is not None

        kinematics = self.drone.get_ground_truth_kinematics()

        pose = kinematics["pose"]
        twist = kinematics["twist"]

        position = pose["position"]
        orientation = pose["orientation"]

        state = State(
            time_stamp=int(kinematics.get("time_stamp", 0)),
            position={
                "x": float(position["x"]),
                "y": float(position["y"]),
                "z": float(position["z"]),
            },
            rotation={
                "w": float(orientation["w"]),
                "x": float(orientation["x"]),
                "y": float(orientation["y"]),
                "z": float(orientation["z"]),
            },
            velocity={
                "x": float(twist["linear"]["x"]),
                "y": float(twist["linear"]["y"]),
                "z": float(twist["linear"]["z"]),
            },
            angular_velocity={
                "x": float(twist["angular"]["x"]),
                "y": float(twist["angular"]["y"]),
                "z": float(twist["angular"]["z"]),
            }
        )

        return state
    @override
    def send(self, actions: Action) -> None:
        actions = FlightController.normalize_action(actions)
        thrusts = actions.motor_thrusts
        self._require_started()

        assert self.drone is not None

        control_signals = {}
        for motor_name, actuator_name in self.MOTOR_TO_ACTUATOR.items():
            control_signals[actuator_name] = float(thrusts[motor_name])

        self.drone.set_control_signals(control_signals)

    def step(
        self,
        dt: float | None = None,
    ) -> None:
        self._require_started()

        if dt is None:
            raise ValueError("dt must be provided.")
        if dt <= 0:
            raise ValueError("dt must be greater than zero.")

        assert self.world is not None

        delta_time_ns = max(1, int(dt * 1_000_000_000))
        self.world.continue_for_sim_time(
            delta_time_ns,
            wait_until_complete=True,
        )
    @override
    def close(self) -> None:
        if not self._started:
            return

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
