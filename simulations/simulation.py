from __future__ import annotations

from Subsystems.flight_controller import FlightController, quaternion_to_rotation_matrix
from Subsystems.action import Action, Motor
from Subsystems.state import State
import numpy as np
from pathlib import Path
from typing import Any, Callable, Dict, override

from projectairsim import Drone, ProjectAirSimClient, World

import time

NANO_SECOND = 1_000_000_000

class ProjectAirSimSimulation(FlightController):

    MOTOR_TO_ACTUATOR = {
        Motor.FL: "Prop_FL_actuator",
        Motor.FR: "Prop_FR_actuator",
        Motor.BL: "Prop_RL_actuator",
        Motor.BR: "Prop_RR_actuator",
    }
    SCENE = "scene_basic_drone.jsonc"
    DEFAULT_DRONE_NAME = "Drone1"
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
    def read(self) -> tuple[State, np.double]:
        self._require_started()

        assert self.drone is not None

        kinematics = self.drone.get_ground_truth_kinematics()

        pose = kinematics["pose"]
        twist = kinematics["twist"]

        position = pose["position"]
        orientation = pose["orientation"]

        time_stamp= np.double(kinematics.get("time_stamp", 0) / NANO_SECOND)

        state = State(
            rotation=quaternion_to_rotation_matrix(
                float(orientation["w"]),
                float(orientation["x"]),
                float(orientation["y"]),
                float(orientation["z"]),
            ),
            position=np.array([
                float(position["x"]),
                float(position["y"]),
                float(position["z"]),
            ], dtype=float),
            velocity=np.array([
                float(twist["linear"]["x"]),
                float(twist["linear"]["y"]),
                float(twist["linear"]["z"]),
            ], dtype=float),
            angular_velocity=np.array([
                float(twist["angular"]["x"]),
                float(twist["angular"]["y"]),
                float(twist["angular"]["z"]),
            ], dtype=float)
        )

        return state,time_stamp
    @override
    def send(self, actions: Action) -> None:
        #super().send(actions)
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

        delta_time_ns = max(1, int(dt * NANO_SECOND))
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
