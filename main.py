import asyncio


from flight_controller import BodyRateCommand, MockFlightController
from simulations.simulation import ProjectAirSimSimulation


# ---------------------------------------------------------------------------
# Temporary test commands
# ---------------------------------------------------------------------------

CONTROL_HZ = 100.0

# This mock controller interprets collective thrust as a normalized [0, 1]
# motor baseline. These values will need tuning for your specific robot config.
TAKEOFF_COLLECTIVE = 0.90
FLIGHT_COLLECTIVE = 0.84

# For now, command a constant roll body rate after takeoff.
# Units are rad/s.
FLIGHT_COMMAND = BodyRateCommand(
    roll_rate=0.20,
    pitch_rate=0.00,
    yaw_rate=0.00,
    collective_thrust=FLIGHT_COLLECTIVE,
)


async def main() -> None:
    flight_controller = MockFlightController()

    simulation = ProjectAirSimSimulation(
        control_hz=CONTROL_HZ,
        display_hz=5.0,
    )

    try:
        simulation.start()

        # ------------------------------------------------------------------
        # TAKEOFF
        # ------------------------------------------------------------------
        #
        # simulation.py owns the takeoff procedure, but it does not know how
        # to control the aircraft. Instead, main supplies a callback which
        # uses the flight controller.
        #
        takeoff_command = BodyRateCommand(
            roll_rate=0.0,
            pitch_rate=0.0,
            yaw_rate=0.0,
            collective_thrust=TAKEOFF_COLLECTIVE,
        )

        def takeoff_motor_supplier(actual_state, dt):
            motors, _state_estimate = flight_controller.update(
                command=takeoff_command,
                actual_state=actual_state,
                dt=dt,
            )
            return motors

        state = simulation.takeoff(
            motor_supplier=takeoff_motor_supplier,
            target_height_m=1.0,
            max_duration_s=3.0,
        )

        # Reset PID history so the normal flight section does not inherit
        # integral/derivative history from the takeoff phase.
        flight_controller.reset()

        # ------------------------------------------------------------------
        # NORMAL CONTROL LOOP
        # ------------------------------------------------------------------
        #
        # Later, this is where SkyJEPA + MPPI can replace FLIGHT_COMMAND:
        #
        #   image/state history
        #          |
        #          v
        #      SkyJEPA
        #          |
        #          v
        #        MPPI
        #          |
        #          v
        #   selected action
        #          |
        #          v
        # motor thrust action -> body rates + collective thrust
        #          |
        #          v
        #   flight_controller.update(...)
        #          |
        #          v
        #   simulation.step(...)
        #
        while True:
            await asyncio.sleep(1 / CONTROL_HZ)
            motor_outputs, state_estimate = flight_controller.update(
                command=FLIGHT_COMMAND,
                actual_state=state,
                dt=simulation.dt,
            )

            state = simulation.step(
                motor_outputs,
                dt=simulation.dt,
                display=True,
            )
    except KeyboardInterrupt as e:
        print("Ending")
    finally:
        final_position = state_estimate["position"]
        print(
            "Finished test. Last controller state estimate: "
            f"N={final_position['x']:.3f}, "
            f"E={final_position['y']:.3f}, "
            f"D={final_position['z']:.3f}"
        )
        simulation.close()



if __name__ == "__main__":
    asyncio.run(main())  # Runner for async main function

