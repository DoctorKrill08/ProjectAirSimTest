import asyncio


from flight_controller import BodyRateCommand, MockFlightController
from simulations.simulation import ProjectAirSimSimulation

CONTROL_HZ = 100.0

TAKEOFF_COLLECTIVE_THRUST = 0.90
FLIGHT_COLLECTIVE_THRUST = 0.84

# For now, command a constant roll body rate after takeoff.
# Units are rad/s.
FLIGHT_COMMAND = BodyRateCommand(
    roll_rate=0.20,
    pitch_rate=0.00,
    yaw_rate=0.00,
    collective_thrust=FLIGHT_COLLECTIVE_THRUST,
)


async def main() -> None:
    flight_controller = MockFlightController()

    simulation = ProjectAirSimSimulation(
        control_hz=CONTROL_HZ,
    )

    try:
        simulation.start()

        takeoff_command = BodyRateCommand(
            roll_rate=0.0,
            pitch_rate=0.0,
            yaw_rate=0.0,
            collective_thrust=TAKEOFF_COLLECTIVE_THRUST,
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

