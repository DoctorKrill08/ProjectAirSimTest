import asyncio
from enum import Enum
import time
import numpy as np
from drone import Drone

from simulations.simulation import ProjectAirSimSimulation
from flight_controller import FlightController, Motor, State, Action

CONTROL_HZ = 100.0
CONTROL_PERIOD = 1 / CONTROL_HZ
TAKE_OFF_TIME = 3 #seconds

SIMULATION_UPDATE_FREQUENCY = 50
SIMULATION_UPDATE_PERIOD = 1 / SIMULATION_UPDATE_FREQUENCY
async def main() -> None:

    simulation = ProjectAirSimSimulation()
    target = State(np.array([0, 0, 10]))
    drone = Drone(target=target, flight_controller=simulation)

    try:
        start_time = time.perf_counter()
        simulation.start()

        while True:
            dt = CONTROL_PERIOD
            await asyncio.sleep(SIMULATION_UPDATE_PERIOD)
            drone.update(
                action = Action(
                    motor_thrusts={
                        Motor.FL: 4.0, #Clockwise
                        Motor.FR: 4.0, #Counter-Clockwise
                        Motor.BL: 4.0, #Counter-Clockwise
                        Motor.BR: 4.0, #Clockwise
                    }
                )
            )
            simulation.step(dt)
            
    except KeyboardInterrupt as e:
        print("Ending")
    finally:
        simulation.close()



if __name__ == "__main__":
    asyncio.run(main())  # Runner for async main function

