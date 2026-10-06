from enum import Enum
import time
import numpy as np
from Subsystems.drone import Drone
import asyncio

from simulations.simulation import ProjectAirSimSimulation
from Subsystems.action import Action, Motor
from Subsystems.state import State

#run with:
# python -m tests.fly_up

CONTROL_HZ = 100.0
CONTROL_PERIOD = 1 / CONTROL_HZ
TAKE_OFF_TIME = 3 #seconds

async def main() -> None:

    simulation = ProjectAirSimSimulation()
    target = State(position=np.array([0, 0, 10])) #dont worry about this yet
    drone = Drone(target=target, flight_controller=simulation)

    try:
        simulation.start()

        while (await drone.update(
                action = Action(
                    motor_thrusts={
                        Motor.FL: 4.0, #Clockwise
                        Motor.FR: 4.0, #Counter-Clockwise
                        Motor.BL: 4.0, #Counter-Clockwise
                        Motor.BR: 4.0, #Clockwise
                    }
                ),
            )            
        ):
            pass
    except KeyboardInterrupt as e:
        print("Ending")
    finally:
        simulation.close()



if __name__ == "__main__":
    asyncio.run(main())  # Runner for main function

