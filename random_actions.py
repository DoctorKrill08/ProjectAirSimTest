from enum import Enum
import time
import numpy as np
import asyncio
from Subsystems.drone import Drone

from simulations.simulation import ProjectAirSimSimulation
from Subsystems.action import Action,ActionSequence, Motor
from Subsystems.state import State

#run with:
# python -m tests.fly_up

TAKE_OFF_TIME = 3 #seconds

RANDOM_ACTION_FREQUENCY = 20
RANDOM_ACTION_LENGTH = 3  # seconds
async def main() -> None:

    simulation = ProjectAirSimSimulation()
    target = State(position=np.array([0, 0, 10])) #dont worry about this yet
    drone = Drone(target=target, flight_controller=simulation)

    try:
        simulation.start()
        hover_sequence = ActionSequence.fly_up(time = TAKE_OFF_TIME)
        drone.action_sequence = hover_sequence
        while (await drone.update()):
            pass

        random_sequence = ActionSequence.generate_random_sequence(
            start_time = drone.time_stamp,
            time = RANDOM_ACTION_LENGTH, 
            frequency = RANDOM_ACTION_FREQUENCY
        )

        drone.action_sequence = random_sequence
        while (await drone.update()):
            pass
            
    except KeyboardInterrupt as e:
        print("Ending")
    finally:
        simulation.close()



if __name__ == "__main__":
    asyncio.run(main())  # Runner for main function

