from enum import Enum
import time
import numpy as np
from Subsystems.drone import Drone
import asyncio

from simulations.simulation import ProjectAirSimSimulation
from Subsystems.action import Action, Motor, ActionSequence
from Subsystems.state import State

async def main() -> None:

    simulation = ProjectAirSimSimulation()
    target = State(position=np.array([0, 0, 10])) #dont worry about this yet
    drone = Drone(target=target, flight_controller=simulation)
    action_sequence = ActionSequence.generate_from_csv("action_sequences/up_and_down.csv")
    drone.action_sequence = action_sequence
    try:
        simulation.start()

        while (await drone.update()):
            pass
    except KeyboardInterrupt as e:
        print("Ending")
    finally:
        simulation.close()



if __name__ == "__main__":
    asyncio.run(main())  # Runner for main function

