from enum import Enum
import time
import numpy as np
from Subsystems.drone import Drone

from simulations.simulation import ProjectAirSimSimulation
from Subsystems.action import Action,ActionSequence, Motor
from Subsystems.state import State

#run with:
# python -m tests.fly_up

CONTROL_HZ = 100.0
CONTROL_PERIOD = 1 / CONTROL_HZ
TAKE_OFF_TIME = 3 #seconds

SIMULATION_UPDATE_FREQUENCY = 100
SIMULATION_UPDATE_PERIOD = 1 / SIMULATION_UPDATE_FREQUENCY

RANDOM_ACTION_FREQUENCY = 20
RANDOM_ACTION_LENGTH = 4  # seconds
def main() -> None:

    simulation = ProjectAirSimSimulation()
    target = State(position=np.array([0, 0, 10])) #dont worry about this yet
    drone = Drone(target=target, flight_controller=simulation)

    try:
        simulation.start()
        hover_sequence = ActionSequence.fly_up(time = TAKE_OFF_TIME)
        action = hover_sequence.get_action(time_stamp=0)
        dt = CONTROL_PERIOD

        while action != None:
            action = hover_sequence.get_action(time_stamp=drone.time_stamp)
            if (action == None):
                break
            time.sleep(SIMULATION_UPDATE_PERIOD)
            drone.update(
                action = action
            )
            simulation.step(dt)

        random_sequence = ActionSequence.generate_random_sequence(
            start_time = drone.time_stamp,
            time = RANDOM_ACTION_LENGTH, 
            frequency = RANDOM_ACTION_FREQUENCY
        )

        action = random_sequence.get_action(time_stamp=0)
        while action != None:
            action = random_sequence.get_action(time_stamp=drone.time_stamp)
            if (action == None):
                break   
            time.sleep(SIMULATION_UPDATE_PERIOD)
            drone.update(
                action = action
            )
            simulation.step(dt)
            
    except KeyboardInterrupt as e:
        print("Ending")
    finally:
        simulation.close()



if __name__ == "__main__":
    main()  # Runner for main function

