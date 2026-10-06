import asyncio
from enum import Enum
import time
from drone import Motor, State, Action, Drone

from simulations.simulation import ProjectAirSimSimulation
from flight_controller import FlightController

CONTROL_HZ = 100.0
CONTROL_PERIOD = 1 / CONTROL_HZ
TAKE_OFF_TIME = 3 #seconds
async def main() -> None:

    simulation = ProjectAirSimSimulation()
    target = State([0,0,10])
    drone = Drone(target=target, flight_controller=simulation)

    try:
        start_time = time.perf_counter()
        simulation.start()

        while True:
            dt = time.perf_counter() - start_time
            if (dt < CONTROL_PERIOD):
                await asyncio.sleep(CONTROL_PERIOD - dt)
                dt = CONTROL_PERIOD
            simulation.send(Action(
                time_stamp=drone.state.time_stamp + dt,
                motor_thrusts={
                Motor.FL: 4.0, #Clockwise
                Motor.FR: 4.0, #Counter-Clockwise
                Motor.BL: 4.0, #Counter-Clockwise
                Motor.BR: 4.0, #Clockwise
            }))
            simulation.step(dt)
            drone.state = simulation.read()
            
    except KeyboardInterrupt as e:
        print("Ending")
    finally:
        simulation.close()



if __name__ == "__main__":
    asyncio.run(main())  # Runner for async main function

